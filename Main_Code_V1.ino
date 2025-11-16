#include <HardwareSerial.h>
#include "DHT.h"
#include "HX711.h"
#include <AccelStepper.h>

// === UART Communication (ESP32 <-> Raspberry Pi) ===
HardwareSerial SerialPi(1);
#define RXD2 13
#define TXD2 14

// === DHT22 Sensor ===
#define DHTPIN 23
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// === Moisture Sensor ===
#define MOISTURE_PIN        34
#define MOISTURE_POWER_PIN  25
int MOISTURE_DRY_RAW = 0;
int MOISTURE_WET_RAW = 2048;

// === Temperature Control (Relays) ===
#define HEATING_PIN  27
#define COOLING_PIN  26

// === HX711 Load Cell (10kg) ===
#define DOUT_10KG 35
#define SCK_PIN   33
HX711 scale10kg;
float calibration_factor_10kg = 450.0;

// === DRV8825 Stepper Motor (for BPA Dispensing) ===
#define DIR_PIN    19
#define STEP_PIN   18
#define ENABLE_PIN 21
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
bool motorEnabled = false;

// === Constants ===
#define AVG_READINGS 5
float avgBuffer10kg[AVG_READINGS];
int avgIndex = 0;

const float TEMP_MIN = 35.0;
const float TEMP_MAX = 55.0;
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 5000;

// === Dispensing Variables ===
bool isDispensing = false;
bool isCoolingDown = false;
float targetDispenseAmount = 0.0;   // Target BPA (kg)
float weightBeforeDispense = 0.0;
float weightTarget = 0.0;
unsigned long dispenseStartTime = 0;
unsigned long cooldownStartTime = 0;
const unsigned long DISPENSE_TIMEOUT = 20000;    // 20s safety cutoff
const unsigned long COOLDOWN_DURATION = 120000;  // 2 minutes cooldown

// === Helper Functions ===
float smooth(float newVal, float buffer[], int size) {
  buffer[avgIndex % size] = newVal;
  float sum = 0;
  for (int i = 0; i < size; i++) sum += buffer[i];
  return sum / size;
}

int readMoistureRaw(uint8_t samples = 10) {
  digitalWrite(MOISTURE_POWER_PIN, HIGH);
  delay(200);
  long sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += analogRead(MOISTURE_PIN);
    delay(5);
  }
  digitalWrite(MOISTURE_POWER_PIN, LOW);
  return (int)(sum / samples);
}

float rawToMoisturePct(int raw) {
  if (MOISTURE_DRY_RAW == MOISTURE_WET_RAW) return 50.0f;
  float pct = 100.0f * (float)(raw - MOISTURE_DRY_RAW) / (float)(MOISTURE_WET_RAW - MOISTURE_DRY_RAW);
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return pct;
}

void enableMotor(bool state) {
  digitalWrite(ENABLE_PIN, state ? LOW : HIGH); // LOW = enabled
  motorEnabled = state;
}

void startDispensing(float manureWeight) {
  if (isCoolingDown) {
    Serial.println("⚠️ Still cooling down, please wait...");
    return;
  }

  // Example formula: 10% of manure weight
  targetDispenseAmount = manureWeight * 0.10;

  if (scale10kg.is_ready()) {
    weightBeforeDispense = scale10kg.get_units(5);
    weightTarget = weightBeforeDispense + targetDispenseAmount;
  }

  Serial.printf("🐔 Manure: %.2f kg → BPA target: %.2f kg\n", manureWeight, targetDispenseAmount);

  enableMotor(true);
  digitalWrite(DIR_PIN, HIGH); // CW direction
  delay(5);
  stepper.setSpeed(800);
  isDispensing = true;
  dispenseStartTime = millis();
}

void stopDispensing() {
  stepper.setSpeed(0);
  enableMotor(false);
  isDispensing = false;
  isCoolingDown = true;
  cooldownStartTime = millis();
  Serial.println("✅ Dispensing complete — starting 2 min cooldown...");
}

void controlTemperature(float currentTemp) {
  // Heating Control
  if (currentTemp < TEMP_MIN) {
    digitalWrite(HEATING_PIN, LOW);   // Turn ON heater (active LOW type)
    Serial.println("Heater: ON (Temperature below minimum)");
  } else {
    digitalWrite(HEATING_PIN, HIGH);  // Turn OFF heater
    Serial.println("Heater: OFF");
  }

  // Cooling Control
  if (currentTemp > TEMP_MAX) {
    digitalWrite(COOLING_PIN, LOW);   // Turn ON cooler (active LOW type)
    Serial.println("Cooler: ON (Temperature above maximum)");
  } else {
    digitalWrite(COOLING_PIN, HIGH);  // Turn OFF cooler
    Serial.println("Cooler: OFF");
  }

  // Print divider for readability
  Serial.println("---------------------------------");
}

void sendMetric(String metric, float value) {
  String payload = "{\"metric\":\"" + metric + "\",\"value\":" + String(value, 2) + "}\n";
  SerialPi.print(payload);
  Serial.println("Sent: " + payload);
}

// === Setup ===
void setup() {
  Serial.begin(115200);
  SerialPi.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("ESP32 Composting + BPA Dispenser Starting...");

  dht.begin();
  scale10kg.begin(DOUT_10KG, SCK_PIN);
  scale10kg.set_scale(calibration_factor_10kg);
  scale10kg.tare();

  pinMode(MOISTURE_POWER_PIN, OUTPUT);
  digitalWrite(MOISTURE_POWER_PIN, LOW);
  analogSetPinAttenuation(MOISTURE_PIN, ADC_11db);

  pinMode(HEATING_PIN, OUTPUT);
  pinMode(COOLING_PIN, OUTPUT);
  digitalWrite(HEATING_PIN, LOW);
  digitalWrite(COOLING_PIN, LOW);

  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);
  enableMotor(false);

  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);

  for (int i = 0; i < AVG_READINGS; i++) avgBuffer10kg[i] = 0;
  randomSeed(analogRead(0));
}

// === Loop ===
void loop() {
  unsigned long currentTime = millis();

  // --- Periodic sensor reading ---
  if (currentTime - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = currentTime;

    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    if (isnan(temperature)) temperature = 25.0;
    if (isnan(humidity)) humidity = 60.0;

    controlTemperature(temperature);

    int moistureRaw = readMoistureRaw();
    float moisturePct = rawToMoisturePct(moistureRaw);

    float weight10kg = 0;
    if (scale10kg.is_ready()) {
      float w = scale10kg.get_units(5);
      if (w < 0) w = 0;
      weight10kg = smooth(w, avgBuffer10kg, AVG_READINGS);
    }

    sendMetric("temperature", temperature);
    sendMetric("humidity", humidity);
    sendMetric("moisture", moisturePct);
    sendMetric("weight10kg", weight10kg);

    Serial.printf("Temp:%.1f°C | Hum:%.1f%% | Moist:%.1f%% | Weight:%.2fkg | CoolingDown:%d\n",
                  temperature, humidity, moisturePct, weight10kg, isCoolingDown);
  }

  // --- Dispensing control ---
  if (isDispensing) {
    if (!scale10kg.is_ready()) return;
    float currentWeight = scale10kg.get_units(3);

    if (currentWeight >= weightTarget || (millis() - dispenseStartTime > DISPENSE_TIMEOUT)) {
      stopDispensing();
    } else {
      stepper.runSpeed();
    }
  }

  // --- Cooldown management ---
  if (isCoolingDown && millis() - cooldownStartTime >= COOLDOWN_DURATION) {
    isCoolingDown = false;
    Serial.println("🟢 Cooldown complete. Ready for next cycle!");
  }

  // --- Listen to commands from Pi ---
  while (SerialPi.available()) {
    String cmd = SerialPi.readStringUntil('\n');
    cmd.trim();
    Serial.println("Received from Pi: " + cmd);

    if (cmd.startsWith("{") && cmd.indexOf("\"dispense\"") != -1) {
      if (scale10kg.is_ready()) {
        float manureWeight = scale10kg.get_units(5);
        if (manureWeight > 0.2) {
          startDispensing(manureWeight);
        } else {
          Serial.println("⚠️ Not enough manure detected to dispense BPA.");
        }
      }
    }
  }

  stepper.runSpeed();
}
