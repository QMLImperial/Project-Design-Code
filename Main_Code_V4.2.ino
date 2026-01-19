#include <HardwareSerial.h>
#include "DHT.h"
#include "HX711.h"
#include <AccelStepper.h>
#include <ModbusMaster.h>

// === UART Communication (ESP32 <-> Raspberry Pi) ===
HardwareSerial SerialPi(1);
#define RXD2 13
#define TXD2 14

// === RS485 / Modbus for NPK sensor ===
#define RS485_RX 16
#define RS485_TX 17
HardwareSerial RS485Serial(2);
ModbusMaster node;

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

// === DC MOTORS (H-Bridge IN1-4) ===
// Motor A
#define DC_A_IN1 4
#define DC_A_IN2 5
bool dcMotorA_Running = false;
// Motor B
#define DC_B_IN3 2
#define DC_B_IN4 12
bool dcMotorB_Running = false;

// === Constants ===
#define AVG_READINGS 5
float avgBuffer10kg[AVG_READINGS];
int avgIndex = 0;

const float TEMP_MIN = 30.0;
const float TEMP_MAX = 55.0;
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 5000;

// === Dispensing Variables ===
bool isDispensing = false;
float targetDispenseAmount = 0.0;
float weightBeforeDispense = 0.0;
float weightTarget = 0.0;
unsigned long dispenseStartTime = 0;
const unsigned long DISPENSE_TIMEOUT = 20000;    // 20s safety cutoff

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

// === Stepper Motor Control ===
void enableMotor(bool state) {
  digitalWrite(ENABLE_PIN, state ? LOW : HIGH);
  motorEnabled = state;
}

void startDispensing(float manureWeight) {
  targetDispenseAmount = manureWeight * 0.004;

  if (scale10kg.is_ready()) {
    weightBeforeDispense = scale10kg.get_units(5);
    weightTarget = weightBeforeDispense + targetDispenseAmount;
  }

  Serial.printf("🐔 Manure: %.2f kg → BPA target: %.2f kg\n", manureWeight, targetDispenseAmount);

  digitalWrite(DIR_PIN, HIGH);
  delay(5);
  stepper.setSpeed(400);

  isDispensing = true;
  dispenseStartTime = millis();
  Serial.println("✅ Dispensing started - stepper running");
}

void stopDispensing() {
  stepper.setSpeed(0);
  isDispensing = false;
  Serial.println("✅ Dispensing stopped");

  // Start both DC motors after dispensing
  if (!dcMotorA_Running) startMotorA(true);
  if (!dcMotorB_Running) startMotorB(true);
}

// === DC Motor Control ===
void startMotorA(bool forward = true) {
  digitalWrite(DC_A_IN1, forward ? HIGH : LOW);
  digitalWrite(DC_A_IN2, forward ? LOW : HIGH);
  dcMotorA_Running = true;
  Serial.println("🚜 Motor A started");
}

void stopMotorA() {
  digitalWrite(DC_A_IN1, LOW);
  digitalWrite(DC_A_IN2, LOW);
  dcMotorA_Running = false;
  Serial.println("🛑 Motor A stopped");
}

void startMotorB(bool forward = true) {
  digitalWrite(DC_B_IN3, forward ? HIGH : LOW);
  digitalWrite(DC_B_IN4, forward ? LOW : HIGH);
  dcMotorB_Running = true;
  Serial.println("🚜 Motor B started");
}

void stopMotorB() {
  digitalWrite(DC_B_IN3, LOW);
  digitalWrite(DC_B_IN4, LOW);
  dcMotorB_Running = false;
  Serial.println("🛑 Motor B stopped");
}

// === Temperature Control ===
void controlTemperature(float currentTemp) {
  digitalWrite(HEATING_PIN, currentTemp < TEMP_MIN ? LOW : HIGH);
  digitalWrite(COOLING_PIN, currentTemp > TEMP_MAX ? LOW : HIGH);
}

// === Metrics ===
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

  RS485Serial.begin(4800, SERIAL_8N1, RS485_RX, RS485_TX);
  node.begin(1, RS485Serial);

  dht.begin();
  scale10kg.begin(DOUT_10KG, SCK_PIN);
  scale10kg.set_scale(calibration_factor_10kg);
  scale10kg.tare();

  pinMode(MOISTURE_POWER_PIN, OUTPUT);
  digitalWrite(MOISTURE_POWER_PIN, LOW);
  analogSetPinAttenuation(MOISTURE_PIN, ADC_11db);

  pinMode(HEATING_PIN, OUTPUT);
  pinMode(COOLING_PIN, OUTPUT);
  digitalWrite(HEATING_PIN, HIGH);
  digitalWrite(COOLING_PIN, HIGH);

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);

  // DC Motors
  pinMode(DC_A_IN1, OUTPUT);
  pinMode(DC_A_IN2, OUTPUT);
  digitalWrite(DC_A_IN1, LOW);
  digitalWrite(DC_A_IN2, LOW);

  pinMode(DC_B_IN3, OUTPUT);
  pinMode(DC_B_IN4, OUTPUT);
  digitalWrite(DC_B_IN3, LOW);
  digitalWrite(DC_B_IN4, LOW);

  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);
  stepper.setSpeed(0);

  for (int i = 0; i < AVG_READINGS; i++) avgBuffer10kg[i] = 0;
  randomSeed(analogRead(0));
}

// === Loop ===
void loop() {
  unsigned long currentTime = millis();

  // --- Sensor reading & temperature control ---
  if (!isDispensing && currentTime - lastSendTime >= SEND_INTERVAL) {
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
  }

  // --- Dispensing timeout ---
  if (isDispensing && millis() - dispenseStartTime > DISPENSE_TIMEOUT) {
    stopDispensing();
    Serial.println("⚠️ Dispense timeout reached!");
  }

  // --- Command Handling ---
  if (SerialPi.available() || Serial.available()) {
    String cmd = SerialPi.available() ? SerialPi.readStringUntil('\n')
                                      : Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.indexOf("\"phase\":\"dispensing\"") != -1) {
      float currentWeight = scale10kg.is_ready() ? scale10kg.get_units(5) : 5.0;
      if (!isDispensing) startDispensing(currentWeight);
    } else if (cmd.indexOf("\"phase\":\"active\"") != -1) {
      if (isDispensing) stopDispensing();
      if (dcMotorA_Running) stopMotorA();
      if (dcMotorB_Running) stopMotorB();
    } else if (cmd.indexOf("\"dispense\"") != -1) {
      float currentWeight = scale10kg.is_ready() ? scale10kg.get_units(5) : 5.0;
      if (!isDispensing) startDispensing(currentWeight);
    }
  }

  stepper.runSpeed();
}
