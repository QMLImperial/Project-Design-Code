#include <HX711_ADC.h>
#include <EEPROM.h>

// ================== ESP32 PIN CONFIG ==================
const int HX711_dout = 18;  // HX711 DOUT → ESP32 GPIO18
const int HX711_sck  = 19;  // HX711 SCK  → ESP32 GPIO19

HX711_ADC LoadCell(HX711_dout, HX711_sck);

// EEPROM
const int calVal_eepromAdress = 0;

// Timing
unsigned long lastPrint = 0;
const unsigned long printInterval = 3000; // 3 seconds

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\nHX711 ESP32 Calibration & Weight Readout");

  EEPROM.begin(512);

  LoadCell.begin();
  unsigned long stabilizingtime = 2000;
  boolean _tare = true;
  LoadCell.start(stabilizingtime, _tare);

  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("❌ HX711 timeout. Check wiring.");
    while (1);
  }

  float calValue;
  EEPROM.get(calVal_eepromAdress, calValue);

  if (isnan(calValue) || calValue == 0) {
    calValue = 1.0;
    Serial.println("⚠ No calibration found. Using default.");
  }

  LoadCell.setCalFactor(calValue);
  Serial.print("Calibration Factor: ");
  Serial.println(calValue);

  Serial.println("Startup complete.");
  Serial.println("Commands:");
  Serial.println("  t = tare");
  Serial.println("  r = recalibrate");
  Serial.println("  c = change calibration value");
}

void loop() {
  LoadCell.update();

  // ===== Serial Commands =====
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 't') LoadCell.tareNoDelay();
    if (cmd == 'r') calibrate();
    if (cmd == 'c') changeSavedCalFactor();
  }

  // ===== Read Weight Every 3 Seconds =====
  if (millis() - lastPrint >= printInterval) {
    lastPrint = millis();

    if (LoadCell.getData() != 0) {
      float kg = LoadCell.getData();
      float lb = kg * 2.20462;

      Serial.print("Weight: ");
      Serial.print(kg, 3);
      Serial.print(" kg | ");
      Serial.print(lb, 3);
      Serial.println(" lb");
    }
  }

  if (LoadCell.getTareStatus()) {
    Serial.println("Tare complete");
  }
}

// ================== CALIBRATION ==================
void calibrate() {
  Serial.println("\n--- CALIBRATION ---");
  Serial.println("Remove all weight.");
  Serial.println("Send 't' to tare.");

  while (!LoadCell.getTareStatus()) {
    LoadCell.update();
    if (Serial.available() && Serial.read() == 't') {
      LoadCell.tareNoDelay();
    }
  }

  Serial.println("Place known weight (kg) and send value:");

  float known_mass = 0;
  while (known_mass == 0) {
    LoadCell.update();
    if (Serial.available()) {
      known_mass = Serial.parseFloat();
    }
  }

  LoadCell.refreshDataSet();
  float newCal = LoadCell.getNewCalibration(known_mass);

  Serial.print("New calibration value: ");
  Serial.println(newCal);

  EEPROM.put(calVal_eepromAdress, newCal);
  EEPROM.commit();

  LoadCell.setCalFactor(newCal);
  Serial.println("Calibration saved.");
}

// ================== MANUAL CAL CHANGE ==================
void changeSavedCalFactor() {
  Serial.println("\nSend new calibration value:");

  float newCal = 0;
  while (newCal == 0) {
    if (Serial.available()) {
      newCal = Serial.parseFloat();
    }
  }

  LoadCell.setCalFactor(newCal);
  EEPROM.put(calVal_eepromAdress, newCal);
  EEPROM.commit();

  Serial.print("Calibration updated: ");
  Serial.println(newCal);
}
