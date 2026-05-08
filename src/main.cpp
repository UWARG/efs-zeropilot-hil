#include <Arduino.h>
#include <Wire.h>

// ===== INA228 register addresses =====
const uint8_t REG_CONFIG     = 0x00;
const uint8_t REG_ADC_CONFIG = 0x01;
const uint8_t REG_SHUNT_CAL  = 0x02;
const uint8_t REG_VBUS       = 0x05;
const uint8_t REG_CURRENT    = 0x07;
const uint8_t REG_POWER      = 0x08;
const uint8_t REG_ENERGY     = 0x09;
const uint8_t REG_CHARGE     = 0x0A;
const uint8_t INA228_ADDR    = 0x45;
volatile uint8_t currentRegister = 0x00;

// ===== Fake register storage =====
uint8_t registers[256][5];
uint8_t registerLength[256];

// ===== Helper: store bytes into fake register =====
void setRegisterBytes(uint8_t reg, const uint8_t *data, uint8_t length) {
  registerLength[reg] = length;

  for (uint8_t i = 0; i < length; i++) {
    registers[reg][i] = data[i];
  }
}

// ===== Fake INA228 initial values =====
void setupFakeINA228Registers() {
  // CONFIG / ADC_CONFIG / SHUNT_CAL
  // 这些先随便给 2 bytes，占位用
  uint8_t config[2]     = {0x80, 0x00};
  uint8_t adcConfig[2]  = {0xF0, 0x02};
  uint8_t shuntCal[2]   = {0x00, 0x00};

  setRegisterBytes(REG_CONFIG, config, 2);
  setRegisterBytes(REG_ADC_CONFIG, adcConfig, 2);
  setRegisterBytes(REG_SHUNT_CAL, shuntCal, 2);

  uint8_t vbus[3]    = {0x39, 0xD0, 0x00};              // fake voltage bytes
  uint8_t current[3] = {0x01, 0x00, 0x00};              // fake current bytes
  uint8_t power[3]   = {0x02, 0x00, 0x00};              // fake power bytes
  uint8_t energy[5]  = {0x00, 0x00, 0x00, 0x10, 0x00};  // fake energy bytes
  uint8_t charge[5]  = {0x00, 0x00, 0x00, 0x08, 0x00};  // fake charge bytes

  setRegisterBytes(REG_VBUS, vbus, 3);
  setRegisterBytes(REG_CURRENT, current, 3);
  setRegisterBytes(REG_POWER, power, 3);
  setRegisterBytes(REG_ENERGY, energy, 5);
  setRegisterBytes(REG_CHARGE, charge, 5);
}

// ===== Fake write register =====
void writeRegister(uint8_t reg, const uint8_t *data, uint8_t length) {
  setRegisterBytes(reg, data, length);

  Serial.print("Wrote register 0x");
  Serial.println(reg, HEX);
}

// ===== Fake read register =====
void readRegister(uint8_t reg) {
  Serial.print("Read register 0x");
  Serial.print(reg, HEX);
  Serial.print(": ");

  for (uint8_t i = 0; i < registerLength[reg]; i++) {
    Serial.print("0x");
    Serial.print(registers[reg][i], HEX);
    Serial.print(" ");
  }

  Serial.println();
}


void onReceive(int numBytes) {
  if (numBytes <= 0) return;

  uint8_t reg = Wire.read();
  currentRegister = reg;

  uint8_t temp[5];
  uint8_t count = 0;

  while (Wire.available() && count < 5) {
    temp[count] = Wire.read();
    count++;
  }

  if (count > 0) {
    writeRegister(reg, temp, count);
  }
}

void onRequest() {
  uint8_t reg = currentRegister;
  uint8_t len = registerLength[reg];

  if (len == 0) {
    Wire.write((uint8_t)0x00);
    return;
  }

  Wire.write(registers[reg], len);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("ESP32-S3 INA228 mock starting...");

  setupFakeINA228Registers();
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.begin(INA228_ADDR);

  readRegister(REG_VBUS);
  readRegister(REG_CURRENT);
  readRegister(REG_POWER);
  readRegister(REG_CHARGE);
  readRegister(REG_ENERGY);

  Serial.println("Fake register table ready.");
}

void loop() {
  delay(1000);
}