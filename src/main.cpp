#include <Arduino.h>
#include <Wire.h>

// I2C pins for ESP32-S3
#define SDA_PIN 8
#define SCL_PIN 9

// LSM6DSOX I2C address
#define LSM6DSOX_ADDR 0x6A

// Registers
#define WHO_AM_I    0x0F
#define CTRL1_XL    0x10
#define CTRL2_G     0x11
#define OUTX_L_G    0x22  // 6 gyro registers start here
#define OUTX_L_A    0x28  // 6 accel registers start here

// Analog pins
const int analogPins[] = {6, 7, 10, 11};

// Write a register
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(LSM6DSOX_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Read multiple bytes
void readRegisters(uint8_t reg, uint8_t *data, uint8_t len) {
  Wire.beginTransmission(LSM6DSOX_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false); // repeated start
  Wire.requestFrom(LSM6DSOX_ADDR, len);
  for (int i = 0; i < len; i++) {
    if (Wire.available()) {
      data[i] = Wire.read();
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // 100 kHz safe start

  // WHO_AM_I check
  uint8_t whoami = 0;
  readRegisters(WHO_AM_I, &whoami, 1);
  Serial.print("WHO_AM_I = 0x");
  Serial.println(whoami, HEX);

  if (whoami != 0x6C) {
    Serial.println("⚠️ LSM6DSOX not found!");
    while (1) delay(100);
  }

  // Configure accelerometer: 104 Hz, ±2 g
  writeRegister(CTRL1_XL, 0b01000000); // ODR=104 Hz, FS=2g

  // Configure gyroscope: 104 Hz, ±2000 dps
  writeRegister(CTRL2_G, 0b01001100); // ODR=104 Hz, FS=2000 dps

  // Initialize analog pins
  for (int i = 0; i < 4; i++) {
    pinMode(analogPins[i], INPUT);
  }
}

void loop() {
  uint8_t rawData[6];
  int16_t gx, gy, gz, ax, ay, az;

  // Gyroscope
  readRegisters(OUTX_L_G, rawData, 6);
  gx = (int16_t)(rawData[1] << 8 | rawData[0]);
  gy = (int16_t)(rawData[3] << 8 | rawData[2]);
  gz = (int16_t)(rawData[5] << 8 | rawData[4]);

  // Accelerometer
  readRegisters(OUTX_L_A, rawData, 6);
  ax = (int16_t)(rawData[1] << 8 | rawData[0]);
  ay = (int16_t)(rawData[3] << 8 | rawData[2]);
  az = (int16_t)(rawData[5] << 8 | rawData[4]);

  // Convert to physical units
  float ax_mg = ax * 0.061f;
  float ay_mg = ay * 0.061f;
  float az_mg = az * 0.061f;

  float gx_dps = gx * 0.07f;
  float gy_dps = gy * 0.07f;
  float gz_dps = gz * 0.07f;

  // Analog reads
  int analogVals[4];
  for (int i = 0; i < 4; i++) {
    analogVals[i] = analogRead(analogPins[i]);
  }

  Serial.print(ax_mg); Serial.print(", ");
  Serial.print(ay_mg); Serial.print(", ");
  Serial.print(az_mg); Serial.print(", ");

  for (int i = 0; i < 4; i++) {
    Serial.print(analogVals[i]);
    if (i < 3) Serial.print(", ");
  }
  Serial.println();
  delayMicroseconds(100);
}
