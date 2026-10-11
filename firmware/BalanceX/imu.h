// =====================================================================================
//  imu.h  -  minimal MPU6050 driver (talks to the chip's registers directly over I2C).
//
//  Why not a library? Fewer things that can break: no library version mismatches, and
//  some MPU6050 libraries refuse to start on clone chips whose WHO_AM_I is not 0x68.
//  Register numbers: "MPU-6000/MPU-6050 Register Map and Descriptions", rev 4.2.
// =====================================================================================
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "config.h"

namespace imu {

constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
constexpr uint8_t REG_CONFIG       = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;   // 14 bytes: accel XYZ, temperature, gyro XYZ
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t REG_WHO_AM_I     = 0x75;

constexpr uint8_t GYRO_FS_500DPS = 1 << 3;   // +-500 deg/s : a falling robot can exceed 250 deg/s
constexpr uint8_t ACCEL_FS_4G    = 1 << 3;   // +-4 g       : motor jerks do not clip the reading
constexpr float GYRO_LSB_PER_DPS = 65.5f;    // datasheet sensitivity for +-500 deg/s
constexpr float ACCEL_LSB_PER_G  = 8192.0f;  // datasheet sensitivity for +-4 g

struct Sample {
  float acc[3];    // g      (X, Y, Z)
  float gyro[3];   // deg/s  (X, Y, Z), calibration offset already removed
};

float   gyroBias[3] = {0.0f, 0.0f, 0.0f};
uint8_t whoAmI = 0;

bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;           // 0 = the chip acknowledged
}

bool readRegs(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;            // false = repeated start
  if (Wire.requestFrom(MPU_I2C_ADDR, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

// Reads the sensor and converts to g and deg/s, WITHOUT removing the gyro offset.
bool readUncalibrated(Sample& s) {
  uint8_t b[14];
  if (!readRegs(REG_ACCEL_XOUT_H, b, sizeof(b))) return false;
  // Each value is a 16-bit signed number, high byte first.
  for (int i = 0; i < 3; i++) {
    const int16_t a = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]);         // bytes 0..5
    const int16_t g = (int16_t)((b[8 + 2 * i] << 8) | b[8 + 2 * i + 1]); // bytes 8..13
    s.acc[i]  = a / ACCEL_LSB_PER_G;
    s.gyro[i] = g / GYRO_LSB_PER_DPS;
  }
  // A sensor that lost power or got disconnected often returns all zeros. A real
  // accelerometer always measures ~1 g of gravity, so treat "no gravity" as a failed read.
  const float gSquared = s.acc[0] * s.acc[0] + s.acc[1] * s.acc[1] + s.acc[2] * s.acc[2];
  return gSquared > 0.04f;   // |acc| > 0.2 g
}

bool read(Sample& s) {
  if (!readUncalibrated(s)) return false;
  for (int i = 0; i < 3; i++) s.gyro[i] -= gyroBias[i];
  return true;
}

// Starts I2C and configures the MPU6050. Returns false if the chip does not answer.
bool begin() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_CLOCK_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);

  if (!readRegs(REG_WHO_AM_I, &whoAmI, 1)) return false;

  writeReg(REG_PWR_MGMT_1, 0x80);    // reset all registers to defaults (no ACK check: chip resets)
  delay(100);
  bool ok = writeReg(REG_PWR_MGMT_1, 0x01);  // wake up; clock = gyro X PLL (more stable than
  delay(10);                                 // the default internal 8 MHz oscillator)
  ok = ok && writeReg(REG_SMPLRT_DIV, 0);                    // 1 kHz internal sample rate
  ok = ok && writeReg(REG_CONFIG, MPU_DLPF_CFG & 0x07);      // low-pass filter
  ok = ok && writeReg(REG_GYRO_CONFIG, GYRO_FS_500DPS);
  ok = ok && writeReg(REG_ACCEL_CONFIG, ACCEL_FS_4G);
  if (!ok) return false;

  // Read back one setting to prove the writes really reached the chip.
  uint8_t check = 0;
  return readRegs(REG_GYRO_CONFIG, &check, 1) && check == GYRO_FS_500DPS;
}

// Re-start I2C from scratch after repeated read errors (e.g. a loose wire that came back).
bool restart() {
  Wire.end();
  delay(5);
  return begin();
}

// Measures the gyro offset (a still gyro never reads exactly 0). The robot must not move.
// Returns false if it moved (readings spread more than maxSpreadDps) or the sensor failed.
bool calibrateGyro(uint16_t samples, float maxSpreadDps, float& spreadOut) {
  float sum[3] = {0, 0, 0}, lo[3] = {1e9f, 1e9f, 1e9f}, hi[3] = {-1e9f, -1e9f, -1e9f};
  Sample s;
  for (uint16_t n = 0; n < samples; n++) {
    if (!readUncalibrated(s)) return false;
    for (int i = 0; i < 3; i++) {
      sum[i] += s.gyro[i];
      lo[i] = fminf(lo[i], s.gyro[i]);
      hi[i] = fmaxf(hi[i], s.gyro[i]);
    }
    delay(2);
  }
  spreadOut = 0.0f;
  for (int i = 0; i < 3; i++) spreadOut = fmaxf(spreadOut, hi[i] - lo[i]);
  if (spreadOut > maxSpreadDps) return false;
  for (int i = 0; i < 3; i++) gyroBias[i] = sum[i] / samples;
  return true;
}

}  // namespace imu
