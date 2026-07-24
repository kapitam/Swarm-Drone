#include "imu.h"

#include <Arduino.h>
#include <Wire.h>
#include "../config/config.h"
#include "swarmcore/types.h"

namespace imu {

#if defined(IMU_MPU6050)

static constexpr uint8_t kAddr = 0x68;
static constexpr float kGyroScale = 1.0f / 16.4f;    // ±2000 dps [dps/LSB]
static constexpr float kAccelScale = 1.0f / 4096.0f; // ±8 g [g/LSB]

static bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(kAddr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool init() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  if (!writeReg(0x6B, 0x01)) return false;  // PWR_MGMT_1: wake, PLL clock
  writeReg(0x1A, 0x03);                     // CONFIG: DLPF 42 Hz
  writeReg(0x19, 0x01);                     // SMPLRT_DIV: 500 Hz
  writeReg(0x1B, 0x18);                     // GYRO_CONFIG: ±2000 dps
  writeReg(0x1C, 0x10);                     // ACCEL_CONFIG: ±8 g
  return true;
}

bool read(float gyro[3], float accel[3]) {
  Wire.beginTransmission(kAddr);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(int(kAddr), 14) != 14) return false;
  int16_t r[7];
  for (int i = 0; i < 7; ++i) {
    r[i] = int16_t((Wire.read() << 8) | Wire.read());
  }
  // r: ax ay az temp gx gy gz. Map chip axes -> body (x fwd, y left, z up):
  // mounting-dependent; identity assumed until the PCB fixes orientation.
  accel[0] = r[0] * kAccelScale;
  accel[1] = r[1] * kAccelScale;
  accel[2] = r[2] * kAccelScale;
  gyro[0] = sc::deg2rad(r[4] * kGyroScale);
  gyro[1] = sc::deg2rad(r[5] * kGyroScale);
  gyro[2] = sc::deg2rad(r[6] * kGyroScale);
  return true;
}

const char* name() { return "MPU6050"; }

#elif defined(IMU_ICM42688)

static constexpr uint8_t kAddr = 0x68;  // AP_AD0 low
static constexpr float kGyroScale = 1.0f / 16.4f;    // ±2000 dps
static constexpr float kAccelScale = 1.0f / 4096.0f; // ±8 g

static bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(kAddr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool init() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  // Bank 0 assumed (reset default). BENCH-UNTESTED: verify on hardware.
  if (!writeReg(0x4E, 0x0F)) return false;  // PWR_MGMT0: gyro+accel low-noise
  delay(1);                                 // required 200 us after PWR_MGMT0
  writeReg(0x4F, 0x06);                     // GYRO_CONFIG0: ±2000 dps @ 1 kHz
  writeReg(0x50, 0x26);                     // ACCEL_CONFIG0: ±8 g @ 1 kHz
  return true;
}

bool read(float gyro[3], float accel[3]) {
  Wire.beginTransmission(kAddr);
  Wire.write(0x1F);  // ACCEL_DATA_X1
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(int(kAddr), 12) != 12) return false;
  int16_t r[6];
  for (int i = 0; i < 6; ++i) {
    r[i] = int16_t((Wire.read() << 8) | Wire.read());
  }
  accel[0] = r[0] * kAccelScale;
  accel[1] = r[1] * kAccelScale;
  accel[2] = r[2] * kAccelScale;
  gyro[0] = sc::deg2rad(r[3] * kGyroScale);
  gyro[1] = sc::deg2rad(r[4] * kGyroScale);
  gyro[2] = sc::deg2rad(r[5] * kGyroScale);
  return true;
}

const char* name() { return "ICM42688"; }

#else  // IMU_MOCK (default when no IMU flag set)

bool init() { return true; }

bool read(float gyro[3], float accel[3]) {
  gyro[0] = gyro[1] = gyro[2] = 0.0f;
  accel[0] = accel[1] = 0.0f;
  accel[2] = 1.0f;  // level, 1 g
  return true;
}

const char* name() { return "MOCK"; }

#endif

}  // namespace imu
