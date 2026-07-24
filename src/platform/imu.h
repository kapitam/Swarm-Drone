#pragma once
// IMU fork (build all, decide with the PCB — HANDBOOK "Forks"):
//   IMU_MPU6050   ubiquitous, cheap, proven; higher noise, EOL-ish.
//   IMU_ICM42688  modern flight-controller standard, lower noise. Driver is
//                 register-level minimal and BENCH-UNTESTED until hardware.
//   IMU_MOCK     returns level-and-still readings so the whole stack runs on
//                 a bare devkit with no sensors attached.
// Axes: x forward, y left, z up (gyro [rad/s], accel [g]).

#include <stdint.h>

namespace imu {

bool init();                       // false = sensor missing (task logs, mock continues)
bool read(float gyro[3], float accel[3]);
const char* name();

}  // namespace imu
