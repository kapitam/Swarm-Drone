#pragma once
// Motor output fork (build both, decide on hardware — HANDBOOK "Forks"):
//   MOTORS_ESC_PWM     hobby ESCs, 50 Hz 1000-2000 us pulses (current bench
//                      hardware; ESC needs its own BEC/battery).
//   MOTORS_BRUSHED_PWM coreless brushed motors on MOSFETs, 20 kHz duty
//                      (Crazyflie/ESP-Drone-style PCB drone).
// Same interface either way; mixer outputs are normalized [0..1].
// DShot over RMT is a documented future variant (HANDBOOK), not built yet.

#include <stdint.h>

namespace motors {

void init();
// m: 4 normalized outputs [0..1]. Ordering per mixer (FR, RR, RL, FL).
void write(const float m[4]);
void stop();  // immediate safe output (ESC min pulse / brushed 0 duty)

}  // namespace motors
