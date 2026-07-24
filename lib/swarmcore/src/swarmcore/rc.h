#pragma once
// RC channel normalization: 1000..2000 us convention -> normalized floats
// with deadband. Channel map (matches the original transmitter sketch):
//   ch0 throttle -> 0..1, ch1 roll, ch2 pitch, ch3 yaw -> -1..1

#include "types.h"
#include "packets.h"

namespace sc {

struct RcNorm {
  float throttle = 0.0f;          // 0..1
  float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;  // -1..1
};

inline float rcBipolar(uint16_t us, float deadband = 0.03f) {
  float v = (clampf(float(us), 1000.0f, 2000.0f) - 1500.0f) / 500.0f;
  if (fabsf(v) < deadband) v = 0.0f;
  return v;
}

inline RcNorm normalizeRc(const RcPacket& p) {
  RcNorm n;
  n.throttle = (clampf(float(p.ch[0]), 1000.0f, 2000.0f) - 1000.0f) / 1000.0f;
  n.roll  = rcBipolar(p.ch[1]);
  n.pitch = rcBipolar(p.ch[2]);
  n.yaw   = rcBipolar(p.ch[3]);
  return n;
}

}  // namespace sc
