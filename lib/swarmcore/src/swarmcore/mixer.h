#pragma once
// Quad-X mixer: torque commands -> per-motor normalized outputs [0..1].
// Motor order/geometry (viewed from above, +x forward, +y left):
//   m0 front-right (CCW), m1 rear-right (CW), m2 rear-left (CCW),
//   m3 front-left (CW)  — Betaflight-style numbering.
// Saturation handling: shift the common mode so differential authority
// survives at high/low throttle (air-mode-lite).

#include "types.h"

namespace sc {

struct MixerParams {
  float idleThrottle = 0.05f;  // min spin while armed (0 for brushed OK)
  float minOut = 0.0f, maxOut = 1.0f;
};

struct MotorOutputs {
  float m[4] = {0.0f, 0.0f, 0.0f, 0.0f};
};

inline MotorOutputs mixQuadX(float throttle, float roll, float pitch, float yaw,
                             const MixerParams& p = {}) {
  MotorOutputs o;
  // Signs: roll>0 rolls left (left motors down... see HANDBOOK for the
  // convention table); pitch>0 pitches nose up; yaw>0 = CCW torque.
  o.m[0] = throttle - roll + pitch + yaw;  // front-right, CCW
  o.m[1] = throttle - roll - pitch - yaw;  // rear-right,  CW
  o.m[2] = throttle + roll - pitch + yaw;  // rear-left,   CCW
  o.m[3] = throttle + roll + pitch - yaw;  // front-left,  CW

  // Common-mode shift on saturation.
  float hi = o.m[0], lo = o.m[0];
  for (int i = 1; i < 4; ++i) {
    if (o.m[i] > hi) hi = o.m[i];
    if (o.m[i] < lo) lo = o.m[i];
  }
  float shift = 0.0f;
  if (hi > p.maxOut) shift = p.maxOut - hi;
  else if (lo < p.minOut) shift = p.minOut - lo;
  // If the span itself exceeds the range, scale differential down.
  const float span = hi - lo;
  const float range = p.maxOut - p.minOut;
  float scale = 1.0f;
  if (span > range && span > 1e-6f) scale = range / span;

  for (int i = 0; i < 4; ++i) {
    if (scale < 1.0f) o.m[i] = throttle + (o.m[i] - throttle) * scale;
    o.m[i] = clampf(o.m[i] + shift, p.minOut, p.maxOut);
    if (throttle > 1e-3f && o.m[i] < p.idleThrottle) o.m[i] = p.idleThrottle;
  }
  return o;
}

}  // namespace sc
