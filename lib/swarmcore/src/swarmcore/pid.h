#pragma once
// Small PID with derivative-on-measurement (no setpoint kick), integral clamp
// and output limit. Used by the attitude rate/angle loops.

#include "types.h"

namespace sc {

struct PidParams {
  float kp = 0.0f, ki = 0.0f, kd = 0.0f;
  float iLimit = 0.0f;    // |integrator| clamp (output units)
  float outLimit = 0.0f;  // |output| clamp; 0 = unlimited
};

class Pid {
 public:
  explicit Pid(const PidParams& p = {}) : p_(p) {}
  void setParams(const PidParams& p) { p_ = p; }
  void reset() { i_ = 0.0f; prevMeas_ = 0.0f; primed_ = false; }

  float step(float setpoint, float measurement, float dt) {
    const float err = setpoint - measurement;
    i_ = clampf(i_ + err * p_.ki * dt, -p_.iLimit, p_.iLimit);
    float d = 0.0f;
    if (primed_ && dt > 1e-6f) d = -(measurement - prevMeas_) / dt;
    prevMeas_ = measurement;
    primed_ = true;
    float out = p_.kp * err + i_ + p_.kd * d;
    if (p_.outLimit > 0.0f) out = clampf(out, -p_.outLimit, p_.outLimit);
    return out;
  }

  float integrator() const { return i_; }

 private:
  PidParams p_;
  float i_ = 0.0f;
  float prevMeas_ = 0.0f;
  bool primed_ = false;
};

}  // namespace sc
