#pragma once
// Mahony AHRS (6-axis gyro+accel) — the fusion pick from research doc 03
// (~120-160 us/update on ESP32, ~6-8% of one core at 500 Hz). Quaternion
// complementary filter with PI feedback on the accel gravity direction.

#include "types.h"

namespace sc {

struct MahonyParams {
  float twoKp = 2.0f * 0.5f;  // 2 * proportional gain
  float twoKi = 2.0f * 0.0f;  // 2 * integral gain (0 = off)
};

class Mahony {
 public:
  explicit Mahony(const MahonyParams& p = {}) : p_(p) {}
  void reset() { q0_ = 1.0f; q1_ = q2_ = q3_ = 0.0f; ix_ = iy_ = iz_ = 0.0f; }

  // gx,gy,gz [rad/s]; ax,ay,az in any consistent unit (normalized internally).
  void update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

  float roll() const {   // [rad]
    return atan2f(2.0f * (q0_ * q1_ + q2_ * q3_),
                  1.0f - 2.0f * (q1_ * q1_ + q2_ * q2_));
  }
  float pitch() const {  // [rad]
    float s = 2.0f * (q0_ * q2_ - q3_ * q1_);
    s = clampf(s, -1.0f, 1.0f);
    return asinf(s);
  }
  float yaw() const {    // [rad]
    return atan2f(2.0f * (q0_ * q3_ + q1_ * q2_),
                  1.0f - 2.0f * (q2_ * q2_ + q3_ * q3_));
  }
  void quaternion(float& w, float& x, float& y, float& z) const {
    w = q0_; x = q1_; y = q2_; z = q3_;
  }

 private:
  MahonyParams p_;
  float q0_ = 1.0f, q1_ = 0.0f, q2_ = 0.0f, q3_ = 0.0f;
  float ix_ = 0.0f, iy_ = 0.0f, iz_ = 0.0f;  // integral feedback
};

}  // namespace sc
