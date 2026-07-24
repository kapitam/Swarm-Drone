#pragma once
// Dead-reckoning pose estimator — PLACEHOLDER MODEL, honest about its limits.
//
// Without optical flow / GPS / UWB (hardware addons still being decided), the
// only onboard horizontal-velocity evidence on a drone is the tilt of the
// thrust vector. This integrates a first-order tilt->acceleration model with
// linear drag. It drifts (minutes-scale usefulness), which research doc 01 s2
// anticipates: swarm behaviors degrade to the gradient/RSSI fallback when
// poseValid drops. Replace the velocity source with a PMW3901 flow addon or
// UWB when hardware lands — the interface stays the same.

#include "types.h"

namespace sc {

struct TiltOdometryParams {
  float drag = 0.55f;          // [1/s] linear drag coefficient
  float validForS = 120.0f;    // pose flagged valid this long after a zero
};

class TiltOdometry {
 public:
  explicit TiltOdometry(const TiltOdometryParams& p = {}) : p_(p) {}

  void zero(uint32_t nowMs) {
    pose_ = Pose2D{};
    vel_ = Vec2{};
    zeroMs_ = nowMs;
    hasZero_ = true;
  }

  // roll/pitch/yaw [rad] from the AHRS; only integrates while flying
  // (throttle above hover fraction), else velocity decays.
  void update(float roll, float pitch, float yaw, float throttle,
              bool armed, float dt, uint32_t nowMs) {
    pose_.yaw = yaw;
    const bool thrusting = armed && throttle > 0.25f;
    if (thrusting) {
      // Body-frame accel from tilt (small-angle thrust model).
      const float axB = -tanf(clampf(pitch, -0.5f, 0.5f)) * kGravity;
      const float ayB = tanf(clampf(roll, -0.5f, 0.5f)) * kGravity;
      const float cy = cosf(yaw), sy = sinf(yaw);
      vel_.x += (cy * axB - sy * ayB) * dt;
      vel_.y += (sy * axB + cy * ayB) * dt;
    }
    // Drag (also bleeds velocity to zero when landed).
    const float k = clampf(1.0f - p_.drag * dt * (thrusting ? 1.0f : 4.0f),
                           0.0f, 1.0f);
    vel_ = vel_ * k;
    pose_.p += vel_ * dt;
    nowMs_ = nowMs;
  }

  const Pose2D& pose() const { return pose_; }
  const Vec2& velocity() const { return vel_; }
  bool poseValid() const {
    return hasZero_ && (nowMs_ - zeroMs_) < uint32_t(p_.validForS * 1000.0f);
  }

 private:
  TiltOdometryParams p_;
  Pose2D pose_;
  Vec2 vel_;
  uint32_t zeroMs_ = 0, nowMs_ = 0;
  bool hasZero_ = false;
};

}  // namespace sc
