#pragma once
// Angle-mode attitude controller: outer P loop (angle error -> rate setpoint)
// cascaded into rate PIDs (gyro feedback) -> normalized torque commands for
// the mixer. Gains are conservative placeholders — MUST be bench-tuned on the
// real PCB drone before flight (props off first; see HANDBOOK "First flight").

#include "types.h"
#include "pid.h"

namespace sc {

struct AttitudeParams {
  float angleKp   = 4.5f;                       // angle err -> rate sp [1/s]
  float maxAngle  = deg2rad(25.0f);             // tilt limit
  float maxRate   = deg2rad(200.0f);            // rate sp limit [rad/s]
  float maxYawRate= deg2rad(180.0f);
  PidParams rollRate  {0.12f, 0.10f, 0.004f, 0.15f, 0.5f};
  PidParams pitchRate {0.12f, 0.10f, 0.004f, 0.15f, 0.5f};
  PidParams yawRate   {0.20f, 0.05f, 0.0f,   0.15f, 0.5f};
};

struct AttitudeSetpoint {
  float rollAngle = 0.0f;   // [rad]
  float pitchAngle = 0.0f;  // [rad]
  float yawRate = 0.0f;     // [rad/s]
  float throttle = 0.0f;    // 0..1 (collective, passed through to mixer)
};

struct TorqueCommand {
  float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;  // normalized -1..1
  float throttle = 0.0f;                        // 0..1
};

class AttitudeController {
 public:
  explicit AttitudeController(const AttitudeParams& p = {}) { setParams(p); }
  void setParams(const AttitudeParams& p) {
    p_ = p;
    rollPid_.setParams(p.rollRate);
    pitchPid_.setParams(p.pitchRate);
    yawPid_.setParams(p.yawRate);
  }
  void reset() { rollPid_.reset(); pitchPid_.reset(); yawPid_.reset(); }

  // roll/pitch: current attitude [rad]; gyro: body rates [rad/s].
  TorqueCommand update(const AttitudeSetpoint& sp, float roll, float pitch,
                       float gyroX, float gyroY, float gyroZ, float dt) {
    TorqueCommand out;
    const float rSp = clampf(sp.rollAngle, -p_.maxAngle, p_.maxAngle);
    const float pSp = clampf(sp.pitchAngle, -p_.maxAngle, p_.maxAngle);
    const float rollRateSp  = clampf((rSp - roll) * p_.angleKp, -p_.maxRate, p_.maxRate);
    const float pitchRateSp = clampf((pSp - pitch) * p_.angleKp, -p_.maxRate, p_.maxRate);
    const float yawRateSp   = clampf(sp.yawRate, -p_.maxYawRate, p_.maxYawRate);
    out.roll  = rollPid_.step(rollRateSp, gyroX, dt);
    out.pitch = pitchPid_.step(pitchRateSp, gyroY, dt);
    out.yaw   = yawPid_.step(yawRateSp, gyroZ, dt);
    out.throttle = clampf(sp.throttle, 0.0f, 1.0f);
    return out;
  }

 private:
  AttitudeParams p_;
  Pid rollPid_, pitchPid_, yawPid_;
};

// Horizontal-velocity -> tilt-angle mapping used by autonomous behaviors:
// desired acceleration (P on velocity error) tilts the thrust vector.
struct VelocityToTiltParams {
  float kv = 2.5f;                    // [1/s] vel error -> accel
  float maxTilt = deg2rad(15.0f);     // autonomy tilt budget < manual limit
};

// velDesired/velEst in the shared frame; yaw rotates into body frame.
inline void velocityToTilt(const Vec2& velDesired, const Vec2& velEst, float yaw,
                           const VelocityToTiltParams& p,
                           float& rollAngle, float& pitchAngle) {
  const Vec2 aWorld = (velDesired - velEst) * p.kv;   // desired accel [m/s^2]
  // World -> body.
  const float cy = cosf(yaw), sy = sinf(yaw);
  const float axB = cy * aWorld.x + sy * aWorld.y;    // body forward accel
  const float ayB = -sy * aWorld.x + cy * aWorld.y;   // body left accel
  // Small-angle thrust-vector model: pitch forward (negative) accelerates +x.
  pitchAngle = clampf(-axB / kGravity, -p.maxTilt, p.maxTilt);
  rollAngle  = clampf(ayB / kGravity, -p.maxTilt, p.maxTilt);
}

}  // namespace sc
