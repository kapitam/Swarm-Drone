#include "behavior.h"

namespace sc {

BehaviorOutput BehaviorPipeline::update(const BehaviorInputs& in) {
  BehaviorOutput out;
  const GovernorOutput gov = governor_.update(in.sectors);
  out.reflex = gov.state;

  // ---- MANUAL: sticks -> attitude; only the reflex brake intervenes. ----
  if (in.mode == BehaviorMode::kManual) {
    float pitchCmd = in.rc.pitch;
    if (p_.manualReflexBrake && pitchCmd > 0.0f) {
      // Forward stick scaled by the governor's cap; stop/fear zero it.
      const float scale = (gov.state == ReflexState::kStop ||
                           gov.state == ReflexState::kFear)
                              ? 0.0f
                              : gov.speedCap / p_.governor.vMax;
      pitchCmd *= scale;
    }
    out.setpoint.rollAngle = in.rc.roll * p_.manualMaxTilt;
    out.setpoint.pitchAngle = pitchCmd * p_.manualMaxTilt;
    out.setpoint.yawRate = in.rc.yaw * p_.manualMaxYawRate;
    out.setpoint.throttle = in.rc.throttle;
    return out;
  }

  // ---- Autonomous modes produce a desired horizontal velocity. ----
  Vec2 vDes{};
  switch (in.mode) {
    case BehaviorMode::kHold:
      vDes = {0.0f, 0.0f};
      break;
    case BehaviorMode::kLeaderFollow: {
      const FollowOutput f =
          in.table ? follow_.update(in.self, *in.table, in.nowMs)
                   : FollowOutput{};
      out.followStatus = f.status;
      // LOST / no leader: hold position (zero velocity).
      vDes = (f.status == FollowStatus::kTracking ||
              f.status == FollowStatus::kCoasting)
                 ? f.velocity
                 : Vec2{0.0f, 0.0f};
      break;
    }
    case BehaviorMode::kFlock:
      if (in.table)
        vDes = boids_.compute(in.self, *in.table, in.nowMs,
                              in.hasGoal ? &in.goal : nullptr, false);
      break;
    case BehaviorMode::kDisperse:
      if (in.table)
        vDes = boids_.compute(in.self, *in.table, in.nowMs, nullptr, true);
      break;
    default:
      break;
  }
  out.desiredVel = vDes;

  // ---- BVC: inter-agent filter (shared frame). ----
  Vec2 v = vDes;
  if (in.table) v = bvc_.constrain(v, in.self.pose.p, *in.table, in.nowMs);

  // ---- VFH-lite: steer around environment obstacles (body frame). ----
  const float yaw = in.self.pose.yaw;
  const float cy = cosf(yaw), sy = sinf(yaw);
  Vec2 vBody{cy * v.x + sy * v.y, -sy * v.x + cy * v.y};
  float speed = vBody.norm();
  if (speed > 1e-3f) {
    const float dirBody = atan2f(vBody.y, vBody.x);
    const VfhResult vr = vfh_.update(in.sectors, dirBody);
    out.vfhSteered = vr.steered;
    out.vfhAllBlocked = vr.allBlocked;
    if (vr.allBlocked && dirBody > -kPi / 2 && dirBody < kPi / 2) {
      speed = 0.0f;  // forward escape impossible: stop (escape = future work)
    } else if (vr.steered) {
      vBody = {cosf(vr.headingRad) * speed, sinf(vr.headingRad) * speed};
    }
  }

  // ---- Governor: forward speed cap + stop reflex + fear backoff. ----
  if (gov.state == ReflexState::kFear) {
    vBody = {-gov.backoff, 0.0f};  // straight back
  } else if (vBody.x > 0.0f) {
    if (gov.state == ReflexState::kStop) vBody.x = 0.0f;
    else if (vBody.x > gov.speedCap) vBody.x = gov.speedCap;
  }
  if (speed < 1e-3f && gov.state != ReflexState::kFear) vBody = {0.0f, 0.0f};

  // Back to shared frame.
  v = {cy * vBody.x - sy * vBody.y, sy * vBody.x + cy * vBody.y};
  out.commandedVel = v;

  // ---- Velocity -> attitude. Throttle: manual collective until an altitude
  // sensor addon lands (hover throttle discovery is a bench-tuning step). ----
  velocityToTilt(v, in.self.vel, yaw, p_.velTilt,
                 out.setpoint.rollAngle, out.setpoint.pitchAngle);
  out.setpoint.yawRate = 0.0f;
  if (in.mode == BehaviorMode::kLeaderFollow || in.mode == BehaviorMode::kFlock) {
    // Slowly yaw toward travel direction for sensor pointing.
    const float speedN = v.norm();
    if (speedN > 0.15f) {
      const float dirWorld = atan2f(v.y, v.x);
      out.setpoint.yawRate = clampf(wrapPi(dirWorld - yaw) * 1.5f,
                                    -p_.manualMaxYawRate, p_.manualMaxYawRate);
    }
  }
  out.setpoint.throttle = in.rc.throttle;
  return out;
}

}  // namespace sc
