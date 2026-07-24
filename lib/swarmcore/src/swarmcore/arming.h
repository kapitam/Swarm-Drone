#pragma once
// Arming / failsafe state machine. Motors can only spin in kArmed.
//  - Arm:    throttle low + yaw stick full right held for armHoldMs.
//  - Disarm: throttle low + yaw stick full left held for armHoldMs (or any
//            failsafe/e-stop).
//  - Failsafe: RC link older than rcTimeoutMs while armed -> kFailsafe
//            (motor cut; a controlled-descent option is a future addon once
//            an altitude sensor exists). E-stop (fleet broadcast) latches.

#include <stdint.h>

namespace sc {

struct ArmingParams {
  uint32_t armHoldMs   = 1000;
  uint32_t rcTimeoutMs = 150;   // matches the original sketch's failsafe
  float    throttleLow = 0.05f; // normalized
  float    stickHigh   = 0.85f; // |yaw| beyond this counts as full deflection
  float    lowBatteryV = 3.4f;  // per-cell; 0 disables
};

enum class ArmState : uint8_t { kDisarmed, kArming, kArmed, kFailsafe, kEStop };

class Arming {
 public:
  explicit Arming(const ArmingParams& p = {}) : p_(p) {}
  void setParams(const ArmingParams& p) { p_ = p; }

  void latchEStop() { state_ = ArmState::kEStop; }

  // throttle 0..1, yaw -1..1, rcAgeMs = ms since last valid RC packet.
  ArmState update(float throttle, float yaw, uint32_t rcAgeMs,
                  float batteryPerCellV, uint32_t nowMs) {
    switch (state_) {
      case ArmState::kEStop:
        // Only a disarm gesture releases the latch (operator acknowledgment).
        if (throttle < p_.throttleLow && yaw < -p_.stickHigh) {
          if (gestureStart_ == 0) gestureStart_ = nowMs;
          if (nowMs - gestureStart_ >= p_.armHoldMs) {
            state_ = ArmState::kDisarmed;
            gestureStart_ = 0;
          }
        } else gestureStart_ = 0;
        break;

      case ArmState::kDisarmed:
        if (rcAgeMs <= p_.rcTimeoutMs && throttle < p_.throttleLow &&
            yaw > p_.stickHigh) {
          if (gestureStart_ == 0) gestureStart_ = nowMs;
          state_ = ArmState::kArming;
        } else gestureStart_ = 0;
        break;

      case ArmState::kArming:
        if (rcAgeMs > p_.rcTimeoutMs || throttle >= p_.throttleLow ||
            yaw <= p_.stickHigh) {
          state_ = ArmState::kDisarmed;
          gestureStart_ = 0;
        } else if (nowMs - gestureStart_ >= p_.armHoldMs) {
          state_ = ArmState::kArmed;
          gestureStart_ = 0;
        }
        break;

      case ArmState::kArmed:
        if (rcAgeMs > p_.rcTimeoutMs) { state_ = ArmState::kFailsafe; break; }
        if (p_.lowBatteryV > 0.0f && batteryPerCellV > 0.5f &&
            batteryPerCellV < p_.lowBatteryV) { state_ = ArmState::kFailsafe; break; }
        if (throttle < p_.throttleLow && yaw < -p_.stickHigh) {
          if (gestureStart_ == 0) gestureStart_ = nowMs;
          if (nowMs - gestureStart_ >= p_.armHoldMs) {
            state_ = ArmState::kDisarmed;
            gestureStart_ = 0;
          }
        } else gestureStart_ = 0;
        break;

      case ArmState::kFailsafe:
        // Motor cut is immediate (task_control). Recover only through
        // disarm: link back + throttle low.
        if (rcAgeMs <= p_.rcTimeoutMs && throttle < p_.throttleLow)
          state_ = ArmState::kDisarmed;
        break;
    }
    return state_;
  }

  ArmState state() const { return state_; }
  bool motorsAllowed() const { return state_ == ArmState::kArmed; }

 private:
  ArmingParams p_;
  ArmState state_ = ArmState::kDisarmed;
  uint32_t gestureStart_ = 0;
};

}  // namespace sc
