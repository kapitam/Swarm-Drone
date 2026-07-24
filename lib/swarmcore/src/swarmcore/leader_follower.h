#pragma once
// Leader-follower formation (research doc 01 s3.4, first formation behavior):
// follower regulates a slot offset expressed in the leader's body frame.
// Degradation per the survey: constant-velocity coast on packet gaps, then
// timeout -> LOST (caller must hold/stop).

#include "types.h"
#include "neighbor_table.h"

namespace sc {

struct LeaderFollowerParams {
  uint8_t leaderId  = 0;
  Vec2    offset    = {-1.2f, 0.0f};  // slot in leader body frame [m] (behind)
  float   kp        = 0.9f;           // P on slot error -> velocity
  float   kd        = 0.4f;           // D on slot error rate (leader vel feedfwd)
  float   vMax      = 1.0f;
  uint32_t coastMs  = 400;            // predict through gaps up to this age
  uint32_t lostMs   = 1200;           // beyond this: LOST
};

enum class FollowStatus : uint8_t { kTracking, kCoasting, kLost, kNoLeader };

struct FollowOutput {
  Vec2 velocity;
  FollowStatus status = FollowStatus::kNoLeader;
};

class LeaderFollower {
 public:
  explicit LeaderFollower(const LeaderFollowerParams& p = {}) : p_(p) {}
  void setParams(const LeaderFollowerParams& p) { p_ = p; }
  void setLeader(uint8_t id) { p_.leaderId = id; }
  const LeaderFollowerParams& params() const { return p_; }

  FollowOutput update(const VehicleState& self, const NeighborTable& table,
                      uint32_t nowMs) const;

 private:
  LeaderFollowerParams p_;
};

}  // namespace sc
