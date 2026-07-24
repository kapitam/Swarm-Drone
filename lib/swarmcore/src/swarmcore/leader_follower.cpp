#include "leader_follower.h"

namespace sc {

FollowOutput LeaderFollower::update(const VehicleState& self,
                                    const NeighborTable& table,
                                    uint32_t nowMs) const {
  FollowOutput out;
  const Neighbor* leader = table.byId(p_.leaderId);
  if (!leader || !leader->poseValid()) {
    out.status = FollowStatus::kNoLeader;
    return out;
  }

  const uint32_t age = nowMs - leader->lastHeardMs;
  if (age > p_.lostMs) {
    out.status = FollowStatus::kLost;
    return out;
  }
  out.status = (age > p_.coastMs) ? FollowStatus::kCoasting
                                  : FollowStatus::kTracking;

  // Slot target in shared frame: leader predicted position + R(yaw)*offset.
  const Vec2 lpos = leader->predictedPos(nowMs);
  const float cy = cosf(leader->pose.yaw), sy = sinf(leader->pose.yaw);
  const Vec2 slot{lpos.x + cy * p_.offset.x - sy * p_.offset.y,
                  lpos.y + sy * p_.offset.x + cy * p_.offset.y};

  const Vec2 err = slot - self.pose.p;
  const Vec2 errRate = leader->vel - self.vel;
  out.velocity = (err * p_.kp + errRate * p_.kd + leader->vel).limited(p_.vMax);
  return out;
}

}  // namespace sc
