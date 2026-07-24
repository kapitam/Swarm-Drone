#pragma once
// Behavior pipeline — the arbitration layer (research doc 01 s4.4) run at the
// avoidance tick (50 Hz). Order of authority, top overrides bottom:
//
//   1. mode source     (manual sticks / hold / leader-follower / flock / disperse)
//   2. BVC filter      (inter-agent, neighbor positions)
//   3. VFH-lite        (environment obstacles, steer within sensor FoV)
//   4. Governor        (speed cap + stop reflex + fear backoff) — final say
//
// Output is an AttitudeSetpoint for the control task. In MANUAL the sticks
// command attitude directly and only the reflex brake applies (pilot keeps
// authority; obstacle steering stays out of the loop).

#include "types.h"
#include "rc.h"
#include "boids.h"
#include "leader_follower.h"
#include "vfh_lite.h"
#include "governor.h"
#include "bvc.h"
#include "attitude.h"

namespace sc {

enum class BehaviorMode : uint8_t {
  kManual = 0,
  kHold = 1,
  kLeaderFollow = 2,  // hold a slot offset from the leader (needs shared frame)
  kFlock = 3,
  kDisperse = 4,
  // CONOPS mode: followers reproduce the leader's broadcast COMMANDED
  // velocity (the operator's stick intent on a manually flown leader, or a
  // GPS/computer-driven leader's autonomous command). Needs NO shared
  // position frame — robust to dead-reckoning drift. BVC separation, VFH
  // steering and the governor still apply on top.
  kMimic = 5,
};
constexpr uint8_t kBehaviorModeMax = uint8_t(BehaviorMode::kMimic);

struct BehaviorParams {
  BoidsParams boids;
  LeaderFollowerParams follow;
  VfhParams vfh;
  GovernorParams governor;
  BvcParams bvc;
  VelocityToTiltParams velTilt;
  float manualMaxTilt = deg2rad(25.0f);
  float manualMaxYawRate = deg2rad(180.0f);
  // MANUAL reflex brake: scale pilot forward-pitch by governor cap / vMax.
  bool manualReflexBrake = true;
};

struct BehaviorInputs {
  BehaviorMode mode = BehaviorMode::kManual;
  RcNorm rc;
  uint32_t rcAgeMs = 0;
  VehicleState self;
  SectorArray sectors;          // steering view: V1, or min-fused V1+V2
  // Reflex view: DIRECT-measuring sensor only (ToF). The governor/stop-reflex
  // never runs on learned perception (docs 05/07 rule). When absent, the
  // steering view is used for both.
  SectorArray reflexSectors;
  bool hasReflexSectors = false;
  const NeighborTable* table = nullptr;
  uint32_t nowMs = 0;
  Vec2 goal;                    // optional flock goal
  bool hasGoal = false;
};

struct BehaviorOutput {
  AttitudeSetpoint setpoint;
  Vec2 desiredVel;              // before constraints (telemetry)
  Vec2 commandedVel;            // after constraints (telemetry)
  ReflexState reflex = ReflexState::kCruise;
  FollowStatus followStatus = FollowStatus::kNoLeader;
  bool vfhSteered = false;
  bool vfhAllBlocked = false;
};

class BehaviorPipeline {
 public:
  explicit BehaviorPipeline(const BehaviorParams& p = {})
      : p_(p), boids_(p.boids), follow_(p.follow), vfh_(p.vfh),
        governor_(p.governor), bvc_(p.bvc) {}

  void setParams(const BehaviorParams& p) {
    p_ = p;
    boids_.setParams(p.boids);
    follow_.setParams(p.follow);
    vfh_.setParams(p.vfh);
    governor_.setParams(p.governor);
    bvc_.setParams(p.bvc);
  }
  const BehaviorParams& params() const { return p_; }
  void setLeader(uint8_t id) { p_.follow.leaderId = id; follow_.setLeader(id); }

  BehaviorOutput update(const BehaviorInputs& in);

 private:
  BehaviorParams p_;
  Boids boids_;
  LeaderFollower follow_;
  VfhLite vfh_;
  Governor governor_;
  Bvc bvc_;
};

}  // namespace sc
