#pragma once
// Neighbor table: the swarm's view of its peers, fed by StatePacket beacons.
// Pattern per research doc 01 s6.2: hearing a beacon IS discovery; entries
// expire after a timeout; consumers extrapolate stale state by packet age
// (constant velocity) before use — the single highest-value robustness trick.

#include "types.h"
#include "packets.h"

namespace sc {

struct Neighbor {
  uint8_t  id = 0xFF;
  Pose2D   pose;          // as last reported
  Vec2     vel;
  float    z = 0.0f;
  uint8_t  mode = 0;
  uint8_t  gradient = 0xFF;
  uint8_t  flags = 0;
  float    batteryV = 0.0f;
  uint16_t lastSeq = 0;
  uint32_t lastHeardMs = 0;
  uint16_t rxCount = 0;
  uint16_t lossEstimate = 0;  // cumulative seq gaps (coarse PDR input)
  bool     used = false;

  bool poseValid() const { return flags & kFlagPoseValid; }
  // Constant-velocity extrapolation to 'nowMs'.
  Vec2 predictedPos(uint32_t nowMs) const {
    float dt = (nowMs - lastHeardMs) * 1e-3f;
    if (dt < 0.0f) dt = 0.0f;
    if (dt > 1.0f) dt = 1.0f;  // don't extrapolate past 1 s
    return pose.p + vel * dt;
  }
};

class NeighborTable {
 public:
  static constexpr int kMax = 12;

  // Returns false if the packet failed CRC / was our own echo.
  bool update(const StatePacket& p, uint32_t nowMs, uint8_t selfId);
  // Drop entries not heard for ttlMs. Returns number of live entries.
  int expire(uint32_t nowMs, uint32_t ttlMs = 1500);

  int count() const;
  const Neighbor* byId(uint8_t id) const;
  // Iteration helper: slot may be unused, check ->used.
  const Neighbor& slot(int i) const { return n_[i]; }

  // Smallest gradient among live neighbors (255 if none).
  uint8_t minNeighborGradient(uint32_t nowMs, uint32_t ttlMs = 1500) const;

 private:
  Neighbor n_[kMax];
};

}  // namespace sc
