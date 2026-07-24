#pragma once
// Buffered Voronoi Cell velocity filter (research docs 02 s4, 06 #15):
// inter-agent collision avoidance needing only neighbor POSITIONS (matches
// the StatePacket), O(neighbors) per tick. For each neighbor, the bisector
// half-plane (shifted toward us by the safety radius) constrains our next
// displacement; the desired velocity is projected to satisfy all planes.
// Proven approach on STM32-class hardware (Crazyflie onboard collision
// avoidance is BVC-based).

#include "types.h"
#include "neighbor_table.h"

namespace sc {

struct BvcParams {
  float safetyRadius = 0.5f;  // half of min center-to-center separation [m]
  float horizonS     = 0.5f;  // displacement horizon the plane test uses [s]
  float rangeM       = 4.0f;  // ignore neighbors beyond this [m]
};

class Bvc {
 public:
  explicit Bvc(const BvcParams& p = {}) : p_(p) {}
  void setParams(const BvcParams& p) { p_ = p; }
  const BvcParams& params() const { return p_; }

  // Returns 'desired' clipped so that (pos + v*horizon) stays inside our
  // buffered Voronoi cell w.r.t. every live neighbor.
  Vec2 constrain(const Vec2& desired, const Vec2& selfPos,
                 const NeighborTable& table, uint32_t nowMs) const;

 private:
  BvcParams p_;
};

}  // namespace sc
