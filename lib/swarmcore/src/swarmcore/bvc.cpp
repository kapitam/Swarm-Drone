#include "bvc.h"

namespace sc {

Vec2 Bvc::constrainVelocity(const Vec2& desired, const Vec2& selfPos,
                    const NeighborTable& table, uint32_t nowMs) const {
  Vec2 v = desired;
  // Two passes: projecting onto one plane can violate another; at swarm
  // scale (<=12 neighbors) two sweeps settle all practical cases.
  for (int pass = 0; pass < 2; ++pass) {
    for (int i = 0; i < NeighborTable::kMax; ++i) {
      const Neighbor& nb = table.slot(i);
      if (!nb.used || !nb.poseValid()) continue;
      const Vec2 npos = nb.predictedPos(nowMs);
      const Vec2 diff = npos - selfPos;         // toward neighbor
      const float dist = diff.norm();
      if (dist < 1e-4f || dist > p_.rangeM) continue;

      const Vec2 n = diff.normalized();         // plane normal, toward neighbor
      // Max displacement along n before leaving the buffered cell:
      // half the gap minus the safety radius.
      const float maxAlong = 0.5f * dist - p_.safetyRadius;
      const float step = v.dot(n) * p_.horizonS;

      if (maxAlong <= 0.0f) {
        // Already inside the buffer: forbid any approach; add gentle escape.
        const float vn = v.dot(n);
        if (vn > 0.0f) v = v - n * vn;
        v = v - n * (0.5f * (p_.safetyRadius - 0.5f * dist) / p_.safetyRadius);
      } else if (step > maxAlong) {
        // Clip the approach component so the horizon step stays inside.
        v = v - n * ((step - maxAlong) / p_.horizonS);
      }
    }
  }
  return v;
}

}  // namespace sc
