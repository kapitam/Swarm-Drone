#include "boids.h"

namespace sc {

Vec2 Boids::compute(const VehicleState& self, const NeighborTable& table,
                    uint32_t nowMs, const Vec2* goal, bool separationOnly,
                    BoidsDebug* dbg) const {
  Vec2 sep{}, ali{}, coh{};
  Vec2 centroid{};
  int used = 0;

  for (int i = 0; i < NeighborTable::kMax; ++i) {
    const Neighbor& nb = table.slot(i);
    if (!nb.used || !nb.poseValid()) continue;
    const Vec2 npos = nb.predictedPos(nowMs);
    const Vec2 d = self.pose.p - npos;
    const float dist = d.norm();
    if (dist > p_.rPerception || dist < 1e-4f) continue;
    ++used;

    if (dist < p_.rSeparation) {
      // 1/d falloff, normalized direction away from the neighbor.
      sep += d.normalized() * ((p_.rSeparation - dist) / p_.rSeparation / dist);
    }
    ali += nb.vel - self.vel;
    centroid += npos;
  }

  Vec2 out{};
  if (used > 0) {
    ali = ali * (1.0f / used);
    coh = centroid * (1.0f / used) - self.pose.p;
    out += sep * p_.wSeparation;
    if (!separationOnly) {
      out += ali * p_.wAlignment;
      out += coh * p_.wCohesion;
    }
  }

  Vec2 goalTerm{};
  if (goal && !separationOnly) {
    goalTerm = (*goal - self.pose.p).limited(1.0f) * p_.vFlock;
    out += goalTerm * p_.wGoal;
  } else if (!separationOnly && used > 0) {
    // No explicit goal: keep cruising along current heading (Vicsek-like
    // persistence so the flock doesn't collapse into the centroid).
    out += Vec2{cosf(self.pose.yaw), sinf(self.pose.yaw)} * (p_.vFlock * 0.5f);
  }

  // Geofence shill repulsion: ramps linearly inside wallMargin of each face.
  Vec2 wall{};
  auto ramp = [&](float dToWall) {
    return clampf((p_.wallMargin - dToWall) / p_.wallMargin, 0.0f, 1.0f);
  };
  wall.x += ramp(self.pose.p.x - p_.fenceXMin);   // left face pushes +x
  wall.x -= ramp(p_.fenceXMax - self.pose.p.x);   // right face pushes -x
  wall.y += ramp(self.pose.p.y - p_.fenceYMin);
  wall.y -= ramp(p_.fenceYMax - self.pose.p.y);
  out += wall * (p_.wWall * p_.vFlock);

  if (dbg) {
    dbg->sep = sep; dbg->ali = ali; dbg->coh = coh;
    dbg->goal = goalTerm; dbg->wall = wall; dbg->neighborsUsed = used;
  }
  return out.limited(p_.vMax);
}

}  // namespace sc
