#pragma once
// Boids/potential-field hybrid (research docs 01 s3.1/s3.3, 06 #1 pick):
// separation + alignment + cohesion over the neighbor table, plus rectangular
// geofence "shill" repulsion (Vasarhelyi-style wall term, simplified) and an
// optional goal attraction. Gains are a config blob so simulator-tuned values
// transfer 1:1 (doc 01 s5.1 methodology).

#include "types.h"
#include "neighbor_table.h"

namespace sc {

struct BoidsParams {
  float rPerception = 4.0f;   // neighbor radius [m]
  float rSeparation = 1.2f;   // separation kicks in below this [m]
  float wSeparation = 1.6f;
  float wAlignment  = 0.6f;
  float wCohesion   = 0.35f;
  float wGoal       = 0.5f;
  float vFlock      = 0.6f;   // preferred cruise speed [m/s]
  float vMax        = 1.0f;   // output clamp [m/s]
  // Rectangular geofence (shared frame). Repulsion ramps inside wallMargin.
  float fenceXMin = -10.0f, fenceXMax = 10.0f;
  float fenceYMin = -10.0f, fenceYMax = 10.0f;
  float wallMargin = 1.5f;    // [m]
  float wWall      = 1.2f;
};

struct BoidsDebug {
  Vec2 sep, ali, coh, goal, wall;
  int neighborsUsed = 0;
};

class Boids {
 public:
  explicit Boids(const BoidsParams& p = {}) : p_(p) {}
  void setParams(const BoidsParams& p) { p_ = p; }
  const BoidsParams& params() const { return p_; }

  // Desired horizontal velocity in the shared frame.
  // separationOnly: DISPERSE mode (separation + walls only).
  Vec2 compute(const VehicleState& self, const NeighborTable& table,
               uint32_t nowMs, const Vec2* goal = nullptr,
               bool separationOnly = false, BoidsDebug* dbg = nullptr) const;

 private:
  BoidsParams p_;
};

}  // namespace sc
