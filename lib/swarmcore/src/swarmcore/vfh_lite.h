#pragma once
// VFH-lite steering over the 8-sector forward array (research docs 02, 08 s7):
// binary polar histogram with hysteresis (block at 1400 mm, release at
// 1600 mm), +-1 sector width inflation, then pick the free sector closest to
// the desired direction. Returns a body-frame steering decision; the caller
// (behavior pipeline) applies it to the desired velocity.

#include "types.h"

namespace sc {

struct VfhParams {
  uint16_t blockMm    = 1400;  // sector blocked below this (doc 08 react dist)
  uint16_t hystMm     = 200;   // release at blockMm + hystMm
  uint8_t  inflate    = 1;     // widen obstacles by +-N sectors (vehicle width)
  bool     unknownIsBlocked = false;  // unknown sectors: free (V1 default) —
                                      // the governor still limits speed.
};

struct VfhResult {
  bool  allBlocked = false;    // no free sector in FoV -> caller must stop/escape
  bool  steered    = false;    // desiredDir was blocked, output differs
  float headingRad = 0.0f;     // chosen direction, body frame (0 = forward)
  uint8_t blockedMask = 0;     // bit i = sector i blocked (after inflation)
};

class VfhLite {
 public:
  explicit VfhLite(const VfhParams& p = {}) : p_(p) { reset(); }
  void setParams(const VfhParams& p) { p_ = p; }
  void reset() { for (int i = 0; i < kSectors; ++i) blocked_[i] = false; }

  // desiredDirRad: desired travel direction in BODY frame, wrapped (-pi..pi].
  // Directions outside the sensor FoV pass through unchanged (no data there;
  // inter-agent safety is BVC's job, and the governor caps forward speed).
  VfhResult update(const SectorArray& s, float desiredDirRad);

 private:
  VfhParams p_;
  bool blocked_[kSectors];  // hysteresis state
};

}  // namespace sc
