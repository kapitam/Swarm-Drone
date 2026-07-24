#pragma once
// VL53L5CX 8x8 grid -> 8-sector reduction (research doc 08 s4):
//  - a zone is valid iff target_status in {5,9} AND nb_target_detected == 1
//    (the flight-proven ETH filter),
//  - min-pool each column over the middle rows 2..5 (rows 0-1 = ceiling,
//    6-7 = ground at typical mounting),
//  - clamp to 2000 mm (validity collapses beyond that indoors),
//  - a sector that loses validity holds its last value for up to 3 frames
//    ("invalid is not free"), then degrades to unknown.

#include "types.h"

namespace sc {

struct SectorReduceParams {
  uint8_t  gridSize     = 8;     // 8 (8x8). 4x4 supported with gridSize=4.
  uint8_t  rowMin       = 2;     // inclusive; middle-rows window (8x8)
  uint8_t  rowMax       = 5;     // inclusive
  uint16_t clampMm      = SectorArray::kMaxMm;
  uint8_t  holdFrames   = 3;     // hold-last-valid frames per sector
};

class SectorFilter {
 public:
  explicit SectorFilter(const SectorReduceParams& p = {}) : p_(p) { reset(); }

  void reset();

  // distMm/status/nbTargets are gridSize*gridSize arrays, row-major with
  // row 0 = top of FoV. Returns the filtered sector array.
  const SectorArray& update(const uint16_t* distMm, const uint8_t* status,
                            const uint8_t* nbTargets, uint32_t nowMs);

  const SectorArray& sectors() const { return out_; }

  static bool zoneValid(uint8_t status, uint8_t nbTargets) {
    return (status == 5 || status == 9) && nbTargets == 1;
  }

 private:
  SectorReduceParams p_;
  SectorArray out_;
  uint8_t staleCount_[kSectors];
};

}  // namespace sc
