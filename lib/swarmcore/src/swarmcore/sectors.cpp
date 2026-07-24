#include "sectors.h"

namespace sc {

void SectorFilter::reset() {
  out_ = SectorArray{};
  for (int i = 0; i < kSectors; ++i) staleCount_[i] = 0xFF;
}

const SectorArray& SectorFilter::update(const uint16_t* distMm,
                                        const uint8_t* status,
                                        const uint8_t* nbTargets,
                                        uint32_t nowMs) {
  const int n = p_.gridSize;
  // Map grid columns onto the kSectors output (1:1 for 8x8, 1:2 for 4x4).
  const int colsPerSector = (n >= kSectors) ? 1 : kSectors / n;

  uint16_t best[kSectors];
  uint8_t  cnt[kSectors];
  for (int i = 0; i < kSectors; ++i) { best[i] = SectorArray::kUnknownMm; cnt[i] = 0; }

  int rMin = p_.rowMin, rMax = p_.rowMax;
  if (n == 4) { rMin = 1; rMax = 2; }  // middle rows of a 4x4 grid

  for (int r = rMin; r <= rMax && r < n; ++r) {
    for (int c = 0; c < n; ++c) {
      const int z = r * n + c;
      if (!zoneValid(status[z], nbTargets[z])) continue;
      uint16_t d = distMm[z];
      if (d > p_.clampMm) d = p_.clampMm;
      // Grid column 0 is the sensor's leftmost column; keep sector 0 = left.
      const int s0 = (n >= kSectors) ? c : c * colsPerSector;
      for (int k = 0; k < colsPerSector; ++k) {
        const int s = s0 + k;
        if (s >= kSectors) break;
        if (d < best[s]) best[s] = d;
        if (cnt[s] < 0xFF) ++cnt[s];
      }
    }
  }

  for (int s = 0; s < kSectors; ++s) {
    if (cnt[s] > 0) {
      out_.distMm[s] = best[s];
      out_.validZones[s] = cnt[s];
      staleCount_[s] = 0;
    } else if (staleCount_[s] < p_.holdFrames) {
      // Hold last value: an invalid reading is NOT evidence of free space.
      ++staleCount_[s];
      // keep out_.distMm[s]; validZones stays >0 so consumers still act on it
      if (out_.validZones[s] == 0) out_.validZones[s] = 1;
    } else {
      out_.distMm[s] = SectorArray::kUnknownMm;
      out_.validZones[s] = 0;
    }
  }
  out_.stampMs = nowMs;
  return out_;
}

}  // namespace sc
