#include "vfh_lite.h"

namespace sc {

VfhResult VfhLite::update(const SectorArray& s, float desiredDirRad) {
  VfhResult r;
  r.headingRad = desiredDirRad;

  // 1) Update per-sector binary state with hysteresis.
  for (int i = 0; i < kSectors; ++i) {
    if (!s.known(i)) {
      blocked_[i] = p_.unknownIsBlocked;
      continue;
    }
    if (blocked_[i]) {
      if (s.distMm[i] >= uint16_t(p_.blockMm + p_.hystMm)) blocked_[i] = false;
    } else {
      if (s.distMm[i] < p_.blockMm) blocked_[i] = true;
    }
  }

  // 2) Inflate by +-inflate sectors (vehicle width).
  bool inflated[kSectors];
  for (int i = 0; i < kSectors; ++i) {
    inflated[i] = false;
    for (int k = -int(p_.inflate); k <= int(p_.inflate); ++k) {
      const int j = i + k;
      if (j >= 0 && j < kSectors && blocked_[j]) { inflated[i] = true; break; }
    }
  }
  for (int i = 0; i < kSectors; ++i)
    if (inflated[i]) r.blockedMask |= uint8_t(1u << i);

  // 3) Desired direction outside the forward FoV: nothing to say.
  const float halfFov = deg2rad(kSectorFovDeg) / 2.0f;
  if (desiredDirRad < -halfFov || desiredDirRad > halfFov) return r;

  // 4) Find nearest sector to the desired direction; if free, done.
  int want = 0;
  float bestD = 1e9f;
  for (int i = 0; i < kSectors; ++i) {
    const float d = fabsf(SectorArray::sectorCenterRad(i) - desiredDirRad);
    if (d < bestD) { bestD = d; want = i; }
  }
  if (!inflated[want]) return r;

  // 5) Otherwise pick the free sector with the smallest angular deviation.
  int pick = -1;
  bestD = 1e9f;
  for (int i = 0; i < kSectors; ++i) {
    if (inflated[i]) continue;
    const float d = fabsf(SectorArray::sectorCenterRad(i) - desiredDirRad);
    if (d < bestD) { bestD = d; pick = i; }
  }
  if (pick < 0) {
    r.allBlocked = true;  // caller: stop / escape mode
    return r;
  }
  r.steered = true;
  r.headingRad = SectorArray::sectorCenterRad(pick);
  return r;
}

}  // namespace sc
