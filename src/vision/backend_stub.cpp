#include "backend.h"

#if defined(PERCEPTION_V2_VISION) && defined(VISION_BACKEND_STUB)

#include <Arduino.h>

namespace vision_backend {

bool init() {
  Serial.println("[vision] STUB backend: capture/logging only, no inference");
  return false;  // never "ready": sectors stay unknown
}

bool run(const uint8_t*, sc::SectorArray&, uint8_t predBins[8], uint32_t) {
  for (int i = 0; i < 8; ++i) predBins[i] = 0xFF;
  return false;
}

const char* name() { return "STUB"; }
uint32_t lastRunUs() { return 0; }

}  // namespace vision_backend

#endif
