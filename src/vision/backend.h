#pragma once
// Inference backend fork (HANDBOOK "Forks"):
//   VISION_BACKEND_TFLM  TFLite-Micro 1.3.7 + ESP-NN (bundled in the Arduino
//                        core, doc 09 s1) running the compiled-in model from
//                        model_data.cc. Handles BOTH model forks by output
//                        shape: 8x4 logits (SectorNet-8) or 12x12 depth map
//                        (uPyD-Net-lite, min-pooled to sectors).
//   VISION_BACKEND_STUB  no inference; sectors stay unknown. Proves the
//                        interface + lets the logger run standalone.

#include <stdint.h>
#include "swarmcore/types.h"

namespace vision_backend {

bool init();  // false = no usable model (caller keeps logging regardless)
// img: VISION_IMG_W*VISION_IMG_H grayscale. Returns false if not ready.
// out: sector distances [mm] + validZones set; predBins: 8 bytes (0xFF n/a).
bool run(const uint8_t* img, sc::SectorArray& out, uint8_t predBins[8],
         uint32_t nowMs);
const char* name();
uint32_t lastRunUs();

}  // namespace vision_backend
