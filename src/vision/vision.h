#pragma once
// V2 perception (Build B, XIAO ESP32S3 Sense): camera -> ML depth-to-sectors.
// ADVISORY ONLY: vision sectors are min-fused into VFH steering, but the stop
// reflex / governor stays on the ToF (docs 05/07 rule). With no model compiled
// in (model_data.cc placeholder), the pipeline still runs capture + SD logging
// — dataset collection does not need a trained model.

#include <stdint.h>

namespace vision {

void start();                 // spawns vision (+ logger) tasks; no-op unless
                              // PERCEPTION_V2_VISION is defined
bool cameraHealthy();
bool backendReady();          // model loaded and interpreter allocated
uint32_t inferenceCount();
uint32_t lastInferenceUs();
const uint8_t* lastPredBins();  // 8 bytes, 0xFF = n/a (for logging/telemetry)

}  // namespace vision
