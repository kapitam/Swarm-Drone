#include "backend.h"

#if defined(PERCEPTION_V2_VISION) && defined(VISION_BACKEND_TFLM)

#include <Arduino.h>
#include "../config/config.h"
#include "model_data.h"

// TFLM 1.3.7 + ESP-NN ship precompiled inside the Arduino-ESP32 3.3.x core
// (research doc 09 s1.2; verified by compile test). No extra libraries.
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"

namespace vision_backend {

// Arena in internal SRAM .bss, NOT PSRAM (doc 09 s1.6: PSRAM arena costs
// ~13% + contention with camera DMA). 96 KB budget for SectorNet-8; shrink
// to arena_used_bytes() + 8 KB once measured on target.
alignas(16) static uint8_t tensorArena[96 * 1024];

static const tflite::Model* model = nullptr;
static tflite::MicroInterpreter* interpreter = nullptr;
static TfLiteTensor* input = nullptr;
static TfLiteTensor* output = nullptr;
static bool ready = false;
static uint32_t runUs = 0;

// Representative distance per SectorNet bin, mm (bins: <0.5 / 0.5-1 / 1-2 /
// >2 m-or-free — doc 09 s2.1). Bin 3 maps to "far": beyond the clamp.
static const uint16_t kBinDistMm[4] = {350, 750, 1500, sc::SectorArray::kMaxMm};

bool init() {
  if (g_model_data_len == 0) {
    Serial.println("[vision] no model compiled in (placeholder) - logging-only mode");
    return false;
  }
  model = tflite::GetModel(g_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("[vision] model schema version mismatch");
    return false;
  }
  // Op set covers both model forks (doc 09 s1.5 accelerated-op constraint).
  static tflite::MicroMutableOpResolver<12> resolver;
  resolver.AddConv2D();
  resolver.AddDepthwiseConv2D();
  resolver.AddFullyConnected();
  resolver.AddAveragePool2D();
  resolver.AddMaxPool2D();
  resolver.AddMean();
  resolver.AddReshape();
  resolver.AddSoftmax();
  resolver.AddLogistic();
  resolver.AddResizeNearestNeighbor();
  resolver.AddConcatenation();
  resolver.AddAdd();

  static tflite::MicroInterpreter staticInterpreter(
      model, resolver, tensorArena, sizeof(tensorArena));
  interpreter = &staticInterpreter;
  if (interpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("[vision] AllocateTensors failed (arena too small?)");
    return false;
  }
  input = interpreter->input(0);
  output = interpreter->output(0);
  Serial.printf("[vision] model v%u loaded, arena used %u B\n", kModelVersion,
                unsigned(interpreter->arena_used_bytes()));
  ready = true;
  return true;
}

// SectorNet head: int8 logits [1, 8, 4] (or flat 32). Argmax per sector.
static void decodeSectorNet(const TfLiteTensor* t, sc::SectorArray& out,
                            uint8_t predBins[8]) {
  const int8_t* q = t->data.int8;
  for (int s = 0; s < sc::kSectors; ++s) {
    int best = 0;
    int8_t bestV = q[s * 4];
    for (int b = 1; b < 4; ++b)
      if (q[s * 4 + b] > bestV) { bestV = q[s * 4 + b]; best = b; }
    predBins[s] = uint8_t(best);
    out.distMm[s] = kBinDistMm[best];
    out.validZones[s] = 1;
  }
}

// uPyD-Net-lite head: dense depth map HxW (e.g. 12x12), int8 quantized.
// Min-pool middle rows per column band into 8 sectors; dequantized value is
// meters (calibration of the depth scale is a training-pipeline contract).
static void decodeDepthMap(const TfLiteTensor* t, sc::SectorArray& out,
                           uint8_t predBins[8]) {
  const int h = t->dims->data[1], w = t->dims->data[2];
  const float scale = t->params.scale;
  const int zp = t->params.zero_point;
  const int8_t* q = t->data.int8;
  const int rMin = h / 4, rMax = (3 * h) / 4 - 1;      // middle rows
  for (int s = 0; s < sc::kSectors; ++s) {
    const int cMin = (s * w) / sc::kSectors;
    const int cMax = ((s + 1) * w) / sc::kSectors - 1;
    float minM = 1e9f;
    for (int r = rMin; r <= rMax; ++r)
      for (int c = cMin; c <= cMax; ++c) {
        const float m = (float(q[r * w + c]) - zp) * scale;
        if (m > 0.01f && m < minM) minM = m;
      }
    if (minM < 1e8f) {
      uint32_t mm = uint32_t(minM * 1000.0f);
      if (mm > sc::SectorArray::kMaxMm) mm = sc::SectorArray::kMaxMm;
      out.distMm[s] = uint16_t(mm);
      out.validZones[s] = 1;
      predBins[s] = mm < 500 ? 0 : mm < 1000 ? 1 : mm < 2000 ? 2 : 3;
    }
  }
}

bool run(const uint8_t* img, sc::SectorArray& out, uint8_t predBins[8],
         uint32_t nowMs) {
  if (!ready) return false;
  const uint32_t t0 = micros();
  // uint8 gray -> int8 (full-int8 model I/O per doc 09 s3: zero_point -128).
  int8_t* in = input->data.int8;
  const int n = VISION_IMG_W * VISION_IMG_H;
  for (int i = 0; i < n; ++i) in[i] = int8_t(int(img[i]) - 128);

  if (interpreter->Invoke() != kTfLiteOk) return false;

  out = sc::SectorArray{};
  int outElems = 1;
  for (int i = 0; i < output->dims->size; ++i) outElems *= output->dims->data[i];
  if (outElems == sc::kSectors * 4) decodeSectorNet(output, out, predBins);
  else if (output->dims->size >= 3) decodeDepthMap(output, out, predBins);
  else return false;

  out.stampMs = nowMs;
  runUs = micros() - t0;
  return true;
}

const char* name() { return "TFLM+ESP-NN"; }
uint32_t lastRunUs() { return runUs; }

}  // namespace vision_backend

#endif  // PERCEPTION_V2_VISION && VISION_BACKEND_TFLM
