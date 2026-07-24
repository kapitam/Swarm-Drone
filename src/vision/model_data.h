#pragma once
// Compiled-in .tflite model (int8). PLACEHOLDER: length 0 until a model is
// trained. Regenerate with:  python ml/gen_c_array.py <model.tflite>
// which overwrites model_data.cc (and bumps kModelVersion).

#include <stdint.h>
#include <stddef.h>

extern const uint8_t g_model_data[] __attribute__((aligned(16)));
extern const size_t g_model_data_len;
extern const uint8_t kModelVersion;  // 0 = no model
