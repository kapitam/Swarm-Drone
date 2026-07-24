// PLACEHOLDER model blob — replaced by ml/gen_c_array.py after training.
// With g_model_data_len == 0 the TFLM backend reports "no model" and the
// vision task runs capture + SD logging only (dataset collection mode).
#include "model_data.h"

const uint8_t g_model_data[] __attribute__((aligned(16))) = {0};
const size_t g_model_data_len = 0;
const uint8_t kModelVersion = 0;
