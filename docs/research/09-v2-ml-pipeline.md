# 09 — V2 Camera Perception: ML Pipeline Deep-Dive

**Scope:** Implementation-ready specification of the machine-learning pipeline behind the camera + ML depth-to-sectors perception version defined in [07-camera-depth-version-spec.md](07-camera-depth-version-spec.md): the on-device inference runtime (with exact dependency pins), concrete specs for **both** model forks (SectorNet-8 and µPyD-Net-lite), the PyTorch→int8→C-array conversion pipeline, the camera/SD data-capture configuration on the Seeed XIAO ESP32S3 Sense, and the training/fine-tune recipe. The firmware and host-side training code are being written in parallel; this document is the reference they build against.

**Status:** Research complete (July 2026). All URLs in §8 were retrieved and verified during this research, except where marked "verified in doc 07". One key claim — that the current Arduino-ESP32 core ships TFLite-Micro + ESP-NN precompiled — was additionally **verified by an actual PlatformIO compile** of a TFLM smoke-test sketch for the `seeed_xiao_esp32s3` board (§1.4).

---

## 1. Inference Runtime on ESP32-S3 under Arduino/PlatformIO

### 1.1 The 2026 landscape — what changed since doc 07

Doc 07 §4.4 recommended "TFLite-Micro + ESP-NN first" and assumed it would be consumed as a third-party Arduino library. The situation is now better than that: **the official Arduino-ESP32 core (3.3.x) ships Espressif's `esp-tflite-micro` and `esp-nn` precompiled inside its static-library bundle** — no external ML library is needed at all.

Evidence chain (each step verified directly):

1. The [esp32-arduino-lib-builder manifest](https://github.com/espressif/esp32-arduino-lib-builder/blob/master/main/idf_component.yml) declares `espressif/esp-tflite-micro: ">=1.2.0", require: public` (all targets except ESP32-C2) and `espressif/esp32-camera` as bundled components.
2. The current core's precompiled-libs artifact (`esp32-arduino-libs-idf-release_v5.5`, built 2026-07-20 on ESP-IDF v5.5.5) was downloaded and inspected: it contains `esp32s3/lib/libespressif__esp-tflite-micro.a` (3.0 MB), `esp32s3/lib/libespressif__esp-nn.a` (1.2 MB), full TFLM headers, and both `-lespressif__esp-tflite-micro -lespressif__esp-nn` in the S3 link flags. Its `versions.txt` pins **esp-tflite-micro 1.3.7** and **esp-nn 1.2.3**.
3. The bundle's S3 `sdkconfig` has `CONFIG_NN_OPTIMIZED=y`, and the `.a` contains the S3 assembly kernels (`esp_nn_conv/dot/add/relu/…_esp32s3.S.obj`) — i.e. the **ESP32-S3 vector-instruction paths are compiled in**, not the ANSI-C fallbacks.
4. The [tanakamasayuki library's own README](https://github.com/tanakamasayuki/Arduino_TensorFlowLite_ESP32) (updated 2026-07-03) confirms: *"The official Arduino ESP32 Core now bundles the official library esp-tflite-micro … The official version is optimized (faster) and lets you use the latest release … Note, however, that there are almost no usage examples for the official version."*

`esp-tflite-micro` itself is at **v1.3.7** on the [ESP Component Registry](https://components.espressif.com/components/espressif/esp-tflite-micro) (released 2026-06-03; 1.3.5 was Nov 2025), and its [README benchmark](https://github.com/espressif/esp-tflite-micro) still anchors S3 performance: **person detection (96×96 int8 MobileNetV1-0.25) in 54 ms with ESP-NN vs 2300 ms without — a 42× speedup from the S3 vector kernels**.

### 1.2 Candidate-by-candidate verdicts

| Runtime | State (July 2026) | ESP-NN / S3 vectors? | Verdict for this project |
|---|---|---|---|
| **esp-tflite-micro bundled in Arduino-ESP32 3.3.x** | v1.3.7 + esp-nn 1.2.3 precompiled in the core's libs (verified in the artifact and by compile test §1.4) | **Yes** — S3 assembly kernels, `CONFIG_NN_OPTIMIZED=y` | **Primary pick.** Zero extra dependencies, current TFLM snapshot, official maintenance |
| esp-tflite-micro as IDF managed component | v1.3.7 on the registry; `idf.py add-dependency "espressif/esp-tflite-micro"` | Yes (compiled per-project) | The escape hatch if the project ever moves to Arduino-as-IDF-component / hybrid compile (pioarduino supports this) and needs menuconfig control over the component |
| **tanakamasayuki/TensorFlowLite_ESP32** | v1.0.0, released **2022-05-30**, no releases since; README now carries an explicit **deprecation notice** (2026-07-03) recommending the official bundle | **No** — the repo's `kernels/esp_nn/` directory contains only a README; the 2022 snapshot predates usable ESP-NN integration | Do not use. 4-year-old TFLM snapshot, unaccelerated, formally deprecated by its author |
| **EloquentTinyML 3.x + tflm_esp32** | EloquentTinyML last pushed 2024-07-18 (latest tagged release 2.4.0 from 2021); its required runtime [tflm_esp32](https://github.com/eloquentarduino/tflm_esp32) is v2.0.0, last pushed 2024-07-17, shipped as a **precompiled** `libtflm_esp32.a` per target | Unverifiable from source (precompiled blob; no esp-nn source in tree); the author's own [person-detection tutorial](https://eloquentarduino.com/posts/esp32-cam-person-detection) states "detections take about 4-5 seconds per frame" — the *unaccelerated* performance class | Do not use for inference. The camera helper (`EloquentEsp32Cam`) also targets Arduino core 2.x, incompatible with the 3.3.x core we need |
| **ESP-DL** | [v3.3.8](https://components.espressif.com/components/espressif/esp-dl) (July 2026), IDF ≥5.3, proprietary `.espdl` format, quantized exclusively via [esp-ppq](https://github.com/espressif/esp-dl) (v1.3.5). Actively developed: AutoQuant + `espdl-quantize` agent skills added 2026-05 | Yes (its own S3 kernels) — but **S3 is per-tensor-only** quantization; per-channel Conv/GEMM landed **for ESP32-P4 only** (2026-04, needs esp-ppq ≥1.2.10 / esp-dl ≥3.3.1). S3 also uses power-of-two symmetric scales ([quantization docs](https://docs.espressif.com/projects/esp-dl/en/latest/tutorials/how_to_quantize_model.html)) | Optimization pass only, as doc 07 concluded — and note it was **dropped from the Arduino lib bundle in Oct 2024** (lib-builder commit "Drop support for ESP-DL on Xtensa chips"), so using it means IDF-component surgery. Per-tensor + power-of-two quantization is a known accuracy hazard (doc 07 §7.1) |

### 1.3 Primary recommendation + exact dependency pins

**Use the TFLM 1.3.7 + ESP-NN 1.2.3 runtime already inside the Arduino-ESP32 3.3.x core, under PlatformIO via pioarduino.** PlatformIO's own `espressif32` platform is frozen at Arduino core 2.x; the community [pioarduino platform](https://github.com/pioarduino/platform-espressif32) is the maintained path to core 3.3.x (its stable releases are "Arduino Release v3.3.x based on ESP-IDF v5.5.x"; latest at time of writing: tag `55.03.311` = Arduino v3.3.11 / IDF v5.5.5, released 2026-07-24).

```ini
; platformio.ini — verified to compile TFLM+ESP-NN with zero extra libraries (§1.4)
[env:xiao-s3-sense]
platform = https://github.com/pioarduino/platform-espressif32/releases/download/55.03.311/platform-espressif32.zip
framework = arduino
board = seeed_xiao_esp32s3          ; ships qio_opi memory_type + BOARD_HAS_PSRAM
board_build.psram_type = opi        ; octal PSRAM — mandatory (doc 07 §2.3)
monitor_speed = 115200
```

Sketch-side, include TFLM directly (headers and link flags are global in the core build):

```cpp
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
```

Pinning policy: pin the **platform release tag** (`55.03.311`), which transitively pins Arduino 3.3.11 / IDF 5.5.5 / esp-tflite-micro 1.3.7 / esp-nn 1.2.3 / esp32-camera (master snapshot; the standalone registry release is [2.1.7, 2026-06-05](https://components.espressif.com/api/components/espressif/esp32-camera)). Record the platform tag in firmware telemetry alongside the model version byte, so every logged dataset is attributable to an exact runtime.

Arduino-IDE users get the identical bits by installing ESP32 core 3.3.x via Boards Manager and selecting `XIAO_ESP32S3` + `PSRAM: OPI PSRAM` (a wrong PSRAM setting fails at camera init with `cam_hal: EV-EOF-OVF` — [independent XIAO review](https://quickfixsurrey.ca/seeed-xiao-esp32-s3-sense-review-2/)).

### 1.4 Compile-test verification (done during this research)

A minimal sketch instantiating `MicroMutableOpResolver` with Conv2D/DepthwiseConv2D/AveragePool2D/FullyConnected/Softmax/Reshape and referencing the `esp_nn_conv_s8` symbol was built with exactly the `platformio.ini` above (no `lib_deps` at all): **SUCCESS in 87 s**, total firmware footprint RAM 22.6 KB (6.9 % of internal SRAM budget) and flash 352 KB (10.5 %) including the Arduino core — the 8 MB-flash / 512 KB-SRAM budget is essentially untouched. This retires doc 07's residual toolchain risk for the primary runtime.

### 1.5 ESP-NN accelerated-op coverage on S3 (design constraint for both models)

From the [ESP-NN README](https://github.com/espressif/esp-nn) (S3 assembly versions, measured opt ratios): conv2d (**5.5–14.2×**), depthwise conv2d (**4.5–6.3×**), elementwise add/mul (3.5–3.8×), max/avg pool (7.8×/3.6×), fully connected (7.8×), ReLU/ReLU6/PReLU (11.5×), softmax (1.4×). Model-level anchors on S3 @240 MHz: person detect 54 ms (arena in PSRAM) / **47 ms (arena in internal RAM)**; MobileNetV3-Small 224×224×3 int8 = 1434 ms.

Both model forks below therefore restrict themselves to: `CONV_2D`, `DEPTHWISE_CONV_2D`, `AVERAGE_POOL_2D`/`MEAN`, `FULLY_CONNECTED`, `RESHAPE`, `SOFTMAX`, `RESIZE_NEAREST_NEIGHBOR`, `CONCATENATION`, `LOGISTIC` — everything hot is ESP-NN-accelerated; resize/concat are memory ops with negligible cost at our sizes. **No LSTM/GRU/attention, no transposed conv, no LeakyReLU** (not in the accelerated set; µPyD-Net-lite substitutes ReLU6, §2.3).

### 1.6 Tensor-arena sizing guidance

- Reference point: the bundled [person_detection example](https://github.com/espressif/esp-tflite-micro/blob/master/examples/person_detection/main/main_functions.cc) uses `kTensorArenaSize = 100 KB + 60 KB scratch` for a 250 k-param 96×96 model, allocated with `heap_caps_malloc(..., MALLOC_CAP_SPIRAM)` — but doc 07 §5.2's placement rule stands: **our arena goes in internal SRAM** (PSRAM arena costs ~13 % even on this small model: 54 vs 47 ms, and far more under camera-DMA + Wi-Fi contention).
- Analytic estimate for our models (activations + ESP-NN scratch): SectorNet-8 peak live tensors ≈ 28 KB (96×96 input + 48×48×8 first feature map) → **budget 96 KB arena**; µPyD-Net-lite48 peak ≈ 12 KB → **budget 64 KB arena**. Both fit internal SRAM alongside the Wi-Fi stack.
- Procedure: allocate generously once, call `interpreter.arena_used_bytes()` after `AllocateTensors()` (or use `RecordingMicroInterpreter`), then shrink to measured + 8 KB margin. Declare as `alignas(16) static uint8_t tensor_arena[...]` in `.bss` (internal SRAM) — **not** `ps_malloc`.
- Weights (`.tflite` C-array, §3.2 step 5) live in flash via `const` — mmapped through the flash cache; at our 30–60 KB model sizes this costs nothing measurable and keeps SRAM free.

---

## 2. Model Specs — Both Forks

Latency estimates below are **MAC-scaled from measured ESP-NN anchors**, not measurements: person detection = MobileNetV1-0.25, 96×96×1 → computed **7.16 MMAC / 211 k params** → measured 54 ms (PSRAM arena) / 47 ms (SRAM arena) on S3 @240 MHz. Scaling ≈ 7.5 ms/MMAC with a ~1.5× conservative band for per-layer overhead and radio/PSRAM contention. Both forks are conv-dominated with the same op mix, so first-order scaling is defensible; treat every latency cell as *design estimate, to be measured in week one on hardware*.

### 2.1 Fork A — SectorNet-8 (sector distance-bin classifier; primary per doc 07)

**Contract:** 96×96×1 grayscale uint8→int8 in; **8 sectors × 4 distance bins** logits out (reshaped to (8, 4), per-sector softmax). Bins per doc 07 §3.2: `<0.5 m / 0.5–1 m / 1–2 m / >2 m-or-free`, derived from VL53L5CX column minima.

**Proposed layer stack** (MobileNet-style depthwise-separable; all ops in the ESP-NN accelerated set):

| # | Layer | Output | MACs | Params |
|---|---|---|---|---|
| 0 | Conv 3×3, s2 (1→8) + ReLU6 | 48×48×8 | 165,888 | 80 |
| 1 | DW 3×3 s2 + PW 8→16 | 24×24×16 | 41,472 + 73,728 | 80 + 144 |
| 2 | DW 3×3 s2 + PW 16→32 | 12×12×32 | 20,736 + 73,728 | 160 + 544 |
| 3 | DW 3×3 s1 + PW 32→48 | 12×12×48 | 41,472 + 221,184 | 320 + 1,584 |
| 4 | DW 3×3 s2 + PW 48→64 | 6×6×64 | 15,552 + 110,592 | 480 + 3,136 |
| 5 | DW 3×3 s1 + PW 64→96 | 6×6×96 | 20,736 + 221,184 | 640 + 6,240 |
| 6 | GlobalAvgPool → FC 96→64 + ReLU6 | 64 | 6,144 | 6,208 |
| 7 | FC 64→32, Reshape (8,4), Softmax(axis=-1) | 8×4 | 2,048 | 2,080 |
| | **Total ("S" config)** | | **1.01 MMAC** | **21.7 k** |

- **Int8 flash size:** ≈ 22 KB weights + flatbuffer overhead → **~30–40 KB `.tflite`** (BatchNorm folds into convs at conversion). Well under doc 07's 64 KB ceiling.
- **Estimated S3 latency (ESP-NN, SRAM arena): ~8–15 ms → 25 ms worst-case with radio contention** → inference is no longer the rate limiter; the ~25–50 ms frame age is (doc 07 §5.3). Sustains the ≥10 Hz sector-output spec with ~4× headroom.
- **"M" width variant** (×1.5 channels: 12-24-48-72-96-144): 2.05 MMAC / 40.0 k params / ~55 KB int8 / est. 15–30 ms. Train both; keep M only if S under-fits (doc 07 targeted 25–50 k params — S sits just below, M inside).
- **Optional 2-frame input** (t, t−100 ms stacked as 2 channels, TinyNav-style motion cue): only layer 0 doubles → +0.17 MMAC, +72 params. Decide after v1 ablation, per doc 07.
- **Head duplication note:** the single FC 64→32 with reshape is deliberately minimal; if per-sector heads prove better, 8 separate FC 64→4 heads cost identical MACs (TFLM handles them as one FC with block-sparse weights only if hand-packed — keep the fused FC).

**Loss:** mean over sectors of cross-entropy with **distance-aware label smoothing**: instead of one-hot bin targets, smooth mass onto *adjacent distance bins only* (e.g. [0.8, 0.15, 0.05, 0] for bin 0 truth, symmetric interior) — ordinal structure without an ordinal head, tolerant of ToF teacher noise at bin edges. Weight the `<0.5 m` bin's rows ×2–4 in the loss (near-recall is the safety-critical metric, doc 07 §7.2). Mask sectors whose teacher column had >4 invalid zones (doc 07 §4.2) out of the loss. A plain-CE ablation and an explicit ordinal-regression head (CORAL-style, 3 cumulative sigmoids/sector) are cheap follow-ups if bin confusion off-by-two errors show up.

### 2.2 Fork B — µPyD-Net-lite (dense relative depth, decimated to sectors)

**Lineage (real, verified):** PyD-Net (Poggi et al., IROS 2018, 1.9 M params, pyramidal encoder + per-level shallow decoders) → **µPyD-Net** (Peluso, Cipolletta, Calimera, Poggi, Tosi, Aleotti, Mattoccia): first in [CVPRW 2020 "Enabling Monocular Depth Perception at the Very Edge"](https://openaccess.thecvf.com/content_CVPRW_2020/papers/w28/Peluso_Enabling_Monocular_Depth_Perception_at_the_Very_Edge_CVPRW_2020_paper.pdf) (48×48 or 32×32 input, 0.1 M params, int8, Cortex-M7-class), then the journal version [IEEE TCSVT 32(3):1524-1536, 2022, doi 10.1109/TCSVT.2021.3077395](https://cris.unibo.it/bitstream/11585/819845/5/main_iris.pdf). Exact original architecture from the TCSVT text: *shallow pyramidal encoder of six 3×3 convs (channels 8, 8, 16, 16, 32, 32) with leaky-ReLU α=0.125, three pyramid levels, three compact decoders restoring input resolution, ≈100 k params total*. It is the same network the ToF-teacher on-device-learning paper (Nadalini et al., [arXiv:2512.00086](https://arxiv.org/html/2512.00086), verified in doc 07) fine-tunes with VL53L5CX pseudo-labels — which is why it is our fork-B baseline: the training recipe transfers verbatim.

**Proposed µPyD-Net-lite config for the S3** (changes vs. original, each justified):

- Keep the 6-conv pyramidal encoder exactly (8,8,16,16,32,32 @ 24²/12²/6² for 48×48 input).
- **Drop the finest decoder** — output at 12×12 (level 2) instead of 48×48. The consumer is an 8-sector min-pool; 12×12 already oversamples it 1.5× per sector column. This deletes ~45 % of the original's decoder compute for zero interface loss. (A 3-decoder 24×24 variant is specced below for visualization/debug builds; a 64×64-input variant scales all maps ×1.78 in MACs — only worth it if 48×48 proves texture-starved.)
- **ReLU6 replaces leaky-ReLU** (α=0.125): leaky-ReLU is not ESP-NN-accelerated and quantizes worse (asymmetric negative tail); ReLU6 is the standard int8-friendly choice. This deviates from the paper and must be validated once against a leaky baseline on the host.
- **Nearest-neighbor upsample + concat** between levels (as PyD-Net does) — `RESIZE_NEAREST_NEIGHBOR` + `CONCATENATION` are TFLM builtins; no transposed conv.
- Output head: 1-channel sigmoid (relative inverse depth, as in the lineage) at 12×12.

| Block | Output | MACs | Params |
|---|---|---|---|
| Encoder L1 (conv s2 1→8, conv 8→8) | 24×24×8 | 373,248 | 664 |
| Encoder L2 (conv s2 8→16, conv 16→16) | 12×12×16 | 497,664 | 3,488 |
| Encoder L3 (conv s2 16→32, conv 32→32) | 6×6×32 | 497,664 | 13,888 |
| Decoder @6×6 (32→32→16→8→1) | 6×6×1 | 541,728 | 15,105 |
| Decoder @12×12 (concat ↑disp: 17→16→8→4→1) | 12×12×1 | 565,056 | 3,953 |
| **Total (2-decoder, 12×12 out)** | | **2.48 MMAC** | **37.1 k** |
| *(3-decoder variant, 24×24 out)* | *24×24×1* | *3.37 MMAC* | *38.7 k* |

- **Int8 flash size:** ~50 KB `.tflite`. **Estimated S3 latency: ~20–35 ms (2-decoder) / ~28–45 ms (3-decoder)** with ESP-NN + SRAM arena — comfortably ≥10 Hz, unlike the ~1 Hz-class full µPyD-Net ports doc 07 §3.1 found ([JPsparks S3 build ~6 s](https://github.com/JPsparks/APAI_ESP_MonocularDepthEstimation), verified in doc 05/07 — that port was unaccelerated and full-size; the 25–40× gap vs our estimate is ESP-NN + half the decoders + no float ops).
- **Min-pool-to-sectors postproc** (runs in the inference task, <0.1 ms): the 12×12 inverse-depth map ⇒ per sector k (8 columns of 1.5 map-columns each): `d_k = 1 / max(inv_depth[rows 2..9, cols ⌊1.5k⌋..⌈1.5k+1⌉])` — max of inverse depth = min distance; drop the top/bottom 2 rows (sky/floor bands, tuned on data). Then **scale-calibrate to metric** with the ToF: relative depth has unknown scale per scene, so maintain a rolling least-squares fit of `1/inv_depth → tof_min_m` per sector over the last N=100 co-observed frames (the teacher is co-mounted; when it is absent, freeze the last calibration and flag `confidence` low). Bin into the same 4 bins as fork A and emit the identical `perception_frame_t`.
- **Training:** identical data, but the loss is the lineage's dense one: per-pixel berHu (or L1) on inverse depth against the ToF 8×8 pseudo-labels upsampled to 12×12 with validity masking, plus an edge-aware smoothness term (weight ~1e-2) as in PyD-Net. 8×8→12×12 label upsampling uses nearest-neighbor + per-zone validity dilation; pixels with no valid zone are masked.

### 2.3 Fork-decision criteria (measure after both are trained on the same base dataset)

Run both forks through the doc 07 §7.2 gates on the same held-out environment, same fine-tune budget (3 k samples), and decide on:

| Criterion | Measure | Pre-registered expectation |
|---|---|---|
| **Sector accuracy vs ToF** | % (sector,frame) bin-exact; % within ±1 bin; `<0.5 m` recall | A wins if its direct supervision beats B's post-processed dense output; B wins if dense supervision regularizes better on 20–30 k samples (the doc 07 §3.3 hypothesis) |
| **Latency / rate** | p50/p95 `invoke()` + end-to-end sector rate, radio on | A ≈ 2–3× faster (1.0 vs 2.5 MMAC); both should clear ≥10 Hz — if B misses it, A wins by default |
| **Robustness** | Accuracy drop: new environment *before* fine-tune; low-light sessions; off-by-≥2-bin rate (dangerous confusions) | Key open question — dense-depth pretraining may transfer better across scenes; sector heads may latch onto scene-specific texture |
| **Scale stability (B only)** | Drift of the rolling metric calibration when the ToF is masked for 60 s | If B needs continuous ToF rescaling to stay metric, its camera-solo story collapses → A wins for camera-only builds |
| **Quantization damage** | Float→int8 delta on all metrics (per-fork) | A has the easier quant surface (classification tolerates quant noise; doc 07 §7.1) |
| **Side utility** | B's dense map is a debugging/visualization asset and a phase-2 (peer-detection backbone) head start | Tie-breaker only |

Both forks emit the same `perception_frame_t`, so the firmware, logging, and evaluation harness are fork-agnostic; keeping both alive until the numbers land costs one extra training script, not a firmware branch.

---

## 3. Quantization Toolchain 2026: PyTorch → int8 `.tflite` → C array

### 3.1 The three paths, honestly compared

| Path | State July 2026 | Full-int8 (weights **and** activations int8, int8 I/O — what TFLM needs) | Verdict |
|---|---|---|---|
| **(A) PyTorch → ONNX → onnx2tf → TFLite** | [onnx2tf v2.5.0](https://pypi.org/project/onnx2tf/2.5.0/), actively maintained; default backend switched to `flatbuffer_direct` in [v2.4.0 (2026-04)](https://github.com/PINTO0309/onnx2tf/releases/tag/2.4.0) (faster, TensorFlow no longer required at install); author states maintenance continues "as long as there is demand", recommends nobuco/litert-torch longer-term | **Yes, first-class**: `-oiqt` emits integer-quantized tflite with **per-channel weights (default)**, calibration via `-cind <input> <calib.npy> <mean> <std>`, and `-iqd int8 -oqd int8` for int8 I/O. Handles the NCHW→NHWC transposition problem automatically | **Primary path for our PyTorch training pipeline** |
| **(B) PyTorch → litert-torch (ex ai-edge-torch)** | Renamed [google-ai-edge/litert-torch](https://github.com/google-ai-edge/ai-edge-torch), v0.8.0 (2026-01-26), converter officially **Beta**; quantizes via PT2E/torchao ([docs](https://github.com/google-ai-edge/ai-edge-torch/blob/main/docs/pytorch_converter/README.md)) | **Fragile.** Static full-int8 via `PT2EQuantizer + get_symmetric_quantization_config(is_dynamic=False)` exists, but real-world reports document crashes and silent fallbacks to dynamic-range quant (float activations — **unusable on TFLM conv kernels**): [issue #150](https://github.com/google-ai-edge/ai-edge-torch/issues/150), [ailia's conversion write-ups](https://tech.ailia.ai/en/quantization-with-ai-edge-torch-1efe17b93cd7/). QAT via PT2E is explicitly "not supported yet / untested" (issue #150 maintainer comment) | Track it (it is Google's strategic path and removes the ONNX hop), **don't build on it yet**. Re-evaluate at v1.0 |
| **(C) Train in Keras, skip conversion friction** | `tf.lite.TFLiteConverter` full-int8 flow unchanged and canonical ([LiteRT PTQ docs](https://developers.google.com/edge/litert/conversion/tensorflow/quantization/post_training_quantization)) | **Yes — the reference implementation** of exactly the recipe TFLM wants | Fallback, not primary: the project standardized on PyTorch (doc 07 §4.4), and at 20–40 k params porting the model class is trivial *if* path A ever breaks. Keep the model definition framework-portable (plain conv/dw/fc) |

Post-conversion add-on: Google's [ai-edge-quantizer](https://github.com/google-ai-edge/ai-edge-quantizer) can apply `STATIC_WI8_AI8` recipes to an FP32 `.tflite` — a possible future replacement for the converter-embedded calibration, but it solves a problem paths A/C already solve, so it is noted and not used.

### 3.2 Reference pipeline (fork A shown; fork B identical apart from the model)

Host environment pins (Python 3.11/3.12 — [ai-edge-litert has no 3.13 wheels](https://pypi.org/project/ai-edge-litert/2.1.3/)):

```
torch>=2.4          # training
onnx>=1.16
onnx2tf==2.5.0      # converter (flatbuffer_direct backend)
ai-edge-litert==2.1.3   # host-side interpreter for bit-exact int8 validation
numpy, opencv-python    # calib/eval tooling
```

**Step 1 — export ONNX** (static shapes; NCHW is fine, onnx2tf transposes):

```python
model.eval()
torch.onnx.export(model, torch.zeros(1, 1, 96, 96), "sectornet8.onnx",
                  input_names=["img"], output_names=["logits"],
                  opset_version=17, dynamo=False)   # classic exporter is fine at this size
```

**Step 2 — build the calibration tensor.** Sample 500–2000 records across *all* recording sessions and lighting conditions (LiteRT docs say ~100–500 suffice; TinyNav used its full training set and kept >99.7 % of float accuracy — verified in doc 07). Save as NHWC float32 in the network's input scaling:

```python
np.save("calib.npy", frames[:, :, :, None].astype(np.float32) / 255.0)  # (N,96,96,1) in [0,1]
```

**Step 3 — convert + quantize (per-channel weights int8, int8 activations, int8 I/O):**

```bash
onnx2tf -i sectornet8.onnx -o tflite_out \
        -oiqt -qt per-channel \
        -cind img calib.npy "[[[[0.0]]]]" "[[[[1.0]]]]" \
        -iqd int8 -oqd int8
# -> tflite_out/sectornet8_full_integer_quant.tflite
```

(The equivalent Keras-path incantation, for path C: `converter.optimizations=[tf.lite.Optimize.DEFAULT]`, `converter.representative_dataset=gen`, `converter.target_spec.supported_ops=[tf.lite.OpsSet.TFLITE_BUILTINS_INT8]`, `converter.inference_input_type = converter.inference_output_type = tf.int8` — [official full-int8 recipe](https://developers.google.com/edge/litert/conversion/tensorflow/quantization/post_training_integer_quant).)

**Step 4 — validate int8 on the host before flashing** (`ai-edge-litert` interpreter): run the full validation set through float and int8 models; require sector-accuracy delta <1 % absolute and near-bin (<0.5 m) recall delta <0.5 %; also assert every tensor in the flatbuffer is int8/int32 (no lingering float ops) and input/output dtypes are int8. If a layer quantizes badly, first try more/better calibration data, then selective int16 activations for that layer — the [ESP-DL MobileNetV2 tutorial's mixed-precision recovery](https://docs.espressif.com/projects/esp-dl/en/latest/tutorials/how_to_quantize_model.html) pattern applies conceptually to TFLM too (`inference_input_type` stays int8; TFLM supports int16 activations for a subset of ops — measure before adopting).

**Step 5 — embed as C array** (the [official LiteRT-Micro step](https://developers.google.com/edge/litert/microcontrollers/build_convert)):

```bash
xxd -i tflite_out/sectornet8_full_integer_quant.tflite > src/model_data.cc
sed -i 's/unsigned char/alignas(16) const unsigned char/' src/model_data.cc
sed -i 's/unsigned int/const unsigned int/' src/model_data.cc
```

`const` keeps the blob in flash (rodata, mmapped); `alignas(16)` satisfies flatbuffer alignment. Prepend a generated header comment with: git hash of the training run, dataset/session manifest hash, calibration-set hash, and the **model version byte** that firmware reports in telemetry (doc 07 §4.4). The firmware side instantiates `MicroMutableOpResolver<N>` with exactly the §1.5 op list and dequantizes outputs with `output->params.scale/zero_point`.

**Step 6 — on-target microbenchmark:** time 100× `invoke()` (radio on) and log `arena_used_bytes()`; these two numbers go into the fork-decision table (§2.3) and gate any ESP-DL optimization detour.

### 3.3 If PTQ accuracy is not enough

Order of escalation (stop at the first that passes §2.3 deltas): (1) larger/richer calibration set; (2) retrain with quant-friendly hygiene (ReLU6 everywhere, no per-channel scales >2^7 spread — check with a flatbuffer inspector); (3) int16 activations for offending layers; (4) QAT — in Keras via `tensorflow-model-optimization` if on path C; on the PyTorch path PT2E-QAT→litert-torch is explicitly untested by the maintainers (issue #150), so QAT effectively forces path C for that model. At 20–40 k params and 4-bin targets, TinyNav-class evidence (<0.3 % PTQ loss, verified in doc 07) says we are unlikely to get past step 1.

---

## 4. Camera + Data Capture on the XIAO ESP32S3 Sense

### 4.1 96×96 grayscale: capture direct, no downscale pass

- `FRAMESIZE_96X96` and `PIXFORMAT_GRAYSCALE` are both first-class in the esp32-camera driver ([sensor.h enums](https://github.com/espressif/esp32-camera/blob/master/driver/include/sensor.h)); the OV2640 path scales on-sensor/in-driver. Field evidence: 96×96 **works in GRAYSCALE where it fails in JPEG** ("this problem only exist if pixformat is set to PIXFORMAT_JPEG, I have tried PIXFORMAT_GRAYSCALE without any problem" — [esp32-camera #436](https://github.com/espressif/esp32-camera/issues/436)). **Verdict: capture grayscale 96×96 directly** (9,216 B/frame); no QQVGA+CPU-downscale pass, no JPEG decode. Fork B additionally 2×2-box-downsamples 96×96→48×48 in ~0.1 ms during the int8 normalize copy.
- **Known S3 + grayscale pitfall:** `cam_hal: EV-VSYNC-OVF` / "Failed to get the frame on time" on ESP32-S3 in non-JPEG modes at `xclk_freq_hz = 20 MHz`; the documented fix is **dropping XCLK to 10 MHz** ([esp32-camera #612](https://github.com/espressif/esp32-camera/issues/612)). Bring-up procedure: start at 20 MHz (higher sensor frame rate), fall back to 16/10 MHz at the first overflow error. At 96×96 even 10 MHz XCLK keeps the sensor far above our 10 Hz floor.
- **No runtime mode switching in non-JPEG modes:** the driver cannot change frame size/pixel format on the fly outside JPEG ([#612 maintainer comment](https://github.com/espressif/esp32-camera/issues/612), [#514](https://github.com/espressif/esp32-camera/issues/514)). Doc 07 §4.2's optional "QVGA JPEG archive every 500 ms" therefore requires a full `esp_camera_deinit()/esp_camera_init()` cycle (~300 ms class) — demote it to a *stationary-robot* option or drop it; the 96×96 stream is the dataset.

```c
camera_config_t cfg = {
  // XIAO ESP32S3 Sense pin map (Seeed wiki): XCLK 10, SIOD 40, SIOC 39,
  // Y9..Y2 = 48,11,12,14,16,18,17,15, VSYNC 38, HREF 47, PCLK 13, PWDN/RESET -1
  .xclk_freq_hz = 20000000,            // drop to 10 MHz on EV-VSYNC-OVF (#612)
  .pixel_format = PIXFORMAT_GRAYSCALE,
  .frame_size   = FRAMESIZE_96X96,
  .fb_count     = 2,                   // double-buffer; fb_count=1 costs 25-35 % fps
  .fb_location  = CAMERA_FB_IN_PSRAM,
  .grab_mode    = CAMERA_GRAB_LATEST,  // always newest frame, never stale
};
```

The camera SCCB bus (GPIO39/40) is separate from the user I²C (D4/D5 = GPIO5/6), so the **VL53L5CX teacher does not share a bus with the camera** ([XIAO pin-multiplexing wiki](https://wiki.seeedstudio.com/xiao_esp32s3_pin_multiplexing/)).

### 4.2 Achievable rates with the radio on

Measured anchors on this exact board ([independent review, Arduino core 3.1.1, fb_count=2](https://quickfixsurrey.ca/seeed-xiao-esp32-s3-sense-review-2/)): JPEG HQVGA (240×176) = **36.4 fps while Wi-Fi-streaming / 28.1 fps while SD-writing**; QVGA = 27.9/21.7. Grayscale 96×96 is a strictly smaller DMA/PSRAM load than HQVGA-JPEG and skips the encoder, so the ~30 fps class is the floor expectation at 20 MHz XCLK (roughly halved at 10 MHz XCLK). ESP-NOW pose/telemetry traffic (≤32 B at 10–25 Hz) is negligible next to the MJPEG streams these figures already tolerate (doc 07 §5.3, [esp32-camera #499](https://github.com/espressif/esp32-camera/issues/499), verified in doc 07). **Capture will not be the bottleneck; the 10 Hz logging/inference spec has ≥2× margin end-to-end.**

### 4.3 SD logging on the Sense expansion board

**Wiring facts** ([Seeed filesystem wiki](https://wiki.seeedstudio.com/xiao_esp32s3_sense_filesystem/)): the microSD slot is **SPI-mode only** — CS = GPIO21 (internal), SCK/MISO/MOSI = **GPIO7/8/9, which are the XIAO's only exposed SPI pins (D8/D9/D10)**. FAT32 ≤32 GB (no exFAT in the stock SD lib). Init: `SD.begin(21)`.

**Integration consequence (new finding, affects doc 07 §2.5's pin budget):** the SD card and any external SPI peripheral — i.e. the **nRF24** in the project's standard radio option — share one physical SPI bus. Seeed's J3 solder-pad exists precisely because the SD's pull-ups load the bus, and community reports show SD + second-SPI-device setups failing until clocks are lowered and CS discipline is enforced ([Seeed forum thread](https://forum.seeedstudio.com/t/xiao-esp32s3-sense-error-while-using-the-sd-card-and-tft-display-simultaneously/295113)). Recommendation for the camera-perception build: **use ESP-NOW as the swarm radio on this board and keep the SPI bus exclusively for the SD card** (the ToF teacher is I²C). If nRF24 is non-negotiable, both devices must live on the shared bus with separate CS, SD at ≤20 MHz, nRF24 at ≤10 MHz, strict mutex around transactions — flyable, but it buys contention during every log write; treat it as a fallback topology.

**Throughput verdict:** measured on ESP32-S3 SPI@20 MHz ([atomic14 benchmarks](https://github.com/atomic14/esp32-sdcard-msc)): Arduino `SD` single-sector writes **0.28 MB/s**; IDF multi-sector **0.48 MB/s**; multi-sector via background writer task **0.92 MB/s**. Our §4.4 record is 9,439 B → 10 Hz = **~94 KB/s ≈ 3× under the worst-case path and ~10× under the buffered path**. Verdict: **10 Hz frame+ToF+state logging is comfortably feasible**; implement as a ring of 4–8 records in PSRAM flushed by a low-priority logger task in ≥32 KB sequential chunks (guards against the 100–200 ms SD-card internal-GC stalls that single-record writes expose), on core 0, never blocking capture/inference on core 1 (doc 07 §5.2 task map).

### 4.4 Logging record format (v2 — supersedes doc 07 §4.2 sketch)

Binary, little-endian, fixed 9,439-byte records appended to one file per session (`/sess_<bootcount>_<modelver>.bin`); a 64-byte session header (magic, format version, firmware git hash, model version byte, platform tag string, sensor config) precedes records.

```c
typedef struct __attribute__((packed)) {
  uint32_t magic;            // 0x53454331 'SEC1'
  uint16_t seq;              // wraps; detects dropped records
  uint32_t t_cap_ms;         // esp_timer at frame capture (VSYNC), shared clock
  uint32_t t_tof_ms;         // timestamp of the paired ToF frame (reject skew > 40 ms)
  uint8_t  img[96*96];       // grayscale, exactly the inference input
  uint16_t tof_mm[64];       // VL53L5CX 8x8 distances, row-major
  uint8_t  tof_status[64];   // per-zone validity (5/9 = valid, per ST docs)
  int16_t  v_mm_s;           // odometry forward speed at capture
  int16_t  yaw_mrad_s;       // gyro yaw rate at capture
  uint8_t  aec_value_hi, aec_value_lo; // sensor exposure (for §5 lighting analysis)
  uint8_t  agc_gain;         // sensor gain
  uint8_t  model_ver;        // model running during collection (0 = none)
  uint8_t  pred_bins[8];     // the live model's sector bins (0-3, 0xFF = n/a) — makes
                             // every drive a shadow-mode evaluation for free
  uint8_t  crc8;             // header+tail integrity (img excluded, cheap)
} log_record_t;               // 9,439 B; 10 Hz ≈ 94 KB/s
```

ToF pairing: VL53L5CX free-runs at 15 Hz (doc 02/07); each camera frame takes the latest completed ToF frame; records with skew >40 ms or >2 fully-invalid sector columns are kept on disk but flagged at label-derivation time (drop at train time, doc 07 §4.2). Labels (sector = min over valid zones of column, then bin) are derived **offline on the host** — raw zones on disk, so bin edges can be re-tuned without re-driving.

---

## 5. Training Recipe

### 5.1 Dataset targets (sanity-checked against doc 07 §4.3 — they hold)

| Purpose | Target | Sanity check |
|---|---|---|
| Base model | **20–30 k samples**, ≥5 environments, ≥3 lighting conditions, both robot speeds | At 10 Hz this is 33–50 min of driving. Consistent with the reference deployment's ~7.4 k/environment total ([arXiv:2512.00086](https://arxiv.org/html/2512.00086), verified in doc 07) × ~5 environments, and with PULP-Frontnet-class ~30 k (doc 05 §3.1). At 21–40 k params the models are ~1 param/sample — regularization headroom, not data starvation |
| Per-environment fine-tune | **~3 k samples** (5–10 min) | Exactly the published fine-tune size that recovered RMSE 4.9→0.6 m; their ablation shows 2.3–4.5 k loses ≤2.5 % vs full-set fine-tuning (same source, verified in doc 07) |
| Held-out eval | 1 full session (~2 k) per environment, **never** touched by training or calibration | Split by session (below) |

Class balance warning: naive driving yields >80 % "far/free" bins. Enforce during collection (deliberately approach walls/obstacles in every session) and during training (per-bin loss reweighting by inverse frequency, capped at 4×; report per-bin metrics, never the blended accuracy alone).

### 5.2 Augmentation (domain robustness)

- **Photometric** (from the reference paper's hyperparameters, verified in doc 07): random gamma 0.8–1.2, brightness 0.5–2.0, per-image gain jitter; plus additive Gaussian + salt noise matched to high-`agc_gain` frames (the logged gain field tells us the real noise distribution to match).
- **Horizontal flip** with **sector-order mirroring** (labels reverse; fork B flips the map) — doubles data for free.
- **Motion/vibration blur**: convolve with random linear kernels (length 1–6 px, random angle ±30° of horizontal) plus small random row-shear to imitate rolling-shutter jello. This is our addition, motivated by doc 07 §7.1's vibration risk; it must never fully substitute for *collecting real driving data at speed*, which stays the primary defense.
- **Not** used: geometric crops/rotations beyond flip (they desynchronize image sectors from ToF columns), cutout over more than one sector width (masks the label's evidence).

### 5.3 Splits and metrics

- **Split by recording session, never by shuffled frame** (adjacent frames are near-duplicates; doc 07 §4.4). Keep whole environments out for the domain-shift eval.
- **Metrics** (both forks, evaluated per sector, radio-on data): (1) bin-exact accuracy; (2) **off-by-one-bin rate** (should absorb almost all errors) and off-by-≥2 rate (safety-relevant confusions, target <1 %); (3) **`<0.5 m` bin recall** (target ≥95 %, doc 07 §7.2) and its precision (false near-alarms herd the robot); (4) **min-distance MAE** in meters, computed from bin midpoints (fork A) or the calibrated dense map (fork B) against the ToF column minimum — the one scalar comparable across forks and to the 0.6 m RMSE literature anchor; (5) per-bin confusion matrices sliced by `agc_gain` tercile (lighting) and by `|yaw_rate|` (motion blur exposure).
- **Training config (fork A):** Adam 3e-3 cosine→3e-5, batch 64, ~60 epochs base / ~120 epochs fine-tune with early stopping on held-out session loss (fine-tune schedule per the reference paper, verified in doc 07); label smoothing per §2.1; minutes per run on any modern GPU at this size. Fork B: same optimizer, berHu+smoothness loss (§2.2).

### 5.4 Per-environment fine-tune + update workflow (tightened from doc 07 §4.5)

1. Drive 5–10 min with logging on (the ToF runs anyway as reflex sensor); `pred_bins` in every record gives the *current* model's shadow-mode score on the way in.
2. Pull the session (SD swap, or Wi-Fi pull of the session file when docked).
3. Host: derive labels → fine-tune from the base checkpoint (~120 epochs, early stop) → PTQ with a calibration set drawn from **base + new session** (never new-only — guards against calibration drift) → host int8 validation gates (§3.2 step 4).
4. Flash: model version byte increments. Two supported mechanisms, in order of preference: (a) **full-firmware OTA** with the model compiled in (C array) — simplest, uses the project's existing OTA partition scheme (doc 04/07), model+firmware always consistent; (b) **model-only update**: `.tflite` file on SD/LittleFS loaded at boot into a PSRAM buffer and passed to `tflite::GetModel()` — enables model swaps without reflash, at the cost of a version-compatibility check (op set!) in firmware. Start with (a); add (b) only if iteration cadence demands it.
5. Post-update, the rolling camera-vs-ToF sector-agreement telemetry (doc 07 §4.5) is the canary: an update that drops agreement >5 % absolute against its predecessor within one drive auto-flags for rollback.

---

## 6. Consolidated Decisions

| Question | Decision | Basis |
|---|---|---|
| Runtime | **Bundled esp-tflite-micro 1.3.7 + ESP-NN 1.2.3** via pioarduino `55.03.311` (Arduino 3.3.11 / IDF 5.5.5), zero extra libs | §1.1–1.4, compile-verified |
| int8 conv/dwconv on ESP-NN S3 fast path? | **Yes** — S3 assembly kernels compiled into the bundle (`CONFIG_NN_OPTIMIZED=y`); conv 5.5–14×, dwconv 4.5–6.3× measured | §1.5 |
| Arena | Internal SRAM, `alignas(16)` static; budget 96 KB (A) / 64 KB (B), then shrink to `arena_used_bytes()`+8 KB | §1.6 |
| Fork A spec | 96×96×1 → ds-conv stack → 8×4 softmax; **1.01 MMAC / 21.7 k params / ~30–40 KB int8 / est. 8–15 ms (≤25 ms radio-on)**; CE + adjacent-bin label smoothing | §2.1 |
| Fork B spec | µPyD-Net-lite (Peluso/Poggi lineage): 48×48 in, 2-decoder, 12×12 inverse-depth out; **2.48 MMAC / 37.1 k params / ~50 KB int8 / est. 20–35 ms**; min-pool→8 sectors + rolling ToF scale calibration | §2.2 |
| PyTorch→tflite | **onnx2tf 2.5.0 `-oiqt`** (per-channel weights, int8 activations + I/O, `-cind` calibration); litert-torch tracked but not load-bearing; Keras converter as fallback; validate with ai-edge-litert 2.1.3; embed with `xxd -i` + `alignas(16) const` | §3 |
| Camera | **Direct `FRAMESIZE_96X96` + `PIXFORMAT_GRAYSCALE`** on OV2640, fb_count=2, PSRAM, GRAB_LATEST; XCLK 20→10 MHz fallback; no runtime mode switching | §4.1–4.2 |
| SD | **Feasible with ~3–10× margin at 10 Hz** (94 KB/s vs 0.28–0.92 MB/s measured); buffered multi-record writes; **SPI bus is shared with the only exposed SPI header → prefer ESP-NOW over nRF24 on this board** | §4.3 |
| Data/training | 20–30 k base / 3 k fine-tune confirmed; session-based splits; per-bin + off-by-one + near-recall + min-distance-MAE metrics; fine-tune→PTQ→OTA loop with shadow-mode logging and agreement canary | §5 |

---

## 7. Open Items for the Firmware/Training Agents

1. Measure real `invoke()` latency + `arena_used_bytes()` for both forks on the XIAO (the §2 numbers are MAC-scaled estimates; §3.2 step 6).
2. Confirm 96×96 grayscale fps at XCLK 20 vs 10 MHz on the actual OV2640 batch (newer Sense units ship OV3660 — same driver path, re-verify).
3. Validate the ReLU6-for-leaky substitution in fork B against a float leaky baseline before committing the quantized config.
4. Decide ESP-NOW vs shared-bus nRF24 per §4.3 at the system level (affects doc 06/08 radio architecture for the camera build only).
5. Wire `pred_bins` shadow-mode logging and the sector-agreement canary into telemetry from day one — they are the cheapest evaluation instruments this project will ever get.

---

## 8. References

**Runtime & acceleration**
- esp-tflite-micro (README, S3 54 ms / 42× ESP-NN benchmark table) — https://github.com/espressif/esp-tflite-micro ; registry versions (1.3.7, 2026-06-03) — https://components.espressif.com/components/espressif/esp-tflite-micro
- ESP-NN kernel library (S3 assembly opt ratios, person-detect 54/47 ms, MobileNetV3 1434 ms) — https://github.com/espressif/esp-nn
- esp32-arduino-lib-builder component manifest (bundles esp-tflite-micro ≥1.2.0, esp32-camera; libs release idf-release_v5.5) — https://github.com/espressif/esp32-arduino-lib-builder/blob/master/main/idf_component.yml ; https://github.com/espressif/esp32-arduino-lib-builder/releases
- pioarduino platform (stable releases = Arduino 3.3.x / IDF 5.5.x; tag 55.03.311) — https://github.com/pioarduino/platform-espressif32 ; https://github.com/pioarduino/platform-espressif32/releases
- tanakamasayuki/Arduino_TensorFlowLite_ESP32 (v1.0.0 2022; 2026-07-03 deprecation notice recommending the bundled official runtime) — https://github.com/tanakamasayuki/Arduino_TensorFlowLite_ESP32 ; Arduino library index entry — https://docs.arduino.cc/libraries/tensorflowlite_esp32/
- EloquentTinyML (last push 2024-07; requires tflm_esp32) — https://github.com/eloquentarduino/EloquentTinyML ; tflm_esp32 precompiled runtime v2.0.0 — https://github.com/eloquentarduino/tflm_esp32 ; person-detection tutorial ("4-5 seconds per frame") — https://eloquentarduino.com/posts/esp32-cam-person-detection
- ESP-DL v3.3.8 (news timeline: P4-only per-channel 2026-04, AutoQuant 2026-05, esp-ppq) — https://github.com/espressif/esp-dl ; registry — https://components.espressif.com/components/espressif/esp-dl ; quantization spec (S3 per-tensor, power-of-two, PTQ/QAT, espdl_quantize_torch) — https://docs.espressif.com/projects/esp-dl/en/latest/tutorials/how_to_quantize_model.html
- person_detection example (arena 100 KB + 60 KB scratch, SPIRAM alloc pattern) — https://github.com/espressif/esp-tflite-micro/blob/master/examples/person_detection/main/main_functions.cc

**Models**
- µPyD-Net journal version: Peluso et al., "Monocular Depth Perception on Microcontrollers for Edge Applications", IEEE TCSVT 32(3), 2022 (architecture: six 3×3 convs 8-8-16-16-32-32, leaky-ReLU 0.125, 3 decoders, ~100 k params, 48×48/32×32) — https://cris.unibo.it/bitstream/11585/819845/5/main_iris.pdf (doi 10.1109/TCSVT.2021.3077395)
- µPyD-Net conference version: Peluso et al., CVPRW 2020 (0.1 M params vs PyD-Net 1.9 M; KITTI numbers) — https://openaccess.thecvf.com/content_CVPRW_2020/papers/w28/Peluso_Enabling_Monocular_Depth_Perception_at_the_Very_Edge_CVPRW_2020_paper.pdf
- Poggi publications index (PyD-Net IROS 2018 lineage) — https://mattpoggi.github.io/publications/
- Verified in doc 07 and reused: VL53L5CX-teacher on-device-learning paper (3 k fine-tune, RMSE 4.9→0.6 m, photometric aug, 120-epoch schedule) — https://arxiv.org/html/2512.00086 ; TinyNav (23 k params, 30 ms, full-train-set calibration, <0.3 % PTQ loss) — https://arxiv.org/abs/2603.11071 ; µPyD-Net unaccelerated S3 port (~6 s) — https://github.com/JPsparks/APAI_ESP_MonocularDepthEstimation

**Quantization toolchain**
- onnx2tf v2.5.0 (`-oiqt`/`-qt`/`-cind`/`-iqd`/`-oqd`; maintenance statement) — https://github.com/PINTO0309/onnx2tf ; https://pypi.org/project/onnx2tf/2.5.0/ ; v2.4.0 flatbuffer_direct default — https://github.com/PINTO0309/onnx2tf/releases/tag/2.4.0
- litert-torch (ex ai-edge-torch) v0.8.0, converter Beta; PT2E quantization docs — https://github.com/google-ai-edge/ai-edge-torch ; https://github.com/google-ai-edge/ai-edge-torch/blob/main/docs/pytorch_converter/README.md ; static-int8 friction + QAT-unsupported maintainer comments — https://github.com/google-ai-edge/ai-edge-torch/issues/150 ; third-party conversion/quantization reports — https://tech.ailia.ai/en/quantization-with-ai-edge-torch-1efe17b93cd7/ ; https://medium.com/axinc-ai/convert-models-from-pytorch-to-tflite-with-ai-edge-torch-0e85623f8d56
- LiteRT full-int8 PTQ recipes (representative dataset, TFLITE_BUILTINS_INT8, int8 I/O) — https://developers.google.com/edge/litert/conversion/tensorflow/quantization/post_training_quantization ; https://developers.google.com/edge/litert/conversion/tensorflow/quantization/post_training_integer_quant
- LiteRT-Micro C-array embedding (`xxd -i`, const) — https://developers.google.com/edge/litert/microcontrollers/build_convert
- ai-edge-quantizer (post-hoc recipes incl. STATIC_WI8_AI8) — https://github.com/google-ai-edge/ai-edge-quantizer
- ai-edge-litert host interpreter (2.1.3 PyPI; Python 3.9–3.12, Linux/macOS) — https://pypi.org/project/ai-edge-litert/2.1.3/ ; LiteRT repo (v2.1.6) — https://github.com/google-ai-edge/litert

**Camera & storage**
- esp32-camera driver enums (FRAMESIZE_96X96, PIXFORMAT_GRAYSCALE) — https://github.com/espressif/esp32-camera/blob/master/driver/include/sensor.h ; registry (2.1.7, 2026-06-05) — https://components.espressif.com/components/espressif/esp32-camera
- 96×96 works in grayscale / fails in JPEG — https://github.com/espressif/esp32-camera/issues/436 ; S3 grayscale EV-VSYNC-OVF → XCLK 10 MHz fix; no non-JPEG runtime mode switching — https://github.com/espressif/esp32-camera/issues/612 ; framesize-change buffer hazard — https://github.com/espressif/esp32-camera/issues/514
- XIAO ESP32S3 Sense microSD (SPI mode, CS=21, SCK/MISO/MOSI=GPIO7/8/9 shared with header SPI, J3 pads, FAT32 ≤32 GB) — https://wiki.seeedstudio.com/xiao_esp32s3_sense_filesystem/ ; full pin multiplexing table (camera pins, PDM mic, SCCB on 39/40) — https://wiki.seeedstudio.com/xiao_esp32s3_pin_multiplexing/ ; SD + second SPI device conflict reports — https://forum.seeedstudio.com/t/xiao-esp32s3-sense-error-while-using-the-sd-card-and-tft-display-simultaneously/295113
- ESP32-S3 SD throughput benchmarks (SPI 20 MHz: 0.28/0.48/0.92 MB/s by write strategy; SDIO 2.34 MB/s) — https://github.com/atomic14/esp32-sdcard-msc
- XIAO Sense measured fps under Wi-Fi stream vs SD write (HQVGA 36.4/28.1, QVGA 27.9/21.7; fb_count=1 penalty 25–35 %; OPI-PSRAM setting requirement) — https://quickfixsurrey.ca/seeed-xiao-esp32-s3-sense-review-2/
- Verified in doc 07 and reused: radio/PSRAM contention on camera pipelines — https://github.com/espressif/esp32-camera/issues/499 ; SRAM-vs-PSRAM arena placement penalties — https://zediot.com/blog/esp32-s3-tinyml-memory-quantization-realtime-inference/
