# 07 — Camera + ML Depth-Estimation Version: Design Spec

**Scope:** Concrete, buildable design for the **camera + ML depth-estimation variant** of the obstacle-avoidance stack — one of several parallel perception versions the project is building. This document specifies hardware, model, training pipeline, firmware integration, and exit criteria. It does **not** relitigate whether ML depth should be the baseline: [05-ml-depth-estimation.md](05-ml-depth-estimation.md) already concluded it should not, and that verdict stands. This version exists as the **experimental track** foreseen in doc 05 §6, built exactly the way the field itself does it — **with the VL53L5CX ToF as the ground-truth teacher** and the learned output kept in an **advisory role** under the shared VFH+ / speed-governor / stop-reflex architecture of [02-obstacle-avoidance.md](02-obstacle-avoidance.md) §7.

**Status:** Design spec (research phase). All cited URLs were retrieved and verified during research (July 2026), except where marked "verified in doc 02/05".

---

## 1. Overview — relationship to doc 05

Doc 05's verdict was: *don't replace the ToF with a camera network — the sensor you already own is what the ML community uses to fix ML depth.* This version embraces that irony instead of fighting it:

- **The VL53L5CX is the teacher, not the competitor.** The flagship 2025 paper on MCU monocular depth ([Nadalini et al., arXiv:2512.00086](https://arxiv.org/html/2512.00086)) pairs a monocular camera with a VL53L5CX exactly as we will: the ToF produces 8×8 pseudo-labels while driving, the network learns from them, and RMSE in a new environment drops from 4.9 m to 0.6 m after fine-tuning on just **3 k self-labeled samples**. We copy this recipe, with the training loop on a laptop instead of on-device (their on-device training needs a GAP9's FP16 SIMD; the ESP32-S3 has no such capability).
- **The output contract is the shared perception interface.** Every perception version (ToF, lidar, camera-ML) must emit the same thing: a K-sector polar distance estimate at some rate, feeding the VFH+ steering layer and the speed governor of doc 02 §7.2. The camera version is *advisory*: the stop-reflex layer is never driven by the network. When the ToF is co-mounted (recommended), the reflex stays on the ToF; when the camera flies solo, a conservative hard speed cap substitutes.
- **What this version can uniquely learn to do later:** see beyond the ToF's 4 m ceiling, and (phase 2) put a "peer robot in sector k" bit into the same interface — the semantic tasks doc 05 §6 listed as the only legitimate reasons to revisit vision ML.

Success = the network's sector estimates agree with the ToF teacher well enough, fast enough, that the advisory layer demonstrably helps (or at least does not hurt) — measured criteria in §7.

---

## 2. Hardware Selection

### 2.1 Why the classic ESP32-CAM (AI-Thinker) is only the budget fallback

The ~$7.50 AI-Thinker ESP32-CAM is the cheapest camera board in existence, but it is the wrong chip for this version: its Xtensa **LX6 cores have no vector instructions**, so int8 CNN inference runs ~7× slower than on an ESP32-S3 (Espressif's own benchmark: 96×96 int8 person detection takes **380 ms on classic ESP32 vs 54 ms on ESP32-S3** with ESP-NN kernels — [esp-tflite-micro readme](https://components.espressif.com/components/espressif/esp-tflite-micro/versions/1.3.5/readme), verified in doc 05). It also has only **4 MB of slower QPI PSRAM** (vs 8 MB octal on the S3 boards), no native USB (needs an FTDI adapter to program — [Maker Advisor board comparison](https://makeradvisor.com/esp32-camera-cam-boards-review-comparison/)), and measured QVGA streaming of ~12 FPS vs ~22–25 FPS on S3 boards ([independent XIAO review with side-by-side table](https://quickfixsurrey.ca/seeed-xiao-esp32-s3-sense-review/)). Keep one on the shelf for driver bring-up; do not build this version on it.

### 2.2 ESP32-S3 camera board comparison

| Board | Price | MCU / PSRAM | Camera | Extras | Fit for this version |
|---|---|---|---|---|---|
| **Seeed XIAO ESP32S3 Sense** | **$13.99** ([Seeed store](https://www.seeedstudio.com/XIAO-ESP32S3-Sense-p-5639.html)) | ESP32-S3R8, **8 MB octal (OPI) PSRAM**, 8 MB flash | Detachable **OV2640** (newer units ship OV3660); official **OV5640 upgrade $11.99**; 24-pin DVP connector | PDM mic, microSD slot, native USB-C, LiPo charging, 21×17.5 mm, 11 GPIOs ([Seeed wiki](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/), [Makerguides spec table](https://www.makerguides.com/getting-started-with-xiao-esp32-s3-sense/)) | **Primary pick.** Smallest/lightest (drone-compatible), octal PSRAM, camera detachable/upgradable, SD slot for data logging. GPIO budget is tight (see §2.4) |
| **Freenove ESP32-S3-WROOM CAM (FNK0085)** | **$19.99** ([Freenove store](https://store.freenove.com/products/fnk0085)) | ESP32-S3-WROOM, **8 MB octal PSRAM**, 8/16 MB flash | OV2640 66.5° (module socket also takes 120°/160° wide-FoV OV2640 variants — [Prusa firmware board doc](https://github.com/prusa3d/Prusa-Firmware-ESP32-Cam/blob/master/doc/Freenove%20ESP32-S3-Wroom/README.md)) | microSD, USB-C, WS2812 LED, **18 broken-out GPIOs** ([Component Advisor comparison](https://componentadvisor.com/best-esp32-camera-boards-for-beginners/)) | **Ground-vehicle alternate.** Same silicon as the XIAO with a friendlier GPIO budget for a full robot build (nRF24 SPI + I²C + PWM). Bigger (43×21 mm) |
| **Espressif ESP32-S3-EYE** | $35–47.50 ([espboards.dev](https://www.espboards.dev/esp32/esp32-s3-eye/), [Adafruit](https://www.adafruit.com/product/5955)) | ESP32-S3-WROOM-1 (S3R8), **8 MB octal PSRAM**, 8 MB flash | OV2640, 66.5° FoV | 240×240 LCD, mic, IMU, microSD; reference board for ESP-WHO ([getting-started guide](https://github.com/espressif/esp-who/blob/master/docs/en/get-started/ESP32-S3-EYE_Getting_Started_Guide.md)) | Best-documented dev/benchmark platform, but 2–3× the price for an LCD a robot doesn't need. Buy at most one, as the model-bring-up bench board |
| **Generic "ESP32-S3-CAM" (AliExpress class)** | ~$10–15 ([Alibaba S3-CAM buying guide](https://smartbuy.alibaba.com/buyingguides/esp32-s3-cam)) | Varies — **check the module code**: N8R8 = 8 MB octal PSRAM, N8R2 = 2 MB quad | OV2640 | Varies, thin documentation | Acceptable only if confirmed R8 (octal). Quad-PSRAM (R2) boards measurably break camera DMA at higher clocks (§2.3) |
| **AI-Thinker ESP32-CAM** (fallback) | ~$7.50 ([Maker Advisor](https://makeradvisor.com/esp32-camera-cam-boards-review-comparison/)) | Classic ESP32 LX6 — **no vector ISA**, 4 MB QPI PSRAM | OV2640 | No USB, GPIO conflicts | Budget fallback only; ~7× slower inference (§2.1) |

### 2.3 PSRAM: octal matters, and why

Two independent reasons to insist on **R8 (8 MB octal/OPI) PSRAM** variants:

1. **Camera DMA integrity.** On the S3, higher camera clocks push frames to PSRAM via EDMA; an Espressif maintainer states directly that this "is not working all that great, especially on QSPI PSRAM (R2 models). R8 models with OPI PSRAM do better" — the corruption shows up as missing bytes/flicker ([esp32-camera issue #499](https://github.com/espressif/esp32-camera/issues/499)).
2. **Inference throughput.** ESP-DL's published kernel benchmarks are measured against octal PSRAM configs, and inference latency is documented to be highly sensitive to PSRAM frequency/bus width settings ([esp-dl operator performance](https://github.com/espressif/esp-dl/blob/master/operator_performance.md), [esp-dl issue #247 on config-dependent 2× slowdowns](https://github.com/espressif/esp-dl/issues/247)). Running a model's hot tensors from PSRAM instead of SRAM can cost 50–80 % of frame rate ([ZedIoT ESP32 edge-AI architecture guide](https://zediot.com/blog/esp32-edge-ai-architecture/)) — the mitigation (§5.2) is arena-in-SRAM, weights-in-flash/PSRAM, frames-in-PSRAM, which the 8 MB octal parts support comfortably.

### 2.4 Camera sensor choice: OV2640 vs OV3660 vs OV5640

| Sensor | Resolution | Rate (driver-level) | Low light | Price | Notes |
|---|---|---|---|---|---|
| **OV2640** | 2 MP (1600×1200) | 25–30 FPS @ VGA, ~50 FPS @ QVGA | Average (~1.5 lux @ F2.0 — [Alibaba OV2640 guide](https://smartbuy.alibaba.com/buyingguides/ov2640)) | $2–5 | Default on every board; on-chip JPEG; 66.5° stock FoV, 120°/160° lens variants exist ([Prusa board doc](https://github.com/prusa3d/Prusa-Firmware-ESP32-Cam/blob/master/doc/Freenove%20ESP32-S3-Wroom/README.md)) |
| **OV3660** | 3 MP (2048×1536) | ~60 FPS @ VGA, 45 FPS @ 720p | **Good** | $6–12 | Fastest at a given resolution; newer XIAO Sense units ship it ([Seeed wiki](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/)) |
| **OV5640** | 5 MP (2592×1944) | 90 FPS @ VGA claim, 30 FPS @ 720p | Average | $8–15 | Optional autofocus; official XIAO add-on $11.99 ([Seeed store](https://www.seeedstudio.com/XIAO-ESP32S3-Sense-p-5639.html)) |

Numbers from the [espboards.dev sensor comparison](https://www.espboards.dev/blog/esp32-camera-modules-compared/) and the [esp32-camera driver's supported-sensor table](https://components.espressif.com/components/espressif/esp32-camera/versions/2.0.10). **The network eats 96×96 grayscale (§3); megapixels are irrelevant.** What matters: frame rate at small sizes (all three are fine), low-light gain (OV3660 best), and FoV match with the teacher. The stock 66.5° lens is nearly identical to the VL53L5CX's 63° diagonal FoV — a lucky alignment that makes teacher-student label pairing almost trivial (§4.2). **Decision: use whatever OV2640/OV3660 ships on the board; do not fit wide-FoV lenses in v1** (a 120° lens would see sectors the teacher can't label). The OV5640 upgrade is only worth it if phase-2 peer detection needs the resolution.

### 2.5 Pick

- **Drone + default: Seeed XIAO ESP32S3 Sense ($13.99).** Octal PSRAM, detachable camera, SD slot for dataset logging, LiPo charging, and at 21×17.5 mm the only option that flies. The 11-GPIO budget covers this version's needs: I²C (2 pins — VL53L5CX teacher shares the bus), SPI for nRF24 (4), 2–3 PWM out, with the camera and SD on the B2B connector/internal pins.
- **Ground vehicle: Freenove ESP32-S3-WROOM CAM ($19.99)** when the build needs more pins (encoders, extra sonar, status LEDs).
- One **ESP32-S3-EYE** on the bench as the ESP-WHO/ESP-DL reference platform is a reasonable luxury, not a requirement.

Total per-robot delta over the ToF-only version: **~$14–20** (the VL53L5CX is already in the BOM as teacher + reflex sensor).

---

## 3. Model Selection

### 3.1 The landscape, with numbers

Doc 05 §2 established the dense-depth numbers; the additions here are the sector/end-to-end alternatives:

| Approach | Params / int8 size | Input → output | Platform & measured speed | Source |
|---|---|---|---|---|
| **µPyD-Net** (dense relative depth) | 107 k / ~110 KB | 48×48 RGB → 48×48 depth | 651 ms on STM32F7; the only public ESP32-S3 build reports ~6 s full cycle | [TCSVT 2021 via arXiv:2512.00086](https://arxiv.org/html/2512.00086); [JPsparks S3 deployment](https://github.com/JPsparks/APAI_ESP_MonocularDepthEstimation) (verified in doc 05) |
| **FastDepth** (dense) | 1.35 MB int8, 2.7 MB RAM | 224×224 → depth | needs an NPU (STM32N6) or Jetson; **out of reach on S3** | [ST model card](https://huggingface.co/STMicroelectronics/fastdepth) (verified in doc 05) |
| **PULP-Dronet v3 / Tiny v3** (steering + collision prob.) | 320 kB → **2.9 k params** | 200×200 gray → 2 scalars | 34–139 FPS on GAP8 (8+1 cores — not transferable, but proves tiny nets suffice for avoidance) | [IEEE IoT-J 2024](https://doi.org/10.1109/jiot.2024.3431913) (verified in doc 05) |
| **TinyNav** (end-to-end depth→control) | **23 k params** | 20×(24×24) ToF-depth frames → steer + throttle | **30 ms inference** on an ESP32-P4-class MCU with TFLM+ESP-NN; 40 collision-free laps on trained-like tracks | [arXiv:2603.11071](https://arxiv.org/abs/2603.11071), [code](https://github.com/regularpooria/TinyNav) |
| **Espressif reference points on ESP32-S3** | person-detect ~250 k / MobileNetV2 96×96 | 96×96 int8 | person detection **54 ms** (TFLM+ESP-NN) / **62 ms** (ESP-DL, 13.7 FPS end-to-end); MobileNetV2 **108 ms** (8.1 FPS end-to-end, 1.4 MB peak PSRAM) | [esp-tflite-micro readme](https://components.espressif.com/components/espressif/esp-tflite-micro/versions/1.3.5/readme) (verified in doc 05); [S3-vs-P4 benchmark](https://quickfixsurrey.ca/esp32-s3-vs-esp32-p4-ai-benchmark/) |

Two structural lessons:

1. **Dense depth maps are the wrong output for this MCU.** The avoidance layer consumes K sector distances (doc 02 §3.2); producing 2 304 depth pixels (48×48) at ~1 Hz to then collapse them into 8 numbers is paying ~100× compute for information the consumer throws away. µPyD-Net on the S3 is a ~1 Hz-class pipeline ([JPsparks](https://github.com/JPsparks/APAI_ESP_MonocularDepthEstimation), verified in doc 05) — below the rate at which VFH+ steering is useful at any speed above a crawl.
2. **Sector-classification networks are 10–50× cheaper and match the interface exactly.** PULP-Dronet proves collision avoidance survives distillation to 2.9 k params; TinyNav proves a ~23 k-param net runs in 30 ms with TFLM+ESP-NN on Espressif silicon. Our teacher even *natively emits sector labels*: min-pooling each column of the VL53L5CX's 8×8 frame yields exactly an 8-sector distance vector.

### 3.2 Primary: "SectorNet-8" — K-sector distance-bin classifier

- **Input:** 96×96×1 grayscale, captured directly at `FRAMESIZE_96X96` from the sensor (the driver scales on-chip; no CPU resize). Optionally 2 stacked frames (t, t−100 ms) as channels for motion cues, TinyNav-style — decide after v1 ablation.
- **Output:** 8 sectors × 4 distance bins, softmax per sector. Bins (log-spaced, teacher-limited): **<0.5 m / 0.5–1 m / 1–2 m / >2 m-or-free**. 8 sectors ≙ the VL53L5CX's 8 columns ≙ the VFH+ sector granularity already planned over the 63° FoV.
- **Architecture:** MobileNet-style depthwise-separable stack, 4–5 blocks, width ~16–48 channels, global pooling, one shared dense layer, 8 classification heads. Target **25–50 k params, 5–10 MMAC, int8 flash ≤ 64 KB, tensor arena ≤ 200 KB** (fits internal SRAM — critical, see §5.2). All ops chosen from the ESP-NN/ESP-DL accelerated set (conv2d, dwconv2d, ReLU, pooling — [esp-dl operator performance](https://github.com/espressif/esp-dl/blob/master/operator_performance.md)); no LSTM/GRU/attention, which TFLM+ESP-NN don't accelerate ([TinyNav §II-C](https://arxiv.org/html/2603.11071v1)).
- **Expected latency on ESP32-S3 @ 240 MHz:** the 62 ms ESP-DL person-detect (≈10× our MACs at the same input size) bounds us from above; TinyNav's 30 ms at 23 k params (on a faster P4) from below. **Design estimate: 25–50 ms inference → 15–25 Hz inference-limited, ~10–15 Hz end-to-end** with capture and radio active (§5.4). Budget conservatively: the spec target is **≥ 10 Hz sector output**.

Why classification bins, not regression: (a) the teacher is noisy and 4 m-capped, so fine-grained metric regression learns noise; (b) VFH+ thresholds distances into free/blocked anyway; (c) per-sector softmax confidence is a free input for the governor ("low confidence → treat sector as unknown → slow down"), mirroring the staleness rule of doc 02 §6.

### 3.3 Fallback: µPyD-Net-lineage dense depth, decimated to sectors

If SectorNet-8 fails to learn (e.g. sector labels prove too coarse a signal to converge), fall back to the published-and-working recipe: µPyD-Net 48×48 with the [garagehq/upydnet-espdl](https://github.com/garagehq/upydnet-espdl) int8 ESP-DL build (verified in doc 05 — note their finding that naive per-tensor int8 breaks the model and cross-layer equalization is required), min-pool the 48×48 output into 8 columns, and accept the ~1 Hz-class rate with a correspondingly lower speed cap (§6). This fallback exists to de-risk the *training* question, not the latency question — if even dense supervision can't make a camera useful here, the track parks (§7).

Explicitly rejected: FastDepth-lineage (needs an NPU; [ST model card](https://huggingface.co/STMicroelectronics/fastdepth), verified in doc 05) and end-to-end control output à la TinyNav/PULP-Dronet — learned steering would bypass the shared VFH+/BVC/reflex stack that keeps all perception versions comparable and safe, violating the version contract (§5.1).

---

## 4. Training & Data Pipeline

### 4.1 The trick: ToF as automatic ground-truth teacher

This is the recipe of [arXiv:2512.00086](https://arxiv.org/html/2512.00086) (whose IoT node pairs an OV5647 camera with **the same VL53L5CX we already plan to buy**), adapted from GAP9 on-device learning to laptop-side training:

- Mount the VL53L5CX rigidly ~1–2 cm from the camera lens, boresight-aligned (parallax at that baseline is negligible beyond ~0.3 m; both FoVs ≈ 63–66°, §2.4).
- Drive the robot around (manual RC, or later the ToF-version autopilot) while the firmware logs synchronized (image, ToF frame, state) tuples to microSD. **Zero manual labeling.**
- Every new environment is 5–10 minutes of driving away from a fresh fine-tuning set. This is not optional polish — it is load-bearing: the paper's network was **RMSE 4.9 m wrong** in a new environment before ToF-supervised fine-tuning and 0.6 m after, on only **3 k samples**.

### 4.2 Data logging format

One binary record per sample, appended to a session file on microSD (the XIAO/Freenove SD slot; SD write throughput at this rate is trivial — a full QVGA-JPEG logger on the same board sustains ~20 FPS, [independent XIAO measurement](https://quickfixsurrey.ca/seeed-xiao-esp32-s3-sense-review-2/)):

```
record {
  uint32  magic; uint16 version;
  uint32  t_ms;              // esp_timer, shared clock with control loop
  uint8   img[96*96];        // grayscale, exactly the inference input
  uint16  tof_mm[64];        // VL53L5CX 8x8 distances
  uint8   tof_status[64];    // per-zone validity (drop zones with bad status)
  int16   v_mm_s, yaw_mrad_s;// odometry/IMU state at capture
  uint8   aec_value; uint8 agc_gain;  // exposure metadata (§7 lighting analysis)
}   // ~9.5 KB/record; 10 Hz logging ≈ 95 KB/s
```

Optionally also store a QVGA JPEG every 500 ms as a higher-resolution archive for future models. Log at 10 Hz; the ToF runs 8×8 @ 15 Hz so every record gets a ToF frame ≤ 67 ms old — timestamp both and reject pairs with skew > 40 ms.

**Label derivation (offline):** per sector k = ToF column k, take min over the 8 rows *of zones with valid status*, then bin. Discard samples where > 2 sectors are entirely invalid. Horizontal-flip augmentation mirrors the sector order.

### 4.3 Dataset size

| Purpose | Size | Basis |
|---|---|---|
| Base model (multi-environment) | **20–30 k samples** across ≥ 5 rooms/outdoor spots, varied lighting | IDSIA's field dataset is 3 k train / 7.4 k total for *one* environment ([arXiv:2512.00086](https://arxiv.org/html/2512.00086) §V); PULP-Frontnet-class deployments used ~30 k (doc 05 §3.1); at 10 Hz this is < 1 h of total driving |
| Per-environment fine-tune | **~3 k samples** (5–10 min of driving) | the paper's RMSE 4.9→0.6 m result used exactly 3 k; their KITTI/NYUv2 ablation shows 2.3–4.5 k-sample fine-tunes lose ≤ 2.5 % accuracy vs full-dataset fine-tunes |

### 4.4 Training → quantization → deployment

1. **PyTorch training.** Per-sector cross-entropy (label-smoothed), Adam, batch 32; photometric augmentation copied from the ODL paper (random gamma 0.8–1.2, brightness 0.5–2.0, per-channel color 0.8–1.2, horizontal flip — [arXiv:2512.00086](https://arxiv.org/html/2512.00086) hyperparameter section). Train/val split by *session*, never by shuffled frame (adjacent frames are near-duplicates).
2. **Int8 post-training quantization** with the full training set as the representative/calibration dataset — feasible at this scale and what TinyNav did, preserving > 99.7 % of float accuracy on their heads ([arXiv:2603.11071](https://arxiv.org/html/2603.11071v1) §II-C).
3. **Runtime: TFLite-Micro + ESP-NN first** (PlatformIO/Arduino-compatible, matches the project's current toolchain, and the 54 ms S3 reference number is from this stack). **ESP-DL as the optimization pass** if more speed is needed — with eyes open: on the S3, ESP-DL/ESP-PPQ is **per-tensor-only** quantization (per-channel is P4-only), which measurably hurts (Espressif's own MobileNetV2 walkthrough: 71.9 % float → 60.5 % int8 per-tensor, recovered by selectively promoting bad layers to int16 — [esp-dl deployment tutorial](https://github.com/espressif/esp-dl/blob/a00707d/docs/en/tutorials/how_to_deploy_mobilenetv2.rst)). Check layerwise quantization error before trusting any converted model; the [garagehq µPyD-Net notes](https://github.com/garagehq/upydnet-espdl) (verified in doc 05) show what per-model surgery looks like when it goes wrong.
4. **Deployment:** model blob in flash (OTA-updatable partition), version byte reported in telemetry so logs are always attributable to a model version.

### 4.5 Per-environment fine-tuning / calibration workflow

1. Arrive in new environment → drive 5–10 min with logging on (the ToF is running anyway — it's the reflex sensor).
2. Pull the SD card (or Wi-Fi-sync the session file), fine-tune the base model ~120 epochs with early stopping (the ODL paper's schedule; minutes on any GPU, tens of minutes on CPU at this model size).
3. Re-quantize, re-flash via OTA.
4. **Continuous online validation for free:** whenever camera and ToF both produce sectors, the firmware logs the per-sector bin agreement. This one scalar (rolling agreement %) is the health metric for §7 — the teacher never stops grading the student.

---

## 5. Firmware Integration Design

### 5.1 The shared perception interface (version contract)

Every perception version emits, over a single-slot overwrite queue (`xQueueOverwrite`, per doc 04 §3.4):

```c
typedef struct {
  uint32_t t_capture_ms;      // timestamp of the *sensor frame*, not of publication
  uint16_t sector_dist_mm[8]; // estimated min distance per sector (UINT16_MAX = free/unknown-far)
  uint8_t  confidence[8];     // 0-255; governor treats low-confidence as absent data
  uint8_t  source;            // PERCEPTION_SRC_TOF / _CAMERA_ML / _LIDAR ...
} perception_frame_t;
```

The VFH+ layer, speed governor, and telemetry consume this identically for all versions. **The stop-reflex never consumes `_CAMERA_ML` frames**: it reads the ToF directly when fitted; camera-only builds run under a hard speed cap instead (§6). Camera bins are converted to `sector_dist_mm` as the bin's *lower* edge (conservative).

### 5.2 Task and memory placement (ESP32-S3, per doc 04 architecture)

| Task | Core | Priority | Notes |
|---|---|---|---|
| Control loop (governor, VFH+, reflex, PWM) | 1 | highest | unchanged from doc 04 §7; µs–ms per tick, preempts inference freely |
| **Inference task** | 1 | low | the 25–50 ms `invoke()` runs *under* the control task's priority; being preempted adds µs-scale delays only. Never on core 0 — blocking inference there starves Wi-Fi and causes disconnect/reboot cycles ([ZedIoT edge-AI guide](https://zediot.com/blog/esp32-edge-ai-architecture/)) |
| Camera capture (driver task + DMA) | 0 | mid | `esp_camera` with `fb_count=2`, `CAMERA_FB_IN_PSRAM`, `grab_mode=CAMERA_GRAB_LATEST` — double-buffered DMA into PSRAM, and the app always gets the newest frame, never a stale queued one ([esp32-camera driver docs](https://components.espressif.com/components/espressif/esp32-camera/versions/2.0.10); config pattern as in the [XIAO measurement article](https://quickfixsurrey.ca/seeed-xiao-esp32-s3-sense-review-2/)). Single-buffer costs 25–35 % frame rate |
| ESP-NOW / nRF24 / telemetry | 0 | below Wi-Fi task | unchanged from doc 04 |

Memory placement (the difference between 10 Hz and 3 Hz): **tensor arena in internal SRAM** (≤ 200 KB fits alongside the Wi-Fi stack's ~300 KB take); **weights** in flash (mmap) or PSRAM; **frame buffers** in PSRAM. Running hot tensors from PSRAM costs 50–80 % of frame rate ([ZedIoT](https://zediot.com/blog/esp32-edge-ai-architecture/)); PSRAM is also contended by camera DMA and Wi-Fi buffers, which is why the arena must not live there ([ZedIoT TinyML bottleneck analysis](https://zediot.com/blog/esp32-s3-tinyml-memory-quantization-realtime-inference/)).

Grayscale 96×96 capture (9.2 KB/frame) instead of JPEG/QVGA keeps PSRAM bandwidth pressure minimal and skips JPEG decode entirely — the preprocessing step is one int8 normalization pass (~1 ms).

### 5.3 Pipeline and expected end-to-end latency

```
sensor exposure+readout ─ DMA → PSRAM fb ─ copy+normalize → SRAM arena ─ invoke() ─ sectors → queue → VFH+/governor tick
      ~25–50 ms (frame age, 20–40 FPS at 96×96)        ~1–2 ms            25–50 ms      <1 ms      next 100 Hz tick ≤10 ms
```

**Design number: t_react ≈ 100–150 ms typical, 200 ms worst-case** (add one 20 ms servo period for actuation) — remarkably close to the ToF version's 100–130 ms (doc 02 §6), because the VL53L5CX's 15 Hz frame age dominates there just as capture+inference dominates here. The difference is not latency but *trustworthiness of the distance estimate* — hence the tighter speed envelope in §6.

Wi-Fi/ESP-NOW coexistence: the radio lives on core 0 and mostly contends for PSRAM bandwidth and packet-time, not CPU on core 1. Evidence from streaming workloads suggests expecting a 20–40 % throughput haircut when the radio is busy (camera+Wi-Fi pipelines run at roughly half their radio-off clock rates in practice — [esp32-camera issue #499 measurements](https://github.com/espressif/esp32-camera/issues/499), [ESP32-S3 MJPEG streaming write-up](https://codebobby.com/articles/esp32-s3-jpeg-stream.html)). ESP-NOW pose broadcasts (10–25 Hz, ≤ 32 B) are negligible next to an MJPEG stream, so the **≥ 10 Hz sector-output target already includes radio-on margin**; validate with the §7 benchmark.

### 5.4 Camera settings for a moving robot

The OV2640/OV3660 are **rolling-shutter** sensors that expose line-by-line; under auto-exposure in dim rooms the exposure time stretches toward a full frame time and moving scenes smear ([ArduCAM manual-exposure explainer](https://blog.arducam.com/manual-exposure-ov2640/), [ArduCAM issue confirming blur + the light/exposure trade](https://github.com/ArduCAM/ArduCAM_ESP32S_UNO/issues/15)). Firmware policy: cap `aec_value` (or disable AEC and manage exposure ourselves — `set_exposure_ctrl(s,0)` + `set_aec_value(...)`, the community-standard knobs: [esp32-cam-demo shutter workaround](https://github.com/igrr/esp32-cam-demo/issues/81), [esp32.com settings thread](https://esp32.com/viewtopic.php?t=14376)) so exposure never exceeds ~10 ms at driving speed, log `aec_value`/`agc_gain` per frame (§4.2), and let the governor slow down when gain maxes out (dark = blind = slow — same rule as stale ToF data).

### 5.5 Phase 2 option: the camera doubles as peer-robot detector

The same backbone can grow a second head — per-sector "peer robot present" logits — trained on frames where ESP-NOW pose broadcasts say a teammate was in-FoV (self-labeling again, radio as teacher this time). This slots into doc 05 §6's closing suggestion (sector-level detection as a redundant hint into VFH+/BVC) with zero interface changes: a detection just annotates a sector. The GAP8 nano-drone results (48 Hz, 15 cm error at ≤ 2 m — doc 05 §3.1) remain the upper bound the S3 will not reach; expect a few Hz of coarse sector hints and treat it strictly as advisory. Not in v1 scope.

---

## 6. Performance & Safe-Speed Estimate

Using doc 02 §6's model, `d_needed = v·t_react + v²/(2·a_brake) + margin`, with t_react = 0.2 s (worst-case, §5.3), margin 0.2 m, a_brake = 3 m/s² ground / 2 m/s² air:

| Configuration | Speed | d_needed | Verdict |
|---|---|---|---|
| Camera-ML advisory **+ ToF reflex on board** (recommended) | ground 1–2 m/s / drone 0.5–1 m/s | 0.57 m @ 1 m/s; 1.27 m @ 2 m/s | Same envelope as the ToF version (doc 02 §6) — the reflex layer's guarantees are unchanged; the camera only *adds* advisory sectors |
| **Camera-only** experiment (network is sole range source) | **ground ≤ 1 m/s hard cap; drone ≤ 0.5 m/s** | 0.57 m @ 1 m/s — but the binding constraint is estimate trust, not geometry | The `<0.5 m` and `0.5–1 m` bins must be near-perfect for this to be safe; run only in padded test areas until §7 criteria pass. Fine-tuned dense-depth error in the reference work was still RMSE 0.6 m — assume sector bins carry equivalent uncertainty |
| Fallback model (µPyD-Net-to-sectors, ~1 Hz) | ground ≤ 0.3 m/s | t_react ≈ 1.2 s → 0.6 m of pure reaction distance at 0.3 m/s | Walking-pace demos only |

Expected steady-state performance (to be validated, §7): **10–15 Hz sector output, ~100–150 ms typical latency, ≥ 85 % sector-bin agreement with the ToF after per-environment fine-tuning** — i.e. a second opinion arriving at the same rate as the primary sensor, useful for redundancy, >4 m hints, and as the platform for phase-2 semantics.

Power: camera ~80 mW ([arXiv:2512.00086](https://arxiv.org/html/2512.00086) uses this figure for its always-on camera); measured whole-board draw of the XIAO streaming QVGA is ~105 mA ≈ 0.5 W ([independent review](https://quickfixsurrey.ca/seeed-xiao-esp32-s3-sense-review/)); PSRAM adds ~20–40 mA static ([ZedIoT](https://zediot.com/blog/esp32-edge-ai-architecture/)). Budget **~0.6–0.8 W** for the perception subsystem — noticeable on a drone (minutes of flight time), a rounding error on a ground vehicle.

---

## 7. Risks & Exit Criteria

### 7.1 Risk register

| Risk | Evidence | Mitigation |
|---|---|---|
| **Domain shift** — model is confidently wrong in unseen environments | RMSE 4.9 m before fine-tuning in the reference deployment ([arXiv:2512.00086](https://arxiv.org/html/2512.00086)) | Per-environment fine-tune is a *mandatory* workflow step (§4.5); online ToF-agreement metric alarms when it degrades; camera never owns the reflex |
| **Lighting sensitivity** — low light → long exposure → blur, or high gain → noise | OV2640 usable to ~1.5 lux but rolling shutter smears moving scenes under AEC ([ArduCAM issue](https://github.com/ArduCAM/ArduCAM_ESP32S_UNO/issues/15), [manual-exposure explainer](https://blog.arducam.com/manual-exposure-ov2640/)) | Exposure cap + gain telemetry + governor slow-down on maxed gain (§5.4); collect training data across lighting conditions; OV3660 preferred for low light |
| **Motor vibration + rolling shutter** — line-wise exposure turns vibration into shear/jello | Line-by-line exposure is inherent to the sensor class ([ArduCAM](https://blog.arducam.com/manual-exposure-ov2640/)); vibration blur is a documented failure mode ([OV2640 buying guide](https://smartbuy.alibaba.com/buyingguides/ov2640)) | Soft-mount the camera (foam tape / TPU bracket), short exposure cap, and *train on real driving data* so residual artifacts are in-distribution — a hidden advantage of self-supervised collection |
| **Quantization accuracy loss** — S3 is per-tensor-only in ESP-DL | MobileNetV2 71.9 % → 60.5 % per-tensor example ([esp-dl tutorial](https://github.com/espressif/esp-dl/blob/a00707d/docs/en/tutorials/how_to_deploy_mobilenetv2.rst)); µPyD-Net needed cross-layer equalization ([garagehq notes](https://github.com/garagehq/upydnet-espdl), verified in doc 05) | Small network + TFLM PTQ with full-set calibration first (TinyNav lost < 0.3 %); layerwise error audit before any ESP-DL migration; int16 promotion for offending layers |
| **Throughput collapse under Wi-Fi/PSRAM contention** | 50–80 % FPS loss with arena in PSRAM; ~½ camera clock when streaming ([ZedIoT](https://zediot.com/blog/esp32-s3-tinyml-memory-quantization-realtime-inference/), [issue #499](https://github.com/espressif/esp32-camera/issues/499)) | Arena in SRAM, octal-PSRAM boards only, core pinning per §5.2, measure with radio on |
| **Power draw on drones** | ~0.6–0.8 W subsystem (§6) | Duty-cycle: camera version primarily targeted at ground vehicles first; drone deployment only after ground validation |
| **Teacher blind spots leak into labels** — ToF fails on glass, black surfaces, sunlight, >4 m | ToF limits documented in doc 02 §2.2 | Drop invalid-status zones from labels (§4.2); accept that the student inherits the teacher's ceiling — beyond-4 m learning uses the ">2 m/free" bin only and stays advisory |

### 7.2 Measurable success criteria (gates to keep investing)

Evaluated on a held-out environment after one fine-tuning round, camera + ToF co-mounted, ESP-NOW pose broadcast active at 10 Hz:

1. **Sector accuracy:** ≥ 85 % of (sector, frame) pairs agree with the ToF teacher's bin, and ≥ 95 % within ±1 bin, over a 10-minute drive. Critically: **≥ 95 % recall on the `<0.5 m` bin** (missing a near obstacle is the failure that matters).
2. **Rate:** sustained **≥ 10 Hz** `perception_frame_t` output with radio active (measured over 10 min, 5th-percentile ≥ 8 Hz).
3. **Latency:** capture-to-queue ≤ 150 ms at the 95th percentile (timestamped end-to-end).
4. **Closed-loop non-inferiority:** with the camera feeding VFH+ (ToF reflex still armed), the standard corridor/slalom course of doc 02 §7.4 completes at 1 m/s with no reflex-layer interventions that the ToF-only version also wouldn't trigger.
5. **Fine-tune workflow cost:** new-environment adaptation (drive → train → OTA) completes in under one hobby evening without hand-labeling.

### 7.3 Parking criteria (what kills the track)

Park the version (keep the code, stop the investment) if any of these hold after the fallback model (§3.3) has also been tried:

- Sector accuracy stays **< 70 %** (or near-bin recall < 85 %) after two fine-tuning rounds in a benign indoor environment — the approach can't learn the task on this data.
- Sustained rate **< 5 Hz** with the radio on despite SRAM-arena placement — the silicon can't run the task alongside its other jobs.
- The quantization/toolchain surgery per model iteration exceeds the effort of the entire ToF version's integration — the lifecycle cost doc 05 §5 warned about has materialized.
- The closed-loop test shows the advisory layer *causing* interventions (false "blocked" sectors herding the robot into real obstacles) that persist after retraining.

Parking is cheap by design: the hardware ($14–20 board) remains useful as an FPV/logging module, the dataset and logging infrastructure feed any future perception work, and the shared interface means nothing downstream ever depended on the camera specifically.

---

## 8. References

**Hardware — boards**
- Seeed XIAO ESP32S3 Sense product page ($13.99; OV5640 add-on $11.99) — https://www.seeedstudio.com/XIAO-ESP32S3-Sense-p-5639.html
- Seeed XIAO ESP32-S3 series wiki (OV2640/OV3660 shipping variants, 8 MB PSRAM/flash) — https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/
- XIAO ESP32S3 Sense spec walkthrough (Makerguides) — https://www.makerguides.com/getting-started-with-xiao-esp32-s3-sense/
- XIAO ESP32S3 Sense independent review (measured FPS/power, board comparison table) — https://quickfixsurrey.ca/seeed-xiao-esp32-s3-sense-review/ ; FPS-by-resolution measurements — https://quickfixsurrey.ca/seeed-xiao-esp32-s3-sense-review-2/
- Freenove ESP32-S3-WROOM CAM (FNK0085, $19.99) — https://store.freenove.com/products/fnk0085
- Freenove ESP32-S3 board details incl. camera-module variants (Prusa firmware docs) — https://github.com/prusa3d/Prusa-Firmware-ESP32-Cam/blob/master/doc/Freenove%20ESP32-S3-Wroom/README.md
- ESP32-S3-EYE overview (Espressif) — https://www.espressif.com/en/products/devkits/esp32-s3-eye/overview ; getting-started guide — https://github.com/espressif/esp-who/blob/master/docs/en/get-started/ESP32-S3-EYE_Getting_Started_Guide.md ; pricing — https://www.espboards.dev/esp32/esp32-s3-eye/ ; https://www.adafruit.com/product/5955
- ESP32-S3-EYE board support package — https://components.espressif.com/components/espressif/esp32_s3_eye/versions/6.0.0/readme
- ESP32 camera board round-ups — https://componentadvisor.com/best-esp32-camera-boards-for-beginners/ ; https://makeradvisor.com/esp32-camera-cam-boards-review-comparison/
- Generic ESP32-S3-CAM buying guide — https://smartbuy.alibaba.com/buyingguides/esp32-s3-cam

**Hardware — camera sensors & driver**
- OV2640 vs OV3660 vs OV5640 comparison (frame rates per resolution) — https://www.espboards.dev/blog/esp32-camera-modules-compared/
- esp32-camera driver (supported sensors, fb_count/grab-mode semantics) — https://components.espressif.com/components/espressif/esp32-camera/versions/2.0.10
- OPI-vs-QSPI PSRAM camera DMA behavior (esp32-camera issue #499) — https://github.com/espressif/esp32-camera/issues/499
- OV2640 module buying guide (low-light, rolling-shutter limits) — https://smartbuy.alibaba.com/buyingguides/ov2640
- OV2640 manual exposure / rolling-shutter mechanics (ArduCAM) — https://blog.arducam.com/manual-exposure-ov2640/
- Motion blur on OV2640 + exposure/light trade-off (ArduCAM issue) — https://github.com/ArduCAM/ArduCAM_ESP32S_UNO/issues/15
- Shutter-speed workaround for motion smear — https://github.com/igrr/esp32-cam-demo/issues/81 ; sensor settings reference thread — https://esp32.com/viewtopic.php?t=14376

**Models & inference**
- Multi-modal on-device learning for MCU depth with VL53L5CX teacher (Nadalini et al. 2025; µPyD-Net, 3 k-sample fine-tune, RMSE 4.9→0.6 m, hyperparameters, IDSIA-µMDE dataset) — https://arxiv.org/html/2512.00086 ; code/dataset — https://github.com/idsia-robotics/ultralow-power-monocular-depth-ondevice-learning
- TinyNav: end-to-end TinyML navigation from ToF depth (23 k params, 30 ms, TFLM+ESP-NN) — https://arxiv.org/abs/2603.11071 ; HTML — https://arxiv.org/html/2603.11071v1 ; firmware — https://github.com/regularpooria/TinyNav
- ESP32-S3 vs ESP32-P4 inference benchmark (MobileNetV2 96×96: 108 ms / 8.1 FPS on S3; person detect 62 ms / 13.7 FPS) — https://quickfixsurrey.ca/esp32-s3-vs-esp32-p4-ai-benchmark/
- ESP-DL operator kernel benchmarks (S3 SIMD speedups, octal-PSRAM test config) — https://github.com/espressif/esp-dl/blob/master/operator_performance.md
- ESP-DL MobileNetV2 deployment tutorial (per-tensor-only quantization on S3, int16 mixed-precision recovery) — https://github.com/espressif/esp-dl/blob/a00707d/docs/en/tutorials/how_to_deploy_mobilenetv2.rst
- ESP-DL inference-time pitfalls (PSRAM/cache config sensitivity) — https://github.com/espressif/esp-dl/issues/247
- Verified in doc 05 and reused here: esp-tflite-micro benchmarks (380 ms ESP32 / 54 ms ESP32-S3 person detection) — https://components.espressif.com/components/espressif/esp-tflite-micro/versions/1.3.5/readme ; µPyD-Net on ESP32-S3 (~6 s cycle) — https://github.com/JPsparks/APAI_ESP_MonocularDepthEstimation ; µPyD-Net ESP-DL int8 builds + quantization notes — https://github.com/garagehq/upydnet-espdl ; FastDepth NPU requirement — https://huggingface.co/STMicroelectronics/fastdepth ; Tiny-PULP-Dronet v3 — https://doi.org/10.1109/jiot.2024.3431913

**System integration**
- TinyML-on-ESP32-S3 bottleneck analysis (PSRAM contention, Wi-Fi concurrency, arena sizing) — https://zediot.com/blog/esp32-s3-tinyml-memory-quantization-realtime-inference/
- ESP32 edge-AI architecture guide (SRAM-vs-PSRAM placement, core pinning, 50–80 % PSRAM penalty) — https://zediot.com/blog/esp32-edge-ai-architecture/
- ESP32-S3 MJPEG-over-Wi-Fi pipeline (LCD_CAM DMA, PSRAM double-buffering, Wi-Fi/core interactions) — https://codebobby.com/articles/esp32-s3-jpeg-stream.html
