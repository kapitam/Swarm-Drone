# 05 — ML Depth Estimation for Obstacle Avoidance & Inter-Robot Localization

**Scope:** Should machine learning be added to the obstacle-avoidance stack for *depth estimation* — both depth to environment obstacles and distance/relative position between robots — on this project's hardware (ESP32 DOIT DevKit v1 today, ESP32-S3 as the upgrade path, PlatformIO/Arduino, nRF24L01 RC link, ESP-NOW planned for swarm state exchange)? This builds on [02-obstacle-avoidance.md](02-obstacle-avoidance.md) (which selected VL53L5CX + VFH+ + BVC as the classical baseline and assessed generic TinyML as "phase 2") and answers the narrower question: does *learned depth* earn a place in that stack?

**Status:** Research phase. All cited URLs were retrieved and verified during research (July 2026).

---

## 1. Executive Summary

**Verdict: No — do not add ML depth estimation now. Conditional revisit criteria in §6.**

- **For environment obstacles, ML depth is a strictly worse VL53L5CX.** The entire academic state of the art in monocular depth on MCUs is one model family: µPyD-Net, 107 k parameters, producing **48×48 px depth maps in 651 ms on an STM32F7** ([Peluso et al., IEEE TCSVT 2021](https://doi.org/10.1109/tcsvt.2021.3077395); latency figure restated in [Rusci et al. 2025, arXiv:2512.00086](https://arxiv.org/abs/2512.00086)). The already-selected **VL53L5CX ToF directly measures an 8×8 metric depth map at 15 Hz for $33 with near-zero CPU cost** — ~10× the frame rate of the best MCU depth network, with true metric scale, no training, no dataset, and no domain-shift risk.
- **The decisive irony:** the flagship 2025 paper on MCU monocular depth ([arXiv:2512.00086](https://arxiv.org/abs/2512.00086)) uses a **VL53L5CX as the ground-truth teacher** to fine-tune its neural network in the field — and needs that fine-tuning because the network's error in a new environment was **RMSE 4.9 m before adaptation, 0.6 m after**. The sensor this project already chose is what the ML research community uses to *fix* ML depth. Adding the student when you already own the teacher is backwards.
- **For inter-robot localization, learned vision works — but only on hardware we don't have.** The ETH/IDSIA/Bologna line of work localizes a peer nano-drone visually at **48 Hz within 95 mW, mean in-field error ~15 cm, usable range only ~2 m**, and it requires a GAP8 8+1-core parallel accelerator (the $240 Crazyflie AI-deck) ([Bitcraze store](https://store.bitcraze.io/products/ai-deck-1-1), [ICRA 2023 drone2drone paper](https://idsia-robotics.github.io/nanorobotics/assets/pdf/2023_icra_drone2drone.pdf)). On an ESP32-S3 the same class of CNN would run at a few Hz at best (Espressif's own S3 benchmark: 54 ms for a 96×96 person-detection classifier — a depth/pose regressor is heavier; the only public ESP32-S3 µPyD-Net build reports ~6 s per full cycle) ([esp-tflite-micro benchmarks](https://components.espressif.com/components/espressif/esp-tflite-micro/versions/1.3.5/readme), [JPsparks ESP32-S3 deployment](https://github.com/JPsparks/APAI_ESP_MonocularDepthEstimation)).
- **The non-ML alternative is better on every axis for cooperative robots:** dead-reckoned **pose broadcast over ESP-NOW** (already planned, ~free) plus, if metric accuracy is ever needed, **UWB ranging (DW3000, ~$21–44/robot)** which achieves **~3 cm ranging error and ~10 cm relative-position error** fused in an EKF — demonstrated on 13 Crazyflies whose STM32F405 (168 MHz, 192 KB) is *weaker* than our ESP32 ([SEU-NetSI RA-L 2025 paper](https://fengshan.seu-netsi.net/papers/LSLCWC-RAL25.pdf), [open-source firmware](https://github.com/SEU-NetSI/crazyflie-firmware)). Vision ML cannot beat 360°-coverage radio ranging that costs less than the camera+compute it would replace.
- **When ML depth becomes worth revisiting:** only for *non-cooperative* targets (robots/obstacles that don't broadcast and that the ToF can't see, e.g. beyond 4 m or requiring semantic identification), and only after the fleet has standardized on ESP32-S3-or-better compute with a camera already on board for other reasons. Decision criteria in §6.

---

## 2. Monocular Depth Estimation on MCUs — the Real Numbers

### 2.1 The model landscape

The "tiny depth network" literature is thin. Below the multi-million-parameter mobile/edge tier there is essentially **one** peer-reviewed architecture family for true MCU-class (<500 mW) hardware:

| Model | Params / size | Input → output | Demonstrated platform | Latency / rate | Source |
|---|---|---|---|---|---|
| **µPyD-Net** | 107 k params (int8 ≈ ~110 KB class) | 48×48 RGB → 48×48 depth | STM32F7 @ ~400 mW | **651 ms/frame (~1.5 FPS)** | [TCSVT 2021](https://doi.org/10.1109/tcsvt.2021.3077395), latency restated in [arXiv:2512.00086](https://arxiv.org/abs/2512.00086) |
| **µPyD-Net (ODL variant)** | 107 k params | 48×48 → 48×48 | GAP9 (9+1-core RISC-V, BF16 SIMD), ~100 mW SoC | inference real-time class; **on-device fine-tuning 17.8 min / 1.2 MB** | [arXiv:2512.00086](https://arxiv.org/html/2512.00086) |
| **PyD-Net** (parent) | 1.9 M params | full-res pyramid | Raspberry Pi 3 | 2 Hz | cited in [arXiv:2512.00086](https://arxiv.org/html/2512.00086) |
| **FastDepth** | MobileNetV1 encoder; int8 build **1.35 MB weights + 2.7 MB RAM** | 224×224 → depth | Jetson TX2: 178 FPS; **STM32N6 (with dedicated NPU): 24.5 ms** | see left | [MIT RLE page](https://www.rle.mit.edu/fastdepth-fast-monocular-depth-estimation-on-embedded-systems/), [ST model zoo card](https://huggingface.co/STMicroelectronics/fastdepth) |
| **PULP-Dronet v3 / Tiny-PULP-Dronet v3** (not depth — steering + collision probability) | 320 kB → **2.9 k params / 2.9 kB** | 200×200 gray → steer + collision | GAP8 (8+1-core) | 34–**139 FPS**, 0.7 mJ/inference, ~100 mW | [IEEE IoT-J 2024](https://doi.org/10.1109/jiot.2024.3431913), [arXiv:2407.12675](https://arxiv.org/html/2407.12675), [repo](https://github.com/pulp-platform/pulp-dronet) |

Key readings of this table:

- **FastDepth-class models are out of reach.** ST's own deployment table marks FastDepth as supported *only* on STM32MP2 (Linux-class) and STM32N6 (Cortex-M55 + dedicated 600 GOPS NPU); the int8 224×224 build needs ~2.7 MB of RAM ([ST Hugging Face card](https://huggingface.co/STMicroelectronics/fastdepth)). The ESP32-S3 has no NPU and ~512 KB internal SRAM; even with 8 MB PSRAM the memory bandwidth isn't there.
- **µPyD-Net is the ceiling for our class of chip, and it is a 48×48, ~1.5 FPS, relative-scale ceiling.** The output is a low-resolution *relative* depth map whose metric calibration collapses under domain shift (§2.3).
- **The famous nano-drone results (PULP-Dronet at 139 FPS) are not depth estimation** — they are steering-angle + collision-probability classification — and they run on GAP8, a 9-core parallel accelerator architecturally unlike the ESP32. This was already the conclusion of [02-obstacle-avoidance.md](02-obstacle-avoidance.md) §5, and nothing found here changes it.

### 2.2 What actually runs on ESP32-family silicon

- Espressif's own benchmark ladder for CNN inference (96×96 int8 person detection, `invoke()` time): **classic ESP32 380 ms, ESP32-S3 54 ms, ESP32-C3 426 ms, ESP32-P4 73 ms** with ESP-NN optimized kernels ([esp-tflite-micro v1.3.5 readme](https://components.espressif.com/components/espressif/esp-tflite-micro/versions/1.3.5/readme)). A depth *decoder* network does strictly more work per pixel than a classifier of the same input size.
- Two public µPyD-Net-on-ESP32 artifacts exist (both 2025-era, both niche):
  - [garagehq/upydnet-espdl](https://github.com/garagehq/upydnet-espdl): int8 `.espdl` builds of µPyD-Net for ESP32-S3 and ESP32-P4 via Espressif's ESP-DL quantization toolchain. Notably documents that naive per-tensor int8 quantization *breaks* the model (encoder features ~±1 concatenated with decoder activations spanning [−1, +95]) and cross-layer equalization is needed to recover accuracy — a taste of the engineering cost per model.
  - [JPsparks/APAI_ESP_MonocularDepthEstimation](https://github.com/JPsparks/APAI_ESP_MonocularDepthEstimation): a university-course deployment on a Freenove ESP32-S3 CAM (OV2640 at 240×240, 5×5 decimation to 48×48). Reported **total cycle time ≈ 6–7 s** (capture + preprocess + inference + SD write). Even discounting the SD write generously, this is a ~1 Hz-class pipeline producing a 48×48 relative-depth image.
- For comparison on the same S3 chip: Espressif's ESP-DL face detection on 240×240 frames reaches ~10–15 FPS ([ESP32-S3 AI camera walkthrough](https://zbotic.in/esp32-s3-ai-camera-tinyml-object-detection-on-device/)), and a from-scratch 20 k-param 64×64 classifier runs at 6.3 FPS with ~235 KB inference memory ([arXiv:2604.23012](https://arxiv.org/html/2604.23012v1)). Detection/classification at 5–15 FPS is achievable on the S3; dense depth regression at useful rates is not demonstrated anywhere.

**Camera hardware reality (fine, not the bottleneck):** the official [esp32-camera driver](https://github.com/espressif/esp32-camera) supports OV2640 (1600×1200), OV3660, OV5640 (2592×1944), OV7670/7725 and several monochrome sensors on ESP32, ESP32-S2 and ESP32-S3 over the parallel DVP interface with DMA into PSRAM. ESP32-CAM/S3-CAM boards with OV2640 cost ~$8–15. Frame rates of 12–60 FPS are reachable at QVGA–SVGA with XCLK tuning ([driver issue threads](https://github.com/espressif/esp32-camera/issues/15)). The camera can feed a network far faster than the network can eat.

### 2.3 The domain-shift problem — measured, not hypothetical

The most instructive result found is the IDSIA/Bologna on-device-learning paper ([arXiv:2512.00086](https://arxiv.org/html/2512.00086), code at [idsia-robotics on GitHub](https://arxiv.org/html/2512.00086)):

- Their IoT node (GAP9Shield, mountable on a Crazyflie 2.1) pairs an OV5647 camera with **an ST VL53L5CX 8×8 ToF — the exact sensor already selected in [02-obstacle-avoidance.md](02-obstacle-avoidance.md)** — used as the pseudo-label source.
- A µPyD-Net pre-trained off-device scored **RMSE 4.9 m when deployed in a new real environment**. Only after collecting 3 k VL53L5CX-labeled samples and fine-tuning *on the MCU* for 17.8 min did RMSE drop to **0.6 m**.
- Translation for this project: a monocular depth net trained on public datasets (KITTI/NYUv2) or in simulation is **meters wrong** in your living room / backyard until it is re-calibrated against a direct depth sensor. The re-calibration sensor is the $33 part we already planned to install. At that point the network adds latency, power, flash, and failure modes — and no information the ToF didn't already provide within its 4 m range.

---

## 3. Learned Inter-Robot Localization vs the Alternatives

### 3.1 What vision ML achieves on the best available MCU-class hardware

The ETH/IDSIA/Bologna nano-drone line is the honest upper bound for "detect and localize a peer robot with a camera and an MCU":

- **Drone-to-drone, ICRA 2023** ([PDF](https://idsia-robotics.github.io/nanorobotics/assets/pdf/2023_icra_drone2drone.pdf), [arXiv:2303.01940](https://arxiv.org/abs/2303.01940)): PULP-Frontnet CNN on a Crazyflie + AI-deck (GAP8, QVGA monochrome Himax camera). Localizes a 10 cm peer drone **up to ~2 m away**, test-set R² = 0.42 / RMSE 18 cm, **in-field mean error 15 cm, closed-loop control error 17 cm**, running **48 FPS at 95 mW** (GAP8 at max performance). Required dedicated dataset collection with mocap ground truth plus augmentation.
- **FCNN follow-up, 2024** ([SUPSI record](https://doi.org/10.71910/supsi.12087)): a fully-convolutional successor at **39 Hz / 101 mW**, R² improved to 0.47/0.55 on image coordinates on a ~30 k-image dataset; 37 % lower tracking error in flight. Still GAP8, still ~2 m-class range, still one target class.
- **Self-supervised variant** ([arXiv:2105.12797](https://ar5iv.labs.arxiv.org/html/2105.12797)): sidesteps mocap labeling by using **UWB ranging as the label source** (again: the classical sensor teaches the network), plus a Blender sim pipeline; deployed on the AI-deck.
- Related NAS work confirms the compute wall: MobileNetV2-class pose networks would run at ~4.6 FPS on GAP8 where PULP-Frontnet runs at 48 ([arXiv:2303.01931](https://arxiv.org/pdf/2303.01931)).

Hardware economics: the AI-deck (GAP8 + camera + WiFi NINA module) retails at **$240** ([Bitcraze store](https://store.bitcraze.io/products/ai-deck-1-1), $195 at [Seeed](https://www.seeedstudio.com/Crazyflie-AI-deck-V1-1-p-5112.html)). An ESP32-S3-CAM at ~$10 can run *detection-class* networks at 5–15 FPS (§2.2), so a cut-down "is there a robot in this sector" detector is conceivable — but the published accuracy numbers above were achieved with 8 parallel RISC-V cores, a curated 30 k-image dataset, and a 10 cm target at ≤2 m. Expect worse on an S3 with a hobby dataset, on robots of varying appearance, outdoors.

### 3.2 Non-ML options for inter-robot ranging on ESP32

- **UWB two-way ranging (DW1000/DW3000).** Qorvo specs the DWM3000 at **<10 cm ranging precision, <15 cm 2D / <30 cm 3D location accuracy**, channels 5/9, ⅓ the power of DW1000 ([Qorvo product page](https://www.qorvo.com/products/p/DWM3000)). Integrated ESP32+DW3000 boards: **$43.80 (Makerfabs)**, DW1000 version $39.80 ([Makerfabs](https://www.makerfabs.com/esp32-uwb-dw3000.html)); bare DWM3000 modules are ~$20. Arduino-level demo libraries exist but multi-node time-multiplexing is DIY ([Makerfabs caveat](https://www.makerfabs.com/esp32-uwb-dw3000.html), [hands-on tutorial](https://how2electronics.com/ranging-localization-with-esp32-uwb-dw3000-module/), [CircuitDigest positioning build](https://circuitdigest.com/microcontrollers-projects/diy-indoor-uwb-positioning-system-using-esp32-and-qorvo-dwm3000)); ~10 cm accuracy requires per-module antenna-delay calibration. Crucially, a *swarm-grade* protocol + EKF stack is published and open-source (§4).
- **ESP-NOW / WiFi / BLE RSSI.** Free (radio already present), but the measured reality is harsh: an ESP32 BLE study got usable estimates only **within ~4 m LOS at <25 % error**, with fluctuations, overestimation beyond 5 m, and an instant 6 dBm hit from one wall ([ELKHA 2025 evaluation](https://doi.org/10.26418/elkha.v17i2.97739)); an ESP-NOW-based localization experiment had to hand-fit path-loss parameters per environment ([MDPI Network 2025](https://doi.org/10.3390/network5020011)); ESP-NOW link studies emphasize how strongly reflections/obstacles move RSSI ([IEEE SIST 2023](https://doi.org/10.1109/sist58284.2023.10223585); practitioner consensus: "close vs far" classification only, [Arduino forum thread](https://forum.arduino.cc/t/esp32-rssi-and-neo-6m/1363066)). Verdict: RSSI is a **proximity hint** (±meters), never a ranging sensor. Useful free input for "neighbor is near, slow down" hysteresis; useless for BVC-grade geometry.
- **WiFi FTM (802.11mc).** Supported on ESP32-S2/S3/C2/C3/C5/C6 — **not** on the classic ESP32 ([ESP-IDF FTM example](https://github.com/espressif/esp-idf/blob/master/examples/wifi/ftm/README.md)); Espressif's own docs warn RTT distance "is not accurate" and wants LOS at ≥ −70 dBm ([ESP-IDF Wi-Fi driver guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-guides/wifi-driver/security-and-roaming.html)). Independent measurements: ESP32-S2 pairs show **~1 m error at 0.5 m true distance and 2–3 m error at 1.0–1.5 m indoors** (20 MHz-only limitation) ([Baddeley et al. benchmark](https://michaelbaddeley.com/wp-content/uploads/2023/05/singh2023benchmarking.pdf), [arXiv:2303.03766](https://doi.org/10.48550/arxiv.2303.03766)); a dedicated ESP32 FTM study found outdoors ~90 % of samples <2 m error (40 MHz) but indoors only 40 % <2 m and ~10 % of samples >8 m off ([arXiv:2401.16517](https://arxiv.org/html/2401.16517v1)); multipath causes systematic overestimation ([Zubow et al., Computer Communications 2023](https://www2.informatik.hu-berlin.de/~zubow/zubow2023towards-ftm.pdf)). Verdict: meter-class at best, at robot-to-robot distances (0.5–3 m) it is *worse than useless* indoors. Not a building block for collision avoidance.
- **IR intensity beacons.** The Kilobot proves the concept at the low end: IR LED + photodiode gives neighbor distance with **±2 mm accuracy, <1 mm precision — but only within ~10 cm range, and no bearing** ([Rubenstein et al., RAS 2013](https://users.eecs.northwestern.edu/~mrubenst/RAS2013.pdf); still the standard minimal-swarm sensing model in current work, e.g. [Swarm Intelligence 2025](https://link.springer.com/article/10.1007/s11721-025-00251-4)). Scaling IR to meters means multi-emitter/multi-detector rings and sunlight rejection — bespoke hardware effort disproportionate for this project. Verdict: elegant for tabletop microrobots, wrong range class for RC cars/drones.
- **Ultrasound between robots.** Already assessed in [02-obstacle-avoidance.md](02-obstacle-avoidance.md) §2.2 for environment sensing: every HC-SR04-class unit emits the same uncoded 40 kHz signature, so co-located robots jam each other; multirotor prop noise adds to it. Making it work robot-to-robot requires coded/synchronized transducers (custom hardware + tight time sync). Verdict: not competitive.
- **Pose broadcast (dead reckoning over radio).** Each robot broadcasts its dead-reckoned pose at ~10 Hz over ESP-NOW (16–32 B packets — negligible). Accuracy equals odometry drift, so it degrades with time-since-anchor, but for BVC-style mutual avoidance over seconds-long horizons it is the *already-planned* mechanism ([01-swarm-algorithms.md](01-swarm-algorithms.md), [02-obstacle-avoidance.md](02-obstacle-avoidance.md)). This is also what the canonical outdoor swarm did: Vásárhelyi et al.'s 10-drone flock shared GPS position/velocity over XBee broadcast at 10 Hz, 5 Hz GPS, no inter-robot sensing at all ([IROS 2014 paper](https://hal.elte.hu/~vasarhelyi/doc/vasarhelyi2014outdoor.pdf)).

### 3.3 Comparison table

| Approach | Relative accuracy | Range | Update rate | Cost per robot | Compute burden | ESP32 feasibility | Integration effort |
|---|---|---|---|---|---|---|---|
| **ML vision (peer detection CNN)** | ~15–18 cm in-field *on GAP8* ([ICRA'23](https://idsia-robotics.github.io/nanorobotics/assets/pdf/2023_icra_drone2drone.pdf)); unquantified on ESP32 | ~2 m (10 cm target); FoV-limited (~60–87°) | 39–48 Hz on GAP8; ~1–5 Hz realistic on ESP32-S3 | ~$10–15 (S3-CAM) but see compute; $240 for GAP8-class deck | Extreme: saturates one S3 core + PSRAM | S3 only; classic ESP32 hopeless (380 ms for a mere classifier) | Very high: dataset, training, quantization (per-model surgery, §2.2), per-environment validation |
| **UWB TWR (DW3000)** | **~3 cm ranging** ([SEU-NetSI](https://fengshan.seu-netsi.net/papers/LSLCWC-RAL25-Poster-ICRA26.pdf)); ~10 cm relative pose w/ EKF; 360°, no LOS pointing | 10–20+ m ([how2electronics test](https://how2electronics.com/ranging-localization-with-esp32-uwb-dw3000-module/)) | >16 Hz per pair; 333 Hz protocol demonstrated ([arXiv:2003.05853](https://arxiv.org/pdf/2003.05853)) | ~$20 (module) – $44 (ESP32-integrated board) | Trivial–low (SPI + EKF ~µs–ms) | Proven pairing; Arduino demos exist; swarm protocol needs porting | Moderate: antenna-delay calibration + porting an open-source swarm-ranging protocol |
| **ESP-NOW / BLE RSSI** | ±1–2 m at best <4 m LOS; unusable through walls ([ELKHA 2025](https://doi.org/10.26418/elkha.v17i2.97739)) | ~4 m usable, degrades fast | per-packet (10+ Hz) | **$0** (radio present) | Trivial | Native | Trivial — but only as a coarse proximity hint |
| **WiFi FTM (802.11mc)** | 1–3 m error *at 0.5–1.5 m true distance* indoors ([benchmark](https://michaelbaddeley.com/wp-content/uploads/2023/05/singh2023benchmarking.pdf)); ~2 m-class outdoors ([arXiv:2401.16517](https://arxiv.org/html/2401.16517v1)) | 10s of m | bursts, ~Hz | $0 on S3/C6 (not classic ESP32) | Low | S2/S3/C2/C3/C5/C6 only | Low — but accuracy disqualifies it at swarm distances |
| **IR intensity beacon (Kilobot-style)** | ±2 mm (!) but ≤10 cm range, no bearing ([RAS 2013](https://users.eecs.northwestern.edu/~mrubenst/RAS2013.pdf)) | ~10 cm (tabletop) | 30 kb/s channel | ~$1 parts, custom board | Trivial | Feasible but wrong scale | High (custom hardware) for meter-scale variants |
| **VL53L5CX ToF (already selected)** | Direct metric depth per 8×8 zone, cm-class | 0.02–4 m, 63° FoV | 15 Hz (8×8) / 60 Hz (4×4) | $32.50 | Near-zero | Proven (SparkFun lib) | Already planned; sees *any* object, cooperative or not — but cannot ID which robot it sees |
| **Pose broadcast over ESP-NOW (dead reckoning)** | = odometry drift (cm/s-class growth, resettable); exact ID/intent info | = radio range (10s–100s m) | 10–25 Hz easily | **$0** | Trivial | Native; already planned | Low; the Vásárhelyi-swarm pattern ([IROS 2014](https://hal.elte.hu/~vasarhelyi/doc/vasarhelyi2014outdoor.pdf)) |

Reading: **for cooperative robots** (ours all broadcast), pose-broadcast + optional UWB correction dominates ML vision on accuracy, range, coverage (360° vs camera FoV), cost, compute and effort. **For non-cooperative objects**, the VL53L5CX already covers 0–4 m — the only gap ML vision could fill is >4 m detection or semantic classification, neither of which the current avoidance stack needs.

---

## 4. The Sensor-Fusion Middle Ground: What Published MCU Swarms Actually Use

No published MCU-class swarm uses learned vision as its primary inter-robot localization. The working recipes are all "broadcast own state + occasional direct ranging correction":

- **Crazyflie UWB+EKF swarms (the closest template for this project).** Each drone broadcasts one UWB "ranging message" carrying timestamps + its velocity, yaw rate and height; neighbors recover ToF distance from six timestamps and fuse it with the shared states in a tiny EKF. Results: **0.03 m mean ranging error, ~0.1 m relative-localization error, 20 s convergence, demonstrated on 5 and 13 real Crazyflies** — all running on a 168 MHz STM32F405 with 192 KB RAM alongside flight control ([RA-L 2025 paper](https://fengshan.seu-netsi.net/papers/LSLCWC-RAL25.pdf), [ICRA'26 poster](https://fengshan.seu-netsi.net/papers/LSLCWC-RAL25-Poster-ICRA26.pdf), earlier version [arXiv:2003.05853](https://arxiv.org/pdf/2003.05853), [project page](https://shushuai3.github.io/autonomous-swarm/), [open firmware incl. DW3000 driver](https://github.com/SEU-NetSI/crazyflie-firmware)). The ESP32 has more CPU and RAM than this proof point.
- **Vásárhelyi outdoor flock:** GPS at 5 Hz + XBee broadcast of ID/position/velocity at 10 Hz, communication range 50–100 m, no inter-robot sensing whatsoever ([IROS 2014](https://hal.elte.hu/~vasarhelyi/doc/vasarhelyi2014outdoor.pdf)). Pure pose-broadcast is enough for flocking when an absolute reference (GNSS) bounds drift.
- **Kilobots:** IR intensity ranging only (no bearing), and a decade of swarm algorithms built on that minimal signal ([RAS 2013](https://users.eecs.northwestern.edu/~mrubenst/RAS2013.pdf), [Swarm Intelligence 2025](https://link.springer.com/article/10.1007/s11721-025-00251-4)) — evidence that even *distance-only* neighbor information supports rich collective behavior.
- Even the **ML camp fuses classically**: the drone2drone CNN's output is consumed by a Kalman filter on the STM32 ([arXiv:2303.01940](https://arxiv.org/abs/2303.01940)), and its self-supervised sibling gets labels from UWB ([arXiv:2105.12797](https://ar5iv.labs.arxiv.org/html/2105.12797)).

**Implication for this project:** the architecture already sketched in docs 01–03 (dead-reckoned pose over ESP-NOW at ~10 Hz + BVC) *is* the published-consensus baseline. The natural accuracy upgrade, if drift ever hurts, is a DW3000 per robot + a port of the SEU-NetSI swarm-ranging EKF — not a camera and a neural network.

---

## 5. Cost–Benefit Analysis

**Costs of adding ML depth (per robot and per project):**

| Cost axis | ML monocular depth / peer detection | VL53L5CX (baseline) | Pose broadcast + UWB (upgrade path) |
|---|---|---|---|
| Hardware | ~$10–15 ESP32-S3-CAM (min) … $240 GAP8-class deck (for published performance) | $32.50 | $0 … ~$21–44 |
| Compute | One S3 core + PSRAM saturated for ~1–5 Hz of 48×48 *relative* depth | ~zero (I²C reads) | ~zero (EKF is µs–ms) |
| Power | Camera ~80 mW + inference load on a 240 MHz core (100s of mW) continuous | ToF 313 mW active (duty-cyclable) ([arXiv:2512.00086](https://arxiv.org/html/2512.00086)) | radio already on; DW3000 ≈ ⅓ of DW1000 power ([Makerfabs](https://www.makerfabs.com/esp32-uwb-dw3000.html)) |
| Latency to a usable obstacle signal | 200 ms–1 s+ | 67 ms (15 Hz) | 40–100 ms (10–25 Hz packets) |
| Engineering (one-off) | Dataset collection/labeling or sim pipeline + sim-to-real gap; per-model quantization surgery ([upydnet-espdl notes](https://github.com/garagehq/upydnet-espdl)); per-environment fine-tuning (documented RMSE 4.9 m → 0.6 m only *after* field adaptation, [arXiv:2512.00086](https://arxiv.org/abs/2512.00086)); revalidation whenever robot appearance/environment changes | install library, read frames | antenna calibration + protocol port (open source exists) |
| Failure modes | silent mis-estimation under domain shift, lighting, motion blur; FoV-blind | reflectivity/sunlight limits, 4 m ceiling | drift between corrections; packet loss (already the modeled failure mode, doc 01) |

**Benefits ML depth would uniquely provide:**

1. Detection of **non-broadcasting, non-ToF-visible** objects: things beyond 4 m, or requiring semantic identity ("that's a robot, not a chair leg", "which robot is it?").
2. Passive sensing with **zero RF footprint** and no per-target hardware.
3. Range/bearing to peers **without adding a $20–44 module per robot** — relevant only at fleet sizes where UWB cost dominates (and even at 20 robots, UWB ≈ $400–880 total, far below the engineering hours ML would consume).

For the current mission profile — small fleet, cooperative robots that all broadcast, speeds ~0.5–2 m/s, avoidance horizon of a few meters (see latency analysis in [02-obstacle-avoidance.md](02-obstacle-avoidance.md) §6) — none of these benefits bind. The VL53L5CX covers non-cooperative obstacles inside the reaction envelope; ESP-NOW pose broadcast covers cooperative peers at any range the radio reaches; UWB is a drop-in metric upgrade with published MCU-grade firmware.

---

## 6. Final Recommendation & Decision Criteria

**Recommendation: do not add ML depth estimation — neither for environment obstacles nor for inter-robot localization — in the current phase.**

1. **Environment depth:** keep the VL53L5CX + VFH+ plan. A learned monocular substitute on our silicon would deliver ~10× lower frame rate, ~48×48 relative (not metric) depth, and meters of error until field-calibrated *against the very ToF sensor it would replace*.
2. **Inter-robot:** implement dead-reckoned **pose broadcast over ESP-NOW** (10–25 Hz, 16–32 B packets) feeding BVC, per docs 01–03. This matches the published consensus (Vásárhelyi flock; Crazyflie swarms).
3. **If/when metric relative accuracy is needed** (drift visibly breaks formation keeping or BVC margins): add **one DW3000 UWB module per robot (~$21–44)** and port the open-source [SEU-NetSI swarm-ranging protocol + EKF](https://github.com/SEU-NetSI/crazyflie-firmware) — a proven ~0.1 m relative-localization stack on weaker hardware than ours. This is the correct "phase 2", not vision ML.
4. **Treat RSSI as a free coarse proximity bit** (near/far hysteresis), never as ranging. Skip WiFi FTM entirely at swarm distances (1–3 m errors *at* 1 m separations indoors).

**Revisit ML depth estimation only when ALL of the following hold:**

- **A concrete non-cooperative-target requirement exists** — e.g. detecting moving obstacles or foreign robots that don't broadcast, at ranges beyond the ToF's 4 m, or requiring visual identification — that the classical stack demonstrably fails to handle.
- **The fleet has standardized on ESP32-S3 (or better)** with PSRAM and a camera already mounted for another purpose (FPV, logging), so the marginal hardware cost is ~zero. On the S3, budget for detection-class networks (5–15 FPS, "robot in sector k" + coarse size-based distance), not dense depth; treat published GAP8 numbers (39–48 Hz, 15 cm error, 2 m range) as an upper bound that the S3 will not reach.
- **Someone is willing to own the ML lifecycle:** dataset collection (or a Blender-style sim pipeline as in [arXiv:2105.12797](https://ar5iv.labs.arxiv.org/html/2105.12797)), quantization debugging, and re-validation per environment — with the VL53L5CX and/or UWB used as the label source, following the field's own practice ([arXiv:2512.00086](https://arxiv.org/abs/2512.00086)).
- **Aggregate cost still favors it:** the vision approach must beat "add a $21–44 UWB module" on the actual requirement, not in principle.

A cheaper intermediate experiment, if visual peer detection ever becomes tempting: run a sector-classification CNN (Tiny-PULP-Dronet-style, ~3–15 k params — proven at 2.9 kB / 139 FPS on GAP8, so a few-Hz S3 port is plausible, [arXiv:2407.12675](https://arxiv.org/html/2407.12675)) as a *redundant hint* into the existing VFH+ histogram, while all safety-critical geometry stays on the ToF, pose broadcast, and (if fitted) UWB. That keeps ML in an advisory role where its failure modes cannot cause collisions.
