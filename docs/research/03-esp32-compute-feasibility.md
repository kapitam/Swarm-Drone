# ESP32 Compute Feasibility for Swarm Coordination + Obstacle Avoidance

Research document — 2026-07. Scope: can ESP32-family MCUs run a 100–500 Hz vehicle control loop,
a 10–50 Hz swarm coordination tick, sensor fusion, and (optionally) small TinyML models — and which
family member should this project target? Current repo hardware: **ESP32 DOIT DevKit v1**
(ESP32-WROOM-32, classic dual-core Xtensa LX6 @ 240 MHz), Arduino framework on PlatformIO,
nRF24L01 on VSPI, LEDC PWM for ESC/servo.

---

## 1. Executive Summary

- **The classic ESP32 in the repo is compute-sufficient for the core workload.** Real projects run
  full flight controllers on it: Espressif's own [ESP-Drone](https://github.com/espressif/esp-drone)
  (a Crazyflie port) runs the Crazyflie 1 kHz stabilizer loop stack on ESP32, and
  [ESP-FC](https://github.com/rtlopez/esp-fc) sustains **1–4 kHz PID loops** on a classic ESP32 with an
  SPI gyro. Our target of 100–500 Hz control + 10–50 Hz swarm tick is 2–40× less demanding than what
  these projects already achieve.
- Sensor fusion is cheap if you use Mahony/Madgwick (measured **~120–160 µs per update on ESP32**,
  i.e. ~6–8 % of one core at 500 Hz); a full EKF is 3–5× more expensive and is the point where the
  classic ESP32 starts to feel loaded (ESP-Drone explicitly warns about the Kalman task's CPU cost).
- Memory is not the bottleneck for swarm state (a 50-neighbor table is ~2–4 KB), but the **Wi-Fi/BT
  stacks cost ~60–130 KB of the ~350 KB usable heap**. With ESP-NOW (which requires the Wi-Fi stack)
  expect roughly 200–250 KB free heap under Arduino — comfortable, not luxurious.
- **ESP-NOW is the standout radio option** for swarm coordination: 1–25 ms latency, 250-byte payloads,
  broadcast to unlimited unencrypted listeners, and it is already used for drone control links.
  The nRF24L01 works but adds a second 2.4 GHz radio (interference with ESP-NOW/Wi-Fi) and its
  32-byte payload is restrictive. Wi-Fi mesh libraries (painlessMesh) are too slow/heavy for
  real-time coordination.
- **TinyML is the one axis where the classic ESP32 is genuinely weak**: the S3's 128-bit vector
  instructions make it **~7× faster** on int8 CNN inference (54 ms vs 380 ms for person detection).
  Tiny MLP/1-D models (IMU gesture-scale, ≤ 20 K params) still fit on the classic chip.
- **Verdict (detail in §8):** keep the classic ESP32 for the current vehicles — it is enough.
  Standardize new hardware on **ESP32-S3** (same price class, ~34 % higher IPC, vector ML
  instructions, more RAM headroom, native USB). Avoid C3/C6 (single core, no FPU) and P4 (no radio)
  for this use case.

---

## 2. ESP32 Family Comparison

| | **ESP32 (classic, in repo)** | **ESP32-S3** | **ESP32-C3** | **ESP32-C6** | **ESP32-P4** |
|---|---|---|---|---|---|
| CPU | 2× Xtensa LX6, 240 MHz | 2× Xtensa LX7, 240 MHz | 1× RISC-V, 160 MHz | 1× RISC-V 160 MHz (+ LP core) | 2× RISC-V, 400 MHz (+ LP core) |
| CoreMark (dual/single) | 994–1080 dual (4.14–4.50/MHz) | 1329.92 dual (5.54/MHz) | ~409 @160 MHz (2.55/MHz) | similar class to C3 | 6.92/MHz ≈ ~2490 dual @360 MHz (community-verified) |
| FPU | ✅ single-precision (not in ISRs by default) | ✅ single-precision | ❌ none (soft float) | ❌ none | ✅ single-precision, **one FPU per core** |
| Double precision | software-emulated | software-emulated | software-emulated | software-emulated | software-emulated (~4–10× slower than float) |
| SIMD / ML accel | none (generic C only in ESP-NN) | **128-bit PIE vector instr.** (ESP-NN asm kernels) | none | none | PIE/QACC SIMD |
| SRAM | 520 KB | 512 KB | 400 KB | 512 KB | **768 KB** |
| PSRAM | up to 8 MB (QSPI) | up to 8–16 MB **octal** SPI | no | no | up to 32 MB in-package |
| Radio | Wi-Fi 4 + **BT Classic** + BLE 4.2 | Wi-Fi 4 + BLE 5.0 | Wi-Fi 4 + BLE 5.0 | **Wi-Fi 6** + BLE 5.3 + Zigbee/Thread | **none** (needs companion chip) |
| USB | UART bridge only | **native USB OTG** | USB-serial (JTAG) | no OTG | OTG HS |
| Fit for this project | baseline, proven | best all-round upgrade | too weak (no FPU, 1 core) | too weak for control+fusion | no radio → disqualifying for swarm |

Sources: [ESP32 datasheet](https://documentation.espressif.com/esp32_datasheet_en.pdf) (older revisions list
994.26 CoreMark dual-core / 4.14 per MHz; current revision lists 1079.96 / 4.50),
[ESP32-S3 datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf)
(1329.92 CoreMark), [espressif/coremark component](https://components.espressif.com/components/espressif/coremark/versions/1.1.0~1/readme?language=en)
(C3: 409.2 @160 MHz), [ESP32-P4 benchmark issue #14177](https://github.com/espressif/esp-idf/issues/14177)
(P4 6.92 CoreMark/MHz confirmed on silicon v1.3),
[DroneBot Workshop 2026 selection guide](https://dronebotworkshop.com/esp32-2026/),
[WizzDev SoC comparison](https://wizzdev.com/blog/espressif-soc-esp32/).

### 2.1 FPU details (verified)

- **All Espressif FPUs are single-precision only** (ESP32, S3, H4, P4). `double` is always
  software-emulated. Source: [Espressif Developer Portal, "Floating-Point Units on Espressif SoCs" (Oct 2025)](https://developer.espressif.com/blog/2025/10/cores_with_fpu/).
- **The FPU is NOT usable from ISRs by default** on the classic ESP32. FPU registers are not saved
  on ISR entry, so a float op in an ISR corrupts the interrupted task's FPU state. Confirmed by
  Espressif staff on the [ESP32 forum](https://esp32.com/viewtopic.php?t=831). ESP-IDF later added
  `CONFIG_FREERTOS_FPU_IN_ISR` (explicitly **experimental, Level-1 interrupts only**) — see the
  [feature commit](https://github.com/espressif/esp-idf/commit/86034ad0491bcc0beedba2c0250552a6c0b04d5f)
  and [issue #722](https://github.com/espressif/esp-idf/issues/722).
  **Design rule: keep float math out of ISRs; do it in tasks.**
- **Lazy FPU context switching pins float-using tasks to a core**: the first `float` use in a task
  causes IDF FreeRTOS to pin it to the current core. Source:
  [ESP-IDF FreeRTOS guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/freertos_idf.html).
- **Measured FPU cycle costs** (Espressif internal test, [issue #14177](https://github.com/espressif/esp-idf/issues/14177)):
  on **ESP32-S3** (LX7, very close to classic LX6 for scalar float): add/sub/mul ≈ 5 cycles,
  div ≈ 53, `sqrtf` ≈ 63, `sinf` ≈ 83, `powf` ≈ 610. On **P4**: div 19, sqrt 22, sin 62.
  Linpack single-float: S3 ≈ **24.3 MFLOPS**, P4 ≈ **67.4 MFLOPS**.
- **Double-precision penalty**: on S3, `cos()` in double ≈ 1619 cycles vs 121 in float (~13×);
  mixed expressions ~12× slower ([Espressif FPU blog benchmark](https://developer.espressif.com/blog/2025/10/cores_with_fpu/)).
  A 2026 [N=64 FFT benchmark](https://www.pschatzmann.ch/home/2026/07/17/microcontroller-fft-ifft-performance-benchmark-n64/)
  measured ESP32 float FFT at 75 µs vs 756 µs in double (10×), and S3 ~17 % faster than classic ESP32
  in float. **Design rule: `float` everywhere, `sinf/sqrtf/fmodf`, compile with `-Wdouble-promotion`**
  (see the [ESPHome PR doing exactly this](https://github.com/esphome/esphome/pull/17252)).

---

## 3. Compute Budget Analysis (with numbers)

Cycle budget at 240 MHz: **480,000 cycles per 2 ms tick (500 Hz)**, 2.4 M cycles per 100 Hz tick,
per core. Numbers below are for one core; the classic ESP32 has two (Wi-Fi/BT stack consumes much
of core 0).

### 3.1 What real ESP32 flight controllers achieve

| Project | Chip | Achieved loop rates | Notes |
|---|---|---|---|
| [ESP-FC](https://github.com/rtlopez/esp-fc) | ESP32 / ESP32-S3 | **up to 4 kHz gyro/PID loop** (SPI gyro); docs recommend 1–2 kHz keeping CPU < 50 % ([setup docs](https://github.com/rtlopez/esp-fc/blob/master/docs/setup.md)) | Betaflight-4.2-compatible: full rate/attitude PID cascade, gyro filters, dynamic notch, DSHOT, blackbox |
| [ESP-Drone](https://github.com/espressif/esp-drone) (Espressif's Crazyflie port) | ESP32 / S2 / S3 | Crazyflie task structure: **1 kHz stabilizer loop, 500 Hz attitude/rate PID, 100 Hz position PID** ([Crazyflie stabilizer.c](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/stabilizer.c), [controller docs](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/controllers/)) | The [ESP-Drone system docs](https://docs.espressif.com/projects/espressif-esp-drone/en/latest/system.html) note the EKF (`KALMAN_TASK`) "consumes a large amount of CPU" — fine on dual-core ESP32, but on single-core ESP32-S2 its priority must be lowered or the task watchdog fires. |
| [madflight](https://madflight.com/AHRS/) | ESP32 (among others) | 1 kHz-class loops; publishes measured filter timings (below) | Arduino-ecosystem FC framework |

Takeaway: a 100–500 Hz control loop is a *fraction* of demonstrated capability on this exact chip.

### 3.2 Sensor fusion cost

Measured on ESP32 @ 240 MHz by [madflight](https://madflight.com/AHRS/):

| Filter | Time per update (ESP32) | Cycles | % of one core @ 500 Hz | % @ 1 kHz |
|---|---|---|---|---|
| Mahony (gyro+acc) | **120 µs** | ~29 K | 6 % | 12 % |
| Madgwick (gyro+acc) | **160 µs** | ~38 K | 8 % | 16 % |
| Madgwick full 9-DOF (gyro+acc+mag) | **600 µs** | ~144 K | 30 % | 60 % |
| Complementary filter | ≈ Mahony (same order; 109 scalar ops for Madgwick 6-DOF per [Madgwick's report via MikroE](https://www.mikroe.com/blog/sensor-fusion-embedded-systems)) | — | ~5 % | ~10 % |

EKF cost: comparative studies consistently put Kalman-family filters at **3–5× the cost** of
Mahony/Madgwick ([IEEE ISITIA 2023 comparison](https://doi.org/10.1109/isitia59021.2023.10220994):
Mahony 950 µs vs Kalman 1376 µs on their test MCU;
[Sensors journal study](https://pmc.ncbi.nlm.nih.gov/articles/PMC8069451/): Kalman 3.96 ms vs
Mahony 0.76 ms in 32-bit float — also showing fixed-point 16-bit is ~6× cheaper than float where
needed). A small altitude/position KF is cheap (a 2-state KF is ~5–10 µs, a 6-state EKF well under
100 µs at 240 MHz, per [community estimates](https://zbotic.in/esp32-sensor-fusion-combine-imu-baro-and-gps-data/));
the expensive case is a full Crazyflie-style 9+-state EKF at high rate — that is what ESP-Drone
flags as the CPU hog.

### 3.3 Swarm coordination tick

A boids/potential-field update is O(N) in tracked neighbors with roughly 50–150 float ops per
neighbor (distance, normalization, 3 rule accumulators). Order-of-magnitude estimate using the
measured FPU costs above (≈5 cycles/FLOP, ~65 cycles per `sqrtf`):

- 20 neighbors × ~150 FLOPs ≈ 3,000 FLOPs ≈ **20–40 K cycles ≈ 100–170 µs per tick**.
- At 50 Hz that is **< 1 % of one core**. Even 100 neighbors at 50 Hz stays ~2–3 %.

Obstacle avoidance on top (a handful of range sensors → potential field / VFH-lite) adds another
few thousand FLOPs per tick — negligible. **The swarm layer is essentially free next to sensor
fusion; the budget question is entirely about the estimator and the radio stack.**

### 3.4 Total budget picture (classic ESP32, dual core)

| Task | Core | Rate | Cost |
|---|---|---|---|
| IMU read (SPI/I2C, DMA or task) | 1 | 500 Hz | ~2–5 % |
| Mahony/Madgwick fusion | 1 | 500 Hz | 6–8 % |
| Rate+attitude PID + LEDC output | 1 | 500 Hz | 1–2 % |
| Swarm tick (boids, 20 neighbors) | 1 | 20–50 Hz | < 1 % |
| ESP-NOW / nRF24 packet handling | 0 | ~50 Hz | few % |
| Wi-Fi/BT stack, housekeeping, logging | 0 | — | bulk of core 0 |

Comfortable ~2–4× headroom on core 1. Switching the estimator to a full EKF at 250–500 Hz consumes
most of that headroom — possible (ESP-Drone does it) but no longer relaxed.

---

## 4. Memory Budget Analysis

The classic ESP32 has 520 KB SRAM, but usable heap is far less after static allocation, IRAM code,
and cache. Measured under **Arduino** ([scottyob.com breakdown, Feb 2025](https://www.scottyob.com/post/2025-02-27-esp32-memory/)):

| State | Free heap |
|---|---|
| Bare sketch, nothing running | **~353 KB** (of ~390 KB total heap) |
| + Wi-Fi connected | ~263 KB (Wi-Fi costs **~58 KB**; an [Arduino-forum measurement](https://forum.arduino.cc/t/esp32-using-more-than-50-000-bytes-of-memory-for-wifi/1378106) puts it at 45–50 KB) |
| + BLE as well | **~148 KB** (BLE costs another ~75 KB) |

Under **ESP-IDF** the picture is similar (~180 KB main DRAM heap region + D/IRAM regions; see
[heap init log in the IDF docs](https://docs.espressif.com/projects/esp-idf/en/v6.0/esp32/api-reference/system/mem_alloc.html)).
IRAM pressure is real: heavy use of `IRAM_ATTR` and Wi-Fi eats the shared D/IRAM pool
([example: 124 KB IRAM investigation](https://github.com/espressif/esp-idf/issues/10281)).

**Swarm state fits trivially.** A neighbor record of `{id, pos[3], vel[3], heading, timestamp, rssi}`
in float is ~36–40 bytes:

| N neighbors | Table size | + double-buffer + workspace |
|---|---|---|
| 10 | ~0.4 KB | ~2 KB |
| 50 | ~2 KB | ~8 KB |
| 200 | ~8 KB | ~25 KB |

Even with FreeRTOS task stacks (ESP-Drone uses ~20 tasks with 1–4× base stack size each — see its
[task configuration](https://docs.espressif.com/projects/espressif-esp-drone/en/latest/system.html)),
queues, and logging buffers, the workload fits in ~50–80 KB of heap. **Rule of thumb for this
project: with ESP-NOW active (Wi-Fi stack on, BT off), expect ~200–250 KB free heap under Arduino —
ample.** Enable BT Classic/BLE simultaneously and it gets tight (~150 KB); avoid running both
stacks.

---

## 5. Radio / Communication Options

| | **nRF24L01 (current)** | **ESP-NOW** | **Wi-Fi mesh (painlessMesh / ESP-WIFI-MESH)** |
|---|---|---|---|
| Latency | < 10 ms typical | **1–25 ms typical, often < 5 ms** | 10–50 ms + 20–50 ms per hop |
| Payload | **32 B** | 250 B | KB-scale (JSON in painlessMesh) |
| RAM cost | ~0 (no stack; RF24 lib is tiny) | Wi-Fi stack ~50–60 KB | painlessMesh: 20–30 KB core + **5–10 KB per connection** + JSON churn |
| CPU cost | Low, but beware driver model (below) | Low (handled by Wi-Fi task on core 0) | High — mesh maintenance, JSON parse; ~10–20 msgs/s across whole mesh |
| Neighbor scale | 6 hardware RX pipes; broadcast possible | **20 registered peers (7 encrypted default, ≤17 max); unlimited unencrypted broadcast listeners** | painlessMesh practical 10–20 nodes; ESP-WIFI-MESH up to 1000 (but multi-hop latency) |
| Coexistence | Separate 2.4 GHz radio → **interferes with ESP-NOW/Wi-Fi on same board/area**; needs antenna separation and channel planning | Shares chip radio; coexists with Wi-Fi STA on same channel | Occupies the Wi-Fi radio fully |

Sources: [ESP-NOW API docs](https://docs.espressif.com/projects/esp-idf/en/v5.0.1/esp32/api-reference/network/esp_now.html)
(20 peers, 17 encrypted max, CCMP), [IIETA swarm-robot ESP-NOW study 2025](https://iieta.org/journals/jesa/paper/10.18280/jesa.590507)
(10–25 ms measured swarm latency, stability advantage over AP Wi-Fi),
[Zbotic ESP-NOW guide](https://zbotic.in/esp-now-protocol-ultra-fast-peer-to-peer-esp32-communication/)
(1–10 ms, 50–100-node unencrypted star networks reported working),
[painlessMesh FAQ](https://github.com/Alteriom/painlessMesh/wiki/Faq) and
[architecture wiki](https://github.com/Alteriom/painlessMesh/wiki/Mesh_architecture) (memory/latency
numbers), [nRF24 2.4 GHz coexistence guide](https://zbotic.in/2-4ghz-interference-wifi-bluetooth-nrf24l01-coexistence/).

**nRF24L01 gotcha specific to ESP-IDF:** the blocking `spi_device_transmit()` path costs a FreeRTOS
tick per transaction, capping a naive nRF24 driver at ~3,200 B/s regardless of SPI clock — use
polling transactions / the Arduino RF24 library's approach, or budget accordingly
([nopnop2002/esp-idf-mirf throughput analysis](https://github.com/nopnop2002/esp-idf-mirf),
[issue #5](https://github.com/nopnop2002/esp-idf-mirf/issues/5)). SPI register writes themselves are
microseconds ([measured 1.6 µs for 2 bytes @10 MHz](https://medium.com/@antonbronnfjell/a-multi-transmitter-2-4-c7f9d92afa78)).

**Recommendation:** ESP-NOW broadcast as the primary swarm channel (state beacons at 10–50 Hz,
250 B is enough for pose + velocity + intent + a few neighbor digests), nRF24L01 retained only if a
dedicated low-jitter point-to-point RC link is wanted — but do not run both radios blind on
overlapping channels. painlessMesh is not suitable for real-time coordination (its own docs advise
broadcasting no faster than every 2–5 s per node).

---

## 6. TinyML on ESP32 vs ESP32-S3

Espressif's [ESP-NN](https://github.com/espressif/esp-nn) provides optimized kernels for
[TFLite-Micro (esp-tflite-micro)](https://github.com/espressif/esp-tflite-micro); the S3 gets
hand-written assembly using its 128-bit PIE vector instructions, while the classic ESP32 only gets
generic C optimizations. Official model-level numbers (int8, `invoke()` time, internal RAM):

| Model | ESP32 @240 MHz | **ESP32-S3 @240 MHz** | ESP32-C3 @160 MHz | ESP32-P4 @360 MHz |
|---|---|---|---|---|
| Person detection (Visual Wake Words, ~250 K params) | 380 ms | **54 ms** (47 ms all-internal-RAM) | 426 ms | 73 ms |
| MobileNetV3-small (224×224, 1000 classes) | — | 1434 ms | — | 1050 ms |

(Without ESP-NN the same person-detection model takes 4084 ms on ESP32 / 2300 ms on S3 — the
optimizations matter more than the silicon.) Source: [ESP-NN README](https://github.com/espressif/esp-nn),
[esp-tflite-micro README](https://github.com/espressif/esp-tflite-micro/tree/master).

Smaller-model reference points on the **S3** ([ZedIoT S3 TinyML benchmarks](https://zediot.com/blog/esp32-s3-tinyml-optimization/)):
keyword spotting (20 K params) ~12 ms; IMU gesture model (5 K params) ~2 ms; MNIST-class CNN ~8 ms.
Scale those by ~7× for the classic ESP32 on conv-heavy models, less for tiny dense models.

[ESP-DL](https://github.com/espressif/esp-dl) (Espressif's own inference stack + ESP-PPQ quantizer,
ONNX/PyTorch → `.espdl`, int8/int16) targets **S3 and P4 with acceleration; ESP32 runs plain-C
operators and is documented as "significantly slower"**
([ESP-DL getting started](https://docs.espressif.com/projects/esp-dl/en/latest/getting_started/readme.html),
[operator support](https://github.com/espressif/esp-dl/blob/master/operator_support_state.md)).

**Realistic budgets for this project:**

- Classic ESP32: only tiny models are compatible with a live control loop — e.g. a ≤ 20 K-param int8
  MLP/1D-CNN over IMU or neighbor features at ~10–100 ms inference, run at a few Hz on core 0.
  Tensor arena must fit in ~100–150 KB of heap alongside the radio stack (WROOM-32 has no PSRAM).
- ESP32-S3: 10–50 Hz inference of 100 K–250 K-param vision/perception models becomes feasible
  (54 ms person detection), with octal PSRAM available for larger arenas. If learned obstacle
  avoidance / visual perception is ever on the roadmap, this is the deciding difference.

---

## 7. Known Bottlenecks and Gotchas

1. **Flash-cache suspension when flash is written** (NVS writes, OTA, logging to flash): during any
   SPI1 flash operation the cache is disabled, all non-IRAM interrupts are masked and other tasks are
   suspended — flash erases can take **tens to hundreds of ms**. Time-critical ISRs must be
   registered with `ESP_INTR_FLAG_IRAM`, and *everything they touch* must be `IRAM_ATTR`/`DRAM_ATTR`,
   or you get postponed interrupts or "cache disabled" crashes.
   [Flash concurrency constraints](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_flash/spi_flash_concurrency.html),
   [interrupt allocation docs](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c2/api-reference/system/intr_alloc.html),
   [memory types / IRAM placement](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/memory-types.html).
   **Consequence for us: never write NVS/flash from the control path; queue writes to a low-priority core-0 task.**
2. **No float in ISRs** (classic ESP32) — see §2.1. Timer ISRs should only capture data/wake tasks.
3. **Wi-Fi-induced jitter**: the Wi-Fi/BT stack runs pinned to core 0 (PRO_CPU) with high-priority
   tasks and IRAM ISRs. **Pin the control loop and fusion to core 1** and keep radio callbacks
   (ESP-NOW receive) short — this is the standard pattern and what ESP-Drone's task layout reflects
   ([ESP-Drone system docs](https://docs.espressif.com/projects/espressif-esp-drone/en/latest/system.html),
   [IDF FreeRTOS SMP notes](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/freertos_idf.html)).
   Also remember lazy-FPU pinning: float-using tasks self-pin to a core.
4. **ADC quality is poor and ADC2 conflicts with Wi-Fi**: the ADC is nonlinear (S-curve, unusable
   below ~0.15 V and above ~2.45 V at 11 dB attenuation), noisy (±10–30 LSB raw), and **ADC2
   (GPIO0/2/4/12–15/25–27) cannot be used while Wi-Fi is active**. Use ADC1 (GPIO32–39), the
   eFuse/line-fitting calibration (`analogReadMilliVolts()` / `adc_cali_raw_to_voltage()`), a 100 nF
   bypass cap, and multisampling — or an external ADC for battery monitoring.
   [IDF ADC calibration docs](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/adc/adc_calibration.html),
   [ADC1/ADC2 + Wi-Fi behavior](https://www.universal-solder.ca/troubleshooting-adc-inputs/),
   [nonlinearity/noise detail](https://zbotic.in/esp32-adc-accuracy-fix-noise-and-non-linearity-issues/).
5. **Brownout resets with motors + Wi-Fi TX**: Wi-Fi transmit bursts alone spike 180–500 mA; ESC/servo
   inrush on a shared rail dips the 3.3 V rail below the ~3.0 V brownout threshold and resets the MCU
   mid-flight. Mitigations: separate motor/logic supplies with common ground, 470–1000 µF low-ESR bulk
   capacitance + 0.1 µF ceramic at the module, short thick power wiring. Do **not** disable the
   brownout detector on a vehicle.
   [espboards.dev brownout guide](https://www.espboards.dev/troubleshooting/issues/power/esp32-brownout-reset/),
   [Universal Solder troubleshooting](https://www.universal-solder.ca/troubleshooting-esp32-brownout/),
   [current-draw table](https://www.oceanremote.net/troubleshooting/esp32-brownout-detector/).
6. **SPI driver task overhead (ESP-IDF)**: blocking SPI transactions can cost a scheduler tick each —
   relevant to the nRF24L01 path (§5).
7. **Mesh-library heap fragmentation**: painlessMesh's per-message JSON allocation fragments the heap
   over long runs ([painlessMesh FAQ](https://github.com/Alteriom/painlessMesh/wiki/Faq)) — one more
   reason to prefer fixed-size binary ESP-NOW frames.
8. **Single-core family members are a trap for this workload**: ESP-Drone documents that on the
   single-core ESP32-S2 the EKF task starves the system (task watchdog) unless deprioritized —
   expect the same or worse on C3/C6, which also lack an FPU
   ([ESP-Drone system docs](https://docs.espressif.com/projects/espressif-esp-drone/en/latest/system.html)).

---

## 8. Final Verdict: Classic ESP32 or ESP32-S3?

**The classic ESP32 currently in the repo is sufficient — keep it for the existing vehicles.**
The evidence: ESP-FC runs 1–4 kHz Betaflight-style loops on this exact chip; ESP-Drone runs the full
Crazyflie stack (1 kHz stabilizer, 500 Hz PID, complementary or EKF estimation) on it; measured
Mahony/Madgwick costs (~120–160 µs) leave a 500 Hz control loop using well under 20 % of one core;
the swarm coordination tick is computationally trivial (< 1 % for 20 neighbors at 50 Hz); and swarm
state for even 200 neighbors fits in tens of KB against ~200–250 KB of free heap with ESP-NOW
running. With the standard discipline — control/fusion pinned to core 1, radio on core 0,
float-only math, no flash writes or float in ISRs, ADC1 only, separate motor power — there is
comfortable headroom for 100–500 Hz control + 10–50 Hz coordination + Mahony-class fusion.

**But standardize new hardware on the ESP32-S3.** The classic ESP32 hits its ceiling on exactly the
directions this project is likely to grow:

1. **TinyML is ~7× faster on the S3** (380 ms → 54 ms person detection; ESP-DL only accelerates
   S3/P4). Any learned obstacle-avoidance/perception component effectively requires the S3.
2. **~34 % more per-clock CPU** (5.54 vs 4.14 CoreMark/MHz) and ~17 % faster float FFT — free
   headroom for upgrading to an EKF estimator or higher loop rates.
3. **More memory headroom**: up to 8–16 MB octal PSRAM options and modules with 8–16 MB flash,
   versus the WROOM-32's bare 520 KB SRAM.
4. **Same toolchain** (PlatformIO/Arduino/ESP-IDF), same dual-core 240 MHz architecture, same LEDC/
   SPI peripherals for the ESC/servo and nRF24 wiring, and ESP-NOW works identically — migration
   cost is a board definition and pin map, not a rewrite. Plus native USB OTG for flashing/debugging.
5. What you lose: BT Classic and the two DACs (neither used here), and the S3 has slightly fewer
   heap-friendly bytes free by default due to cache config — immaterial for this workload.

Chips to avoid for this project: **C3/C6** (single core, no FPU — fusion + radio + control on one
core is exactly the configuration ESP-Drone warns about) and **P4** (fastest CPU/FPU but no radio,
so it would need an ESP32-C-series companion chip — unwarranted complexity at this scale).
