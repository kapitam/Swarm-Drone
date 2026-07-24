# Research index (annotated)

Ten documents, written in order. Each was produced by a dedicated research
agent with web sources verified at retrieval time (2023–2026 sources
preferred). Read 00 first; every doc ends with its own reference list.

**If you are about to implement something, check the table below first —
the answer is probably already researched.**

| # | File | Question it answers | Load-bearing findings (used by the code today) |
|---|---|---|---|
| 00 | [00-overview.md](00-overview.md) | Index + cross-cutting conclusions | Compute is never the bottleneck on ESP32; sensing + radio reliability are. ESP-NOW over nRF24 for swarm state. Classic ESP32 fine, S3 for new boards. |
| 01 | [01-swarm-algorithms.md](01-swarm-algorithms.md) | Which swarm coordination algorithms fit this hardware? | Boids/potential-field hybrid is the #1 pick (implemented in `swarmcore/boids.*`); leader–follower #2 (`leader_follower.*`); Kilobot hop-count gradients as positioning-free fallback (`gradient.h`). 16-byte pose beacon @ 10 Hz; **extrapolate stale neighbors by packet age** (implemented in `neighbor_table.h`); phase-jitter beacons. Vásárhelyi communication-aware tuning = the method for WP7 gain tuning. |
| 02 | [02-obstacle-avoidance.md](02-obstacle-avoidance.md) | Which sensors + avoidance algorithms? | VL53L5CX best value; VFH-style polar steering (`vfh_lite.*`); BVC for inter-robot (`bvc.*`, needs only positions); layered reflex architecture (`governor.*`); latency math → 0.5–1 m/s safe airborne speed. Lidar/mmWave/ultrasonic assessed for other builds. |
| 03 | [03-esp32-compute-feasibility.md](03-esp32-compute-feasibility.md) | Does it all fit on an ESP32? | Yes with margin (ESP-Drone/ESP-FC precedents). Mahony fusion ~120–160 µs/update (chosen, `mahony.*`). Gotchas honored in code: ADC2 unusable with Wi-Fi, no NVS writes armed, core pinning, single-precision FPU only, no FPU in ISRs. |
| 04 | [04-rtos-and-firmware-architecture.md](04-rtos-and-firmware-architecture.md) | RTOS choice + task architecture | Arduino already runs on IDF FreeRTOS; the task table in §7.2 is implemented 1:1 in `src/` (control 500 Hz core 1 timer-ISR paced; comms core 0; overwrite queues; callbacks only enqueue). Migration path to ESP-IDF when tracing/menuconfig needed. |
| 05 | [05-ml-depth-estimation.md](05-ml-depth-estimation.md) | Should ML depth be added? | Not as baseline (ToF measures depth directly). Key insight that shaped Build B: **use the ToF as the automatic training teacher**. Inter-robot: pose broadcast now, DW3000 UWB (~$21–44, ~3 cm) if drift hurts — before any vision ML. Revisit criteria listed. |
| 06 | [06-swarm-algorithms-catalog.md](06-swarm-algorithms-catalog.md) | Quick reference | One-line summary + verdict for all 17 algorithm families investigated (coordination + inter-agent avoidance). |
| 07 | [07-camera-depth-version-spec.md](07-camera-depth-version-spec.md) | Design of the camera/ML version | XIAO ESP32S3 Sense ($14) chosen; SectorNet-8 concept (8 sectors × 4 bins); self-supervised data collection while driving; per-environment fine-tune mandatory; advisory-only integration; **exit criteria** (<70% agreement after 2 fine-tunes → park the track). |
| 08 | [08-v1-tof-integration.md](08-v1-tof-integration.md) | Exact VL53L5CX driver recipe | Implemented verbatim in `src/platform/tof_vl53l5cx.cpp` + `swarmcore/sectors.*`: SparkFun lib 1.0.3, `setWireMaxPacketSize(128)` before `begin()`, 8x8@15 Hz CONTINUOUS CLOSEST, status {5,9} + nb==1 validity, min-pool rows 2–5, clamp 2 m, hold-last 3 frames, governor knots 150/400/700/1400/2000 mm. Also: multi-sensor addressing, PCB electrical rules (2.2 kΩ pullups, INT wiring, bezel exclusion zone), interference between drones. |
| 09 | [09-v2-ml-pipeline.md](09-v2-ml-pipeline.md) | Exact ML runtime + training recipe | Implemented in `src/vision/` + `ml/`: TFLM 1.3.7 + ESP-NN inside Arduino core 3.3.x (pioarduino 55.03.311 pin — **do not casually upgrade**), 96 KB SRAM arena, ESP-NN-safe op set, onnx2tf `-oiqt` int8 path, 96×96 gray direct capture, XCLK 20→10 MHz fallback, SD record format (9,439 B), SD-owns-SPI finding (→ ESP-NOW RC on XIAO), both model-fork specs + decision criteria. |

## How this maps to the plan and code

- Versions/builds strategy: [`../plan/avoidance-method-versions.md`](../plan/avoidance-method-versions.md)
  (V1–V6 perception versions, L1–L4 localization versions, builds A–F).
- Work-package outline: [`../plan/firmware-implementation-outline.md`](../plan/firmware-implementation-outline.md).
- Current implementation state + procedures: [`../HANDBOOK.md`](../HANDBOOK.md).
- What to build next (with research pointers): [`../ROADMAP.md`](../ROADMAP.md).
