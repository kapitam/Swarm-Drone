# Obstacle-Avoidance Method Versions (parallel tracks)

**Decision:** instead of committing to a single perception method, the project builds **separate,
swappable versions** of the obstacle-avoidance stack. Every version plugs into the same firmware
interfaces, so versions can be compared head-to-head on the same vehicles and mixed within one fleet.

Sources: sensor/algorithm analysis in [research doc 02](../research/02-obstacle-avoidance.md),
ML verdict in [doc 05](../research/05-ml-depth-estimation.md), camera version design in
[doc 07](../research/07-camera-depth-version-spec.md).

## 0. The shared contract (what makes versions swappable)

Two firmware interfaces isolate the method choice from everything else:

1. **`PerceptionSource`** (environment obstacles): every version outputs the same thing — a
   **K-sector polar array of nearest-obstacle distances** (K = 8 forward sectors to start) at its
   own native rate, plus a quality/age flag. The VFH+ steering layer, speed governor, and
   stop-reflex consume this array and don't care who produced it.
2. **`NeighborSource`** (inter-robot): every version maintains the same **neighbor table**
   `{id → relative or shared-frame pose, velocity, age, quality}` consumed by the BVC filter and
   the flocking behaviors.

The stop-reflex/speed-governor safety layer always stays owned by a *direct-measuring* sensor or a
conservative speed cap — never by a learned estimator (doc 05/07 rule).

## 1. Environment-perception versions

| | V1 ToF (baseline) | V2 Camera + ML depth | V3 2D Lidar | V4 mmWave radar | V5 Ultrasonic budget | V6 Comms-only |
|---|---|---|---|---|---|---|
| Sensor | VL53L5CX 8×8 ToF | OV2640/OV3660 camera | LD19/D500 or RPLidar C1 | HLK-LD2450 | 3× HC-SR04 | none (geofence only) |
| MCU needed | classic ESP32 OK | **ESP32-S3 w/ PSRAM** (XIAO ESP32S3 Sense) | classic ESP32 OK (UART parse) | classic ESP32 OK (UART) | classic ESP32 OK | classic ESP32 OK |
| Added cost/robot | ~$33 | ~$14 (board incl. camera) | ~$50–99 | ~$20 | ~$3–6 | $0 |
| Update rate | 15 Hz (8×8) / 60 Hz (4×4) | 10–15 Hz sectors | 10 Hz scan, 360° | ~10 Hz, tracks up to 3 moving targets | 10–20 Hz/sensor | n/a |
| Range / FoV | 4 m / 63° | scene-dependent / ~65° | 12 m / 360° | 6+ m / 120°, **moving targets only** (CW radar) | 2–4 m / 3 narrow cones | n/a |
| Sector-array fit | native (8 columns → 8 sectors) | native (network outputs sectors) | best quality (full 360° histogram) | sparse (target list → sectors) | coarse (3 sectors) | all-clear constant |
| Strengths | direct metric depth, cheap CPU, works in dark | richest information, doubles for peer detection later, cheapest hardware | best geometry, enables mapping later | detects fast movers, immune to lighting, sees through dust | dirt cheap, trivial code | zero hardware |
| Weaknesses | 63° FoV (blind sides), IR interference robot-to-robot possible | domain shift, lighting, needs training pipeline, advisory-only | weight/power, 10 Hz, moving parts | mostly blind to static obstacles | crosstalk between robots, acoustic noise, narrow cones | blind to everything unmapped |
| Role | **reference implementation** every other version is scored against | **experimental track** (full spec in doc 07) | ground "flagship"/mapper variant | add-on layer for dynamic obstacles | classroom/large-cheap-fleet variant | controlled-arena bring-up & absolute fallback |
| Effort | low (driver + min-pool) | high (training pipeline is the real cost) | medium (parser exists, mount/power work) | medium-low | low | trivial |

Notes:
- **V1** is the yardstick: doc 05's key finding is that the ToF also serves as the *automatic
  ground-truth teacher* for V2 — so V2 vehicles carry both sensors during data collection, which
  is exactly the co-mount the training pipeline needs.
- **V2** per doc 07: "SectorNet-8" ~25–50k-param int8 classifier, 96×96 grayscale → 8 sectors ×
  4 distance bins, 25–50 ms inference on ESP32-S3, self-supervised labels from the co-mounted ToF,
  mandatory per-environment fine-tune (~5–10 min of driving), parked if sector agreement stays
  <70% after two fine-tune rounds.
- **V4** is best treated as a *layer* combined with V1/V3/V5 (radar handles moving robots/people,
  the other sensor handles static geometry) rather than a standalone version.
- **V6** exists so the swarm stack (flocking + BVC inter-robot avoidance) can be developed and
  demoed in an empty, geofenced arena before any perception hardware arrives.

## 2. Inter-robot localization/avoidance versions

| | L1 Pose broadcast (baseline) | L2 UWB ranging | L3 Camera peer detection | L4 RSSI + right-of-way |
|---|---|---|---|---|
| Mechanism | each robot broadcasts dead-reckoned pose over ESP-NOW; BVC filter on positions | DW3000 modules, swarm-ranging protocol + relative-localization EKF | V2's camera + small detector CNN estimates bearing/range to peers | coarse near/far from RSSI; fixed yield rules (lower ID yields) |
| Added cost/robot | $0 | ~$21–44 | $0 on V2 hardware | $0 |
| Accuracy | drift-limited (odometry) | ~3 cm ranging / ~0.1 m relative pose (proven on 13-drone STM32 swarm) | ~15 cm at ≤2 m (Crazyflie AI-deck evidence; expect worse on S3) | binary/coarse |
| Depends on | WP3 estimation + WP4 comms | L1 running (fuses with it) | V2 track maturing first | nothing (works with radio only) |
| Role | **baseline**, always on | upgrade **triggered only if** M3/M4 formation tests show drift breaks station-keeping | phase-3 research extension of V2 | **always-on last-resort fallback** under radio degradation |

## 3. Compatibility matrix and named builds

Any environment version pairs with any localization version. The concrete builds worth assembling:

| Build name | Env + loc | Hardware | Purpose |
|---|---|---|---|
| **Build A "Reference"** | V1 + L1 (+L4 fallback) | classic ESP32 + VL53L5CX | the recommended first build; scores all others |
| **Build B "Vision"** | V2 (+V1 co-mounted) + L1 | XIAO ESP32S3 Sense + VL53L5CX teacher | the camera/ML experimental track |
| **Build C "Scout"** | V3 + V4 + L1 | classic ESP32 + LD19 + LD2450 | one-per-fleet ground flagship: mapping + moving-target watch |
| **Build D "Classroom"** | V5 + L1 + L4 | classic ESP32 + 3× HC-SR04 | cheapest possible large fleet |
| **Build E "Arena"** | V6 + L1 + L4 | bare classic ESP32 | swarm-algorithm bring-up before sensors arrive |
| **Build F "Precision"** | V1 + L1 + L2 | Build A + DW3000 | only if drift trigger fires |

## 4. Impact on the implementation outline

Amendments to [firmware-implementation-outline.md](firmware-implementation-outline.md):

- **WP6 is generalized**: it now delivers the `PerceptionSource`/`NeighborSource` interfaces, the
  shared VFH+/governor/stop-reflex consumers, and then **one driver module per version** (V1 first,
  V5 and V6 nearly free, V2 driver on the S3 board per doc 07, V3/V4 as ordered).
- **New WP6b (Vision track)**: data logger (camera frame + ToF sector labels to SD), host training
  pipeline (PyTorch → int8 → TFLM/ESP-NN), OTA model update, camera-vs-ToF agreement telemetry.
  Runs in parallel with the main line after M2, on Build B hardware only.
- **Milestone M5 splits**: M5a = Build A avoidance working (unchanged scope); M5b = Build B camera
  advisory sectors live with agreement telemetry; M5c+ = additional builds as hardware lands.
- **Fleet heterogeneity is a feature**: version identity travels in the telemetry/beacon so mixed
  fleets (e.g. one Build C scout among Build A vehicles) are first-class.

## 5. Suggested order of execution

1. **Build E → Build A** on current hardware (this is the existing M0–M5 path, unchanged).
2. **Build B in parallel** once one XIAO ESP32S3 Sense + VL53L5CX pair is in hand — data collection
   piggybacks on every Build A test drive.
3. **Build D** whenever fleet expansion outpaces the sensor budget.
4. **Build C** when mapping/dynamic-obstacle experiments start; **Build F** only on the drift trigger.
