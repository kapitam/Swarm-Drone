# Roadmap — what to build next, with the research already done for you

Each item below says **why**, **what the research already established** (so
you don't re-research it), **where the code seam is**, and **when it's done**
(acceptance). Items are ordered; skipping ahead is fine if hardware forces it.
Fork decisions live in `HANDBOOK.md` §4 — several items below produce exactly
the measurements those decisions need.

Conventions for agents: core logic → `lib/swarmcore` + a native test;
hardware glue → `src/platform` or a task; new perception = publish a
`sc::SectorArray`; never touch the reflex/governor path with learned outputs.

---

## Phase 1 — prove the stack on the bench (no new code)

### 1.1 Bench-run Build A and Build B
- **Why:** nothing has run on hardware yet; every later item assumes this.
- **Already established:** full procedure in `HANDBOOK.md` §5.2 (telemetry
  keys, expected values, ToF 2–3 s boot upload is normal — doc 08 §2.2).
- **Done when:** telemetry shows stable 500 Hz loop (`loop_max_us` < 2000),
  arm/disarm/failsafe transitions verified, `min_fwd_mm` tracks a hand,
  two boards see each other (`nbrs:1`).

### 1.2 Measure the radio (feeds item 3.1 and the comms research loop)
- **Why:** the boids tuning methodology needs OUR measured loss/latency.
- **Already established:** what to measure and why — doc 01 §5.1/§6; the
  counters already exist (`espnow` triple in telemetry, per-neighbor
  `lossEstimate` in the table).
- **Seam:** none needed for basics; optionally add a telemetry field dumping
  per-neighbor PDR from `Neighbor::rxCount/lossEstimate`.
- **Done when:** you have PDR + latency numbers at 2–4 m for 2–3 robots.

## Phase 2 — make it fly

### 2.1 First flight + PID tuning session
- **Already established:** gains are placeholders (`sc::AttitudeParams`);
  tuning order rate→angle→hover throttle; Crazyflie/ESP-Drone ladders as
  reference (doc 04 §4). Props-off motor-direction check against the mixer
  table (HANDBOOK §7).
- **Seam:** `lib/swarmcore/src/swarmcore/attitude.h` (gains); consider a
  `kCmdSetParam` param id per gain for live tuning (packet + handler exist).
- **Done when:** stable manual hover, tethered.

### 2.2 Altitude sensing + throttle hold  ← FIRST HARDWARE ADDON TO BUY
- **Why:** every autonomous mode currently passes manual throttle through.
- **Already established:** doc 02 sensor table — VL53L1X downward (~$10,
  4 m, I2C, same Wire bus) is the cheap pick; barometer as drift-prone
  complement; ESP-Drone runs 100 Hz position hold on this chip class.
- **Seam:** publish height via `StateBus`; add an altitude PID producing
  `setpoint.throttle` — the seam is marked in `behavior.cpp` ("Throttle:
  manual collective until an altitude sensor addon lands") and
  `task_control.cpp` consumes the setpoint unchanged. Update `VehicleState.z`.
- **Done when:** HOLD keeps altitude ±15 cm indoors.

### 2.3 Optical-flow velocity odometry (replaces the placeholder estimator)
- **Why:** `TiltOdometry` drifts by design; flocking quality is bounded by
  pose quality (doc 01 §2).
- **Already established:** PMW3901 (~$10–15, SPI, works on Crazyflie's
  weaker MCU — doc 02 §sensors; doc 05 comparison table). Pairs with 2.2's
  down-ranger for metric scaling.
- **Seam:** same interface as `sc::TiltOdometry` (pose/velocity/poseValid) —
  swap inside `task_control.cpp`; keep TiltOdometry as fallback. SPI note:
  on DevKit share VSPI with nRF24 (separate CS, mutex) or use HSPI; on XIAO
  the SD owns SPI (doc 09 §4.3) — flow sensor needs the I2C-based
  alternative or a different pin budget: check before buying for Build B.
- **Done when:** `pose_ok` stays true through a 2-minute flight; position
  error < 0.5 m over 1 minute of hover.

## Phase 3 — make it a swarm

### 3.1 Simulator + communication-aware boids tuning (WP7)
- **Why:** default boids gains are hand-set; the research's #1 methodology
  lesson is to tune against YOUR measured radio (item 1.2's numbers).
- **Already established:** doc 01 §5.1 (Vásárhelyi CMA-ES/grid method,
  fitness = speed + collision-freedom + cohesion); gains are already one
  blob (`BoidsParams`) so tuned values transfer 1:1.
- **Seam:** new host-side `sim/` (Python, 2-D kinematics + measured
  loss/latency replay). Mirror `boids.cpp` math exactly (or bind it —
  swarmcore compiles on host already, pybind/ctypes both fine).
- **Done when:** simulated 5-robot flock at measured PDR shows no collisions
  and cohesion; gains flashed via config header; 2–3 real vehicles flock.

### 3.2 Leader–follower demo (first multi-vehicle milestone)
- **Already established:** doc 01 ranks this the fastest meaningful demo
  (human-driven leader removes autonomous navigation). Implementation exists
  (`leader_follower.*`, `kCmdSetLeader`, coast/lost handling).
- **Done when:** follower holds a 1.2 m slot behind a manually flown leader,
  and stops safely when you power the leader off mid-flight.

### 3.3 Firefly beacon sync → coarse TDMA (only if 1.2 shows collisions)
- **Already established:** doc 01 §6.3 — jittered beacons suffice at ≤10
  robots; firefly sync is the upgrade and doubles as dead-robot detection.
- **Seam:** `task_swarm.cpp` beacon scheduler (`nextBeacon` phase nudging
  toward heard-beacon phases).

### 3.4 Event-triggered broadcasting (bandwidth headroom, later)
- **Already established:** doc 01 §5.2 — only worth it once airtime is
  measured; repeat event packets 2–3× + keep a heartbeat floor.
- **Seam:** `sendBeacon()` gating: transmit when own state deviates from
  what neighbors would extrapolate (the extrapolator already exists on the
  RX side — reuse the same constant-velocity predictor for self).

## Phase 4 — vision track (Build B), in parallel any time after 1.1

### 4.1 First end-to-end model
- **Already established:** entire recipe in `ml/README.md` + doc 09 (which
  toolchain versions, why, and fallbacks). No model is needed to start —
  logging runs standalone.
- **Done when:** a model (any quality) round-trips: SD logs → train → int8 →
  `gen_c_array.py` → flashed → `inf` counters live in telemetry.

### 4.2 Fork decision: SectorNet-8 vs µPyD-Net-lite
- **Already established:** decision metrics + expected numbers (doc 09 §2/§6;
  `ml/eval.py` prints them); on-target latency microbenchmark procedure
  (doc 09 §3.2 step 6).
- **Done when:** table filled for both forks on the same held-out sessions;
  owner picks; loser's script stays in `ml/models/`.

### 4.3 Vision-in-the-loop flight + agreement telemetry
- **Already established:** advisory-only integration + >70% agreement gate +
  parking criteria (doc 07 §7); shadow-mode `pred_bins` already logged in
  every SEC1 record for offline scoring.
- **Done when:** VFH steers earlier with vision fused than with ToF alone in
  a controlled A/B, with zero governor regressions.

## Phase 5 — the PCB

### 5.1 PCB design checklist (fold into the schematic review)
Everything below is already researched — cite the doc in the review:
- ToF: 2.2 kΩ I2C pullups for 1 MHz, INT pin wired to a GPIO
  (interrupt-driven readout beats polling by up to one poll interval), LPn +
  I2C_RST on GPIOs for the recovery ladder, recessed bezel sized to the
  55.5°×61° exclusion zone, no cover glass, AVDD decoupling (doc 08 §5/§6).
- Power: brownout is the #1 ESP32+motors failure (doc 03 §gotchas) — bulk
  capacitance + separate logic rail; battery divider on an **ADC1** pin
  (ADC2 is dead with Wi-Fi on), set `BATT_DIVIDER`/`BATT_CELLS`.
- Motor fork decision (HANDBOOK §4): brushed MOSFET stages vs ESC pads;
  DShot-over-RMT is a documented future variant if brushless.
- IMU fork decision: verify the ICM-42688 driver on a breakout FIRST
  (it is bench-untested); mount orientation → axis map in `imu.cpp`.
- Optional per doc 08 §multi-sensor: side/rear ToF footprints (LPn
  address-change procedure documented).
- Then: add `BOARD_PCB_V1` to `src/config/pins.h` + one env to
  `platformio.ini`. Nothing else should change.

## Phase 6 — robustness & infrastructure (as needed)

- **Escape behavior** when `vfhAllBlocked` (currently: stop). Doc 02
  recommends bug/wall-follow escape; seam: `behavior.cpp` where
  `allBlocked` is handled.
- **UWB relative localization (L2)** only if 3.1/3.2 show dead-reckoning
  drift breaking formation: DW3000 ~$21–44, ~3 cm ranging, open-source
  swarm-ranging EKF precedent on weaker MCUs (doc 05 §2 — the comparison
  table is decision-ready).
- **OTA updates:** doc 09 §5 evaluated both mechanisms — full-firmware OTA
  with compiled-in model (preferred, always-consistent) vs model-only from
  SD/LittleFS; needs a partition-scheme change (HANDBOOK §8 note).
- **ESP-IDF migration** (menuconfig, SystemView jitter tracing): staged path
  documented in doc 04 §6 — pioarduino (done) → Arduino-as-component →
  native. Trigger: needing trace-level scheduling proof or sdkconfig control.
- **Magnetometer or flow-aided yaw** if absolute heading drift matters for
  formations (currently gyro-integrated; noted HANDBOOK §5.3).
- **Watch-items from research:** ToF warm-up drift (log `silicon_temp_degc`
  when SparkFun exposes it / ULD 2.0 — doc 08 §3.4); robot-to-robot ToF IR
  interference at close range (doc 08 §5.3 mitigation: CLOSEST target order
  already set); XCLK EV-VSYNC-OVF fallback already implemented.

---

## Anti-goals (researched and deliberately rejected — don't re-add casually)

- ML depth as the SAFETY layer (doc 05/07: advisory only, ever).
- painlessMesh / RF24Mesh for coordination (doc 03/01: wrong latency class /
  wrong topology — raw broadcast + neighbor table is the pattern).
- Full Kilobot shape assembly without ranging hardware (doc 01 §4.2).
- MARL/TinyML *coordination* policies before the classical stack works
  (doc 01 §5.3 — phase-2+ with sim-to-real burden).
- Upgrading the pinned platform (pioarduino 55.03.311) without re-verifying
  TFLM/ESP-NN/camera (doc 09 pin policy).
