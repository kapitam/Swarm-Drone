# Swarm Drone Handbook — the handoff document

**Purpose.** Everything needed to continue this project without re-reading the
whole repository or the chat history that produced it: what exists, what is
proven vs untested, which decisions are still open (the "forks"), and the
exact procedures for bring-up, training, and tuning. Written 2026-07-24.

Companion documents: [ROADMAP.md](ROADMAP.md) (what to build next, research
pre-digested per item), [research/README.md](research/README.md) (annotated
research index), and per-directory READMEs in `lib/swarmcore`, `src`, `ml`,
`test` for code navigation.

---

## 1. Project state at a glance

| Layer | State |
|---|---|
| Research (docs/research/00..09) | **Done.** 9 documents, ~150 verified citations. |
| Core algorithms (`lib/swarmcore`) | **Implemented + 26 native unit tests passing.** |
| Firmware task architecture (`src/`) | **Implemented, compiles for all 4 envs.** Runs on bench hardware; NOT yet flight-tested (no finished PCB drone exists yet). |
| Fork variants (motors/IMU/RC/perception/ML) | **All build.** Hardware-dependent ones are bench-untested (marked below). |
| ML pipeline (`ml/`) | **Implemented** (both model forks). Needs real SD logs to train — there is deliberately no pretrained model in the repo. |
| PCB | Not designed yet. Firmware is pin-map-abstracted (`src/config/pins.h`) so the PCB only needs a new `BOARD_*` block. |

**What "for a PCB drone" means here:** the vehicle is a quad-X drone
(attitude control + mixer are quad code); hardware specifics (motor type,
IMU part, radio choice, sensor placement) are all still open, so each is a
compile-time fork rather than a hardcoded choice.

---

## 2. Research index (docs/research/)

| Doc | Topic | One-line conclusion |
|---|---|---|
| 00 | Overview/index | Cross-cutting: compute is not the bottleneck; sensing + radio are. |
| 01 | Swarm algorithms survey | Boids/potential-field hybrid first (communication-aware tuned), leader–follower demo, Kilobot gradients as positioning-free fallback. |
| 02 | Obstacle avoidance | VL53L5CX + VFH-style steering + BVC inter-agent + stop-reflex layer. All classical algorithms tick in <1 ms on ESP32. |
| 03 | ESP32 compute feasibility | Classic ESP32 suffices (ESP-Drone/ESP-FC prove it); standardize new boards on ESP32-S3; ESP-NOW over nRF24 for swarm state. |
| 04 | RTOS & architecture | Use IDF FreeRTOS via Arduino (it's already there); control task pinned core 1, timer-ISR paced; comms on core 0; overwrite queues. |
| 05 | ML depth estimation verdict | Not as baseline: ToF measures depth directly; UWB/pose-broadcast beats vision ML for inter-robot. Revisit criteria listed. |
| 06 | Algorithm catalog | Quick-reference of all 17 families investigated. |
| 07 | Camera version spec | XIAO ESP32S3 Sense + "SectorNet-8" int8 sector classifier; ToF-as-teacher self-supervised training; advisory-only; exit criteria. |
| 08 | V1 ToF integration | SparkFun lib 1.0.3, 8x8@15 Hz CONTINUOUS/CLOSEST, status {5,9} filter, rows 2–5 min-pool, stop 400 mm / react 1400 mm governor. |
| 09 | V2 ML pipeline | TFLM 1.3.7 + ESP-NN ships inside Arduino core 3.3.x (pioarduino 55.03.311, verified); onnx2tf int8 path; 96x96 gray capture; SD logging feasible 3–10x margin. |

Planning docs: `docs/plan/firmware-implementation-outline.md` (work packages),
`docs/plan/avoidance-method-versions.md` (versions V1–V6, builds A–F).

---

## 3. Firmware architecture

### 3.1 Task layout (research doc 04 §7.2; see `src/config/config.h`)

| Task | Core | Prio | Rate | Role |
|---|---|---|---|---|
| `tCtrl` | 1 | 20 | 500 Hz, GPTimer ISR → task notification | IMU read → Mahony AHRS → arming/failsafe → angle+rate PIDs → quad-X mixer → LEDC motors; publishes state snapshot; feeds task watchdog. |
| `tAvoid` | 1 | 15 | 50 Hz | BehaviorPipeline: mode source → BVC → VFH-lite → governor → `AttitudeSetpoint` via length-1 overwrite queue. |
| `tSwarm` | 0 | 17 | event + 10 Hz beacon | Owns ESP-NOW RX queue: neighbor beacons → table, operator commands, RC-over-ESP-NOW; broadcasts our `StatePacket` (phase-jittered); publishes SwarmSnapshot. |
| `tRadioNrf` | 0 | 18 | 250 Hz poll | (RC_LINK_NRF24 fork) sole owner of the nRF24/VSPI; publishes RC. |
| `tTof` | 0 | 14 | 15 Hz data | VL53L5CX service: blocking I2C reads, sector reduction, raw-grid publish. |
| `tVision` | 0 | 12 | 10 Hz | (Build B) camera capture → TFLM inference → advisory sectors; SD dataset records while armed. |
| `tLog` | 0 | 4 | event | SD writer (PSRAM ring, never blocks capture). |
| `tTelem` | 0 | 10 | 5 Hz | JSON telemetry line on USB serial + control-loop rate supervision (health checks folded here — deviation from doc 04's separate tHealth, for task-count economy). |

Data flow: single-writer snapshots in `src/state_bus.h` (spinlock-guarded
copies) + one overwrite queue for the setpoint. The control task never blocks
on anything.

### 3.2 The perception contract (why V1/V2 are interchangeable)

Every perception source produces the same `sc::SectorArray`: 8 forward polar
sectors of nearest-obstacle mm over the 63° FoV. Consumers:

- **VFH-lite steering** uses the *steering view* = ToF min-fused with vision
  (vision can only make it more conservative);
- **Governor / stop-reflex** uses the *reflex view* = **ToF only, always**
  (docs 05/07 rule: learned perception never owns safety);
- **BVC** uses the neighbor table, not sectors.

Adding a V3 lidar later = one driver publishing a SectorArray. Nothing else
changes.

### 3.3 Radio protocol (`lib/swarmcore/src/swarmcore/packets.h`)

All packets ≤32 B (fit one nRF24 frame AND ESP-NOW), crc8-sealed:

- `StatePacket` (25 B, 10 Hz broadcast, phase-jittered): id, seq, x/y/z cm,
  vx/vy cm/s, heading mrad, mode, gradient hop-count, flags, battery, t_ms.
  Consumers extrapolate stale neighbors by packet age (constant velocity) —
  the single highest-value robustness trick from the research.
- `CmdPacket` (13 B): SET_MODE, **ESTOP** (fleet latch), ZERO_POSE,
  SET_PARAM (param 1 = robot id, NVS-persisted only while disarmed),
  SET_LEADER.
- `RcPacket` (8 B): legacy 4×uint16 channels — byte-compatible with the
  original transmitter sketch. Over ESP-NOW it travels wrapped in
  `RcEspNowPacket` (11 B).

Fleet identity: `ROBOT_ID_DEFAULT` build flag, NVS override. Gradient seed =
id 0 (operator station).

### 3.4 Behavior modes (runtime, via CmdPacket SET_MODE)

`MANUAL` (sticks → attitude; reflex brake only), `HOLD`, `LEADER_FOLLOW`
(slot offset behind leader, coast→lost degradation), `FLOCK` (boids +
geofence), `DISPERSE` (separation only). E-stop overrides everything.

---

## 4. Forks awaiting decision (all built, all compiling)

Pick by building the matching env; decide when hardware/testing says so.
`platformio.ini` is the single source of truth for flag combinations.

| Fork | Variants (flag) | Status | Decision criterion |
|---|---|---|---|
| Perception | `PERCEPTION_V1_TOF` / `+PERCEPTION_V2_VISION` | V1 code follows doc 08 exactly; V2 runs capture+logging today, inference once a model is embedded | V1 is the reference; V2 gate = >70% sector agreement vs ToF after fine-tune (doc 07 exit criteria) |
| ML model | `sectornet_s` / `sectornet_m` / `mupyd` (ml/ scripts; firmware auto-detects output shape) | Both forks train/export through the same pipeline | `ml/eval.py` fork-decision metrics + on-target `invoke()` latency (doc 09 §6): expect SectorNet ~8–15 ms vs uPyD ~20–35 ms |
| Motors | `MOTORS_ESC_PWM` / `MOTORS_BRUSHED_PWM` | Both implemented (LEDC core-3 API); ESC path matches original hardware; brushed is the Crazyflie-style PCB assumption | PCB decision: brushless+ESC vs coreless brushed. DShot-over-RMT documented as future variant, not built |
| IMU | `IMU_MPU6050` / `IMU_ICM42688` / `IMU_MOCK` | MPU6050 register driver standard; **ICM42688 driver bench-untested**; mock lets bare boards run | ICM-42688-P is the modern FC standard (lower noise) — verify its driver on hardware before committing the PCB footprint |
| RC link | `RC_LINK_NRF24` / `RC_LINK_ESPNOW` | Both implemented; nRF24 path byte-compatible with existing transmitter | Keep nRF24 while the old TX is the only controller; ESP-NOW RC is mandatory on XIAO (SD owns SPI) and removes a radio from the BOM |
| Board | `BOARD_DEVKIT_V1` / `BOARD_XIAO_S3` / *(future `BOARD_PCB_V1`)* | Pin maps in `src/config/pins.h` | PCB bring-up = add one block there |

**How to decide the ML fork with data:** fly Build B (no model needed —
logging works standalone), train both (`ml/README.md` workflow), compare
`eval.py` outputs + on-target latency, embed the winner. Keep the loser's
training script; it costs nothing.

---

## 5. Safety and bring-up procedures

### 5.1 Non-negotiables already enforced in code

- Motors only spin in `ARMED` (stick gesture: throttle low + yaw right 1 s;
  disarm: yaw left 1 s). Failsafe (RC loss >150 ms) cuts motors immediately.
- Fleet E-stop (`CmdPacket kCmdEStop`, broadcast) latches until an explicit
  disarm gesture acknowledges it.
- Low battery → failsafe (per-cell threshold 3.4 V; disabled when no divider).
- Stop-reflex authority is ToF-only; vision is advisory.
- Control task feeds the task watchdog; a stalled loop resets the MCU.
- No NVS writes while armed (flash-cache stall, docs 03/04).

### 5.2 Bench bring-up order (per build)

1. **Bare board, props off, motors disconnected.** Flash; watch the 5 Hz
   telemetry JSON on USB serial (115200). Check: `arm` transitions with stick
   gestures, `rc_age` sane, `loop_max_us` well under 2000, heap stable.
2. **IMU sanity:** `rpy` tracks hand tilts; still-board gyro noise small.
   (MPU6050/ICM42688 axis orientation is mounting-dependent — verify signs
   against §"Conventions" below and fix in `imu.cpp` mapping if the PCB
   mounts the chip rotated.)
3. **ToF:** `min_fwd_mm` responds to a hand at 0.3/0.7/1.5 m; `tof_ok:1`.
   Expect a 2–3 s boot pause (86 KB sensor firmware upload — normal).
4. **Radio:** two boards powered → `nbrs:1` on both; `espnow` counters climb.
5. **Motors, PROPS OFF:** arm; verify per-motor response direction against
   the mixer table (§"Conventions"); verify failsafe cut (kill TX).
6. **First flight (tethered/caged):** MANUAL only. Tune rate PIDs first
   (`sc::AttitudeParams` in `lib/swarmcore/src/swarmcore/attitude.h` —
   conservative placeholders), then angle P, then hover throttle. Only then
   try HOLD, then LEADER_FOLLOW with a second vehicle, then FLOCK.

### 5.3 What is NOT verified yet (honest list)

- **No flight testing whatsoever.** Attitude gains are textbook-shaped
  placeholders; the quad has never flown (no finished drone hardware).
- ICM-42688 driver register writes are per-datasheet but untested on silicon.
- Altitude is open: throttle is manual collective in every mode (no baro /
  downward ranger yet — first hardware addon to buy; `setpoint.throttle`
  passthrough marks the seam).
- Yaw is gyro-integrated (no magnetometer): drifts slowly; fine for reflexes
  and relative behaviors, not for long absolute headings.
- Pose is tilt-integrated dead reckoning (`estimator.h`) — minutes-scale
  drift by design; `poseValid` degrades and behaviors fall back (gradient
  mode). The real fix is the planned optical-flow (PMW3901) or UWB addon.
- Vision inference on-target: the TFLM backend compiles and follows doc 09's
  verified recipe, but no real model has run yet (needs SD logs → training).
- nRF24 RX path is ported faithfully from the original sketch but not
  re-tested against the physical transmitter.

---

## 6. V2 vision: end-to-end workflow (summary of `ml/README.md`)

1. Build B hardware = XIAO ESP32S3 Sense + VL53L5CX on D4/D5 I2C + SD card
   (FAT32 ≤32 GB). Camera and ToF co-mounted, same forward view.
2. Fly/drive armed → `sess_*.bin` accumulate (9,439 B records @ 10 Hz:
   image + raw ToF 8x8 + attitude + live-model shadow predictions).
3. `ml/train.py` both forks → `ml/export.py` (ONNX → onnx2tf full-int8) →
   `ml/eval.py` (fork-decision metrics) → `ml/gen_c_array.py --version N` →
   rebuild `xiao_s3_vision`.
4. Per-environment fine-tune is mandatory before trusting a new venue
   (5–10 min of local data; doc 07's 4.9 m→0.6 m RMSE lesson).
5. Track the camera-vs-ToF agreement in telemetry (`inf` counters + logged
   `pred_bins`); park the vision track per doc 07 exit criteria if it can't
   reach 70% agreement after two fine-tune rounds.

## 7. Conventions (get these right before touching control code)

- Frames: world = shared dead-reckoned frame (x fwd at boot, z up);
  body = x forward, y left, z up. Angles: roll + = right side down,
  pitch + = nose up, yaw CCW +. `wrapPi` everywhere.
- Mixer (quad-X, Betaflight-style numbering, viewed from above):
  m0 front-right CCW, m1 rear-right CW, m2 rear-left CCW, m3 front-left CW.
  `mixQuadX` signs are unit-tested (`test_mixer_roll_sign_convention`).
- Sectors: index 0 = leftmost of the forward 63° FoV, 7 = rightmost;
  `sectorCenterRad(i)` gives the body-frame bearing.
- Units: SI floats internally; packets cm / mrad / dV; distances in sectors mm.

## 8. Build/test/CI cheat sheet

```bash
pio test -e native                    # 26 unit tests, <1 s
pio run                               # default env (devkit_v1_tof)
pio run -e xiao_s3_vision -t upload   # flash Build B
pio run -e devkit_v1_tof -t monitor   # watch telemetry JSON
```

Platform pin: pioarduino `55.03.311` (Arduino 3.3.11 / IDF 5.5.5) — this
transitively pins TFLM 1.3.7 + ESP-NN 1.2.3 + esp32-camera. Flash use on the
classic DevKit is ~83% of the default 1.3 MB app partition; if it overflows
later, switch that env to a `huge_app.csv` partition table (4 MB flash is
plenty; OTA would need a different scheme).

## 9. Suggested next steps (in order)

> The expanded version of this list — each item with the relevant research
> findings inlined, the code seam to modify, and acceptance criteria — is
> [ROADMAP.md](ROADMAP.md). Use that as the working guide; this is the recap.

1. Buy: 1× XIAO ESP32S3 Sense, 2× VL53L5CX breakout, (optional 2nd devkit),
   props-off bench rig. Later: PMW3901 flow + VL53L1X down-ranger for
   altitude/velocity, or DW3000 UWB if formation drift demands it.
2. Bench-run Build A on the existing devkit (procedure §5.2 steps 1–5).
3. Bench-run Build B; collect first SD sessions by walking the board around;
   run the ML pipeline once end-to-end (even a mediocre first model proves
   the loop); measure real `invoke()` latency for the fork decision.
4. Design the PCB: pick motors (fork!), IMU (fork!), add `BOARD_PCB_V1` pin
   map. Wire the ToF INT pin to a GPIO (doc 08: interrupt-driven readout).
5. First tethered flight → PID tuning session → then swarm behaviors with 2
   vehicles (leader-follow before flock).
6. Firmware follow-ups queued in `docs/plan/firmware-implementation-outline.md`:
   altitude hold, optical-flow odometry, simulator-tuned boids gains (WP7),
   firefly TDMA, OTA.
