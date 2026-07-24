# Firmware Implementation Outline

**Status update (2026-07-24): first firmware build LANDED.** WP0–WP6 (skeleton, vehicle
abstraction, estimation placeholder, ESP-NOW comms, behaviors, versioned avoidance incl. the
WP6b vision track) plus the ml/ training pipeline now exist and compile for all four envs; see
`docs/HANDBOOK.md` for the authoritative current-state document. Open decision D1 is resolved:
**the platform is a quad PCB drone** (hardware specifics still open as build-time forks).
Remaining work from this outline: WP7 simulator/tuning, altitude/velocity sensing addons,
flight testing. The original outline follows for traceability.

Current baseline in the repo before this build: a single-file Arduino superloop (`src/main.cpp`)
that receives nRF24L01 RC packets and drives one ESC, with a 150 ms failsafe.

---

## 0. Open decisions to settle during review

These change the plan's shape; everything else is incremental.

| # | Decision | Options / research guidance |
|---|---|---|
| D1 | **Ground vehicles or aerial drones first?** | Repo is named Swarm-Drone but doc 01 analyzed ground RC vehicles, and the current sketch drives one ESC. Aerial adds full attitude control (rate PID, mixing, tuning risk); ground lets the swarm stack mature safely first. Recommendation: ground first, keep interfaces vehicle-agnostic (WP2). |
| D2 | RC link stays on nRF24L01 or migrates to ESP-NOW? | Research says ESP-NOW for swarm state; nRF24 RC link can stay for bring-up (avoids re-doing the transmitter). Both radios on 2.4 GHz → channel plan needed. |
| D3 | IMU selection | Doc 03 assumes Mahony/Madgwick-class fusion; needs an IMU on the BOM (e.g. MPU-6050/ICM-42688 class). Required for heading; wheel/ESC odometry alone drifts fast. |
| D4 | Build system migration point | Doc 04: stay PlatformIO short-term via the pioarduino fork; migrate to ESP-IDF + Arduino-as-component when we need menuconfig/tracing. Decide whether WP0 does the pioarduino switch immediately. |
| D5 | Fleet size + hardware budget for sensors | VL53L5CX (~$33/robot) per doc 02; UWB (~$21–44/robot) explicitly deferred per doc 05 unless dead-reckoning drift proves unacceptable. |

## 1. Work packages

Ordered so each is testable on its own; dependencies noted. "Source" points at the research doc
that specifies the design.

### WP0 — Repo restructure and build hygiene
- Move to a modular layout: `src/` split into `control/`, `estimation/`, `comms/`, `swarm/`,
  `avoid/`, `platform/` (drivers), `config/` (one pin-map + one priority-ladder header, Crazyflie-style).
- PlatformIO env updates (pioarduino per D4), `lib/` for internal libraries so host-side unit
  tests can compile swarm/control logic without Arduino headers.
- CI: build + host unit tests on push. *Source: doc 04 §6.*

### WP1 — FreeRTOS task skeleton (the architectural core)
- Implement the doc 04 §7.2 task table: `tCtrl` (core 1, prio 20, 500 Hz, woken by esp_timer ISR
  via task notification), `tAvoid` (core 1, prio 15, 50 Hz), `tRadioNrf` (core 0, prio 18,
  IRQ-driven), `tSwarm` (core 0, prio 17), `tTelem` (core 0, prio 10), `tHealth` (core 0, prio 8),
  optional `tLog` (core 0, prio 3).
- Inter-task plumbing: single-slot overwrite queues for setpoints and neighbor state; double-buffered
  state snapshot published by `tCtrl`; log stream buffer.
- Task watchdog feeding, rate supervisor (cycle counters checked by `tHealth`), brownout-safe
  arm/disarm state machine, motors-safe failsafe path.
- Acceptance: measured `tCtrl` jitter under Wi-Fi load (SystemView or GPIO-toggle + scope). *Source: doc 04 §3, §7.*

### WP2 — Vehicle abstraction and manual control (port of existing sketch)
- `VehicleOutputs` interface (ESC/servo via LEDC/MCPWM) with ground (throttle+steer) implementation;
  aerial implementation deferred per D1.
- Port the existing nRF24 RC receive path into `tRadioNrf` (sole SPI owner, IRQ→task notification,
  parse `ControlPacket` → setpoint queue), keep the 150 ms failsafe semantics but route through the
  arm/disarm state machine.
- Acceptance: manual driving works exactly as today, but under the RTOS skeleton. *Source: doc 04 §5.1.*

### WP3 — State estimation
- IMU driver (D3) read inside `tCtrl`; Mahony (or Madgwick) fusion for heading/attitude
  (~120–160 µs/update budget per doc 03).
- Odometry: ESC/throttle model or wheel encoder if fitted; dead-reckoned pose `(x, y, θ, v)` in a
  shared frame; re-zero command from operator.
- Own-state packet builder: `uint8 id, int16 x_cm, y_cm, vx_cm_s, vy_cm_s, heading_mrad, uint16 seq`
  (13–16 B, fits one nRF24 frame). *Source: doc 01 §2, doc 03 §2.*

### WP4 — Swarm comms layer (ESP-NOW)
- ESP-NOW broadcast init on a channel coexisting with the nRF24 link (D2 channel plan).
- Beacon TX at 10 Hz with per-robot phase jitter; RX callback = copy + enqueue only (never block in
  the Wi-Fi task), drained by `tSwarm`.
- Neighbor table `{id → last state, RSSI, last-heard}`, expiry at 3× beacon period, and
  **constant-velocity extrapolation of stale neighbor state by packet age** (the single highest-value
  robustness trick per docs 01/05).
- Link instrumentation from day one: per-neighbor packet-delivery ratio, latency estimate, airtime
  counters — these numbers feed the WP7 simulator's radio model.
- Later (phase 2, deferred): firefly beacon sync → coarse TDMA; event-triggered broadcasting.
  *Source: doc 01 §6, doc 04 §5.2–5.3.*

### WP5 — Swarm behaviors
- Behavior-tree (or equivalent explicit-arbitration) layer in `tSwarm`/`tAvoid` hosting: manual,
  hold, leader–follower, flock, disperse, failsafe-stop; mission byte broadcast so the fleet
  switches coherently.
- **B1 Leader–follower** (first multi-robot behavior): followers track offset slots from the
  human-driven leader's broadcast pose; PD on slot error; constant-velocity coast on packet gaps;
  timeout → stop.
- **B2 Boids/APF hybrid** (first true flocking): separation + alignment + cohesion + wall/geofence
  repulsion (shill-style), gains loaded from a config blob tuned in WP7's simulator.
- **B3 Gradient fallback**: hop-count gradient beacons (operator = seed), follow-the-gradient homing
  and RSSI-threshold dispersion — active when pose quality degrades. *Source: doc 01 §7.2, doc 06.*

### WP6 — Obstacle avoidance (versioned; see [avoidance-method-versions.md](avoidance-method-versions.md))
- Deliver the shared consumers first: `PerceptionSource` (K-sector polar distance array) and
  `NeighborSource` (neighbor table) interfaces, VFH+ steering, speed governor, stop-reflex, and the
  BVC inter-agent filter — all version-agnostic.
- Then one driver module per perception version, starting with **V1 ToF** (VL53L5CX, I²C, 8×8 @
  15 Hz; handle the ~90 KB firmware upload at boot) as the reference implementation; V5 ultrasonic
  and V6 comms-only are nearly free; V3 lidar / V4 mmWave as hardware lands.
- **WP6b — Vision track (Build B)**: the camera + ML depth version per doc 07 spec — SD data logger
  (frames + co-mounted ToF sector labels), host training pipeline (PyTorch → int8 → TFLM/ESP-NN),
  OTA model update, camera-vs-ToF agreement telemetry. Runs in parallel after M2 on ESP32-S3
  camera hardware; advisory-only (stop-reflex stays with the ToF/speed cap).
  *Source: doc 02 §recommendation, docs 05 & 07, versions plan.*

### WP7 — Simulator and tuning pipeline (host-side, not firmware)
- Minimal 2-D kinematic simulator (Python) with the vehicle motion model and a radio model replaying
  WP4's *measured* loss/latency distributions.
- Same boids/APF gain semantics as firmware (shared header or codegen) so tuned gains transfer directly.
- Grid search or CMA-ES over the ~8–10 gains against Vásárhelyi-style fitness (speed, collisions,
  cohesion). *Source: doc 01 §5.1.*

### WP8 — Telemetry, ground station, and testing
- Telemetry packets (state + health + link stats) → simple ground-station logger/plotter; parameter
  set/get over the link (no NVS writes while armed).
- Host unit tests (Unity) for estimation, neighbor table, extrapolation, boids math, VFH+, BVC;
  on-target smoke tests; SystemView tracing setup for jitter regressions. *Source: doc 04 §6.*

## 2. Milestones (integration order)

| Milestone | Contents | Work packages |
|---|---|---|
| M0 bench | RTOS skeleton boots, jitter measured, watchdogs proven | WP0–WP1 |
| M1 single vehicle | Manual RC drive under RTOS, failsafe verified | WP2 |
| M2 pose exchange | 2+ vehicles broadcasting/receiving pose, link stats collected | WP3–WP4 |
| M3 first swarm demo | Leader–follower with human leader | WP5(B1) |
| M4 flocking | Boids/APF with simulator-tuned gains, geofence | WP5(B2), WP7 |
| M5a avoidance (Build A) | ToF + VFH+ + stop reflex + BVC active during flocking | WP6 |
| M5b vision track (Build B) | Camera advisory sectors live, agreement telemetry vs ToF | WP6b |
| M6 robustness | Gradient fallback, degraded-radio behavior, endurance runs | WP5(B3), WP8 |

## 3. Explicitly deferred (with triggers)

- **Event-triggered comms** — after M4 airtime measurements justify it.
- **UWB ranging** — only if M3/M4 show dead-reckoning drift breaks formation keeping (doc 05).
- **TinyML (coordination or depth)** — after the classical stack works; requires ESP32-S3 + camera
  + a training-data owner (docs 01 §5.3, 05).
- **Aerial vehicle support** — after D1 review; the WP2 interface keeps the door open.
- **ESP-IDF native migration** — when menuconfig/tracing needs outgrow pioarduino (doc 04 §6.1).
