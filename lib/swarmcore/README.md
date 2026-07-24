# swarmcore — portable core logic

Everything algorithmic lives here, **with zero Arduino/ESP dependencies**, so
it compiles on the host and is covered by `pio test -e native`
(`test/test_swarmcore/`). The ESP32 firmware in `src/` is a thin shell around
this library. If you change behavior, change it here and extend the tests.

Namespace `sc::`. Units: SI floats internally (m, m/s, rad); packets use
cm / mrad / deciVolt; sector distances are mm. Angle convention: body x
forward, y left, z up; `wrapPi` everywhere.

## File map (src/swarmcore/)

| File | What it implements | Research basis |
|---|---|---|
| `types.h` | `Vec2`, `Pose2D`, `VehicleState`, `SectorArray` (the 8-sector perception contract: `kUnknownMm`, `minForwardMm()`, `sectorCenterRad()`) | docs 02/08 |
| `packets.h` | Wire formats ≤32 B, crc8-sealed: `StatePacket` (25 B swarm beacon), `CmdPacket` (mode/E-stop/zero-pose/param/leader), `RcPacket` (legacy 4-ch, byte-compatible with the original transmitter), `RcEspNowPacket` | doc 01 §2/§6 |
| `sectors.h/.cpp` | `SectorFilter`: VL53L5CX 8x8 grid → 8 sectors. Validity = status∈{5,9} && nb==1; min-pool rows 2–5; clamp 2000 mm; hold-last-valid 3 frames (**"invalid is not free"**) | doc 08 §4 (verbatim) |
| `neighbor_table.h/.cpp` | Fixed-size (12) peer table from beacons: expiry, seq-gap loss estimate, oldest-eviction, **`predictedPos()` constant-velocity extrapolation by packet age** — the key lossy-radio trick | doc 01 §6.2 |
| `gradient.h` | Kilobot hop-count gradient (min(heard)+1; seed=0). The positioning-free fallback layer | doc 01 §4.2 |
| `boids.h/.cpp` | Separation/alignment/cohesion + goal + heading persistence + rectangular geofence "shill" repulsion. All gains in `BoidsParams` (simulator-tunable blob) | doc 01 §3.1/§5.1 |
| `leader_follower.h/.cpp` | Slot offset in leader body frame; PD + leader-velocity feedforward; TRACKING → COASTING (predict ≤400 ms) → LOST (caller holds) | doc 01 §3.4 |
| `vfh_lite.h/.cpp` | Binary polar histogram: block <1400 mm, release +200 mm hysteresis, ±1-sector inflation, pick free sector nearest desired direction; `allBlocked` when trapped | docs 02, 08 §7 |
| `governor.h/.cpp` | Speed cap + stop reflex, piecewise-linear knots 150 (fear/back-away) / 400 (stop) / 700 / 1400 / 2000 mm; BLIND crawl when sensor dead | doc 08 §7 |
| `bvc.h/.cpp` | Buffered Voronoi Cell velocity filter (inter-drone): clip desired velocity against bisector half-planes, positions only, O(neighbors) | docs 02 §4, 06 #15 |
| `pid.h` | PID with derivative-on-measurement, integral clamp, output limit | — |
| `mahony.h/.cpp` | 6-axis Mahony AHRS quaternion filter (~130 µs/update class on ESP32) | doc 03 |
| `attitude.h` | Angle-mode cascade (angle P → rate PIDs) + `velocityToTilt()` (desired velocity → roll/pitch). **Gains are untuned placeholders** | doc 04 case studies |
| `mixer.h` | Quad-X mix with common-mode shift + differential rescale on saturation, idle floor. Motor order m0 FR-CCW, m1 RR-CW, m2 RL-CCW, m3 FL-CW | — |
| `arming.h` | DISARMED→ARMING→ARMED→FAILSAFE(+ latched ESTOP) state machine; stick gestures; 150 ms RC-loss cut; low-battery | original sketch semantics preserved |
| `estimator.h` | `TiltOdometry` — **placeholder** dead-reckoning (tilt→accel model with drag). Replace via same interface when optical-flow/UWB addon lands | doc 01 §2 honest-gap |
| `rc.h` | 1000–2000 µs → normalized channels with deadband | — |
| `behavior.h/.cpp` | `BehaviorPipeline` — the arbitration layer: mode source (manual/hold/follow/flock/disperse) → BVC → VFH → governor → `AttitudeSetpoint`. Manual keeps pilot authority (reflex brake only). Separate *steering* vs *reflex* sector views (reflex = ToF only) | docs 01 §4.4, 05/07 rule |

## Invariants to preserve

1. **No Arduino headers** — keeps native tests possible (`constrain` name is
   already avoided; beware Arduino macros like `min`/`max`/`abs` shadowing).
2. Packet structs are packed + static_asserted; changing them breaks fleet
   compatibility and `ml/dataset.py` — bump consciously, in both places.
3. The reflex/governor path must stay learned-perception-free.
4. Every behavior must degrade safely with stale/missing data (that's why
   extrapolation, coast/lost states, hold-last sectors, BLIND crawl exist).
