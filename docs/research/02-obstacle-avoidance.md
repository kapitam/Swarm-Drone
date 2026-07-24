# 02 — Obstacle Avoidance on ESP32-Class Vehicles

**Scope:** Obstacle-avoidance sensing and algorithms feasible on the project's target platform — ESP32 DOIT DevKit v1 (dual-core Xtensa LX6 @ 240 MHz, ~520 KB SRAM, single-precision FPU, Arduino/PlatformIO, nRF24L01 radio) — for small ground vehicles and drones operating in a swarm. Covers environment obstacles (static/dynamic) and inter-agent collision avoidance.

**Status:** Research phase. All cited URLs were retrieved and verified during research (July 2026).

---

## 1. Executive Summary

- **Sensing is the bottleneck, not compute.** A 240 MHz ESP32 executes any of the classical reactive avoidance algorithms (potential fields, VFH-style histogram steering, bug behaviors, ORCA/BVC for a handful of neighbors) in well under 1 ms per tick. What limits safe speed is sensor update rate, field of view, and data latency.
- **Best sensing value for this platform:** the ST **VL53L5CX** multizone ToF sensor — an 8×8 "depth image" (or 4×4 @ 60 Hz), 63° FoV, up to 4 m, over I²C, ~$33, with a maintained Arduino library (SparkFun). It provides just enough spatial structure for histogram-style steering without the CPU/RAM burden of a camera or the weight/power/parsing cost of a spinning lidar.
- **Recommended algorithm pairing:** a **VFH-style polar-histogram reactive layer** for environment obstacles + **Buffered Voronoi Cell (BVC) inter-agent avoidance** using positions shared over nRF24L01, with a hard **stop/escape reflex** as the lowest layer. BVC is proven on Crazyflie's STM32F405 (a *weaker* MCU than the ESP32) inside the shipping Bitcraze firmware.
- **TinyML is not worth it on the plain ESP32** for this task: the reference person-detection CNN takes ~380 ms/inference even with ESP-NN optimized kernels (~2.6 FPS). The nano-drone state of the art (PULP-Dronet / Tiny-PULP-Dronet v3) achieves 139 FPS only on a GAP8 8-core parallel accelerator. The *ideas* transfer (tiny quantized CNNs, ~3–100 KB), the *performance* does not. An ESP32-S3 (54 ms/inference) would change this calculus, but classical methods remain the right first implementation.
- **Realistic safe speeds** (latency analysis, §6): ~1–2 m/s for ground vehicles with a VL53L5CX @ 15 Hz; ~0.5–1 m/s for drones (consistent with the TU Delft SGBA swarm, which flew Crazyflies at ~0.5 m/s using four 1-D ToF rangers on an STM32F4).

---

## 2. Sensor Options

### 2.1 Comparison table

| Sensor | Type | Cost (approx.) | Interface | Update rate | Range / FoV | ESP32 CPU/RAM burden | ESP32 driver support |
|---|---|---|---|---|---|---|---|
| HC-SR04 | Ultrasonic 1-D | ~$1–3 | GPIO trigger/echo | ≤ ~16 Hz (≥60 ms cycle recommended) | 2–400 cm, ~15° beam | Trivial; echo timing via interrupt. Blocking `pulseIn` wastes up to 38 ms if naive | Ubiquitous (NewPing etc.); trivial to bit-bang |
| Sharp GP2Y0A21YK0F | IR triangulation 1-D | ~$8–12 | Analog (ADC) | ~26 Hz (38±10 ms update period) | 10–80 cm | Trivial (one ADC read) | Native ADC; note 4.5–5.5 V supply, noisy 30 mA bursts |
| VL53L0X / **VL53L1X** | 1-D laser ToF | ~$5–15 (breakout) | I²C (400 kHz) | up to 50 Hz (L1X, short mode) | up to 400 cm (L1X), FoV 15–27° programmable | Trivial; a few registers per read | Pololu & Adafruit Arduino libs, ST ULD |
| **VL53L5CX** | 8×8 multizone ToF | $32.50 (SparkFun Qwiic) | I²C (400 kHz–1 MHz) | 60 Hz @ 4×4, 15 Hz @ 8×8 | 2–400 cm/zone, 63° diagonal FoV | Low: 64 distances/frame; **~90 KB firmware blob** stored in MCU flash, uploaded to sensor at boot (~1.4 s @ 400 kHz) | SparkFun Arduino lib (works on ESP32); RJRP44 ESP-IDF component |
| VL53L8CX | 8×8 multizone ToF (newer) | ~$25–40 (SATEL) | I²C / SPI | 60 Hz @ 4×4, 15 Hz @ 8×8 | ~400 cm, 65° FoV | Same class as L5CX | ESP-IDF driver exists (RJRP44 family); Arduino support thinner than L5CX |
| LD19 / LDROBOT D500 kit | 2-D spinning DToF lidar | $50–99 | UART 230400, TX-only (PWM pin → GND) | 10 Hz scan (5–13 Hz), 4500 samples/s | 12 m (white), 360°, 0.167°–0.72° resolution | Moderate: continuous 230 kbaud parsing (~12 pts/packet); a full 360° scan buffer of ~450 points ≈ 2–4 KB; run parser on core 0 | Community ESP32/Arduino tutorials & parsers (LudovaTech), kaiaai/LDS |
| RPLidar C1 | 2-D spinning DToF lidar | ~$69 | UART 460800 (motor control over UART, no PWM pin) | 10 Hz scan, 5000 samples/s | 0.05–12 m, 360°, ~0.72° | Same class as LD19; 460 kbaud needs hardware UART | kaiaai/LDS (C1 support merged), Robotto RPLIDAR_C1_ESP32 |
| PMW3901 | Optical flow (velocity, not range) | ~$20–30 (Pimoroni/Bitcraze breakout) | SPI @ 2 MHz | 121 FPS internal; poll as needed | 80 mm–∞ working height, 42° FoV | Trivial (2 delta registers); 6–9 mA | Bitcraze official Arduino lib; documented ESP32 wiring |
| HLK-LD2450 | 24 GHz FMCW mmWave radar | ~$10–20 | UART 256000 (needs hardware UART) | ~10 Hz effective target reports | 6 m, ±60° azimuth / ±35° elevation; tracks ≤3 moving targets (X/Y position + speed) | Trivial: 30-byte frames, 3 target blocks | ESPHome component; simple frame parser in Arduino |
| ESP32-CAM (OV2640) + TinyML | Monocular vision | ~$8–12 (board) | On-board (DVP camera) | ~2.6 FPS (380 ms/inference, 96×96 int8 person-detect w/ ESP-NN @ 240 MHz) | Scene-dependent; no metric depth | Very high: ~300 KB flash model + ~100 KB tensor arena (PSRAM), one core saturated | esp-tflite-micro + ESP-NN (ESP-IDF); Arduino TFLM ports exist |

Prices marked "~" are street prices observed across vendors in 2025–2026 and vary; the SparkFun VL53L5CX ($32.50), LD19/D500 ($50–99 across witmotion/sensorlidar/Amazon/RobotShop) and RPLidar C1 (~$69, Amazon comparison table) were directly observed.

### 2.2 Notes per sensor

**HC-SR04 (ultrasonic).** Datasheet: 2–400 cm, 40 kHz burst, 15° effective angle, 15 mA, 10 µs TTL trigger; the manufacturer recommends a ≥60 ms measurement cycle to avoid trigger/echo collision, capping one sensor at ~16 Hz ([SparkFun/DigiKey datasheet](https://www.digikey.com/htmldatasheets/production/1979760/0/0/1/hc-sr04.html), [Handson Tech user guide](https://www.handsontec.com/pdf_files/hc-sr04-User-Guide.pdf)). Multiple sensors must be triggered *sequentially* to avoid crosstalk ([DroneBot Workshop](https://dronebotworkshop.com/hc-sr04-ultrasonic-distance-sensor-arduino/)) — so 4 sensors ≈ 4 Hz effective per direction. **Swarm-specific caveat:** every robot emits the same 40 kHz signature, so ultrasonic robots in close proximity will hear each other's pings; there is no per-robot coding. Verdict: acceptable as a cheap redundant bumper on slow ground vehicles; poor as the primary swarm sensor. Useless on multirotors (prop noise, weight of the big transducers, narrow beam).

**Sharp GP2Y0A21YK0F (IR triangulation).** 10–80 cm analog output, update period 38±10 ms (~26 Hz), 30 mA in bursts, 4.5–5.5 V supply ([Sharp datasheet](https://www.micro-semiconductor.jp/datasheet/bc-GP2Y0A21YK0F.pdf), [Pololu product page](https://www.pololu.com/product/136)). Simple and immune to acoustic crosstalk, but short range, nonlinear output, sensitive to surface reflectivity and sunlight. Verdict: side/rear proximity fill-in at best; superseded by 1-D ToF at similar cost.

**VL53L0X / VL53L1X (1-D laser ToF).** The VL53L1X ranges to 400 cm at up to 50 Hz over 400 kHz I²C, with a programmable 15–27° FoV, and has a lightweight Pololu Arduino library plus ST's full API ([ST datasheet](https://www.st.com/resource/en/datasheet/vl53l1x.pdf), [ST product page](https://www.st.com/en/imaging-and-photonics-solutions/vl53l1x.html?icmp=tt16126_gl_pron_jul2020), [pololu/vl53l1x-arduino](https://github.com/pololu/vl53l1x-arduino)). The older VL53L0X is ~2 m and is what Bitcraze pairs with the PMW3901 on the Flow breakout ([PX4 docs](https://docs.px4.io/main/en/sensor/pmw3901)). This is exactly the sensor class used (×4, front/back/left/right) on the Crazyflie Multi-ranger deck that flew the SGBA swarm experiments (§4.3). Verdict: excellent secondary sensors — side coverage, downward altimeter for drones, cross-checking. Multiple units share one I²C bus via the XSHUT pin address-reassignment dance.

**VL53L5CX (8×8 multizone ToF) — the standout.** 64 independent zones at up to 400 cm, 63° diagonal FoV, 4×4 @ 60 Hz or 8×8 @ 15 Hz, I²C up to 1 MHz ([ST product page](https://www.st.com/en/imaging-and-photonics-solutions/vl53l5cx.html), [datasheet](https://cdn.sparkfun.com/assets/6/e/3/0/6/vl53l5cx-datasheet.pdf), [UM2884 ULD manual](https://www.pololu.com/file/0J1885/um2884-a-guide-to-using-the-vl53l5cx-multizone-timeofflight-ranging-sensor-with-wide-field-of-view-ultra-lite-driver-uld-stmicroelectronics.pdf)). Two ESP32-relevant integration facts ([SparkFun hookup guide](https://learn.sparkfun.com/tutorials/qwiic-tof-imager---vl53l5cx-hookup-guide), [SparkFun forum](https://community.sparkfun.com/t/vl53l5cx-firmware/63818)):
- The sensor's ~90 KB firmware must live in MCU flash and be uploaded over I²C at every power-on — a non-issue for the ESP32's 4 MB flash, but boot takes ~1.4 s at 400 kHz+ I²C (~9.4 s at 100 kHz — run the bus fast).
- SparkFun's Arduino library works on ESP32 (they explicitly recommend ESP32-class boards); an ESP-IDF port of ST's ULD also exists ([RJRP44/VL53L5CX-Library](https://github.com/RJRP44/V53L5CX-Library), [ESP component registry](https://components.espressif.com/components/rjrp44/vl53l5cx/versions/4.0.1/readme?language=en) — note that component is IDF-only, not Arduino).

An 8×8 frame is 64 × 2-byte distances plus status — a few hundred bytes per frame, trivially converted into an 8-column polar histogram. This is effectively a 15 Hz depth camera for $33 with near-zero CPU cost. The newer **VL53L8CX** is the same 8×8/65° concept with improved ranging and an SPI option; ESP-IDF drivers exist in the same family, but Arduino-ecosystem support is currently thinner, so the L5CX is the pragmatic choice today.

**LD19 / LDROBOT D500 and RPLidar C1 (cheap 2-D spinning lidars).** The LD19 (sold as "D300/D500 kit", internally STL-19P) is a DToF 360° scanner: 4500 samples/s, 10 Hz rotation, 12 m range, one-way UART at 230400 baud — it just streams; ground the PWM pin and parse ([LD19 development manual](https://www.elecrow.com/download/product/SLD06360F/LD19_Development%20Manual_V2.3.pdf), [LudovaTech ESP32/Arduino tutorial](https://github.com/LudovaTech/lidar-LD19-tutorial), kit pricing: [$50](https://witmotion-sensor.com/products/ldrobot-d500-lidar-kit-dtof-laser-radar-lidar-scanner-360-30000lux-5000hz-support-ros1-ros2-for-indoor-and-outdoor)–[$99](https://www.robotshop.com/products/hiwonder-ld19-d500-lidar-developer-kit-360-dtof-laser-scanner-supports-ros1-2-raspberry-pi-jetson-nano)). The SLAMTEC RPLidar C1 is similar (12 m, 5000 samples/s, 10 Hz, UART 460800, motor controlled over UART, ~$69) and has working ESP32 Arduino libraries ([Waveshare wiki](https://www.waveshare.com/wiki/RPLIDAR_C1), [kaiaai/LDS C1 support](https://github.com/kaiaai/LDS/issues/4), [Arduino forum + Robotto lib](https://forum.arduino.cc/t/rplidar-c1-arduino-c-code-needed/1433682)). Burden on ESP32: continuous UART parsing (dedicate core 0), ~2–4 KB scan buffer, 5 V @ hundreds of mA, ~42–110 g. Verdict: superb 360° awareness for a *ground* flagship rover and the natural VFH input; too heavy/power-hungry for small drones, and 10 Hz scan rate (not compute) becomes the latency floor.

**PMW3901 (optical flow).** Not an obstacle sensor — it measures ego-velocity over the ground (X/Y pixel deltas), 80 mm–∞ working range, 42° FoV, 4-wire SPI @ 2 MHz, ~121 FPS internally, 6–9 mA ([PixArt datasheet via Bitcraze](https://wiki.bitcraze.io/_media/projects:crazyflie2:expansionboards:pot0189-pmw3901mb-txqt-ds-r1.00-200317_20170331160807_public.pdf), [Pimoroni breakout](https://shop.pimoroni.com/en-us/products/pmw3901-optical-flow-sensor-breakout), [PX4 docs](https://docs.px4.io/main/en/sensor/pmw3901)). Official Bitcraze Arduino driver works on ESP32 ([bitcraze/Bitcraze_PMW3901](https://github.com/bitcraze/Bitcraze_PMW3901), [ESP32 wiring tutorial](https://circuitdigest.com/microcontroller-projects/interfacing-pmw3901-optical-flow-sensor-with-esp32)). **Why it matters here:** every velocity-space avoidance method (DWA, ORCA, BVC, CBF) needs a velocity estimate; for drones without GPS/mocap, PMW3901 + a downward VL53L1X is the proven minimal odometry stack (it is exactly the Crazyflie Flow deck used in the SGBA swarm). Pair with the IMU in a complementary/Kalman filter.

**HLK-LD2450 (24 GHz mmWave radar).** Tracks up to 3 *moving* targets to 6 m, reporting Cartesian X/Y and radial speed per target at UART 256000 (5 V, ≥200 mA, 3.3 V logic) ([Hi-Link product page](https://www.hlktech.com/en/Goods-226.html), [module manual](https://www.laskakit.cz/user/related_files/hlk-ld2450_1t2r_motion_target_detection_and_tracking_module_manual__v1-0.pdf), [ESP32 integration notes](https://www.espboards.dev/sensors/ld2450/), [ESPHome component](https://github.com/hareeshmu/esphome-docs/blob/ld2450/components/sensor/ld2450.rst)). Strengths: sees through dust/low light, gives *velocity* directly, dirt cheap. Weaknesses: firmware is tuned for tracking *humans indoors*; it filters static objects, so it cannot detect walls; multi-radar mutual interference in a dense swarm is uncharacterized. Verdict: interesting *complement* for detecting moving agents (people, peer robots) crossing a vehicle's path — not a primary obstacle sensor.

**ESP32-CAM + TinyML.** Espressif's own benchmark: TFLite-Micro person detection (96×96 int8) takes **4084 ms without / 380 ms with ESP-NN** optimized kernels on a plain ESP32 @ 240 MHz; the ESP32-S3 does it in 54 ms ([esp-tflite-micro performance table](https://components.espressif.com/components/espressif/esp-tflite-micro/versions/1.3.5/readme)). Independent builds confirm ~400–500 ms in practice ([i2ds-sentinel](https://github.com/edward62740/i2ds-sentinel)). Model ≈ 300 KB flash + ~100 KB tensor arena (needs PSRAM) ([ESP32-CAM person-detection walkthrough](https://zbotic.in/ai-tinyml-with-person-detection-on-esp32-cam-offline/)). At 2–3 FPS with no metric depth, this cannot anchor obstacle avoidance on the target board. See §5 for the fuller TinyML assessment.

---

## 3. Reactive Avoidance Algorithms (single vehicle vs. environment)

For each: what it is, computational/memory cost, and an ESP32 feasibility verdict.

### 3.1 Artificial Potential Fields (APF)

**Description.** Goal exerts attraction, obstacles exert repulsion (typically inverse-square within an influence radius); command the vector sum. The classic entry-level reactive controller.

**Complexity/memory.** O(number of obstacle points) per tick — with 64 ToF zones or even a 450-point lidar scan, this is microseconds of float math on the ESP32's FPU. Memory: negligible (no map needed for the pure reactive form).

**Known failure modes.** Well documented: **local minima** (attractive and repulsive forces cancel → robot stalls, especially in U-shaped obstacles), oscillation in narrow corridors/near obstacles, strong sensitivity to gain tuning, goals-near-obstacles unreachable. These limitations and the 2023–2026 fixes (virtual obstacles/virtual hill escape, rotational fields, hybridization with sampling planners) are surveyed in: [Engineering Research Express review, 2026](https://doi.org/10.1088/2631-8695/ae73e7); [improved-APF via local path information, 2024](https://doi.org/10.1177/17298806241278172); [APF applications survey](https://doi.org/10.54097/7zg0w183); [enhanced virtual-hill escape, Applied Sciences 2024](https://doi.org/10.3390/app14188292); [APF-CPRM fusion, 2024](https://doi.org/10.1109/icpics62053.2024.10796201).

**ESP32 verdict: trivially feasible; use only as a component.** Fine as the inter-agent repulsion term or as a first prototype, but do not ship it as the sole avoidance layer — local minima will strand vehicles in cluttered rooms. A wall-following escape mode (i.e., a bug behavior, §3.4) is the standard cheap remedy.

### 3.2 Vector Field Histogram (VFH / VFH+)

**Description.** Reduce range data to a 1-D **polar histogram** of obstacle density around the robot (e.g., 64–144 angular sectors), threshold to find candidate open "valleys," and steer toward the valley closest to the goal heading. VFH+ adds robot-width inflation, masking of directions unreachable given kinematics, and hysteresis for smoother selection.

**Complexity/memory.** Per tick: one pass over range readings to fill the histogram + one pass over sectors — O(readings + sectors). Historical benchmark: full VFH ran in **27 ms on late-1980s/1990s hardware** vs. ~250 ms for DWA on comparable-era robots ([LIACS thesis survey](https://theses.liacs.nl/pdf/2017-2018-SmitsT.pdf)); on a 240 MHz ESP32 with 64 ToF zones the tick cost is tens of microseconds. Memory: the histogram itself is <1 KB; a full VFH occupancy grid (optional — you can feed instantaneous readings directly) at 100×100 cells × 1 byte = 10 KB, easily within SRAM.

**Robustness evidence.** In a comparative implementation study on the Khepera IV (STM32-class educational robot), VFH was the only algorithm that failed in none of the test scenarios and was judged robust with acceptable computational cost, while DWA was found more costly and "advised against" on that hardware ([EPFL DISAL student project report](https://disalw3.epfl.ch/teaching/student_projects/ay_2020-21/ws/DISAL-SP143_summary.pdf)).

**Limitations.** Purely local (can still dither between valleys — mitigated by VFH+ hysteresis); no explicit dynamics/velocity reasoning; needs decent angular coverage from the sensors (an 8×8 ToF gives 8 forward sectors over 63°; a 2-D lidar gives all 360°).

**ESP32 verdict: feasible with large margin — the recommended primary algorithm.** Maps naturally onto both the VL53L5CX (columns → sectors) and the LD19/C1 (bins → sectors).

### 3.3 Dynamic Window Approach (DWA)

**Description.** Sample the space of (v, ω) commands reachable within one control period given acceleration limits ("dynamic window"), forward-simulate each candidate's arc, score by clearance + heading + speed, pick the best. A one-step model-predictive controller.

**Complexity/memory.** O(velocity samples × trajectory points × obstacle checks) per tick. A modest grid (e.g., 10 v × 20 ω × 20 points × 64 zone checks ≈ 256 k distance evaluations) is milliseconds on the ESP32 FPU — feasible but 10–100× the cost of VFH. The dominant cost is exactly this iterative arc-point collision checking; a 2023 reformulation with non-discrete (arc-based) path representation exists specifically to cut it ([Mathematics 11(21):4424, 2023](https://doi.org/10.3390/math11214424)). Historical and embedded comparisons consistently find DWA slower and more tuning-sensitive than VFH on small robots ([EPFL Khepera IV study](https://disalw3.epfl.ch/teaching/student_projects/ay_2020-21/ws/DISAL-SP143_summary.pdf), [LIACS thesis](https://theses.liacs.nl/pdf/2017-2018-SmitsT.pdf)).

**ESP32 verdict: feasible (dedicate core 1, ~20–50 Hz with coarse sampling), but not the best first choice.** Its advantage — respecting acceleration limits — matters most for fast, heavy vehicles. For our small vehicles, VFH + a velocity governor achieves similar safety with a fraction of the complexity. Revisit if the ground vehicles get fast enough that kinodynamic feasibility becomes the failure mode.

### 3.4 Bug Algorithms (Bug0/1/2, wall-following)

**Description.** Move toward the goal; on encountering an obstacle, follow its boundary until a leave condition (progress toward goal restored) is met. Requires only 1-D range sensing + heading + coarse odometry. Provably complete (Bug1/Bug2) under idealized sensing.

**Complexity/memory.** Essentially zero — a small finite-state machine and a couple of stored positions. This is the most resource-frugal navigation family known; TU Delft published a systematic comparative study of bug variants specifically motivated by tiny-robot constraints ([McGuire et al., *A comparative study of bug algorithms for robot navigation*, RAS 2019](https://doi.org/10.1016/j.robot.2019.103261)).

**MCU flight proof.** The **Swarm Gradient Bug Algorithm (SGBA)** ran entirely on the Crazyflie 2.0's STM32F4 with four 1-D laser rangers + optical flow, navigating a swarm of 33 g drones through a real office building and back ([Science Robotics 2019](https://www.science.org/doi/10.1126/scirobotics.aaw9710), [open-access PDF](https://repository.ubn.ru.nl/bitstream/handle/2066/214783/214783.pdf), [code](https://github.com/tudelft/SGBA_code_SR_2019)).

**Limitations.** Paths are far from optimal; boundary-following is slow; leave-condition tuning matters; degrades with bad odometry.

**ESP32 verdict: trivially feasible — keep as the escape/fallback behavior** (local-minimum escape for APF/VFH, and lost-comms degraded mode), not as the primary planner.

### 3.5 Control Barrier Functions (CBFs)

**Description.** Encode safety as h(x) ≥ 0 (e.g., distance to obstacle minus margin) and filter a nominal command through the optimization "minimally modify u so that ḣ ≥ −α(h)". Usually a small quadratic program (QP); gives formal forward-invariance (safety) guarantees rather than heuristics.

**MCU evidence.** In 2025, NTNU-ARL integrated a **composite CBF** safety filter — collapsing all range-sensor points into a single scalar constraint with an **analytical (closed-form) QP solution** — directly into the PX4 autopilot's cascaded controller, running embedded on flight-controller-class hardware and filtering acceleration setpoints from raw onboard range measurements ([Misyats et al., ICUAS 2025, arXiv:2504.15850](https://arxiv.org/html/2504.15850v1), [Harms et al., ICRA 2025, arXiv:2502.04101](https://arxiv.org/pdf/2502.04101), [code: ntnu-arl/composite_cbf](https://github.com/ntnu-arl/composite_cbf), [PX4 fork: ntnu-arl/PX4-CBF](https://github.com/ntnu-arl/PX4-CBF), [project docs](https://ntnu-arl.github.io/unified_autonomy_stack/cbf/)).

**Complexity/memory.** The general CBF-QP with many constraints needs an embedded QP solver — heavy and jitter-prone for an ESP32 control loop. The composite/analytic formulation avoids the solver entirely: per tick it is one pass over range points (weights + gradient) plus closed-form algebra — comparable to APF cost, with actual guarantees.

**ESP32 verdict: feasible in the analytic/composite form; overkill for v1 but the right upgrade path.** A single-constraint velocity-space CBF ("don't let closing speed toward nearest obstacle exceed what braking allows") is a few lines of float math and makes an excellent hard safety layer under VFH. Avoid iterative QP solvers on this MCU.

---

## 4. Inter-Agent Collision Avoidance in the Swarm

Our swarm setting: each vehicle knows (at best) its own velocity estimate and receives neighbors' states over nRF24L01 broadcasts. Whatever we choose must run per-agent, per-tick, for k ≈ 2–10 relevant neighbors.

### 4.1 Velocity Obstacles: VO / RVO / ORCA

**Description.** Each neighbor induces a cone of ego-velocities leading to collision within a horizon; ORCA linearizes each into a half-plane constraint and picks the velocity closest to the preferred one via a low-dimensional **linear program**, with each agent taking half the avoidance responsibility (reciprocity prevents oscillation) ([ORCA project page](https://gamma.cs.unc.edu/ORCA/), [RVO2 C++ library](https://github.com/snape/RVO2)).

**Computational cost.** Per agent per tick: build k half-planes (a handful of float ops each) + solve a 2-D incremental LP over those k constraints — **O(k)** in practice; global simulations handle thousands of agents in milliseconds on desktop CPUs ([RVO2](https://gamma.cs.unc.edu/RVO2/), [an implementation's documented internals](https://docs.rs/raasta/latest/src/raasta/rvo.rs.html)). For k ≤ 10 this is trivially within ESP32 budget (well under 100 µs). Recent applied work continues to build on ORCA for dense UAV traffic ([ORCA-A* hybrid, SESAR 2024](https://www.sesarju.eu/sites/default/files/documents/sid/2024/papers/SIDs_2024_paper_059%20final.pdf)).

**Caveats for us.** ORCA needs neighbors' *velocities* (radio-shared or estimated), assumes holonomic agents (NH-ORCA variants exist for differential drive), and its guarantees erode with stale state — relevant given nRF24 packet loss. Crazyflie-scale experiments comparing MPC-ORCA and MPC-CBF exist but ran the optimization off-board under motion capture ([JINT 2024 study on Crazyflie 2.1](https://doi.org/10.1007/s10846-024-02202-3)) — evidence of algorithm quality, not of on-MCU deployment.

**ESP32 verdict: feasible (2-D LP is cheap); second choice after BVC** because it demands velocity exchange and more careful handling of packet loss.

### 4.2 Buffered Voronoi Cells (BVC) — proven on an MCU weaker than ours

**Description.** Each robot computes its Voronoi cell w.r.t. neighbor *positions*, shrinks (buffers) it by the robot radius, and constrains its motion/setpoint to remain inside. Cells are disjoint ⇒ collision-free by construction. **O(k)** per tick, same asymptotics as ORCA, but requires **only relative positions — no velocity exchange** ([Zhou, Wang, Bandyopadhyay, Schwager, RA-L 2017](https://doi.org/10.1109/lra.2017.2656241), [PDF](https://msl.stanford.edu/papers/zhou_fast_2017.pdf)). Uncertainty-aware extensions exist ([B-UAVC, Auton. Robots 2022](https://autonomousrobots.nl/assets/files/publications/22_zhu_auro.pdf)).

**The key MCU feasibility evidence:** Bitcraze ships **onboard BVC ("BVCA") in the production Crazyflie firmware**, running on an STM32F405 (Cortex-M4 @ 168 MHz — slower than one ESP32 core) as a setpoint filter between commander and controller, using peer positions received over the shared radio channel ([collision_avoidance.c source](https://github.com/bitcraze/crazyflie-firmware/blob/2022.09/src/modules/src/collision_avoidance.c), [PR #628 with real-hardware validation](https://github.com/bitcraze/crazyflie-firmware/pull/628)). Design notes from that implementation worth stealing: it is suitable for low/medium spatial contention; it modifies setpoints without the commander knowing; and aggressive trajectory-tracking controllers can go unstable when the filtered setpoint jumps — a plain PID handles it well.

**ESP32 verdict: feasible with strong precedent — the recommended inter-agent layer.** Maps directly onto our architecture: nRF24L01 position broadcasts → neighbor table → per-tick cell projection of the velocity/position setpoint.

### 4.3 Rule-based / right-of-way schemes

**Description.** Priority rules ("lower ID yields", "always dodge right", altitude/lane separation for drones) resolved with 1-bit coordination or none. The SGBA swarm used exactly this class: drones broadcast presence over the radio and applied simple right-of-way + preferred-direction rules to avoid each other and de-conflict exploration directions, entirely on the STM32F4 ([Science Robotics 2019](https://www.science.org/doi/10.1126/scirobotics.aaw9710), [PDF](https://repository.ubn.ru.nl/bitstream/handle/2066/214783/214783.pdf)).

**Cost:** near-zero. **Weaknesses:** no guarantees in dense conflicts; deadlock/livelock possible in symmetric encounters; degrades unpredictably beyond a few simultaneous participants.

**ESP32 verdict: trivially feasible — use as the comms-degraded fallback** (e.g., on prolonged nRF24 packet loss: drop speed, apply "dodge right + lower-ID-yields"), and for drones add static altitude offsets per agent ID as a free vertical separation layer.

### 4.4 What the swarm layer needs from the radio

BVC/ORCA quality is bounded by neighbor-state freshness. nRF24L01 payloads (32 B) comfortably carry id + x/y(/z) + v + heading + timestamp; a 5–20 Hz broadcast per agent is enough for BVC at our speeds (position error from staleness at 1 m/s and 100 ms age is 0.1 m — fold it into the buffer radius). Design rule: **inflate the BVC buffer by v·(max expected packet age)** and enforce a speed cap tied to time-since-last-neighbor-update.

---

## 5. Lightweight Learned Approaches (TinyML) — honest assessment

**State of the art on nano-drones.** The PULP-Dronet line runs vision-based navigation (collision probability + steering angle from a forward camera) fully onboard 27–33 g drones — but on a **GAP8**: a 9-core parallel ultra-low-power RISC-V SoC purpose-built for embedded CNN inference ([pulp-platform/pulp-dronet](https://github.com/pulp-platform/pulp-dronet/)). Evolution: PULP-Dronet v2 was 320 KB of weights / ~41 MMAC / 19 FPS (peak RAM ~400 KB, nearly filling GAP8's 512 KB L2) ([Tiny-PULP-Dronets, AICAS 2022, arXiv:2407.02405](https://arxiv.org/abs/2407.02405)); the 2024 distillation work produced **Tiny-PULP-Dronet v3: 2.9 KB of parameters, 1.1 MMAC, 139 FPS on GAP8, 0.7 mJ/inference**, navigating obstacle-lined corridors at 0.5 m/s (and 60 % success against dynamic obstacles at 1.5 m/s) ([IEEE IoT Journal 2024](https://doi.org/10.1109/jiot.2024.3431913), [paper summary](https://api.emergentmind.com/papers/2407.12675)).

**What transfers to ESP32, and what doesn't.**
- *Transfers:* the methodology — int8 quantization, aggressive distillation, sub-100 KB (even sub-10 KB) task-specific networks, and the insight that dataset quality matters more than model size. A 2.9 KB network is small enough that raw parameter storage is a non-issue anywhere.
- *Doesn't transfer:* throughput. GAP8 reaches 139 FPS via 8 parallel compute cores with NN-tuned ISA and an autotuned deployment pipeline (NEMO/DORY). The plain ESP32's measured reference point is **380 ms (2.6 FPS) for a 96×96 int8 person-detection CNN with ESP-NN**, 4 s without ([Espressif esp-tflite-micro benchmarks](https://components.espressif.com/components/espressif/esp-tflite-micro/versions/1.3.5/readme)). Even granting a 10× smaller network, a busy camera pipeline (capture + preprocess + inference) competing with control, radio, and sensor tasks on two cores yields perhaps 10–20 FPS with high jitter — while consuming most of the machine.

**Verdict: not worth it on the ESP32 DOIT DevKit v1 for obstacle avoidance.** A VL53L5CX gives metric distances at 15–60 Hz for less CPU than decoding one camera frame; classical geometry on that data is deterministic, debuggable, and swarm-composable. Two situations change the answer: (a) migrating to **ESP32-S3** (54 ms/inference benchmark, vector instructions) where a Tiny-PULP-Dronet-style ~3–30 KB net becomes a plausible *semantic* add-on (e.g., "person ahead" classification feeding the same VFH layer); (b) tasks classical sensors can't do at all (visual classification of obstacle types). Learned *control policies* on top of cheap range sensors (rather than vision) are also a possible later experiment — inference of a tiny MLP on 64 ToF zones costs microseconds — but they still need a training pipeline and offer little over VFH+BVC at this stage.

---

## 6. Latency, Control Loop, and Safe Speed on ESP32

The avoidance loop's total reaction latency is:

```
t_react = t_sensor_age + t_transfer + t_algorithm + t_control + t_actuation
```

Representative numbers for this platform:

| Component | Budget | Basis |
|---|---|---|
| VL53L5CX 8×8 frame age | ≤ 67 ms (15 Hz) + integration; use ~70–100 ms worst case | ST datasheet/UM2884 rates ([UM2884](https://www.pololu.com/file/0J1885/um2884-a-guide-to-using-the-vl53l5cx-multizone-timeofflight-ranging-sensor-with-wide-field-of-view-ultra-lite-driver-uld-stmicroelectronics.pdf)) |
| VL53L5CX 4×4 frame age | ≤ 17 ms (60 Hz) | same |
| I²C readout, 8×8 result | ~2–5 ms @ 400 kHz (hundreds of bytes) | bus math; run I²C at 1 MHz to shrink ([datasheet](https://cdn.sparkfun.com/assets/6/e/3/0/6/vl53l5cx-datasheet.pdf)) |
| 2-D lidar (LD19/C1) scan age | up to 100 ms (10 Hz rotation) | LD19 manual / Waveshare specs |
| HC-SR04, 4 sensors sequential | 240 ms full sweep (60 ms each) | datasheet cycle recommendation |
| VFH histogram + steering tick | ≪ 1 ms on 240 MHz ESP32 | O(64 zones + sectors) float ops (§3.2; historical 27 ms on 1990s hardware, [LIACS](https://theses.liacs.nl/pdf/2017-2018-SmitsT.pdf)) |
| BVC/ORCA tick, k ≤ 10 neighbors | ≪ 1 ms | O(k) (§4.1–4.2) |
| Neighbor state age (nRF24, 10 Hz) | ≤ 100 ms + loss margin | design choice (§4.4) |
| Servo/ESC actuation | one 50 Hz servo period ≈ 20 ms (ESC faster) | standard RC PWM |

So the **environment-avoidance reaction time is dominated by the sensor: ~100–130 ms** with the 8×8 ToF (or ~40–50 ms in 4×4 @ 60 Hz mode); algorithm cost is noise. Required sensing range for speed v:

```
d_needed = v · t_react  +  v² / (2a_brake)  +  margin
```

With t_react = 0.13 s, margin = 0.2 m:

| Vehicle | v | a_brake (est.) | d_needed | Feasible with 4 m ToF? |
|---|---|---|---|---|
| Ground car | 1 m/s | 3 m/s² | 0.13 + 0.17 + 0.2 ≈ **0.5 m** | Yes, ample |
| Ground car | 2 m/s | 3 m/s² | 0.26 + 0.67 + 0.2 ≈ **1.1 m** | Yes |
| Ground car | 3 m/s | 3 m/s² | 0.39 + 1.5 + 0.2 ≈ **2.1 m** | Marginal (low-reflectance targets range < 4 m) |
| Drone | 0.5 m/s | 2 m/s² (gentle) | 0.07 + 0.06 + 0.2 ≈ **0.33 m** | Yes |
| Drone | 1.5 m/s | 2 m/s² | 0.20 + 0.56 + 0.2 ≈ **1.0 m** | Yes, if FoV covers the velocity vector |

Cross-checks from real MCU-class systems: the SGBA Crazyflie swarm flew at **~0.5 m/s** on four 1-D ToF rangers ([Science Robotics 2019](https://www.science.org/doi/10.1126/scirobotics.aaw9710)); Tiny-PULP-Dronet v3 navigated at **0.5 m/s** target speed (1.5 m/s only with degraded success) ([IoT-J 2024](https://doi.org/10.1109/jiot.2024.3431913)). Our multizone sensor is richer than SGBA's, so **1–2 m/s ground / 0.5–1 m/s air are defensible v1 envelopes**, enforced by a speed governor: `v_max = f(d_min_ahead)` so the vehicle can always stop within currently-seen free space.

**Loop architecture on the two cores (firmware-phase guidance):** core 1 runs the 100 Hz control task (governor + VFH steering + BVC setpoint filter + reflex stop); core 0 runs sensor I/O (I²C/UART DMA parsing), nRF24 neighbor table, and telemetry. Timestamp every sensor frame and neighbor packet; the safety layer treats data older than a threshold as absent (and slows down accordingly).

---

## 7. Recommendation for the First Implementation

### 7.1 Sensor suite

**Ground vehicle (v1):**
- 1× **VL53L5CX** forward (8×8 @ 15 Hz normally; switch to 4×4 @ 60 Hz above ~1.5 m/s) — primary avoidance input. ~$33.
- 2× **VL53L1X** angled left/right (50 Hz) for shoulder coverage beyond the 63° cone. ~$10–15 ea.
- Wheel odometry (encoders if available) + IMU for velocity estimate feeding BVC and the governor.
- Optional: 1× HC-SR04 rear for reversing (accepting its swarm-crosstalk limits).

**Drone (v1):**
- 1× **VL53L5CX** forward + 1× **VL53L1X** downward (altitude) + **PMW3901** downward (velocity) — this mirrors the proven Crazyflie Flow + ranging stack.
- Static per-agent altitude offsets as a free extra inter-agent separation layer.

**Flagship/rover option:** **LD19/D500 (~$50) or RPLidar C1 (~$69)** on one ground unit for 360° awareness and mapping experiments; parse on core 0. Not on drones (weight/power).

### 7.2 Algorithm stack (three layers, top-down)

1. **Goal/behavior layer** (mission logic, later phase) emits a preferred velocity.
2. **VFH-style polar histogram** over the fused ToF zones picks the best admissible heading + speed (with robot-width inflation and valley hysteresis à la VFH+). Local-minimum escape: switch to **wall-following (bug) mode** when progress stalls.
3. **BVC inter-agent filter**: project the commanded velocity into the buffered Voronoi cell computed from nRF24-shared neighbor positions (buffer inflated by v · max packet age). Crazyflie's `collision_avoidance.c` is the reference implementation to port.
4. **Reflex layer** (runs every control tick, overrides everything): if min forward distance < stopping distance at current speed → brake/hover; speed governor `v_max = f(d_min)` at all times. This is a degenerate one-constraint CBF and can later be upgraded to the analytic composite-CBF filter (§3.5).

### 7.3 Fallback options

| Risk | Fallback |
|---|---|
| VL53L5CX FoV/range insufficient outdoors (sunlight shortens ToF range) | LD2450 mmWave for moving-obstacle detection; lower speed cap; lidar on ground units |
| nRF24 congestion/loss degrades BVC | Rule-based right-of-way (lower ID yields, dodge right) + hard speed reduction on stale neighbor data; SGBA-style minimal coordination as the model |
| VFH dithering in clutter | VFH+ masking/hysteresis; if still insufficient, coarse DWA on core 1 (§3.3) |
| Arduino-framework friction with the L5CX ULD | SparkFun library first; if it underperforms, wrap ST's ULD directly (RJRP44's IDF port shows the shape) or move to ESP-IDF with Arduino as a component |
| Later need for semantic perception | ESP32-S3 co-processor or upgrade (54 ms/inference benchmark) running a Tiny-PULP-Dronet-style distilled CNN feeding the same VFH layer |

### 7.4 Suggested firmware-phase validation order

1. Bench: VL53L5CX 8×8 → polar histogram visualization over serial; measure real frame age and I²C cost at 400 kHz vs 1 MHz.
2. Single ground vehicle: reflex layer + governor only (drive at wall, verify stop margin at 1 m/s and 2 m/s).
3. Add VFH steering; corridor + slalom tests; tune sector inflation/hysteresis.
4. Two vehicles: nRF24 state broadcast + BVC filter; head-on and crossing encounters at 0.5 → 1.5 m/s.
5. Add bug-mode escape; U-shaped obstacle test.
6. Port stack to drone (PMW3901 velocity estimate replaces wheel odometry) at 0.5 m/s.

---

## 8. References

**Sensors**
- ST VL53L5CX product page — https://www.st.com/en/imaging-and-photonics-solutions/vl53l5cx.html
- VL53L5CX datasheet — https://cdn.sparkfun.com/assets/6/e/3/0/6/vl53l5cx-datasheet.pdf
- UM2884 VL53L5CX ULD user manual — https://www.pololu.com/file/0J1885/um2884-a-guide-to-using-the-vl53l5cx-multizone-timeofflight-ranging-sensor-with-wide-field-of-view-ultra-lite-driver-uld-stmicroelectronics.pdf
- SparkFun VL53L5CX hookup guide — https://learn.sparkfun.com/tutorials/qwiic-tof-imager---vl53l5cx-hookup-guide
- SparkFun Qwiic ToF Imager product page ($32.50) — https://www.sparkfun.com/sparkfun-qwiic-tof-imager-vl53l5cx.html
- SparkFun forum: L5CX 90 KB firmware upload timing — https://community.sparkfun.com/t/vl53l5cx-firmware/63818
- RJRP44 VL53L5CX ESP-IDF library — https://github.com/RJRP44/V53L5CX-Library ; https://components.espressif.com/components/rjrp44/vl53l5cx/versions/4.0.1/readme?language=en
- ST VL53L1X datasheet — https://www.st.com/resource/en/datasheet/vl53l1x.pdf ; product page — https://www.st.com/en/imaging-and-photonics-solutions/vl53l1x.html?icmp=tt16126_gl_pron_jul2020
- Pololu VL53L1X Arduino library — https://github.com/pololu/vl53l1x-arduino
- HC-SR04 datasheet (SparkFun/DigiKey) — https://www.digikey.com/htmldatasheets/production/1979760/0/0/1/hc-sr04.html
- HC-SR04 user guide (Handson Tech) — https://www.handsontec.com/pdf_files/hc-sr04-User-Guide.pdf
- HC-SR04 multi-sensor usage (DroneBot Workshop) — https://dronebotworkshop.com/hc-sr04-ultrasonic-distance-sensor-arduino/
- Sharp GP2Y0A21YK0F datasheet — https://www.micro-semiconductor.jp/datasheet/bc-GP2Y0A21YK0F.pdf ; Pololu product page — https://www.pololu.com/product/136
- LD19 development manual (Elecrow PDF) — https://www.elecrow.com/download/product/SLD06360F/LD19_Development%20Manual_V2.3.pdf
- LD19 ESP32/Arduino tutorial (LudovaTech) — https://github.com/LudovaTech/lidar-LD19-tutorial
- LD19/D500 pricing — https://witmotion-sensor.com/products/ldrobot-d500-lidar-kit-dtof-laser-radar-lidar-scanner-360-30000lux-5000hz-support-ros1-ros2-for-indoor-and-outdoor ; https://www.sensorlidar.com/products/ldrobot-d500-lidar-kit-tof-laser-radar-lidar-scanner-360-30000lux-support-ros1-ros2-for-indoor-and-outdoor-replace-d300-kit ; https://www.robotshop.com/products/hiwonder-ld19-d500-lidar-developer-kit-360-dtof-laser-scanner-supports-ros1-2-raspberry-pi-jetson-nano ; https://www.amazon.com/LDROBOT-Outdoor-Navigation-Scanning-Support/dp/B0DDKXQ23R
- RPLidar C1 specs (Waveshare wiki) — https://www.waveshare.com/wiki/RPLIDAR_C1
- RPLidar C1 on ESP32 (kaiaai/LDS) — https://github.com/kaiaai/LDS/issues/4 ; Arduino forum / Robotto lib — https://forum.arduino.cc/t/rplidar-c1-arduino-c-code-needed/1433682
- PMW3901 datasheet (Bitcraze mirror) — https://wiki.bitcraze.io/_media/projects:crazyflie2:expansionboards:pot0189-pmw3901mb-txqt-ds-r1.00-200317_20170331160807_public.pdf
- Bitcraze PMW3901 Arduino driver — https://github.com/bitcraze/Bitcraze_PMW3901 ; ESP32 tutorial — https://circuitdigest.com/microcontroller-projects/interfacing-pmw3901-optical-flow-sensor-with-esp32 ; PX4 flow sensor docs — https://docs.px4.io/main/en/sensor/pmw3901 ; Pimoroni breakout — https://shop.pimoroni.com/en-us/products/pmw3901-optical-flow-sensor-breakout
- HLK-LD2450 product page — https://www.hlktech.com/en/Goods-226.html ; module manual — https://www.laskakit.cz/user/related_files/hlk-ld2450_1t2r_motion_target_detection_and_tracking_module_manual__v1-0.pdf ; ESP32 notes — https://www.espboards.dev/sensors/ld2450/ ; ESPHome component — https://github.com/hareeshmu/esphome-docs/blob/ld2450/components/sensor/ld2450.rst ; ComponentIndex page — https://componentindex.net/components/ld2450/

**Reactive algorithms**
- APF review 2026 (Engineering Research Express) — https://doi.org/10.1088/2631-8695/ae73e7
- Improved APF via local path information (2024) — https://doi.org/10.1177/17298806241278172
- APF applications survey — https://doi.org/10.54097/7zg0w183
- Enhanced virtual-hill local-minimum escape (Applied Sciences 2024) — https://doi.org/10.3390/app14188292
- APF-CPRM fusion (2024) — https://doi.org/10.1109/icpics62053.2024.10796201
- VFH vs DWA on Khepera IV (EPFL DISAL project) — https://disalw3.epfl.ch/teaching/student_projects/ay_2020-21/ws/DISAL-SP143_summary.pdf
- VFH/DWA background & timing survey (LIACS thesis) — https://theses.liacs.nl/pdf/2017-2018-SmitsT.pdf
- Faster DWA via non-discrete path representation (Mathematics 2023) — https://doi.org/10.3390/math11214424
- Comparative study of bug algorithms (McGuire et al., RAS 2019) — https://doi.org/10.1016/j.robot.2019.103261
- Embedded CBF safety filter in PX4 (ICUAS 2025) — https://arxiv.org/html/2504.15850v1
- Composite CBFs for quadrotor navigation (ICRA 2025) — https://arxiv.org/pdf/2502.04101
- Composite CBF code — https://github.com/ntnu-arl/composite_cbf ; docs — https://ntnu-arl.github.io/unified_autonomy_stack/cbf/

**Swarm inter-agent avoidance**
- ORCA project page — https://gamma.cs.unc.edu/ORCA/
- RVO2 library — https://github.com/snape/RVO2 ; https://gamma.cs.unc.edu/RVO2/
- ORCA implementation internals (O(k) LP, spatial hashing) — https://docs.rs/raasta/latest/src/raasta/rvo.rs.html
- ORCA-A* hybrid for drone traffic (SESAR 2024) — https://www.sesarju.eu/sites/default/files/documents/sid/2024/papers/SIDs_2024_paper_059%20final.pdf
- Buffered Voronoi cells (Zhou et al., RA-L 2017) — https://doi.org/10.1109/lra.2017.2656241 ; PDF — https://msl.stanford.edu/papers/zhou_fast_2017.pdf
- Uncertainty-aware BVC (Zhu et al., Auton. Robots 2022) — https://autonomousrobots.nl/assets/files/publications/22_zhu_auro.pdf
- Crazyflie onboard BVCA source — https://github.com/bitcraze/crazyflie-firmware/blob/2022.09/src/modules/src/collision_avoidance.c ; PR #628 — https://github.com/bitcraze/crazyflie-firmware/pull/628
- MPC-ORCA vs MPC-CBF on Crazyflie 2.1 (JINT 2024) — https://doi.org/10.1007/s10846-024-02202-3
- SGBA swarm exploration (Science Robotics 2019) — https://www.science.org/doi/10.1126/scirobotics.aaw9710 ; open PDF — https://repository.ubn.ru.nl/bitstream/handle/2066/214783/214783.pdf ; code — https://github.com/tudelft/SGBA_code_SR_2019 ; summary — https://hackaday.com/2019/11/06/tiny-drones-navigate-like-real-bugs/

**TinyML**
- PULP-Dronet repository — https://github.com/pulp-platform/pulp-dronet/
- Tiny-PULP-Dronet v3 (IEEE IoT Journal 2024) — https://doi.org/10.1109/jiot.2024.3431913 ; summary — https://api.emergentmind.com/papers/2407.12675
- Tiny-PULP-Dronets (AICAS 2022) — https://arxiv.org/abs/2407.02405
- Espressif esp-tflite-micro benchmarks (person detection: 380 ms ESP32 / 54 ms ESP32-S3 with ESP-NN) — https://components.espressif.com/components/espressif/esp-tflite-micro/versions/1.3.5/readme
- ESP32-CAM person-detection build (~400 ms with ESP-NN) — https://github.com/edward62740/i2ds-sentinel
- ESP32-CAM TinyML walkthrough (model/arena sizes) — https://zbotic.in/ai-tinyml-with-person-detection-on-esp32-cam-offline/
