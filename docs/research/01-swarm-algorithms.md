# Multi-Robot Coordination and Flocking Algorithms on ESP32-Class Microcontrollers

**Research phase document 01 — literature survey**

*Target platform: hobby ESP32-based RC vehicles (240 MHz dual-core Xtensa LX6/LX7, ~520 KB SRAM), PlatformIO + Arduino framework, nRF24L01 radios (32-byte payloads) with ESP-NOW as an alternative (250-byte payloads), hobby ESC/servo outputs, differential/Ackermann ground drive.*

---

## 1. Executive Summary

This survey reviews the algorithm families available for cooperative movement of a small fleet (3–10, up to a few dozen) of microcontroller-class ground vehicles, and assesses each for feasibility on an ESP32 with a lossy, low-bandwidth broadcast radio.

**Key findings:**

1. **Compute is not the bottleneck — sensing and state exchange are.** Every classical flocking/formation algorithm (boids, Vicsek, potential fields, leader–follower, virtual structure, Olfati-Saber consensus flocking) costs `O(N)` floating-point vector arithmetic per control tick for `N` neighbors — microseconds on a 240 MHz dual-core ESP32. The hard part is that all of them need each robot to know the *relative position* (and usually velocity) of its neighbors. On our platform this must come from broadcasting each robot's own pose estimate (odometry/IMU dead reckoning, optionally corrected by external anchors) over the radio, since we have no cameras or UWB ranging.
2. **The strongest existence proof for our hardware class is the Crazyflie ecosystem.** A Crazyflie 2.1 runs its flight stack *plus* swarm ranging *plus* an EKF-based relative localization *plus* formation control on an STM32F4 at 168 MHz with 192 KB RAM — strictly weaker than our ESP32 ([Shan et al., RA-L 2025](https://fengshan.seu-netsi.net/papers/LSLCWC-RAL25.pdf); [Li et al.](https://shushuai3.github.io/autonomous-swarm/)). Vásárhelyi et al.'s 30-drone outdoor flock ([Science Robotics 2018](https://www.science.org/doi/10.1126/scirobotics.aat3536)) also ran its flocking model on embedded onboard computers using only broadcast position/velocity packets — the same information our robots can exchange.
3. **Communication-aware model design matters more than algorithm choice.** Real-world studies show flocking degrades gracefully-to-catastrophically with packet loss: halving packet delivery ratio roughly tripled mission completion time in a swarm mapping study ([Atlas 2.0, ACM SenSys workshop 2021](https://doi.org/10.1145/3485730.3494040)), and lossy links cause flock fragmentation unless controller gains, update period, and loss probability jointly satisfy stability conditions ([Discrete-time flocking with random link failures, IEEE TVT 2024](https://doi.org/10.1109/tvt.2024.3382617)). Vásárhelyi et al. handle this by explicitly modeling delay and limited communication range and then optimizing the model parameters with evolution — the reason that paper is the reference design for "flocking that actually works on real radios."
4. **ESP-NOW measurements are directly encouraging for fleets of our size**: published field studies report >95% packet delivery and <40 ms latency for 5–7 robot meshes indoors ([IIETA JESA 2025](https://iieta.org/journals/jesa/paper/10.18280/jesa.590507)), and useful range up to ~150 m line-of-sight ([FORTEI-ICEE 2024](https://doi.org/10.1109/fortei-icee64706.2024.10824617)). nRF24L01's 32-byte payload comfortably fits an `id + pose + velocity + timestamp` state packet.
5. **Learning-based methods now fit on MCUs, but are a "phase 2" option.** End-to-end RL policies have been flashed onto the Crazyflie's STM32 ([Learning to Fly in Seconds, IEEE RA-L 2024](https://arxiv.org/abs/2311.13081)), 23k-parameter navigation CNNs run at 30 ms/inference on ESP32 ([TinyNav](https://arxiv.org/html/2603.11071v1)), and header-only RL frameworks target ESP32-S3 with PlatformIO ([TinyRL](https://github.com/mohmdelsayed/TinyRL)). But multi-agent RL for flocking is still trained centrally in simulation and faces a sim-to-real gap; rule-based controllers remain the sane first implementation.

**Top recommendation (detail in §7):** start with **(1) a boids/potential-field hybrid tuned per Vásárhelyi's communication-aware methodology**, then **(2) leader–follower formation control** as the first "structured" behavior, and keep **(3) Kilobot-style gradient/hop-count coordination** as the communication-only fallback that needs no positioning at all.

---

## 2. Platform Assumptions and the Sensing Problem

Before assessing algorithms, we fix what a robot on this platform actually knows:

- **Own state:** wheel/ESC odometry + IMU heading (if fitted) → dead-reckoned pose `(x, y, θ)` in a shared frame, drifting over time. No GPS indoors; outdoor GPS is an optional upgrade.
- **Neighbor state:** *only via radio.* nRF24L01 and ESP-NOW provide no angle-of-arrival and only crude RSSI-based range hints. Therefore every algorithm that needs "positions of neighbors" implies each robot broadcasts its own pose estimate at some rate (the approach used by Vásárhelyi et al. 2018, where drones broadcast GNSS position/velocity, and by the Crazyflie P2P swarm demos described on [Bitcraze's research blog](https://www.bitcraze.io/category/research/)).
- **Obstacle sensing:** optional ultrasonic/ToF sensors, as in the ESP32-S3 swarm platform of [Csiszár 2025](https://www.theseus.fi/bitstream/handle/10024/893098/Istvan_Csiszar.pdf?sequence=2).
- **Radio budget:** nRF24L01 at 250 kbps–2 Mbps, 32-byte payloads, no MAC-level mesh unless RF24Network/RF24Mesh is layered on top ([RF24Mesh docs](https://nrf24.github.io/RF24Mesh/index.html)); ESP-NOW at ~1 Mbps effective, 250-byte payloads, true connectionless broadcast.

A practical state packet — `uint8 id, int16 x_cm, int16 y_cm, int16 vx_cm_s, int16 vy_cm_s, int16 heading_mrad, uint16 seq/timestamp` — is **13–16 bytes**. At 10 Hz per robot, 10 robots produce ≤1.6 kB/s of application traffic: trivially within both radios' capacity. Airtime contention, not bandwidth, is the constraint (see §6).

This drives the single most important design conclusion of the survey: **algorithm families divide into those that need metric neighbor state (boids, Vicsek, potential fields, formation control, coverage) and those that only need connectivity/hop-counts (Kilobot-style gradients, firefly sync, consensus on scalars).** The first class needs the broadcast-pose scheme above; the second works even when positioning fails.

---

## 3. Classical Foundations

### 3.1 Reynolds' Boids (separation / alignment / cohesion)

**Reference:** C. W. Reynolds, "Flocks, Herds, and Schools: A Distributed Behavioral Model," SIGGRAPH 1987 — [PDF at red3d.com](https://www.red3d.com/cwr/papers/1987/SIGGRAPH87.pdf).

**Description.** Each agent steers by three local rules evaluated over neighbors within a perception radius: *separation* (steer away from crowding), *alignment* (match average neighbor heading/velocity), *cohesion* (steer toward the neighbor centroid). Emergent polarized, collision-free group motion; no leader, no global state. Reynolds designed it for animation, but it remains the conceptual root of nearly every flocking controller (Olfati-Saber's 2006 paper formally shows its first algorithm "embodies all three rules of Reynolds").

**Math sketch.** For robot `i` with neighbor set `N_i` (positions `q_j`, velocities `p_j`):

```
sep_i =  Σ_{j∈N_i} (q_i − q_j) / |q_i − q_j|²
ali_i = (Σ_{j∈N_i} p_j) / |N_i| − p_i
coh_i = (Σ_{j∈N_i} q_j) / |N_i| − q_i
u_i   = w_s·sep_i + w_a·ali_i + w_c·coh_i          (then clamp to actuator limits)
```

For a car-like robot, `u_i` (a desired velocity vector) is mapped to throttle + steering via a low-level heading controller.

**Compute:** `O(|N_i|)` vector ops per tick; ~30 flops per neighbor. At 20 Hz with 10 neighbors this is negligible (<0.01% of one ESP32 core).
**Memory:** neighbor table of `N × ~20 B` plus 3 gain constants — well under 1 KB.
**Comms:** needs neighbor position + velocity → the 13–16 B state packet at 5–20 Hz. Tolerates stale data for a few ticks because the rules average over neighbors.
**Sensing:** shared-frame pose per robot (broadcast dead reckoning) or relative positions from any ranging system.
**Degradation with packet loss:** graceful in moderate loss (a missing neighbor update just means using the last known state); simulation studies show emergent behavior quality decays steadily with loss and latency and collapses only under heavy degradation ([Zambrano-Martinez et al., WSC 2017 study of degraded comms on emergent behavior](https://doi.org/10.1109/wsc.2017.8248117)).

**ESP32 verdict: ✅ Trivially feasible — the recommended starting point.** The whole controller is a few hundred lines of C++. Risk is not compute but tuning stability with real actuator lag; use Vásárhelyi-style simulation-in-the-loop tuning (§5.3).

### 3.2 Vicsek Model

**Reference:** T. Vicsek, A. Czirók, E. Ben-Jacob, I. Cohen, O. Shochet, "Novel Type of Phase Transition in a System of Self-Driven Particles," Phys. Rev. Lett. 75, 1226 (1995) — [arXiv:cond-mat/0611743](https://arxiv.org/abs/cond-mat/0611743), [DOI 10.1103/PhysRevLett.75.1226](https://doi.org/10.1103/PhysRevLett.75.1226).

**Description.** The statistical-physics minimum of flocking: constant-speed particles that, each step, adopt the *average heading of neighbors within radius r*, plus noise. No separation or cohesion terms — the model studies the order–disorder phase transition of alignment. It is the theoretical ancestor of alignment-based control and of the ELTE drone-flocking line of work (Vicsek is a co-author of both the 2018 Science Robotics paper and the 2020 adaptive-leadership follow-up).

**Math sketch.**

```
θ_i(t+1) = ⟨θ_j⟩_{j: |q_j − q_i| < r}  +  Δθ_noise
q_i(t+1) = q_i(t) + v0 · (cos θ_i, sin θ_i) · Δt
```

**Compute:** `O(|N_i|)` — one circular mean per tick (use `atan2(Σsin, Σcos)`), cheaper than boids.
**Memory:** just neighbor headings; <100 B.
**Comms:** only *heading* needs to be exchanged (2 bytes/robot!) if neighborhood membership is decided by RSSI threshold instead of metric distance. This makes Vicsek the cheapest metric-free-ish flocking rule available.
**Sensing:** heading only (IMU/odometry). Neighborhood radius via RSSI is noisy but acceptable for a demo.
**Degradation:** alignment consensus is robust to symmetric random loss (it is a form of averaging consensus) but constant speed with no separation term means **no collision avoidance** — unusable alone on ground vehicles in a confined space.

**ESP32 verdict: ✅ Feasible as a teaching/demo behavior and as the alignment term inside a hybrid controller; ❌ not sufficient alone** (no separation ⇒ collisions).

### 3.3 Artificial Potential Fields (APF)

**References:** foundational concept surveyed (with UAV-swarm focus) in [Javed et al., "State-of-the-Art Flocking Strategies for the Collective Motion of Multi-Robots," Machines 12(10):739, 2024](https://www.mdpi.com/2075-1702/12/10/739) and [Bandarupalli et al., "Advancement Challenges in UAV Swarm Formation Control: A Comprehensive Review," Drones 8(7):320, 2024](https://doi.org/10.3390/drones8070320); the rigorous multi-agent form is Olfati-Saber's collective potentials (§4.1).

**Description.** Each robot descends the gradient of a potential `U = U_attract(goal) + Σ U_repel(neighbors, obstacles)`. Attractive wells pull toward goals/formation slots; repulsive barriers push away from collisions. Simple, reactive, widely used as the collision-avoidance layer under other coordinators. Known weaknesses: local minima (robot gets stuck where forces cancel) and oscillation in narrow passages — the 2024 reviews above catalog standard fixes (rotational/vortex fields, random perturbation escape, combining with planners).

**Math sketch.** Typical repulsion + quadratic attraction:

```
U_att(q) = ½ k_a |q − q_goal|²
U_rep(q) = ½ k_r (1/d − 1/d0)²   for d < d0, else 0     (d = distance to obstacle/neighbor)
u_i = −∇U = k_a (q_goal − q_i) + Σ k_r (1/d − 1/d0)(1/d²) · d̂
```

**Compute:** `O(|N_i| + obstacles)` per tick; a `sqrt` and a division per interaction — still negligible.
**Memory:** <1 KB (gains + obstacle list).
**Comms:** neighbor positions (same state packet as boids); goal/waypoint broadcast from operator.
**Sensing:** same as boids, plus optional ultrasonic/ToF for unmapped obstacles.
**Degradation:** identical profile to boids; repulsion computed from stale neighbor positions is the main collision risk, so inflate `d0` with expected staleness × speed.

**ESP32 verdict: ✅ Trivially feasible.** Best used as the safety/spacing layer inside every other behavior rather than as the sole coordinator.

### 3.4 Formation Control: Leader–Follower and Virtual Structure

**References:** [Wang et al., "A Survey of Multi-Agent Systems on Distributed Formation Control," Unmanned Systems, 2024](https://doi.org/10.1142/s2301385024500274); [Drones 8(7):320, 2024 review](https://doi.org/10.3390/drones8070320) (covers leader–follower, virtual structure, behavior-based, consensus, APF and AI methods side by side); [Recent Advancement in Formation Control of Multi-Agent Systems: A Review, CMC 83(3), 2025](https://www.techscience.com/cmc/v83n3/61053).

**Description.**
- **Leader–follower:** one robot (or a human-driven RC car!) is the leader; each follower regulates a desired offset `(ρ_des, φ_des)` (distance + bearing) or displacement vector relative to the leader or to a designated parent in a tree. Simple, intuitive, and maps perfectly onto a hobby fleet where one vehicle is manually driven. Weaknesses: single point of failure, error propagation down chains, leader must be tracked reliably.
- **Virtual structure:** the whole formation is treated as one rigid body; a virtual frame `(x_v, y_v, θ_v)` moves along a trajectory and each robot tracks its fixed slot `q_i,des = q_v + R(θ_v)·offset_i`. No physical leader to lose, formation shape is exactly maintained — but someone must compute/broadcast the virtual frame state (mild centralization), and rigid slots are less forgiving of individual robot limits.
- The modern classification (position-/displacement-/distance-/bearing-based, per the Unmanned Systems 2024 survey) is about *what each robot can measure*: displacement-based control needs a shared frame (our broadcast-odometry case); distance-based needs only inter-robot ranges; bearing-based needs only angles.

**Math sketch (displacement-based follower):**

```
e_i = (q_L + R(θ_L)·d_i,des) − q_i        // slot error in shared frame
u_i = K_p e_i + K_d ė_i                    // then map to (v, ω) via unicycle feedback linearization
```

For a virtual structure, replace `q_L, θ_L` by the broadcast virtual-frame state.

**Compute:** `O(1)` per robot per tick (a follower only tracks its parent/frame). Easiest of all families.
**Memory:** slot table + gains, <1 KB.
**Comms:** *lowest requirement of the metric family*: followers need only the leader/frame pose at 10–20 Hz — a single 16-byte broadcast for the whole fleet. Update rate directly bounds tracking error: at leader speed `v` and update period `T`, worst-case slot error ≥ `v·T` (1 m/s @ 10 Hz → 10 cm), so 10–20 Hz is right for RC-car speeds.
**Sensing:** shared-frame odometry; error accumulates with dead-reckoning drift, so periodic re-zeroing or external anchors are needed for long runs.
**Degradation:** leader-packet loss is the critical failure. Standard mitigations: followers coast on the last leader velocity (constant-velocity prediction), and a timeout triggers safe-stop. Event-triggered variants (§5.2) formally reduce leader broadcast rate. Chain topologies amplify noise; star topologies (all followers track the one leader) are preferred at our fleet size.

**ESP32 verdict: ✅ Trivially feasible; leader–follower is the best first *formation* behavior** because a human-driven leader removes the hardest subproblem (autonomous navigation) while exercising the whole comms/estimation stack. Virtual structure is an easy extension once waypoint following works.

---

## 4. Modern Decentralized Coordination

### 4.1 Consensus-Based Flocking (Olfati-Saber framework)

**Reference:** R. Olfati-Saber, "Flocking for Multi-Agent Dynamic Systems: Algorithms and Theory," IEEE Trans. Automatic Control 51(3):401–420, 2006 — [DOI 10.1109/TAC.2005.864190](https://doi.org/10.1109/tac.2005.864190), [PDF mirror at ELTE](https://hal.elte.hu/~vicsek/downloads/papers/flocking_tac06-engineering.pdf). ~5,000 citations; the theoretical backbone of modern flocking.

**Description.** Formalizes Reynolds' rules as gradient descent on a *collective potential* that penalizes deviation from an "α-lattice" (all neighbor distances = d), plus a *velocity consensus* term (the graph-Laplacian alignment that gives the family its name), plus virtual β-agents (projections onto obstacles) for obstacle avoidance and a γ-agent (group objective) for navigation. Provides Lyapunov-style stability guarantees and the famous conclusion that "flocks need no leaders." Its key practical lesson: naive Reynolds rules alone generically *fragment* into subgroups; adding the γ (goal) term is what keeps the flock whole.

**Math sketch (Algorithm 2 of the paper):**

```
u_i = Σ_{j∈N_i} φ_α(‖q_j − q_i‖_σ) n_ij      // lattice-spacing potential term
    + Σ_{j∈N_i} a_ij(q) (p_j − p_i)           // velocity consensus (alignment)
    + c1 (q_r − q_i) + c2 (p_r − p_i)          // γ-agent: group objective feedback
```

where `φ_α` is a smooth finite-range action function and `‖·‖_σ` a smoothed norm.

**Compute:** `O(|N_i|)` with a handful of transcendentals per neighbor (smooth bump functions) — perhaps 2–3× boids cost, still trivial at our scale.
**Memory:** same neighbor table as boids + ~10 parameters.
**Comms:** neighbor position **and velocity** (velocity consensus needs `p_j`), i.e. the full 16-byte state packet, 10–20 Hz.
**Sensing:** as boids.
**Degradation:** best-understood family under loss: discrete-time analyses with Bernoulli link failures give explicit stability conditions coupling controller gains, interaction period, and delivery probability, and show flocking "in expectation" survives moderate loss while fragmentation risk grows with hop count and loss rate ([IEEE TVT 2024](https://doi.org/10.1109/tvt.2024.3382617); [flocking fragmentation under multi-hop lossy networks, FITEE 2024](https://journal.hep.com.cn/fitee/EN/10.1631/FITEE.2300295)).

**ESP32 verdict: ✅ Feasible.** Choose it over raw boids when you want provable cohesion + a goal term; the fixed-point/float cost difference is irrelevant on ESP32's FPU. The main added burden vs. boids is parameter count.

### 4.2 Gradient-Based Pattern Formation (Kilobot-style self-assembly)

**References:** M. Rubenstein, A. Cornejo, R. Nagpal, "Programmable self-assembly in a thousand-robot swarm," Science 345(6198), 2014 — [publisher page](https://www.science.org/doi/10.1126/science.1254295), [Harvard SSR group overview](https://ssr.seas.harvard.edu/kilobots), [IEEE Spectrum technical explainer](https://spectrum.ieee.org/a-thousand-kilobots-self-assemble). Recent extension: [decentralized global coordinate construction on 200 real Kilobots, Swarm Intelligence 2025](https://link.springer.com/article/10.1007/s11721-025-00251-4).

**Description.** The Kilobot result shows large-scale *shape formation* with three communication-only primitives, on hardware far weaker than an ESP32 (8-bit AVR @ 8 MHz, ~2 KB RAM, 10 cm IR broadcast):
1. **Gradient formation:** seed robots broadcast hop-count 0; every robot adopts `min(heard values) + 1` and rebroadcasts — a distributed BFS giving each robot geodesic distance to the seed.
2. **Localization:** robots trilaterate a local coordinate frame from received messages + measured distances to already-localized neighbors.
3. **Edge-following:** unlocalized robots move along the swarm's perimeter until they enter the target shape region, then stop when about to exit the shape or when meeting an equal-gradient stationary robot.

The deep insight for our project: **hop-count gradients require no positioning whatsoever** — only local broadcast and (optionally) coarse range. On our platform, gradient values propagate over nRF24L01/ESP-NOW broadcasts with RSSI thresholding defining "neighbor," enabling behaviors like follow-the-gradient-to-the-operator, relay-chain formation, and dispersion-until-connectivity-thin.

**Math sketch.**

```
grad_i = 1 + min_{j ∈ heard(i)} grad_j          // repeat each message round; seed holds 0
```

Shape assembly adds: localize in seed frame from ≥3 non-collinear localized neighbors (trilateration), then greedy edge-follow until inside the bitmap shape.

**Compute:** integer min over recent messages — the cheapest algorithm in this survey (it ran on 8 MHz AVR).
**Memory:** gradient value + small neighbor cache; the full shape-assembly variant needs the target bitmap (a 32×32 bitmap = 128 B).
**Comms:** tiny periodic beacons (id + gradient + state, ≤6 B) at ~1–2 Hz. Bandwidth negligible; correctness needs only *eventual* delivery, so it is the most loss-tolerant family here.
**Sensing:** none required for gradients; shape assembly needs inter-robot *range*, which nRF24L01/ESP-NOW RSSI provides only coarsely (Kilobots use IR intensity — noisy too, and it worked). Precise shape formation on our radios is unrealistic; gradient behaviors are.
**Degradation:** exceptionally robust — gradients self-heal as messages arrive; the algorithm was explicitly designed for unreliable robots and was proven correct under the paper's assumptions.

**ESP32 verdict: ✅ Gradient/hop-count coordination: trivially feasible and uniquely valuable as the no-positioning fallback. ⚠️ Full Kilobot shape self-assembly: not feasible with RSSI-only ranging** (would need IR/UWB/ultrasonic inter-robot ranging hardware).

### 4.3 Coverage and Dispersion

**References:** J. Cortés, S. Martínez, T. Karataş, F. Bullo, "Coverage Control for Mobile Sensing Networks," IEEE Trans. Robotics & Automation 20(2), 2004 — [PDF at UCSB](https://motion.me.ucsb.edu/pdf/2002j-cmkb.pdf), [arXiv:math/0212212](https://arxiv.org/pdf/math/0212212). Modern unified treatment: [Boldrer et al., "Lloyd-based approach for robots navigation in human-shared environments" / RAS 156:104207, 2022](https://iris.unitn.it/retrieve/c6491d20-cf40-40d8-965d-8e8461c942b2/1-s2.0-S092188902200118X-main.pdf).

**Description.** Robots spread out to optimally cover an area: each robot computes its **Voronoi cell** (region of the arena closer to it than to any neighbor) and moves toward the cell's centroid (**Lloyd's algorithm** as distributed gradient descent). Provably converges to a centroidal Voronoi configuration; distributed (only Voronoi-neighbor positions needed), asynchronous-tolerant, and adaptive to robots joining/leaving. Simpler dispersion variants skip Voronoi entirely and just run pairwise repulsion (an APF special case) or "move away from strongest RSSI" until neighbor count drops to a threshold.

**Math sketch.**

```
V_i = { x ∈ arena : ‖x − q_i‖ ≤ ‖x − q_j‖ ∀j }        // Voronoi cell
C_i = ∫_{V_i} x φ(x) dx / ∫_{V_i} φ(x) dx              // weighted centroid
u_i = −k (q_i − C_i)                                    // Lloyd descent step
```

**Compute:** the only family where compute deserves a thought: exact 2-D Voronoi cell construction from `N` neighbors is `O(N log N)`, but for ≤10 robots a brute-force half-plane clip of the arena polygon (`O(N × E)`, tens of µs) or a coarse grid approximation (e.g. 64×64 arena grid = 4 KB, sum cells nearest to self) is entirely adequate on ESP32.
**Memory:** arena polygon + neighbor table, or the 4–16 KB grid — fine within 520 KB.
**Comms:** neighbor positions at low rate (coverage dynamics are slow; 1–5 Hz suffices).
**Sensing:** shared-frame positions; arena boundary must be known (pre-mapped or geofenced).
**Degradation:** very tolerant — Cortés et al. explicitly prove convergence under asynchronous updates and delayed neighbor information; a lost packet just delays a centroid step. RSSI-threshold dispersion degrades even more gracefully (it needs no positions at all).

**ESP32 verdict: ✅ Feasible (use half-plane clipping or a grid, not a general Voronoi library).** A natural "spread out and hold" demo behavior and the standard prelude to distributed sensing/patrol tasks.

### 4.4 Behavior Trees as the Coordination Architecture

**References:** M. Iovino et al., "A Survey of Behavior Trees in Robotics and AI," Robotics and Autonomous Systems 154:104096, 2022 — [DOI](https://doi.org/10.1016/j.robot.2022.104096), [arXiv PDF](https://arxiv.org/pdf/2005.05842); A. Ligot et al., "Automatic modular design of robot swarms using behavior trees as a control architecture," PeerJ CS 6:e314, 2020 — [DOI](https://doi.org/10.7717/peerj-cs.314); [K. Montague et al., "A Hierarchical Approach to Evolving Behaviour-Trees for Swarm Control," 2024](https://doi.org/10.1007/978-3-031-56852-7_12); [Kuckling, "Recent trends in robot learning and evolution for swarm robotics," Frontiers in Robotics and AI 2023](https://demiurge.be/publications/pdf_author_versions/Kuc2023FRAI.pdf).

**Description.** Not a flocking algorithm but the *arbitration layer* above them: a BT is a tree of control-flow nodes (sequence, fallback/selector, parallel) over condition and action leaves, ticked at a fixed rate. In swarm robotics BTs are used to compose primitive collective behaviors (disperse, aggregate, flock, follow-leader, go-home-on-low-battery) into missions, replacing spaghetti finite-state machines with something modular and human-readable. The swarm-specific literature (Ligot et al.; Montague et al.) mostly studies *automatically synthesizing* BTs by evolution — interesting but optional; hand-written BTs are the practical takeaway. Notably, Ligot et al. found evolved BTs don't beat evolved FSMs in swarm tasks — the argument for BTs is engineering modularity, not raw performance.

**Math sketch.** None needed — a BT tick is a depth-first traversal returning `SUCCESS / FAILURE / RUNNING` per node; memory-less semantics make each tick `O(nodes)`.

**Compute/Memory:** a 30-node hand-written tree ticks in microseconds; each node is a function pointer + small state → a few KB total. Multiple lightweight C++ BT implementations compile for Arduino/ESP32.
**Comms:** none intrinsically; whatever the leaf behaviors need. A useful pattern: broadcast the *mission-level* BT state (1 byte) so the fleet switches behaviors coherently.
**Sensing:** whatever leaves need.
**Degradation:** N/A at the architecture level; a well-designed BT is exactly where you *put* the degradation handling ("if no leader packet for 500 ms → fallback: stop and hold").

**ESP32 verdict: ✅ Trivially feasible; recommended as the firmware control architecture from day one,** with flocking/formation/dispersion as leaf behaviors.

---

## 5. Recent Research Directions (2018–2026)

### 5.1 Communication-Aware Optimized Flocking (Vásárhelyi et al. and follow-ups)

**References:** G. Vásárhelyi, C. Virágh, G. Somorjai, T. Nepusz, A. E. Eiben, T. Vicsek, "Optimized flocking of autonomous drones in confined environments," Science Robotics 3(20):eaat3536, 2018 — [publisher](https://www.science.org/doi/10.1126/scirobotics.aat3536), [author PDF](https://hal.elte.hu/~vasarhelyi/doc/vasarhelyi2018optimized.pdf), [project page](https://vasarhelyi.github.io/drone-project-site/scirob2018.html). Follow-up: B. Balázs, G. Vásárhelyi, T. Vicsek, "Adaptive leadership overcomes persistence–responsivity trade-off in flocking," J. Royal Society Interface 17(167), 2020 — [DOI](https://doi.org/10.1098/rsif.2019.0853), [author PDF](https://hal.elte.hu/~vasarhelyi/doc/balazs2020adaptive.pdf) (the "WillFull" model, validated on 52 real drones). Related extension: [AGDS goal-directed swarm strategy, Complex & Intelligent Systems 2023](https://link.springer.com/article/10.1007/s40747-022-00900-9).

**Description.** The paper that made flocking work on real, communication-limited robot swarms — and the single most relevant reference for this project. Its thesis: idealized flocking models fail on hardware because they ignore *constrained motion, limited communication range, delays, sensor noise, and walls*. The fix is twofold:
1. **A richer (~11-parameter) model:** short-range repulsion; velocity alignment with a *braking-curve-shaped* alignment radius (robots align more strongly with neighbors they could actually collide with given deceleration limits, absorbing actuator constraints into the rule); and "shill agents" — virtual aligned agents on walls/obstacles pushing inward (generalizing Olfati-Saber's β-agents).
2. **Evolutionary tuning:** parameters optimized with CMA-ES in a simulator that *explicitly models communication delay, packet range limits, and sensor noise*, against fitness combining speed, collision-freedom, and cohesion. Validated outdoors on 30 drones, self-organized, with only broadcast GNSS state — no central control.

The 2020 follow-up diagnoses the core weakness of averaging-type flocks — slow information propagation (stable but sluggish) — and fixes it with *adaptive leadership*: agents that detect a needed direction change (e.g. wall ahead) temporarily raise their influence weight, propagating turns quickly while keeping average-consensus stability. 52-drone field validation at 8 m/s.

**Math sketch (core terms, simplified):**

```
v_rep_i = Σ_j  p_rep · (r_rep − d_ij) · (q_i − q_j)/d_ij        for d_ij < r_rep
v_ali_i = Σ_j  C_frict(d_ij) · (p_j − p_i)                      alignment gain shaped by
                                                                 braking curve D(d, a_max)
v_wall_i = shill-agent alignment term toward arena interior
u_i = v_flock·p̂_i + v_rep_i + v_ali_i + v_wall_i    (clamped to v_max, a_max)
```

**Compute:** `O(|N_i|)` with a few extra curve evaluations — same order as Olfati-Saber; the drones ran it on small embedded companion computers. Easily 200+ Hz on ESP32 for 10 neighbors; 20 Hz is plenty.
**Memory:** neighbor table + 11 parameters; <2 KB.
**Comms:** each robot broadcasts position + velocity (our 16 B packet) at a few Hz; the model was *designed* assuming delayed (~ hundreds of ms), range-limited broadcast — precisely our nRF24L01/ESP-NOW regime.
**Sensing:** shared-frame pose (they used GNSS; we use odometry + optional anchors). This is the honest gap: their positioning was globally consistent, ours drifts.
**Degradation:** the best story of any family — robustness to delay/loss is baked into the fitness function during tuning rather than hoped for afterward. The follow-up work exists precisely because the team measured where the 2018 model degraded (responsivity at walls) and fixed it.

**ESP32 verdict: ✅ Feasible and the recommended *methodology* even if we implement a reduced model:** simulate our radio (measured loss/latency from §6), tune boids/APF gains offline against that simulation (CMA-ES or even grid search at our parameter count), then flash. The braking-curve alignment idea transfers directly to RC cars, whose acceleration limits are severe.

### 5.2 Event-Triggered Communication and Control

**References:** X. Ge et al. background and modern review: Y. Wu et al., "A review of event-triggered consensus control in multi-agent systems," J. Control and Decision, 2024 — [DOI](https://doi.org/10.1080/23307706.2024.2388551); tutorial introduction: Nowzari, Garcia, Cortés, "Event-Triggered Communication and Control of Networked Systems for Multi-Agent Consensus," Automatica 2019 — [arXiv:1712.00429](https://arxiv.org/abs/1712.00429); applied to flocking: [distributed resilient flocking via event/self-triggered communication, IET Control Theory & Appl. 2021](https://doi.org/10.1049/cth2.12061); applied to UAV consensus with comm faults: [Intelligence & Robotics 2023](https://www.oaepublish.com/articles/ir.2023.32).

**Description.** Instead of broadcasting state at a fixed rate, a robot transmits **only when its actual state deviates from what neighbors would predict** by more than a threshold: `‖x_i(t) − x̂_i(t)‖ > ε` (static) or a dynamically adapted threshold. Neighbors run the same predictor (e.g. constant-velocity extrapolation) between packets. Theory guarantees consensus/flocking is preserved and rules out Zeno behavior (infinitely fast triggering); reviews report communication reductions of 80%+ in consensus tasks. *Self-triggered* variants compute the next mandatory broadcast time in advance, allowing radio sleep.

**Math sketch (trigger for consensus):**

```
broadcast when:  ‖x̂_i(t) − x_i(t)‖  ≥  c0 + c1·e^{−αt}      (static/dynamic threshold)
between events:  neighbors integrate  x̂_i(t) = x_i(t_k) + p_i(t_k)·(t − t_k)
```

**Compute/Memory:** one predictor per tracked neighbor (constant-velocity: 4 floats) + own-trigger check — negligible.
**Comms:** this *is* the comms optimization: it converts our fixed 10 Hz × N broadcast load into load proportional to maneuvering intensity. On a shared 2.4 GHz channel with nRF24L01 (no CSMA — collisions silently destroy packets), reducing transmission count directly reduces collision probability, so event triggering compounds beneficially.
**Degradation:** caution — the theory usually assumes triggered packets *arrive*. Under loss, a dropped *triggered* packet is worse than a dropped periodic one (neighbors confidently extrapolate wrong state). Practical fix: send each event packet 2–3× (cheap, still far below periodic load) and keep a low-rate periodic heartbeat as a floor.

**ESP32 verdict: ✅ Feasible and genuinely useful — as an optimization layer on top of whichever controller we pick, not as a starting point.** Implement periodic first, measure airtime, then add triggering.

### 5.3 Multi-Agent RL Distilled to Tiny Networks (TinyML)

**References:** survey: [Bui & Kim et al., "A Survey on UAV Control with Multi-Agent Reinforcement Learning," Drones 9(7):484, 2025](https://www.mdpi.com/2504-446X/9/7/484); scalable MARL flocking: [Yan et al., "MARL with Spatial–Temporal Attention for Flocking … Fixed-Wing UAV Fleet," IEEE T-ITS 2024](https://doi.org/10.1109/tits.2024.3505929), [attention-biased RL flocking, IEEE TASE 2025](https://doi.org/10.1109/tase.2025.3640181). MCU deployment evidence: [Eschmann, Albani, Loianno, "Learning to Fly in Seconds," IEEE RA-L 2024, arXiv:2311.13081](https://arxiv.org/abs/2311.13081) + [code](https://github.com/arplaboratory/learning-to-fly) (policy runs on Crazyflie's STM32 under real-time guarantees); [TinyNav: end-to-end TinyML navigation on ESP32, 23k-param CNN, 30 ms inference](https://arxiv.org/html/2603.11071v1) + [code](https://github.com/regularpooria/tinynav); [TinyRL: header-only C++ RL on ESP32-S3 with a PlatformIO example](https://github.com/mohmdelsayed/TinyRL); [ESP32-S3 swarm with on-policy RL + MQTT/simulation bridge (MSc thesis, 2025)](https://www.theseus.fi/bitstream/handle/10024/893098/Istvan_Csiszar.pdf?sequence=2).

**Description.** The modern recipe is **centralized training, decentralized execution (CTDE)**: train in simulation with global information (MAPPO/MADDPG, often with attention over neighbors for swarm-size invariance), deploy per-robot policies that see only local observations. For MCUs, the deployed actor is distilled/quantized to a small MLP or CNN (int8, TFLite-Micro or code-generated C). Inference-side feasibility on our hardware class is now proven: a full quadrotor RPM controller runs on a 168 MHz STM32, and 23k-parameter policies run in 30 ms on ESP32. What is *not* yet mature is MARL *flocking* policies deployed on MCU swarms — the 2025 Drones survey notes most real-world MARL work still uses companion computers, and sim-to-real for multi-agent interaction dynamics remains the open problem.

**Math sketch.** Deployed artifact: `a_i = π_θ(o_i)`, with `o_i` = own state + k-nearest-neighbor relative states (fixed-size, sorted-by-distance observation vector). A 2×64 MLP ≈ 10–20k params ≈ 40–80 KB int8 — fits easily.

**Compute:** ~20k MACs per inference → well under 1 ms on ESP32 (with or without S3 vector instructions). Fine at 20–50 Hz.
**Memory:** 50–300 KB flash for weights, tens of KB RAM tensor arena — fits, with care, in 520 KB SRAM alongside comms stacks.
**Comms:** same state broadcast as classical methods (the policy consumes neighbor states); no extra requirement.
**Sensing:** same as classical; policy quality is very sensitive to observation noise mismatch between sim and reality.
**Degradation:** unknown-by-default — a policy trained with perfect neighbor info can behave arbitrarily badly under stale inputs. Mitigation (per the Vásárhelyi lesson): inject measured packet loss/latency into training. This is doable but is research-grade effort, not weekend firmware.

**ESP32 verdict: ⚠️ Feasible for inference, premature as the first coordination approach.** Sensible phase-2/3 project: train a policy in a simple 2-D simulator against our measured radio model, distill to int8, deploy with TFLite-Micro or TinyRL — after the classical stack works and provides a safety fallback layer.

### 5.4 Crazyflie Swarm Research (STM32-class evidence for our hardware budget)

**References:** [Bitcraze research blog (ICRA 2025 fully-onboard decentralized swarm demo, community research roundup)](https://www.bitcraze.io/category/research/); S. Li et al., "Onboard UWB-based relative localization for lightweight aerial swarms" — [project page + code](https://shushuai3.github.io/autonomous-swarm/); [Shan et al., "Onboard Ranging-Based Relative Localization and Stability for Lightweight Aerial Swarms," IEEE RA-L 2025](https://fengshan.seu-netsi.net/papers/LSLCWC-RAL25.pdf); vision-based neighbor localization on nano-drones: [Bonato et al., ICRA 2023, arXiv:2303.01940](https://ar5iv.labs.arxiv.org/html/2303.01940), [Crupi et al., ICRA 2024](https://doi.org/10.1109/icra57147.2024.10611455).

**Description.** The Crazyflie (STM32F405 @ 168 MHz, 192 KB RAM — *less* compute and RAM than our ESP32) is the academic workhorse for MCU-class swarms, and its results calibrate what we can expect:
- **13-drone fully-onboard swarms** running a many-to-many UWB ranging protocol + EKF relative localization at 16 Hz with <0.2 m error, *plus* flight control, all on the STM32 (RA-L 2025 above; open-source).
- **Fully decentralized behavior:** Bitcraze's ICRA 2025 booth demo ran a decentralized swarm entirely onboard with peer-to-peer broadcast (their P2P API) — no central planner.
- **Sensing trend worth copying:** since 2.4 GHz radios can't measure relative position, the community bolts on a ranging modality (UWB decks; or camera + tiny CNN on a GAP8 co-processor). For our RC cars, the equivalent honest options are broadcast odometry (phase 1), fixed UWB/ultrasonic anchor beacons (phase 2), or per-robot UWB (phase 3, ~$20/robot).

**ESP32 verdict: ✅ Direct evidence our compute/RAM budget supports simultaneous state estimation + swarm ranging + coordination + control.** Also a source of reusable design patterns (broadcast P2P API, TDMA-slotted ranging).

---

## 6. Communication Considerations for nRF24L01 / ESP-NOW Fleets

### 6.1 Measured link characteristics

| Link | Payload | Measured performance (published) | Source |
|---|---|---|---|
| ESP-NOW (broadcast, mesh) | 250 B | PDR >98% (3 robots), >95% (5), >92% (7); latency <20 ms (3) to <40 ms (7); ~8–10 m reliable indoors in that setup | [IIETA JESA 2025 swarm study](https://iieta.org/journals/jesa/paper/10.18280/jesa.590507) |
| ESP-NOW (point, field) | 250 B | 5–46 ms latency; efficient to ~150 m LoS; severe loss >180–200 m | [FORTEI-ICEE 2024 QoS field test](https://doi.org/10.1109/fortei-icee64706.2024.10824617) |
| ESP-NOW (18-node ad hoc) | 200 B | packet loss 0.3% (dense room) → 27.3% (sparse building-wide) | [TU Ilmenau NetSys 2025 evaluation](https://www.db-thueringen.de/servlets/MCRFileNodeServlet/dbt_derivate_00068872/ilm1-202520021_009-012.pdf) |
| nRF24L01(+) | 32 B | no CSMA; hardware auto-ACK/retry unicast only (broadcast unacknowledged); mesh requires RF24Network/RF24Mesh tree overlay | [RF24Mesh docs](https://nrf24.github.io/RF24Mesh/index.html), [RF24Network docs](https://rf24network.readthedocs.io/en/latest/) |

Practical implications: **(a)** our 16-byte state packet fits a single nRF24L01 frame with room for a CRC/seq — no fragmentation ever; **(b)** ESP-NOW is the technically easier and better-instrumented choice (true broadcast, larger payloads, no extra hop overlay needed, built into the ESP32 we already have) — nRF24L01 remains attractive only if 2.4 GHz Wi-Fi congestion or per-unit cost dominates; **(c)** RF24Mesh's tree topology (single path to any node, master-assigned addresses) is designed for sensor networks, not peer flocking — for nRF24L01 flocking, raw broadcast frames on a shared channel/address with application-level sequence numbers are the right primitive, not the mesh layer.

### 6.2 Broadcast/gossip pattern and neighbor discovery

The pattern used by every MCU swarm reviewed (Vásárhelyi drones, Bitcraze P2P, ESP-NOW robot meshes): each robot broadcasts its state beacon at a fixed rate; every receiver maintains a **neighbor table** `{id → last state, RSSI, last-heard time}`; entries expire after a timeout (e.g. 3× beacon period). Neighbor discovery is thus free — hearing a beacon *is* discovery — and "neighborhood" is defined by radio range (optionally an RSSI floor), which conveniently matches the local-interaction assumption of flocking models. Gossip extensions (rebroadcasting others' states with a hop counter) buy multi-hop reach at the cost of airtime; at ≤10 robots in one radio range, don't — single-hop is simpler and the literature on multi-hop lossy flocking shows fragmentation risk grows with hop count ([FITEE 2024](https://journal.hep.com.cn/fitee/EN/10.1631/FITEE.2300295)).

To avoid synchronized-collision loss on these CSMA-less/weak-CSMA links, either jitter each robot's beacon phase randomly (simple, adequate at N≤10) or run coarse TDMA: `slot_i = (t_sync + i·T/N)` — which requires time sync (next).

### 6.3 Time synchronization

Options in increasing precision/complexity:
1. **None** — pure jittered broadcast; algorithms in this survey (boids/APF/Vicsek/gradients/Lloyd) are asynchronous-tolerant, so this is genuinely viable for phase 1.
2. **Firefly / pulse-coupled oscillator sync** — each robot nudges its beacon phase toward the average of heard beacon phases; fully decentralized, self-healing, proven on Kilobots and swarm platforms ([Christensen, O'Grady, Dorigo, "From Fireflies to Fault-Tolerant Swarms of Robots," IEEE Trans. Evolutionary Computation 2009](https://home.iscte-iul.pt/~alcen/pubs/ChrOGrDor2009tec.pdf); [Kilobot sync walkthrough](https://soln.tech/blog/shape_formation_using_kilobots)). Also doubles as decentralized fault detection: a robot that stops flashing on schedule is detectably dead. Milliseconds-level accuracy — enough for TDMA slots and timestamping state packets.
3. **Reference-broadcast from the operator station** — one base beacon; robots timestamp against it. Simplest engineering if a base station exists anyway.

Timestamping matters even without TDMA: consumers should extrapolate a neighbor's broadcast state by `(now − t_packet)` with the packet's velocity before using it — this single trick (dead-reckoning compensation of comm latency) is standard in the Crazyflie/Vásárhelyi systems and substantially flattens the loss-vs-performance curve.

### 6.4 How each family degrades with packet loss and limited neighbor information

| Family | Under packet loss | Under few/limited neighbors | Notes & sources |
|---|---|---|---|
| Boids / APF | Graceful: stale-state extrapolation covers gaps; collision margin must absorb staleness × speed | Works with 2–3 neighbors; degenerates to pair behaviors | Emergence decays with loss+delay ([WSC 2017](https://doi.org/10.1109/wsc.2017.8248117)) |
| Vicsek | Robust (averaging consensus) | Order parameter drops; noise-dominated below critical density | Phase-transition behavior is the point of the model ([PRL 1995](https://arxiv.org/abs/cond-mat/0611743)) |
| Olfati-Saber consensus flocking | Provable "flocking in expectation" bounds vs. loss probability, gains, period | Fragmentation if graph disconnects; γ-term mitigates | [IEEE TVT 2024](https://doi.org/10.1109/tvt.2024.3382617), [FITEE 2024](https://journal.hep.com.cn/fitee/EN/10.1631/FITEE.2300295) |
| Leader–follower | **Brittle**: leader packet is a single point of failure → predict + timeout-stop; prioritizing positional traffic helps most ([AirTight mixed-criticality study](https://eprints.whiterose.ac.uk/id/document/2938296)) | Needs only the leader link — best in sparse fleets | Star > chain topology |
| Virtual structure | Frame broadcast loss stalls everyone identically (formation stays coherent, drifts stale) | N/A (no inter-follower dependence) | |
| Kilobot gradients | Best-in-class: needs only eventual delivery; self-healing | Designed for minimal connectivity | [Science 2014](https://www.science.org/doi/10.1126/science.1254295) |
| Coverage/Lloyd | Proven convergent under async/delayed info | Voronoi cells enlarge; still correct | [Cortés et al. 2004](https://motion.me.ucsb.edu/pdf/2002j-cmkb.pdf) |
| Event-triggered | Reduces load (helps loss) but a dropped *event* packet is costly → repeat events + heartbeat floor | Threshold theory assumes connected graph | [Review 2024](https://doi.org/10.1080/23307706.2024.2388551) |
| MARL/TinyML | Undefined unless loss was in training distribution | Fixed-size observation padding handles it mechanically | [Drones 2025 survey](https://www.mdpi.com/2504-446X/9/7/484) |
| Whole-mission view | Halving PDR ≈ 3× mission time in swarm mapping | — | [Atlas 2.0](https://doi.org/10.1145/3485730.3494040) |

---

## 7. Comparison Table and Recommendation

### 7.1 Comparison

`N` = neighbors heard (≤ fleet size ≈ 10). "Payload" = per-robot broadcast needed by the algorithm.

| Family | Compute / update | RAM | Payload × rate | Sensing needed | Loss robustness | Impl. effort | ESP32 verdict |
|---|---|---|---|---|---|---|---|
| Boids (Reynolds) | `O(N)`, ~30 flops/nbr | <1 KB | 16 B × 5–20 Hz | shared-frame pose+vel | Good | Low | ✅ Ideal first flocking behavior |
| Vicsek | `O(N)`, cheapest | <0.5 KB | 2–4 B × 5 Hz | heading only | Very good | Trivial | ✅ Demo/alignment term only (no separation) |
| Potential fields | `O(N+obst)` | <1 KB | 16 B × 5–20 Hz | pose (+ranger) | Good | Low | ✅ Use as universal spacing/safety layer |
| Leader–follower | `O(1)` | <1 KB | 16 B × 10–20 Hz (leader only) | pose vs. leader | Brittle → mitigable | Lowest | ✅ Best first *formation* behavior |
| Virtual structure | `O(1)` | <1 KB | 16 B × 10–20 Hz (frame) | shared-frame pose | Moderate | Low | ✅ Easy extension of leader–follower |
| Olfati-Saber flocking | `O(N)`, 2–3× boids | <2 KB | 16 B × 10–20 Hz | pose+vel | Analyzed, good | Medium | ✅ When provable cohesion/goal term wanted |
| Vásárhelyi optimized flocking | `O(N)` + offline CMA-ES tuning | <2 KB | 16 B × 2–10 Hz | pose+vel | **Designed-in** | Medium (needs simulator) | ✅ Target methodology for the "real" flocking milestone |
| Kilobot gradients | `O(msgs)`, integer | <0.5 KB | 4–6 B × 1–2 Hz | none | **Best** | Low | ✅ No-positioning fallback; ⚠️ full shape assembly needs real ranging |
| Coverage / Lloyd | `O(N·E)` clip or grid | 4–16 KB | 16 B × 1–5 Hz | pose + arena map | Very good (proven async) | Medium | ✅ Feasible with grid/clipping approximation |
| Behavior trees | `O(nodes)` | few KB | mission byte | per leaves | N/A (hosts fallbacks) | Low | ✅ Adopt as firmware architecture |
| Event-triggered comms | `O(N)` predictors | <1 KB | −80% vs. periodic | as base algo | Good w/ heartbeat | Medium | ✅ Phase-2 optimization layer |
| MARL → TinyML | ~20k MACs/inf (<1 ms) | 50–300 KB flash, tens KB RAM | 16 B × 10–20 Hz | as classical + sim fidelity | Only if trained-in | High (training pipeline) | ⚠️ Feasible inference; premature as first approach |

### 7.2 Final recommendation — top 3 for a first implementation

**#1 — Boids + potential-field spacing, tuned communication-aware (Vásárhelyi methodology).**
Implement Reynolds' three rules with an APF repulsion term for inter-robot spacing and arena walls (shill-agent style), consuming a broadcast neighbor table over ESP-NOW (or raw nRF24L01 broadcast) at ~10 Hz with timestamp-based extrapolation of stale neighbor states. Tune the ~8–10 gains in a simple 2-D simulator that replays *measured* packet loss and latency from our own radios, per Vásárhelyi et al. This is the highest payoff-to-effort coordination behavior: genuinely decentralized, visibly "swarm-like," `O(N)` cheap, robust to loss, and every piece (neighbor table, state packet, extrapolation, safety repulsion) is reusable by everything below.

**#2 — Leader–follower formation control (human-driven leader).**
One RC car stays human-driven and broadcasts its pose; followers hold offsets with PD control on slot error, constant-velocity coasting on packet gaps, and timeout-stop safety. This is the fastest route to a working multi-robot demo (O(1) compute, one 16-byte broadcast for the whole fleet), it exercises the identical comms/estimation stack as #1, and it degrades in well-understood ways. Extend to virtual-structure formations once autonomous waypoint following exists.

**#3 — Kilobot-style gradient/hop-count coordination + firefly synchronization.**
Hop-count gradients over radio beacons (operator station = seed) give follow-the-gradient homing, relay-chain stretching, and RSSI-threshold dispersion with **zero positioning requirements** — the fallback layer that keeps the fleet coordinated when odometry drifts or the shared frame breaks down, and it runs on 6-byte beacons at 1–2 Hz. Firefly sync rides on the same beacons, providing TDMA slots and dead-robot detection nearly for free.

Deferred (with re-evaluation triggers): **event-triggered broadcasting** once airtime measurements justify it; **Olfati-Saber/Vásárhelyi full models** as the maturation of #1; **coverage control** when a mapped arena exists; **TinyML/MARL** after the classical stack works and can serve as its safety envelope; **Kilobot shape assembly** only if inter-robot ranging hardware (UWB/ultrasonic/IR) is added.

---

## 8. References

Classical foundations
1. Reynolds, C. W. — *Flocks, Herds, and Schools: A Distributed Behavioral Model*, SIGGRAPH 1987. https://www.red3d.com/cwr/papers/1987/SIGGRAPH87.pdf
2. Vicsek, T. et al. — *Novel Type of Phase Transition in a System of Self-Driven Particles*, Phys. Rev. Lett. 75:1226, 1995. https://arxiv.org/abs/cond-mat/0611743 · https://doi.org/10.1103/PhysRevLett.75.1226
3. Olfati-Saber, R. — *Flocking for Multi-Agent Dynamic Systems: Algorithms and Theory*, IEEE TAC 51(3), 2006. https://doi.org/10.1109/tac.2005.864190 · https://hal.elte.hu/~vicsek/downloads/papers/flocking_tac06-engineering.pdf
4. Cortés, J., Martínez, S., Karataş, T., Bullo, F. — *Coverage Control for Mobile Sensing Networks*, IEEE TRA 20(2), 2004. https://motion.me.ucsb.edu/pdf/2002j-cmkb.pdf · https://arxiv.org/pdf/math/0212212

Surveys (2024–2026)
5. Javed et al. — *State-of-the-Art Flocking Strategies for the Collective Motion of Multi-Robots*, Machines 12(10):739, 2024. https://www.mdpi.com/2075-1702/12/10/739
6. *Advancement Challenges in UAV Swarm Formation Control: A Comprehensive Review*, Drones 8(7):320, 2024. https://doi.org/10.3390/drones8070320
7. Wang et al. — *A Survey of Multi-Agent Systems on Distributed Formation Control*, Unmanned Systems, 2024. https://doi.org/10.1142/s2301385024500274
8. *Recent Advancement in Formation Control of Multi-Agent Systems: A Review*, CMC 83(3), 2025. https://www.techscience.com/cmc/v83n3/61053
9. *Exploring advancements and emerging trends in robotic swarm coordination and control of swarm flying robots: A review*, Proc. IMechE Part C, 2024. https://doi.org/10.1177/09544062241275359

Kilobots & minimal swarms
10. Rubenstein, M., Cornejo, A., Nagpal, R. — *Programmable self-assembly in a thousand-robot swarm*, Science 345(6198), 2014. https://www.science.org/doi/10.1126/science.1254295 · https://ssr.seas.harvard.edu/kilobots · explainer: https://spectrum.ieee.org/a-thousand-kilobots-self-assemble
11. *Decentralised construction of a global coordinate system in a large swarm of minimalistic robots*, Swarm Intelligence, 2025. https://link.springer.com/article/10.1007/s11721-025-00251-4
12. Christensen, A. L., O'Grady, R., Dorigo, M. — *From Fireflies to Fault-Tolerant Swarms of Robots*, IEEE Trans. Evol. Comp., 2009. https://home.iscte-iul.pt/~alcen/pubs/ChrOGrDor2009tec.pdf
13. Kilobot synchronization & shape formation walkthrough (kilolib-based). https://soln.tech/blog/shape_formation_using_kilobots

Behavior trees
14. Iovino, M. et al. — *A Survey of Behavior Trees in Robotics and AI*, RAS 154:104096, 2022. https://doi.org/10.1016/j.robot.2022.104096 · https://arxiv.org/pdf/2005.05842
15. Ligot, A. et al. — *Automatic modular design of robot swarms using behavior trees as a control architecture*, PeerJ CS 6:e314, 2020. https://doi.org/10.7717/peerj-cs.314
16. Montague, K. et al. — *A Hierarchical Approach to Evolving Behaviour-Trees for Swarm Control*, 2024. https://doi.org/10.1007/978-3-031-56852-7_12
17. Kuckling, J. — *Recent trends in robot learning and evolution for swarm robotics*, Frontiers in Robotics & AI, 2023. https://demiurge.be/publications/pdf_author_versions/Kuc2023FRAI.pdf

Communication-aware flocking (ELTE line)
18. Vásárhelyi, G. et al. — *Optimized flocking of autonomous drones in confined environments*, Science Robotics 3(20):eaat3536, 2018. https://www.science.org/doi/10.1126/scirobotics.aat3536 · https://hal.elte.hu/~vasarhelyi/doc/vasarhelyi2018optimized.pdf · https://vasarhelyi.github.io/drone-project-site/scirob2018.html
19. Balázs, B., Vásárhelyi, G., Vicsek, T. — *Adaptive leadership overcomes persistence–responsivity trade-off in flocking*, J. R. Soc. Interface 17(167), 2020. https://doi.org/10.1098/rsif.2019.0853 · https://hal.elte.hu/~vasarhelyi/doc/balazs2020adaptive.pdf
20. *AGDS: adaptive goal-directed strategy for swarm drones flying through unknown environments*, Complex & Intelligent Systems, 2023. https://link.springer.com/article/10.1007/s40747-022-00900-9

Event-triggered communication
21. *A review of event-triggered consensus control in multi-agent systems*, J. Control & Decision, 2024. https://doi.org/10.1080/23307706.2024.2388551
22. Nowzari, C., Garcia, E., Cortés, J. — *Event-Triggered Communication and Control of Networked Systems for Multi-Agent Consensus*. https://arxiv.org/abs/1712.00429
23. *Distributed resilient flocking control of multi-agent systems through event/self-triggered communication*, IET CTA, 2021. https://doi.org/10.1049/cth2.12061
24. *Event-triggered consensus control method with communication faults for multi-UAV*, Intelligence & Robotics, 2023. https://www.oaepublish.com/articles/ir.2023.32

Crazyflie / MCU-class swarms
25. Bitcraze research blog (decentralized onboard swarm demos, community research). https://www.bitcraze.io/category/research/
26. Li, S. et al. — *Onboard UWB-based Relative Localization and Stability for Lightweight Aerial Swarms* (13 × Crazyflie, 192 KB RAM, open source). https://shushuai3.github.io/autonomous-swarm/
27. Shan, F. et al. — *Onboard Ranging-Based Relative Localization and Stability for Lightweight Aerial Swarms*, IEEE RA-L 2025. https://fengshan.seu-netsi.net/papers/LSLCWC-RAL25.pdf
28. Bonato, S. et al. — *Ultra-low Power Deep Learning-based Monocular Relative Localization Onboard Nano-quadrotors*, ICRA 2023. https://ar5iv.labs.arxiv.org/html/2303.01940
29. Crupi, L. et al. — *High-throughput Visual Nano-drone to Nano-drone Relative Localization using Onboard Fully Convolutional Networks*, ICRA 2024. https://doi.org/10.1109/icra57147.2024.10611455

TinyML / learning on MCUs
30. Eschmann, J., Albani, D., Loianno, G. — *Learning to Fly in Seconds*, IEEE RA-L 2024. https://arxiv.org/abs/2311.13081 · code: https://github.com/arplaboratory/learning-to-fly
31. *TinyNav: End-to-End TinyML for Real-Time Autonomous Navigation on Microcontrollers* (ESP32, 23k params, 30 ms). https://arxiv.org/html/2603.11071v1 · code: https://github.com/regularpooria/tinynav
32. TinyRL — header-only C++ RL for ESP32-S3 (PlatformIO example). https://github.com/mohmdelsayed/TinyRL
33. Csiszár, I. — *Design and Implementation of a Cloud-Based Robot Swarm Control System…* (ESP32-S3 swarm + on-policy RL), MSc thesis, 2025. https://www.theseus.fi/bitstream/handle/10024/893098/Istvan_Csiszar.pdf?sequence=2
34. *A Survey on UAV Control with Multi-Agent Reinforcement Learning*, Drones 9(7):484, 2025. https://www.mdpi.com/2504-446X/9/7/484
35. Yan, C. et al. — *MARL With Spatial–Temporal Attention for Flocking With Collision Avoidance of a Scalable Fixed-Wing UAV Fleet*, IEEE T-ITS 2024. https://doi.org/10.1109/tits.2024.3505929
36. *Attention-Biased Reinforcement Learning Framework for Adaptive and Scalable Flocking of UAV Swarms*, IEEE TASE 2025. https://doi.org/10.1109/tase.2025.3640181

Communication over lossy low-bandwidth links
37. *Swarm Robot Communication Using ESP-NOW Mesh Protocol for Multi-Agent Coordination in Indoor Navigation*, IIETA JESA 59(5), 2025. https://iieta.org/journals/jesa/paper/10.18280/jesa.590507
38. *Field Testing and QoS Analysis of ESP-NOW Communication on ESP32*, FORTEI-ICEE 2024. https://doi.org/10.1109/fortei-icee64706.2024.10824617
39. *A Real-World Evaluation of ESP32's Suitability for Use in Wireless Ad Hoc Networks*, NetSys 2025. https://www.db-thueringen.de/servlets/MCRFileNodeServlet/dbt_derivate_00068872/ilm1-202520021_009-012.pdf
40. RF24Mesh / RF24Network documentation (nRF24L01 networking layers). https://nrf24.github.io/RF24Mesh/index.html · https://rf24network.readthedocs.io/en/latest/
41. *Coordinating a Swarm of Micro-Robots Under Lossy Communication* (Atlas 2.0), ACM SenSys workshops 2021. https://doi.org/10.1145/3485730.3494040
42. *Simulating the effect of degraded wireless communications on emergent behavior*, WSC 2017. https://doi.org/10.1109/wsc.2017.8248117
43. *Discrete-Time Flocking Control in Multi-Robot Systems With Random Link Failures*, IEEE TVT 2024. https://doi.org/10.1109/tvt.2024.3382617
44. *Flocking fragmentation formulation for a multi-robot system under multi-hop and lossy ad hoc networks*, Frontiers of IT & EE, 2024. https://journal.hep.com.cn/fitee/EN/10.1631/FITEE.2300295
45. *AirTight-based mixed-criticality communication for swarm flocking under wireless faults* (University of York). https://eprints.whiterose.ac.uk/id/document/2938296

---

*Document written 2026-07-24 as part of the research phase; all URLs above were retrieved and verified during the survey. Next document in the series should cover simulation tooling and the radio measurement plan needed for the communication-aware tuning loop recommended in §7.2.*
