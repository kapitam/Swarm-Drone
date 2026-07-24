# Catalog of Swarm Algorithms Investigated

**Quick-reference note.** This is the condensed catalog of every coordination algorithm family
investigated during the research phase. Full analysis, math sketches, degradation behavior, and
all citations are in [01-swarm-algorithms.md](01-swarm-algorithms.md) (coordination) and
[02-obstacle-avoidance.md](02-obstacle-avoidance.md) (inter-agent collision avoidance).

Verdicts assume the project platform: ESP32-class MCU, broadcast radio (ESP-NOW or nRF24L01)
with 13–16 byte pose packets, no relative-position sensing, fleet of ~3–10 robots.

## A. Classical foundations

| # | Algorithm | One-line summary | ESP32 verdict |
|---|---|---|---|
| 1 | Reynolds' Boids (1987) | Three local steering rules — separation, alignment, cohesion — over neighbors within a radius; emergent leaderless flocking. ~30 flops/neighbor, <1 KB RAM, needs neighbor pose+velocity at 5–20 Hz. | ✅ Recommended starting point |
| 2 | Vicsek model (1995) | Minimal flocking: each agent adopts the average heading of neighbors plus noise. Cheapest of all (2-byte heading exchange), but has **no separation term → collisions**. | ✅ As alignment term / demo only |
| 3 | Artificial Potential Fields | Gradient descent on attractive (goal) + repulsive (neighbors, obstacles) potentials. Known local-minimum and oscillation issues; standard fixes exist. | ✅ Use as the universal spacing/safety layer |
| 4 | Leader–follower formation | Followers hold a distance/bearing or displacement offset to a leader (can be a human-driven vehicle). O(1) compute; one 16 B leader broadcast serves the whole fleet. Brittle to leader-packet loss → coast-and-timeout mitigations. | ✅ Best first *formation* behavior |
| 5 | Virtual structure formation | The formation is one rigid virtual body; each robot tracks its slot in the broadcast virtual frame. No physical leader to lose; mildly centralized (someone broadcasts the frame). | ✅ Easy extension once waypoint following works |

## B. Modern decentralized coordination

| # | Algorithm | One-line summary | ESP32 verdict |
|---|---|---|---|
| 6 | Olfati-Saber consensus flocking (2006) | Formalized boids: lattice-spacing potential + graph-Laplacian velocity consensus + β-agents (obstacles) + γ-agent (goal), with Lyapunov stability guarantees. Key lesson: raw boids fragments without a goal term. 2–3× boids cost. | ✅ When provable cohesion is wanted |
| 7 | Vásárhelyi optimized flocking (Science Robotics 2018) | The reference design for flocking on real radios: ~11-parameter model (braking-curve-shaped alignment, wall "shill agents") with gains **evolved in a simulator that models packet loss, delay, and range**. Validated on 30 outdoor drones; 2020 adaptive-leadership follow-up on 52. | ✅ Target methodology for the "real" flocking milestone |
| 8 | Kilobot-style gradient / hop-count coordination (Science 2014) | Distributed BFS: seed broadcasts hop 0, everyone adopts min(heard)+1. Needs **no positioning at all**, runs on 6-byte beacons at 1–2 Hz, self-heals under loss (proven on 8 MHz AVR). Full shape self-assembly additionally needs real inter-robot ranging — not feasible with RSSI alone. | ✅ Gradients: the no-positioning fallback layer. ⚠️ Shape assembly: needs ranging hardware |
| 9 | Coverage / Lloyd's algorithm (Cortés 2004) | Robots move toward the centroid of their Voronoi cell → provably converge to optimal spread. Proven convergent under asynchronous, delayed information. Use half-plane clipping or a 4–16 KB grid, not a Voronoi library. | ✅ Natural "spread out and hold" behavior |
| 10 | Behavior trees (arbitration layer) | Not a flocking rule — the architecture that composes behaviors (flock, follow, disperse, go-home, failsafe) and hosts all degradation handling. Microsecond ticks, few KB. | ✅ Adopt as firmware control architecture from day one |

## C. Recent research directions (2018–2026)

| # | Algorithm | One-line summary | ESP32 verdict |
|---|---|---|---|
| 11 | Event-triggered communication | Broadcast only when actual state deviates from what neighbors would extrapolate; 80%+ traffic reduction in consensus tasks. Caveat: a dropped *event* packet is costlier than a dropped periodic one → repeat events + heartbeat floor. | ✅ Phase-2 optimization layer, after airtime is measured |
| 12 | MARL → TinyML distilled policies | Centralized training / decentralized execution; deployed actor is a ~10–20k-param int8 network (<1 ms inference on ESP32). Inference proven on Crazyflie/ESP32; the open problems are training burden, sim-to-real gap, and undefined behavior under un-trained packet loss. | ⚠️ Feasible for inference; premature as first approach |
| 13 | Firefly / pulse-coupled sync | Beacon-phase synchronization for TDMA slots and dead-robot detection, nearly free on top of the beacon traffic; proven on Kilobots. | ✅ Cheap add-on to the beacon layer |

Supporting evidence track: the **Crazyflie ecosystem** (STM32F405, 168 MHz, 192 KB RAM — strictly
weaker than our ESP32) runs flight control + UWB swarm ranging + EKF relative localization +
formation control simultaneously on 13-drone swarms. This is the existence proof that our compute
budget holds for every entry above.

## D. Inter-agent collision avoidance (from doc 02)

These are swarm algorithms too — they resolve robot-vs-robot conflicts rather than produce group motion.

| # | Algorithm | One-line summary | ESP32 verdict |
|---|---|---|---|
| 14 | Velocity Obstacles family (VO / RVO / ORCA) | Choose velocities outside the set that leads to collision; ORCA reduces it to a small linear program per tick. Needs neighbor **velocity** exchange. Runs on MCU-class hardware. | ✅ Feasible; second choice |
| 15 | Buffered Voronoi Cells (BVC) | Each robot constrains its motion to its own Voronoi cell shrunk by a safety buffer; needs only neighbor **positions**, O(k) per tick. Ships in production Crazyflie firmware. | ✅ Recommended inter-agent filter |
| 16 | Rule-based right-of-way | Fixed priority rules (e.g. lower ID yields, always-turn-right). Trivial, no geometry; the degraded-comms fallback. | ✅ Keep as radio-failure fallback |
| 17 | Control Barrier Functions (CBF) | Analytic safety filter guaranteeing forward-invariance of a safe set; QP or closed-form on simple dynamics. | ✅ Later upgrade of the stop-reflex layer |

## Ranked recommendation (unchanged from doc 01 §7.2)

1. **Boids + potential-field spacing, tuned communication-aware** per the Vásárhelyi methodology
   (simulate our measured radio loss/latency, tune gains offline, then flash).
2. **Leader–follower formation** with a human-driven leader — the fastest working multi-robot demo,
   exercising the identical comms/estimation stack.
3. **Kilobot gradients + firefly sync** — the positioning-free fallback and TDMA/fault-detection layer.

Deferred with re-evaluation triggers: event-triggered comms (once airtime is measured), full
Olfati-Saber/Vásárhelyi models (as #1 matures), coverage control (when an arena map exists),
MARL/TinyML (after the classical stack works and can act as its safety envelope), Kilobot shape
assembly (only if UWB/IR/ultrasonic ranging hardware is added).
