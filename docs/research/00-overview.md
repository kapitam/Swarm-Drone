# Swarm Robotics on ESP32 — Research Overview

**Phase: research only.** This folder contains the research groundwork for the swarm firmware.
Firmware writing and OS/build-system work are separate follow-up efforts that should use these
documents as their starting brief.

## Documents

| Doc | Topic | Headline conclusion |
|---|---|---|
| [01-swarm-algorithms.md](01-swarm-algorithms.md) | Swarm coordination & flocking algorithms | Compute is not the bottleneck — sensing and radio state exchange are. Start with a boids/potential-field hybrid tuned communication-aware (Vásárhelyi et al. methodology), leader–follower as first demo, Kilobot-style gradients as positioning-free fallback. |
| [02-obstacle-avoidance.md](02-obstacle-avoidance.md) | Obstacle avoidance sensors & algorithms | VL53L5CX multizone ToF + VFH+ steering for environment obstacles; Buffered Voronoi Cells for inter-agent avoidance (proven on weaker STM32F405 in Crazyflie); a hard stop-reflex layer underneath. Every classical algorithm ticks in <1 ms on ESP32. |
| [03-esp32-compute-feasibility.md](03-esp32-compute-feasibility.md) | Compute/memory/radio feasibility on the ESP32 family | The classic ESP32 in this repo is sufficient (ESP-FC and ESP-Drone prove 500 Hz–1 kHz loops on this exact chip). Standardize *new* hardware on ESP32-S3 (~34% faster per clock, ~7× TinyML, more RAM). ESP-NOW is the recommended swarm radio over nRF24L01. |
| [04-rtos-and-firmware-architecture.md](04-rtos-and-firmware-architecture.md) | RTOS choice & firmware architecture | FreeRTOS is already running under Arduino on ESP32; the decision is how much of it to control. Recommend migrating toward native ESP-IDF (via pioarduino, then Arduino-as-component). Headline architecture: 500 Hz control task pinned to core 1, woken by hardware-timer ISR; all radio/comms on core 0; single-slot overwrite queues between them. |
| [05-ml-depth-estimation.md](05-ml-depth-estimation.md) | Should ML depth estimation be added for obstacle avoidance / inter-robot distance? | **No, not now.** MCU monocular depth is ~1 Hz-class and relative-only, while the VL53L5CX measures metric depth at 15 Hz for $33; learned inter-robot vision needs a GAP8-class accelerator. Pose broadcast (+ optional UWB later) dominates. Doc includes explicit revisit criteria. |
| [06-swarm-algorithms-catalog.md](06-swarm-algorithms-catalog.md) | Quick-reference catalog | Condensed one-table-per-category summary of all 17 algorithm families investigated (coordination + inter-agent avoidance), with verdicts and the ranked recommendation. |
| [07-camera-depth-version-spec.md](07-camera-depth-version-spec.md) | Design spec for the camera + ML depth version | Experimental track: XIAO ESP32S3 Sense (~$14), "SectorNet-8" int8 sector classifier (25–50 ms inference, 10–15 Hz), self-supervised training with the co-mounted VL53L5CX as teacher, advisory-only integration, explicit exit criteria. |

Implementation planning lives in `docs/plan/`:
[firmware-implementation-outline.md](../plan/firmware-implementation-outline.md) (work packages,
milestones, open decisions) and [avoidance-method-versions.md](../plan/avoidance-method-versions.md)
(the parallel perception/avoidance version tracks and named builds A–F).

## Cross-cutting conclusions

1. **Feasibility verdict: yes, comfortably — on the compute side.** All four investigations
   independently converge on the same finding: classical swarm coordination (boids, potential
   fields, consensus flocking) plus reactive obstacle avoidance (VFH+, BVC) plus a 500 Hz control
   loop with Mahony/Madgwick fusion fits on a classic 240 MHz dual-core ESP32 with large margin.
   The existence proofs are Crazyflie (STM32F4, 168 MHz, 192 KB RAM — strictly weaker) and
   Espressif's own ESP-Drone port.

2. **The real constraints are sensing and radio, not CPU.** nRF24L01/ESP-NOW give no relative
   positioning, so every metric algorithm implies each robot broadcasting its dead-reckoned pose
   (a 16–32 byte packet at ~10 Hz). Packet loss, not bandwidth, is the failure mode — algorithms
   must be tuned with delay/loss modeled in (the Vásárhelyi communication-aware approach) and
   carry stale-state extrapolation and rule-based fallbacks.

3. **Radio direction: prefer ESP-NOW over nRF24L01 for swarm state exchange.** 250-byte payloads,
   1–25 ms latency, broadcast to unlimited unencrypted listeners, and no second 2.4 GHz radio
   competing with Wi-Fi coexistence. The nRF24L01 path in the current sketch remains fine for the
   manual RC control link during bring-up.

4. **TinyML is a phase-2 option, not a starting point.** Inference of small policies is feasible
   (30 ms-class on ESP32, ~7× faster on S3), but classical controllers are the sane first
   implementation; learned methods add sim-to-real and training burden without near-term payoff.

5. **Hardware guidance:** keep the DOIT DevKit v1 (classic ESP32) for current vehicles; choose
   ESP32-S3 for any newly purchased boards. Avoid C3/C6 (single core, no FPU) and P4 (no radio)
   for this project.

## Suggested next steps (for the firmware phase)

1. Restructure the firmware skeleton per doc 04: core-pinned FreeRTOS tasks, hardware-timer-paced
   control loop, overwrite queues — while still on Arduino/PlatformIO (pioarduino fork).
2. Add ESP-NOW pose-broadcast alongside the existing nRF24 RC link; define the 32-byte state
   packet (`id, x, y, θ, v, timestamp`).
3. Implement the boids/potential-field controller from doc 01 with parameters exposed for tuning.
4. Bench one VL53L5CX per vehicle and integrate VFH+ steering plus the stop-reflex layer (doc 02).
5. Add the BVC inter-agent filter once ≥2 vehicles exchange poses reliably.
