# Swarm Drone

ESP32 swarm drone firmware: decentralized flocking + obstacle avoidance on
microcontroller-class hardware, with **two interchangeable perception
versions** (VL53L5CX ToF and camera + TinyML depth-to-sectors) and every
major hardware choice kept open as a build-time fork until the PCB is final.

> **Start here: [`docs/HANDBOOK.md`](docs/HANDBOOK.md)** — the complete,
> self-contained handoff document (architecture, forks awaiting decision,
> safety procedures, how to build/flash/train, research summaries).
> Then **[`docs/ROADMAP.md`](docs/ROADMAP.md)** — what to build next, with
> the relevant research pre-digested per item. Every directory has its own
> README for navigation.

## Quick start

```bash
pip install platformio          # or use the PlatformIO IDE

pio test -e native              # 26 host unit tests for the core logic
pio run -e devkit_v1_tof        # Build A  — DevKit v1 + ToF + ESC + nRF24 RC
pio run -e devkit_v1_brushed    # Build A' — brushed motors + ICM42688 + ESP-NOW RC
pio run -e xiao_s3_vision       # Build B  — XIAO ESP32S3 Sense camera+TFLM+ToF+SD
pio run -e devkit_v1_tof -t upload
```

**SAFETY: attitude gains are untuned placeholders. Props off for first
power-up.** Arming is stick-gesture gated (throttle low + yaw right, 1 s);
RC loss for 150 ms cuts motors; a fleet-wide E-stop broadcast latches.

## Repository map

| Path | Contents |
|---|---|
| [`lib/swarmcore/`](lib/swarmcore/README.md) | Portable core logic (no Arduino): packets, sector filter, neighbor table, boids, leader-follower, gradient, VFH-lite, BVC, governor, Mahony AHRS, attitude PIDs, mixer, arming, behavior pipeline. Unit-tested natively. |
| [`src/`](src/README.md) | ESP32 glue: FreeRTOS tasks (control 500 Hz / avoid 50 Hz on core 1; swarm, sensors, telemetry on core 0), drivers for every fork variant, `config/` pin maps. |
| [`ml/`](ml/README.md) | Host-side training pipeline: SEC1 log parser, **both** model forks (SectorNet-8, uPyD-Net-lite), int8 export, eval, firmware embedding. |
| [`test/`](test/README.md) | Native Unity tests (`pio test -e native`). |
| [`docs/HANDBOOK.md`](docs/HANDBOOK.md) | **The handoff document.** |
| [`docs/ROADMAP.md`](docs/ROADMAP.md) | **What to build next** — each item with its research pre-digested, the code seam, and acceptance criteria. |
| [`docs/research/`](docs/research/README.md) | 10 research documents, annotated index in the README. |
| [`docs/plan/`](docs/plan/README.md) | Implementation outline + the versions/builds plan. |

## The fork matrix (decisions you still get to make)

Every fork compiles today; pick per env in `platformio.ini`. Details and
decision criteria: HANDBOOK §"Forks awaiting decision".

- **Perception:** V1 ToF (reference) / V2 camera+ML (advisory, S3) — both at once on Build B
- **ML model:** SectorNet-8 classifier / uPyD-Net-lite dense depth — both trainable now
- **Motors:** ESC PWM 50 Hz / brushed 20 kHz LEDC
- **IMU:** MPU6050 / ICM-42688 / mock
- **RC link:** nRF24L01 (legacy TX) / ESP-NOW
- **Swarm behavior:** manual / hold / leader-follow / flock / disperse (runtime mode)
