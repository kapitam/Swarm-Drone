# src/ — ESP32 firmware layer

The thin, hardware-facing shell around `lib/swarmcore`. Architecture follows
research doc 04 §7.2 exactly; see `docs/HANDBOOK.md` §3 for the task table
and data-flow description.

## Layout

| Path | Contents |
|---|---|
| `main.cpp` | Boot: motors to safe state first, config load, spawn all tasks, print the fork banner. `loop()` self-deletes — everything is FreeRTOS tasks. |
| `state_bus.h` | The ONLY inter-task data path: spinlock-guarded single-writer snapshots (RC, control state, ToF sectors + raw grid, vision sectors, swarm view) + one length-1 overwrite queue (`AttitudeSetpoint`). Control loop never blocks. |
| `config/config.h` | Priority ladder, core pinning, rates, radio constants, safety thresholds. One documented place (Crazyflie-style). |
| `config/pins.h` | Pin maps per `BOARD_*` flag. **The PCB lands here** as a new `BOARD_PCB_V1` block — nothing else should need edits. |
| `config/config_store.*` | NVS persistence (robot id). Writes only while disarmed. |
| `tasks/task_control.cpp` | `tCtrl`: 500 Hz, core 1, prio 20. GPTimer ISR → task notification (esp_timer ISR-dispatch is NOT enabled in Arduino's sdkconfig — that's why GPTimer). IMU→Mahony→arming→attitude→mixer→motors; TWDT-fed. |
| `tasks/task_avoid.cpp` | `tAvoid`: 50 Hz, core 1, prio 15. Builds `BehaviorInputs` (fuses ToF+vision for steering, ToF-only for reflex) → `BehaviorPipeline` → setpoint queue. |
| `tasks/task_swarm.cpp` | `tSwarm`: core 0. Owns the ESP-NOW RX queue; dispatches beacons/commands/RC-over-ESP-NOW; broadcasts our phase-jittered `StatePacket` at 10 Hz; publishes `SwarmSnapshot`. |
| `tasks/task_telem.cpp` | `tTelem`: 5 Hz JSON line on USB serial + control-loop rate supervision (health). Greppable keys — see HANDBOOK §5.2. |
| `platform/motors.*` | Motor fork: `MOTORS_ESC_PWM` (50 Hz, 1000–2000 µs, 14-bit LEDC) / `MOTORS_BRUSHED_PWM` (20 kHz, 10-bit). Arduino core-3 LEDC API (`ledcAttach`/`ledcWrite`). |
| `platform/imu.*` | IMU fork: MPU6050 / ICM42688 (bench-untested) / MOCK. Axis mapping is mounting-dependent — fix signs here when the PCB fixes orientation. |
| `platform/rc_link.*` | RC fork: nRF24 poll task (VSPI, byte-compatible with the original transmitter) / ESP-NOW-fed. |
| `platform/espnow_link.*` | ESP-NOW broadcast: callbacks ONLY copy+enqueue (they run inside the Wi-Fi task — doc 04 §5.2 rule), counters for link stats. |
| `platform/tof_vl53l5cx.*` | V1 perception service (core 0): doc 08 init checklist verbatim, publishes filtered sectors + raw 8x8 grid (for the dataset logger). Re-init loop on failure; governor sees BLIND. |
| `vision/` | V2 perception (S3 only): `vision_task.cpp` (96×96 gray capture, 20→10 MHz XCLK fallback, SD 'SEC1' logger with PSRAM ring), `backend_tflm.cpp` (TFLM from the Arduino core, 96 KB SRAM arena, auto-detects SectorNet vs depth-map output), `backend_stub.cpp`, `model_data.cc` (placeholder until `ml/gen_c_array.py`). |

## Fork flags (set per env in platformio.ini — never in code)

`BOARD_DEVKIT_V1|BOARD_XIAO_S3`, `PERCEPTION_V1_TOF`, `PERCEPTION_V2_VISION`,
`VISION_BACKEND_TFLM|VISION_BACKEND_STUB`, `DATALOGGER_SD`,
`MOTORS_ESC_PWM|MOTORS_BRUSHED_PWM`, `IMU_MPU6050|IMU_ICM42688|IMU_MOCK`,
`RC_LINK_NRF24|RC_LINK_ESPNOW`, `ROBOT_ID_DEFAULT`.

## Rules of the road

- Real-time work on core 1 only; anything that can block (I2C frames, SD,
  Wi-Fi) lives on core 0.
- Radio callbacks never take locks or do work — enqueue and leave.
- New sensors publish through `StateBus`; new perception = one driver that
  fills a `sc::SectorArray` (that is the whole interchangeability contract).
- Safety changes (arming, governor, failsafe) belong in `swarmcore` with
  tests, not here.
