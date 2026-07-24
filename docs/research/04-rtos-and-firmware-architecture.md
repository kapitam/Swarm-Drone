# RTOS Options and Firmware Architecture for the ESP32 Swarm Vehicle

**Research document — no code changes.** Prepared for the swarm-robotics project currently running on an ESP32 DOIT DevKit v1 (classic dual-core ESP32), PlatformIO + Arduino framework, nRF24L01 radio, ESC/servo output, single superloop sketch. Target firmware: fast control loop (PWM/ESC, sensor fusion), obstacle avoidance, swarm coordination messaging, and telemetry with real-time guarantees.

---

## 1. Executive Summary

- **There is no "add an RTOS" decision to make on ESP32 — FreeRTOS is already running.** The Arduino-ESP32 core is built on ESP-IDF, and `loop()` is just one FreeRTOS task (`loopTask`) pinned to core 1. The real decision is *how much of FreeRTOS to use and through which framework*.
- **Recommendation: ESP-IDF's built-in FreeRTOS (IDF FreeRTOS), accessed via native ESP-IDF, optionally through a transitional "Arduino as an ESP-IDF component" phase.** IDF FreeRTOS is a dual-core SMP fork of vanilla FreeRTOS with per-core scheduling and task affinity (`xTaskCreatePinnedToCore`), which maps perfectly onto this project's needs: control loop pinned to core 1, Wi-Fi/radio/comms on core 0.
- **Zephyr and NuttX are credible but wrong for this project today.** Both are officially supported by Espressif, but Zephyr's classic-ESP32 story still has peripheral gaps and constrained SMP, and both would force abandoning the ESP32 Arduino/IDF driver ecosystem (LEDC/MCPWM, ESP-NOW, RF24 library) for little real-time gain. ThreadX (ex-Azure RTOS) has no official ESP32 port; Mongoose OS is a low-activity niche framework. Neither matters here.
- **The strongest architectural precedents are Crazyflie and its official ESP32 port, ESP-Drone.** Both are FreeRTOS task-per-subsystem designs with a 1 kHz stabilizer task at the top of the priority ladder, sensor-interrupt-driven timing, and communication decomposed into link/protocol/application tasks. Betaflight proves the opposite point — a gyro-synchronized cooperative superloop also works — but only because it owns the whole CPU; on ESP32 the Wi-Fi stack already imposes an RTOS, so fighting it is pointless.
- **Headline architecture:** a ~500 Hz control task pinned to core 1 at high priority, woken by a hardware-timer (`esp_timer`/GPTimer) ISR via direct-to-task notification; all radio work (nRF24 IRQ service task, ESP-NOW RX/TX tasks, telemetry) on core 0 below the Wi-Fi task; setpoints and neighbor state passed to core 1 through single-slot overwrite queues so the control loop never blocks. Details and full task table in §7.
- **Tooling:** stay on PlatformIO short-term via the community **pioarduino** fork (official PlatformIO is frozen at Arduino core 2.x / IDF 4.4-era), and plan the migration to native ESP-IDF (`idf.py`) for menuconfig control, unit testing (Unity/CMock, Linux-host tests), and SEGGER SystemView tracing. JTAG on the DevKit v1 requires an external ESP-Prog wired to GPIO12–15.

---

## 2. RTOS Landscape on ESP32

### 2.1 ESP-IDF's built-in FreeRTOS ("IDF FreeRTOS") — the incumbent

ESP-IDF ships a modified FreeRTOS kernel, based on vanilla FreeRTOS v10.5.1, with substantial changes to kernel behavior and API to support **dual-core symmetric multiprocessing (SMP)** on ESP32/ESP32-S3/ESP32-P4 ([ESP-IDF FreeRTOS guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos_idf.html)). The full change list is documented in the source tree ([idf_changes.md](https://github.com/espressif/esp-idf/blob/12f36a02/components/freertos/FreeRTOS-Kernel/idf_changes.md)). The differences that matter for firmware design:

| Vanilla FreeRTOS | IDF FreeRTOS |
|---|---|
| Single core; one global scheduler picks the highest-priority ready task | Two cores run concurrently; **each core independently schedules** the highest-priority ready task *it is allowed to run* |
| No concept of core affinity | Every task has an affinity: core 0, core 1, or `tskNO_AFFINITY`. Created via `xTaskCreatePinnedToCore()` / `xTaskCreateStaticPinnedToCore()`; the vanilla `xTaskCreate()` still exists but maps to `tskNO_AFFINITY` |
| One idle task | Two idle tasks, one pinned per core |
| One tick source | Both cores receive tick interrupts, but only core 0 increments the tick count and unblocks timed-out tasks (`xTaskIncrementTick()` on core 0, `xTaskIncrementTickOtherCores()` on core 1) |
| `taskSELECT_HIGHEST_PRIORITY_TASK()` global | `prvSelectHighestPriorityTaskSMP()` per core — picks highest-priority ready task with compatible affinity not already running on the other core |

Practical consequences:

- **Round-robin between equal-priority tasks is "best effort" across cores**, not the strict time slicing of vanilla FreeRTOS — another reason to give hard-real-time tasks a unique priority and a fixed core.
- Historically, core 0 is called `PRO_CPU` (protocol: Wi-Fi/BT stacks live there by default) and core 1 `APP_CPU` (application) ([ESP-IDF v4.2 SMP guide](https://docs.espressif.com/projects/esp-idf/en/v4.2.5/esp32/api-guides/freertos-smp.html)). This split is a convention baked into default task placement, and it is exactly the split a robot firmware wants.
- IDF FreeRTOS can be built single-core (`CONFIG_FREERTOS_UNICORE`), restoring near-vanilla behavior — irrelevant here but useful for porting to single-core variants (ESP32-C3/S2) later.

**Verdict:** zero-integration-cost, best driver support, dual-core SMP is a genuine asset for isolating the control loop. This is the default choice.

### 2.2 Arduino on ESP32: pure Arduino vs Arduino-as-a-component

**Arduino on ESP32 already runs *on* FreeRTOS.** The Arduino core is a layer over ESP-IDF: at boot, the core creates a FreeRTOS task (`loopTask`, on the core set by `CONFIG_ARDUINO_RUNNING_CORE`, default core 1) that calls `setup()` once and then `loop()` forever. Practically this means:

- Calling `xTaskCreatePinnedToCore()`, using queues, semaphores, and notifications **works today inside an Arduino sketch** — the kernel is there. Community write-ups warn, however, that mixing "Arduino's single-threaded illusion and FreeRTOS's multi-threaded reality" without understanding it breeds priority inversions, undersized stacks, and starved idle tasks/watchdog resets ([Hubble: ESP-IDF vs Arduino, when to switch](https://hubble.com/community/comparisons/esp-idf-vs-arduino-framework-for-esp32-when-to-switch-and-why/)).
- What pure Arduino **cannot** do is reconfigure the platform: no `menuconfig`/`sdkconfig` access, so the FreeRTOS tick rate, task watchdog timeout, Wi-Fi task placement, IRAM options, etc. are fixed at whatever the prebuilt Arduino libs chose.
- **Arduino as an ESP-IDF component** is the officially maintained middle path ([docs](https://docs.espressif.com/projects/arduino-esp32/en/latest/esp-idf_component.html)): an ESP-IDF project pulls the Arduino core in via the IDF Component Manager, giving full `sdkconfig` control while keeping Arduino APIs and libraries (e.g. the `RF24` and `ESP32Servo` libs already in this project's `platformio.ini`). You choose either autostarted `setup()`/`loop()` or your own `app_main()` calling `initArduino()`. Note the Arduino component *requires* `CONFIG_FREERTOS_HZ = 1000` — conveniently the tick rate a 1 ms-granularity robot loop wants anyway. Current compatibility: Arduino core 3.3.x ↔ ESP-IDF v5.5.
- Community guidance (and experience from projects like [cosmoBots/esp_idf_arduino](https://github.com/cosmoBots/esp_idf_arduino)) is that the hybrid is **a bridge, not a destination**: run it while porting drivers, then let ESP-IDF own each subsystem ([Hubble comparison](https://hubble.com/community/comparisons/esp-idf-vs-arduino-framework-for-esp32-when-to-switch-and-why/)).

### 2.3 Zephyr RTOS on ESP32

Espressif has contributed to Zephyr since May 2020 and maintains an official [Zephyr support status page](https://developer.espressif.com/software/zephyr-support-status/). Status as of 2025–2026:

- Espressif declared Zephyr **production-ready starting with Zephyr v4.0, targeting the ESP32-C3**; other chips (including classic ESP32, S2, S3, C6) are supported at varying levels, and new silicon keeps landing upstream (e.g. [ESP32-C5 SoC + board support PR, merged 2026](https://github.com/zephyrproject-rtos/zephyr/pull/105674)). Espressif recommends a rolling-release model tracking upstream rather than pinning old releases.
- Core connectivity is real: SPI, I2C, UART, Wi-Fi with the full Zephyr networking stack, BLE, MCUboot/OTA ([Hubble: Zephyr on ESP32](https://hubble.com/community/guides/getting-started-with-zephyr-rtos-on-esp32-why-it-might-replace-esp-idf-for-your-next-product/)).
- Gaps that matter here: **peripheral drivers lag ESP-IDF** (certain ADC modes, PCNT, LCD, camera, ULP), power management is more limited, and **multi-core support on the classic ESP32 is constrained** — Espressif's own tracking issue notes open questions around SMP/AMP and the XIP/dual-core SPI interactions ([zephyrproject-rtos/zephyr#29394](https://github.com/zephyrproject-rtos/zephyr/issues/29394)). ESP-NOW, central to a cheap swarm mesh, is an Espressif-proprietary protocol exposed through ESP-IDF, not a Zephyr-native API.
- Pros, for completeness: vendor-neutral portability (600+ boards), devicetree/Kconfig discipline, an excellent native BLE stack, strong long-term multi-vendor story.

**Verdict:** the right answer if the swarm later migrates off Espressif silicon or needs Zephyr's networking/BLE stack; the wrong answer for squeezing real-time performance out of a classic ESP32 with ESP-NOW and Espressif-specific peripherals in 2026.

### 2.4 Apache NuttX

Espressif **officially supports NuttX** on the ESP32, ESP32-S and ESP32-C series, alongside Zephyr ([Espressif announcement](https://www.espressif.com/en/news/new_operating_systems_in_ESP32), [getting-started guide](https://developer.espressif.com/blog/2020/11/nuttx-getting-started/)). NuttX is a POSIX-compliant RTOS; Espressif publishes ongoing work including Wi-Fi driver refactors ([apache/nuttx#17008, 2025](https://github.com/apache/nuttx/pull/17008)) and motor-control application notes using MCPWM on ESP32-C6 ([NuttX for motor control, 2025](https://developer.espressif.com/blog/2025/05/nuttx-motor-control-and-sensing/)). Its POSIX model (files, devices, `poll()`) is attractive if the team already thinks in Unix idioms — it is what PX4 runs on for STM32 flight controllers. But on ESP32 it is a smaller community than ESP-IDF, with the same ecosystem cost as Zephyr (no Arduino libraries, its own driver layer). **Worth knowing about; not worth adopting here.**

### 2.5 ThreadX, Mongoose OS, and other alternatives

- **Eclipse ThreadX** (formerly Azure RTOS): Microsoft contributed it to the Eclipse Foundation; the transition to MIT-licensed open source completed in 2024 ([Microsoft announcement](https://techcommunity.microsoft.com/blog/iotblog/azure-rtos-transition-to-open-source-is-now-complete/4105027), [threadx.io](https://threadx.io/)). Technically excellent and safety-certified, but there is **no official Espressif port or ecosystem** — you would hand-port it and rewrite every driver. Not relevant.
- **Mongoose OS**: an IoT firmware *framework* (cloud-connectivity-oriented, C/JavaScript) rather than a robotics RTOS; community activity is very low compared to Zephyr/FreeRTOS ([comparison data](https://www.libhunt.com/compare-mongoose-os-vs-zephyr)). Not relevant.

### 2.6 Comparison table

| Option | Kernel / real-time model | ESP32 (classic) maturity | Dual-core use | Ecosystem fit for this project | Migration cost from current code | Verdict |
|---|---|---|---|---|---|---|
| **Pure Arduino (current)** | IDF FreeRTOS underneath; `loop()` = one task | Mature | Possible (`xTaskCreatePinnedToCore` available) but platform config locked | RF24, ESP32Servo libs work today | None | OK for prototyping; too little control for real-time guarantees |
| **Arduino as ESP-IDF component** | IDF FreeRTOS, full `sdkconfig` | Mature (Arduino 3.3.x ↔ IDF 5.5) | Full | Keeps Arduino libs during transition | Low–moderate (project restructure) | **Recommended transition path** |
| **Native ESP-IDF (IDF FreeRTOS)** | FreeRTOS 10.5.1 fork, SMP, per-core scheduling, task affinity | Reference platform | Full, first-class | ESP-NOW, LEDC/MCPWM, GPTimer, tracing, unit tests all native | Moderate (rewrite Arduino-only drivers) | **Recommended destination** |
| **Zephyr** | Own preemptive kernel, devicetree/Kconfig | Production-ready per Espressif from v4.0 (first on C3); classic-ESP32 peripheral gaps, constrained SMP | Constrained on classic ESP32 | No ESP-NOW API, no Arduino libs | High | Revisit only for multi-vendor future |
| **NuttX** | POSIX RTOS, officially supported by Espressif | Good and improving | Yes | POSIX, PX4 heritage; no Arduino libs | High | Interesting, not justified |
| **Eclipse ThreadX** | Preemptive RTOS, MIT since 2024 | No official ESP32 port | — | None | Very high | Not applicable |
| **Mongoose OS** | Framework over FreeRTOS, cloud-IoT focus | Niche, low activity | — | Wrong domain (cloud IoT, not robotics) | High | Not applicable |

---

## 3. Real-Time Design Patterns for Robot Firmware on (IDF) FreeRTOS

### 3.1 Task decomposition and core pinning

The canonical ESP32 robot layout mirrors the silicon's PRO/APP split:

- **Core 1 (APP_CPU): the hard-real-time domain.** Control loop task (sensor read → fusion → PID → PWM/ESC write) pinned here, at a priority above everything else on that core. Nothing on core 1 should ever hold a lock the control task needs or run long critical sections.
- **Core 0 (PRO_CPU): the "everything with unbounded latency" domain.** The Wi-Fi/BT stacks, LwIP, ESP-NOW callbacks, and system housekeeping already run there by default; put radio service, swarm messaging, telemetry, and logging tasks there too, *below* the Wi-Fi task's priority.

Use `xTaskCreatePinnedToCore()` for every task — explicit affinity makes worst-case interference analyzable, whereas `tskNO_AFFINITY` tasks migrate and defeat per-core reasoning ([IDF FreeRTOS guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos_idf.html)).

### 3.2 Priorities and priority inversion

- Give each real-time task a **unique priority**; reserve equal priorities for genuinely interchangeable background work. Crazyflie's `config.h` is a good model of a documented priority ladder (§4.1).
- **Priority inversion** arises when a high-priority task blocks on a mutex held by a low-priority task that a medium-priority task is preempting. Defenses: use FreeRTOS **mutexes** (which implement priority inheritance) rather than binary semaphores for shared resources like the SPI bus; keep critical sections tiny; better yet, structure the design so the control loop *never takes a mutex* — give shared data a lock-free handoff (single-writer/single-reader, `xQueueOverwrite`, or double buffering). The FreeRTOS book's interrupt-management chapter covers these trade-offs ([Mastering the FreeRTOS Kernel, ch. 7](https://freertos.gitbook.io/mastering-the-freertos-tm-real-time-kernel/mastering.ch07)).
- A classic ESP32 deadlock case: a task holding a lock that an ESP-NOW/Wi-Fi callback then tries to take from the *higher-priority* Wi-Fi task — documented in the wild ([esp-idf-svc#315](https://github.com/esp-rs/esp-idf-svc/issues/315)). Rule: callbacks never take application locks; they only enqueue.

### 3.3 Timing the control loop: `vTaskDelayUntil` vs hardware timers

Three tiers, in increasing precision ([ESP-Techpedia timer comparison](https://docs.espressif.com/projects/esp-techpedia/en/latest/esp-friends/advanced-development/system/timer.html), [esp_timer docs](https://github.com/espressif/esp-idf/blob/master/docs/en/api-reference/system/esp_timer.rst)):

1. **`vTaskDelayUntil()`** — drift-free periodic scheduling, but resolution is the FreeRTOS tick (1 ms at `CONFIG_FREERTOS_HZ=1000`), and wake-up jitter is up to a tick plus scheduling latency. Fine for ≤ 100 Hz tasks (avoidance, telemetry). Note the default tick on some configs is 100 Hz — set it to 1000 explicitly ([esp32.com discussion of tick-boundary behavior](https://esp32.com/viewtopic.php?t=38644)).
2. **`esp_timer` (high-resolution timer)** — microsecond-resolution callbacks; use the ISR-dispatch mode and have the callback do nothing but `vTaskNotifyGiveFromISR()` to the control task. Measured wake-up latency from ISR to `ulTaskNotifyTake()` return is ~15 µs on a 240 MHz ESP32-S3, largely independent of priority/affinity ([esp32.com measurement thread](https://esp32.com/viewtopic.php?t=38644)) — ample for a 500 Hz–1 kHz loop.
3. **GPTimer** — dedicated Timer Group hardware, highest precision, exclusive use of the peripheral, alarm callback in ISR context (must use only `FromISR` APIs) ([GPTimer docs](https://docs.espressif.com/projects/esp-idf/en/v5.1.7/esp32/api-reference/peripherals/gptimer.html)). Choose this over `esp_timer` if you want the control tick isolated from the shared `esp_timer` dispatch task/ISR.

The **best trigger of all is the sensor itself**: if the IMU exposes a data-ready interrupt, drive the loop from that GPIO interrupt (as Crazyflie and Betaflight do, §4) so control always operates on fresh samples with zero sampling/loop drift.

### 3.4 ISR-to-task handoff

FreeRTOS's own guidance, in preference order for this project ([task notifications](https://www.freertos.org/Documentation/02-Kernel/02-Kernel-features/03-Direct-to-task-notifications/01-Task-notifications), [Mastering FreeRTOS ch. 7](https://freertos.gitbook.io/mastering-the-freertos-tm-real-time-kernel/mastering.ch07), [ch. 10](https://freertos.gitbook.io/mastering-the-freertos-tm-real-time-kernel/mastering.ch10)):

- **Direct-to-task notifications** — ~45% faster and ~8 bytes RAM vs unblocking via a semaphore/queue; the right tool whenever exactly one task consumes the event (timer tick → control task, nRF24 IRQ → radio task).
- **Queues** — when multiple data items must be buffered/copied with FIFO order (received radio packets, telemetry records). For "latest value wins" data (setpoints, neighbor state) use a **length-1 queue with `xQueueOverwrite()`** — the Crazyflie codebase explicitly uses this pattern for inputs where stale data should be discarded ([Bitcraze system-task howto](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/development/systemtask/)).
- **Stream buffers** — single-writer/single-reader byte streams (e.g. a UART/telemetry pipe); internally built on task notifications ([stream buffer docs](https://docs.aws.amazon.com/freertos/latest/userguide/inter-task-coordination.html)).

Always use `*FromISR` variants inside ISRs and pass/honor `pxHigherPriorityTaskWoken` + `portYIELD_FROM_ISR()` so the woken task runs immediately.

### 3.5 Avoiding flash-cache stalls (`IRAM_ATTR`)

On ESP32, code normally executes from external flash through the MMU cache. **During any SPI flash write/erase (NVS commits, OTA, core dumps), the cache is disabled: non-IRAM-safe interrupts are masked, other tasks are suspended, and the other core busy-polls** ([SPI flash concurrency constraints](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_flash/spi_flash_concurrency.html)). For a robot this is a direct source of control-loop jitter and missed deadlines. Mitigations:

- Register time-critical interrupts with `ESP_INTR_FLAG_IRAM` and mark the handler *and everything it calls* `IRAM_ATTR`, with constants in `DRAM_ATTR`; otherwise the handler is simply suppressed while the cache is off — or crashes with an Illegal Instruction if mis-placed ([memory types guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/memory-types.html), [esp_attr.h](https://github.com/espressif/esp-idf/blob/master/components/esp_common/include/esp_attr.h)).
- Better: **don't write flash while driving actuators.** Confine NVS/parameter writes to disarmed/idle states.
- Hot control-path functions can also be placed in IRAM purely to avoid cache-miss latency variance.

### 3.6 Watchdogs

ESP-IDF provides an **Interrupt Watchdog** (catches ISRs/critical sections that hog a core) and a **Task Watchdog Timer (TWDT)** which by default watches the idle tasks and can have application tasks subscribed to it ([watchdogs API guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/wdts.html)). Patterns for this firmware:

- Subscribe the control task to the TWDT and reset it each cycle — a hung control loop must reboot into a failsafe (motors off), not freeze at last throttle.
- Keep enough idle time on both cores that the idle-task watchdog is fed; ESP-Drone documents exactly this failure (Kalman task starving core → TWDT trip) and fixes it by priority tuning (§4.2).
- Add an application-level **rate supervisor** (Crazyflie has one) that checks the control loop actually ran N times per second and triggers failsafe if not.

---

## 4. Case Studies: How Existing Flight Controllers / Swarm Robots Structure Firmware

### 4.1 Crazyflie (STM32F4 + FreeRTOS) — task-per-subsystem, interrupt-paced 1 kHz stabilizer

The Bitcraze Crazyflie — the most-studied swarm research platform — is a straight FreeRTOS design ([architecture wiki](https://wiki.bitcraze.io/projects:crazyflie:firmware:arch)). Its priority ladder is defined in one header ([`src/config/config.h`](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/config/config.h)), higher number = higher priority:

| Priority | Tasks (selection) |
|---|---|
| 5 | `STABILIZER` (control loop), `RATE_SUPERVISOR`, `PASSTHROUGH` |
| 4 | `SENSORS` (IMU read/pre-process) |
| 3 | `SYSLINK`/`USBLINK` (radio link), `LPS`/`LIGHTHOUSE`/`OA` deck drivers, `ADC` |
| 2 | `SYSTEM`, `CRTP_TX`, `CRTP_RX`, `KALMAN` (estimator), `ZRANGER`, `CMD_HIGH_LEVEL` |
| 1 | `LOG`, `MEM`, `PARAM`, `WORKER`, `SUPERVISOR` |
| 0 | `PM` (power management), `PROXIMITY`, `CRTP_SRV` |

Key mechanics from [`stabilizer.c`](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/stabilizer.c):

- The stabilizer loop runs at **1 kHz, unblocked by the sensor data-ready interrupt** (`sensorsWaitDataReady()`), not by a time delay — control always consumes a fresh IMU sample. Sub-rate functions run slower by *skipping calls* inside the 1 kHz loop.
- A **rate supervisor** verifies the loop achieves 997–1003 iterations/second and flags failure.
- Communication is layered into tasks: link (`SYSLINK`) → protocol (`CRTP_RX`/`CRTP_TX`) → application modules, connected by queues; modules block on their port queue ([system-task guide](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/development/systemtask/)).
- Memory is statically allocated; stacks sized per task (stabilizer gets 3× minimal stack).

### 4.2 ESP-Drone (Espressif's Crazyflie port to ESP32/S2/S3) — the closest existing analog

[ESP-Drone](https://github.com/espressif/esp-drone) ports the Crazyflie flight kernel (tag 2021.01) onto ESP-IDF FreeRTOS, replacing the nRF radio link with **Wi-Fi UDP carrying CRTP** ([communication docs](https://docs.espressif.com/projects/espressif-esp-drone/en/latest/communication.html)). Its task inventory ([flight control system docs](https://docs.espressif.com/projects/espressif-esp-drone/en/latest/system.html)): `SENSORS`, `KALMAN`, `STABILIZER` (highest priority), `PWRMGNT`, `WIFILINK`, `UDP-RX`, `UDP-TX`, `CRTP-RX`, `CRTP-TX`, `CMDHL`, plus logging/params. Lessons directly transferable to this project:

- **Priorities are a tuning surface across chip variants:** on dual-core ESP32 the CPU-heavy `KALMAN_TASK` can run at high priority, but on single-core ESP32-S2 it must be *lowered* or it starves the idle task and **trips the task watchdog** — Espressif documents this explicitly. Dual-core headroom is a real architectural asset.
- **Stack sizes matter and are configurable** (`BASE_STACK_SIZE` 2048 on ESP32 vs 1024 on S2); tune with `uxTaskGetStackHighWaterMark`.
- The Crazyflie task/queue architecture survives the port intact — evidence that this shape of firmware fits ESP-IDF FreeRTOS well.

### 4.3 Betaflight / iNav (STM32) — the disciplined superloop alternative

Betaflight deliberately **rejects an RTOS** in favor of a cooperative scheduler running from `main()` ([scheduler overview](https://deepwiki.com/betaflight/betaflight/8.1-task-scheduler), [RTOS discussion, betaflight#1681](https://github.com/betaflight/betaflight/issues/1681)):

- **Realtime tasks** (gyro → filter → PID) live *outside* the task queue and are phase-locked to the **gyro's EXTI data-ready interrupt**; since 4.3, SPI gyro reads are DMA + EXTI-triggered, nearly eliminating loop jitter ([4.3 tuning notes](https://betaflight.com/docs/wiki/tuning/4-3-Tuning-Notes), [deep dive](https://betaflight.com/docs/wiki/guides/current/Deep-Dive)).
- **All other tasks** sit in a dynamic priority queue and are dispatched only if their measured execution time fits in the gap before the next gyro cycle ([PR #11354](https://github.com/betaflight/betaflight/pull/11354)).
- The maintainers' rationale: with one dominant task and statically allocated memory, cooperative scheduling avoids preemption overhead and mutexes entirely.

**Lesson for ESP32:** the "gyro-interrupt-paced, everything-else-fits-in-the-gaps" philosophy is worth stealing, but the superloop *implementation* is not portable here — the ESP32's Wi-Fi/BT stack already runs preemptive FreeRTOS tasks that the application cannot schedule around. On ESP32 you get the same effect by pinning a top-priority, interrupt-woken control task to core 1 and exiling everything else to core 0.

### 4.4 Synthesis — lessons applied

1. Task-per-subsystem on FreeRTOS is the proven shape for exactly this class of vehicle (Crazyflie, ESP-Drone).
2. Pace the control loop with a hardware interrupt (IMU data-ready if available, else GPTimer/esp_timer), never with `delay()`-style waiting.
3. Maintain one explicit, documented priority ladder in one header; unique priorities for real-time tasks.
4. Add a rate supervisor + task watchdog from day one.
5. Layer comms into link → protocol → application tasks joined by queues; drop stale setpoints with overwrite semantics.
6. Keep heavy estimation (Kalman) out of the hard 1 kHz path or budget it explicitly; on dual-core ESP32, give it the spare core capacity.

---

## 5. Comms Stack Integration

### 5.1 nRF24L01 under an RTOS

The nRF24L01+ is serviced over SPI with an active-low **IRQ pin** that signals RX-ready / TX-done / max-retries. The recommended RTOS integration is classic deferred interrupt handling (demonstrated in the RF24 ecosystem's FreeRTOS example, [maniacbug pingpair/radio.cpp](https://github.com/maniacbug/FreeRTOS/blob/master/examples/pingpair/radio.cpp), and the [RF24 interrupt example](https://rf24.readthedocs.io/en/stable/examples_2InterruptConfigure_2InterruptConfigure_8ino-example.html)):

1. GPIO ISR on the IRQ pin (falling edge), marked `IRAM_ATTR`, does nothing but `vTaskNotifyGiveFromISR()` (or a binary-semaphore give) to a dedicated **radio task**, then `portYIELD_FROM_ISR()`.
2. The radio task blocks on the notification; on wake it calls `radio.whatHappened(tx, fail, rx)` / `clearStatusFlags()`, drains the RX FIFO into a packet queue, and re-arms. The RF24 docs warn that the pipe-number status is unreliable *during* the IRQ edge, so clear status flags before reading it in the handler path ([RF24 class reference](https://nrf24.github.io/RF24/classRF24.html)).
3. If any other task shares the SPI bus, guard it with a mutex (priority inheritance). Best is to make the radio task the *sole owner* of that SPI bus so no lock is needed.
4. Polling (`radio.available()` in a periodic task) also works and is simpler, at the cost of latency = polling period; with a dedicated core-0 task at 250–500 Hz this is acceptable for command traffic. Note the RF24 library is officially Arduino/Linux-HAL oriented; pure-ESP-IDF use requires the Arduino component or a small shim ([RF24#925](https://github.com/nRF24/RF24/issues/925)).

### 5.2 ESP-NOW: callback execution context

ESP-NOW is the strongest candidate for swarm coordination (connectionless peer-to-peer over 802.11, ≤250-byte payloads in v1). The critical integration fact, straight from Espressif ([ESP-NOW API docs](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/network/esp_now.html)):

> The sending/receiving callback function runs from a **high-priority Wi-Fi task**. Do not do lengthy operations in the callback. Instead, post the necessary data to a queue and handle it from a lower priority task.

Implications:

- Callbacks execute in the Wi-Fi driver task's context on **core 0** — blocking there stalls the entire Wi-Fi driver; taking an application mutex there can deadlock against a lower-priority holder ([field report](https://github.com/esp-rs/esp-idf-svc/issues/315)). Espressif's own example and `esp-now` component both copy the payload and `xQueueSend()` immediately ([espnow example](https://github.com/espressif/esp-idf/blob/master/examples/wifi/espnow/main/espnow_example_main.c), [esp-now component](https://github.com/espressif/esp-now/blob/master/src/espnow/src/espnow.c)).
- Delivery is unacknowledged at application level (MAC-level ack only); add sequence numbers and treat neighbor state as best-effort/latest-wins.
- Send pacing: issue the next `esp_now_send()` only after the previous send callback fires.

### 5.3 Keeping radio work from jittering the control loop

- **Core isolation is the primary defense:** Wi-Fi/ESP-NOW machinery and the nRF24 task live on core 0; the control task owns core 1. Per-core scheduling in IDF FreeRTOS means core-0 load does not preempt core 1 ([IDF FreeRTOS guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos_idf.html)).
- Cross-core data flows only through queues/notifications; the control task uses **zero-block reads** (`xQueueReceive` with timeout 0, or peek a double buffer) so a quiet radio never delays a control cycle.
- Beware shared-resource coupling: flash writes triggered by comms (e.g. logging/NVS on receipt of a config packet) stall *both* cores via the cache-disable mechanism (§3.5) — queue such writes for idle/disarmed periods.
- Decision point for the project: **running nRF24L01 and Wi-Fi/ESP-NOW simultaneously means two radios in the 2.4 GHz band** on one small vehicle (interference + one extra SPI device + wiring). A reasonable roadmap is: keep nRF24 for the existing manual-control link short-term, trial ESP-NOW for swarm messaging + telemetry, and consolidate on ESP-NOW if latency/QoS proves adequate — eliminating the external radio entirely, as ESP-Drone did by moving CRTP onto Wi-Fi ([ESP-Drone comms](https://docs.espressif.com/projects/espressif-esp-drone/en/latest/communication.html)).

---

## 6. Tooling and Ecosystem

### 6.1 Build system: PlatformIO vs native ESP-IDF vs Zephyr `west`

- **PlatformIO (current setup):** the official `platform-espressif32` is effectively **frozen at Arduino core 2.x / the IDF 4.4 era** — Espressif ended the collaboration and PlatformIO declined to package Arduino 3.x ([platformio/platform-espressif32#1225](https://github.com/platformio/platform-espressif32/issues/1225)). The community **pioarduino** fork tracks current releases (Arduino 3.3.9 on ESP-IDF v5.5.4 as of its stable release) and is a drop-in `platform =` URL change ([pioarduino README](https://github.com/pioarduino/platform-espressif32/blob/main/README.md), [version map](https://github.com/sivar2311/platform-espressif32-versions)). PlatformIO also supports `framework = espidf` and `framework = arduino, espidf` hybrid builds, but `sdkconfig` handling and IDF version lag are recurring friction points.
- **Native ESP-IDF (`idf.py` + VS Code/Espressif-IDE extension):** the reference environment — full `menuconfig`, component manager (needed for Arduino-as-component), target/host unit test runners, OpenOCD/GDB and tracing integration, first access to new IDF releases. This is where a real-time firmware with custom `sdkconfig` (tick rate, watchdogs, IRAM options, Wi-Fi task tuning) ultimately needs to live.
- **Zephyr `west`:** excellent meta-tool, only relevant if Zephyr is adopted (§2.3 — not recommended now).

**Recommended path:** switch the existing PlatformIO project to **pioarduino** now (unblocks Arduino 3.x / IDF 5.5-era features like current ESP-NOW APIs with zero workflow change), and structure new firmware as an **ESP-IDF project with Arduino as a component**, dropping the Arduino layer per-subsystem as native drivers replace it.

### 6.2 Debugging and tracing

- **JTAG on the DOIT DevKit v1:** the classic ESP32 has **no built-in USB-JTAG** (that arrived with ESP32-C3/S3). External adapter required — Espressif's ESP-Prog or any FT2232H-class probe — wired to **GPIO15 (TDO), GPIO12 (TDI), GPIO13 (TCK), GPIO14 (TMS)** + solid GND ([configure-other-JTAG guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/jtag-debugging/configure-other-jtag.html), [JTAG debugging guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/jtag-debugging/), [PlatformIO setup/troubleshooting](https://community.platformio.org/t/esp32-debugging-setup-and-troubleshooting/28648)). Caveats: those four GPIOs must be kept free of other functions in the vehicle pin map, GPIO12 is a boot-strapping pin, and deep sleep kills JTAG. PlatformIO supports `debug_tool = esp-prog` directly.
- **SEGGER SystemView / ESP-IDF application-level tracing:** ESP-IDF's `app_trace` library streams FreeRTOS scheduler events (task switches, ISR entry/exit, queue ops) to **SEGGER SystemView** over JTAG or UART with low overhead — the single best tool for *proving* the control loop meets its period and finding jitter sources ([app-trace guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/app_trace.html), [sysview example](https://github.com/espressif/esp-idf/blob/master/examples/system/sysview_tracing/README.md)). Dual-core note: SystemView tracing produces one trace per CPU over JTAG (or select which core when tracing over UART). Requires ESP-IDF-level menuconfig — another argument for leaving pure Arduino.
- Lightweight fallbacks that work today: `vTaskGetRunTimeStats()`/`uxTaskGetStackHighWaterMark()`, GPIO-toggle scope probes around the control loop, and the rate-supervisor pattern from Crazyflie.

### 6.3 Unit testing RTOS firmware

ESP-IDF has two sanctioned test styles ([unit testing guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/unit-tests.html), [Linux-host testing guide](https://docs.espressif.com/projects/esp-idf/en/v5.0.9/esp32/api-guides/linux-host-testing.html)):

1. **On-target tests with Unity:** tests live in a component's `test` subdirectory and run in a unit-test app on the ESP32 — right for driver/integration tests (does the ESC PWM actually output 50 Hz? does the nRF24 loopback work?).
2. **Linux-host tests with Unity + CMock:** components build for the `linux` target with dependencies mocked via CMock (Ruby required); fast, CI-friendly, no hardware. Still officially "under development / experimental," but ideal for the pure-logic core of this firmware — PID controllers, sensor-fusion math, swarm coordination state machines, packet codecs. **Design rule: keep control/estimation/protocol logic in plain C/C++ modules with no direct register or FreeRTOS calls, injected behind small interfaces, so they compile on host.**

PlatformIO's own Unity-based test runner (`pio test`) covers similar ground while the project remains on PlatformIO. (PlatformIO documentation: https://docs.platformio.org/en/latest/advanced/unit-testing/index.html)

---

## 7. Recommended Architecture

### 7.1 Platform decision

**ESP-IDF FreeRTOS on the dual-core ESP32, built as a native ESP-IDF project with Arduino as a component during transition.** Rationale: it is the only option that simultaneously (a) provides real dual-core SMP with task pinning, (b) keeps the ESP-NOW + LEDC/MCPWM + GPTimer + tracing + unit-test ecosystem native, (c) preserves the existing RF24/ESP32Servo code while it is being replaced, and (d) costs nothing to adopt because it is already running underneath the current sketch. Key `sdkconfig` set: `CONFIG_FREERTOS_HZ=1000`, task watchdog enabled with control task subscribed, `ESP_INTR_FLAG_IRAM` for the timer/radio ISRs.

### 7.2 Task table

Priorities on the 0–24 ESP-IDF scale (higher = more urgent; Wi-Fi driver task runs at high priority on core 0, so application tasks stay below it).

| Task | Core | Prio | Rate / trigger | Stack (initial) | Role |
|---|---|---|---|---|---|
| `tCtrl` (control loop) | 1 | 20 | 500 Hz, woken by GPTimer/esp_timer ISR → task notification (upgrade to IMU data-ready GPIO when IMU lands) | 4 KB | Read IMU/sensors (SPI/I2C), sensor fusion, PID, write ESC/servo via LEDC/MCPWM; feed TWDT; publish state snapshot (double buffer) |
| `tAvoid` (obstacle avoidance) | 1 | 15 | 50 Hz, `vTaskDelayUntil` | 4 KB | Read rangefinder queue + state snapshot; compute avoidance setpoint corrections → `qSetpoint` (overwrite) |
| `tRadioNrf` (nRF24 service) | 0 | 18 | Event-driven: IRQ pin ISR → task notification (fallback 250 Hz poll) | 3 KB | Sole owner of nRF24 SPI; drain RX FIFO → `qCmdRx`; send pending TX; parse manual-control packets → `qSetpoint` |
| `tSwarm` (coordination) | 0 | 17 | Event-driven from `qEspNowRx`; TX beacon 10–20 Hz | 4 KB | ESP-NOW RX/TX handling deferred from Wi-Fi-task callbacks; maintain neighbor table → `qNeighborState` (overwrite per peer); run coordination logic → setpoint bias |
| `tTelem` (telemetry) | 0 | 10 | 10–20 Hz, `vTaskDelayUntil` | 3 KB | Package state snapshot + health counters; send via ESP-NOW (or nRF24 TX queue) |
| `tHealth` (supervisor) | 0 | 8 | 2 Hz | 3 KB | Rate-supervisor check on `tCtrl` cycle counter, battery/RSSI monitoring, arm/disarm state, failsafe trigger (motors-safe), deferred NVS writes when disarmed |
| `tLog` (optional) | 0 | 3 | Background | 3 KB | Drain log stream buffer to UART/flash; lowest priority, no real-time claims |
| *(system)* Wi-Fi task, `esp_timer`, IPC, idle×2 | 0 (mostly) | system | — | — | Owned by ESP-IDF; ESP-NOW callbacks run inside Wi-Fi task and only enqueue |

**Queues / IPC:**

- `qSetpoint` — length-1, `xQueueOverwrite`; writers: `tRadioNrf`, `tSwarm`, `tAvoid`; reader: `tCtrl` (zero-timeout read each cycle). Latest command wins; control never blocks.
- `qEspNowRx` — length ~16, written *only* by the ESP-NOW receive callback (copy + enqueue, per Espressif guidance), read by `tSwarm`.
- `qCmdRx` / `qTelemTx` — small FIFO queues for radio packets.
- State snapshot — double-buffered struct (seq-lock or two-slot swap) written by `tCtrl`, read lock-free by `tAvoid`/`tTelem`/`tSwarm`.
- Log pipe — FreeRTOS stream buffer into `tLog`.

### 7.3 Rationale and rate choices

- **500 Hz control** is the sweet spot for a small ESC/servo vehicle: well within ESP32 headroom (Crazyflie/ESP-Drone sustain 1 kHz stabilizers on comparable silicon), one control period = 2 ms ≫ the ~15 µs timer-notification wake-up latency, and standard ESC/servo PWM output updates cannot exploit much more anyway. Design the loop so the rate is a constant that can be raised to 1 kHz later.
- **Core split** follows §3.1/§5.3: everything with unbounded latency (Wi-Fi, ESP-NOW, nRF24, flash, logging) is on core 0; core 1 runs only `tCtrl` and `tAvoid`, with `tCtrl` strictly higher so avoidance can never delay actuation.
- **Priorities are unique per real-time task** and deliberately leave gaps for future tasks; the ladder mirrors Crazyflie's (control > sensors/radio link > estimator/protocol > logging/housekeeping).
- **Failure containment:** TWDT on `tCtrl`, rate supervisor in `tHealth`, failsafe = neutral ESC output on comms-loss timeout or supervisor trip; NVS/flash writes only when disarmed (avoids §3.5 cache stalls in flight).
- **Verification loop:** SystemView traces over ESP-Prog JTAG to measure actual `tCtrl` period jitter under full radio load; host-side Unity/CMock tests for PID, fusion, and swarm protocol logic; on-target Unity tests for drivers.

### 7.4 Migration sketch from the current sketch

1. Repoint `platformio.ini` at pioarduino (Arduino 3.x / IDF 5.5-era) — no code changes.
2. Inside the still-Arduino project, split the superloop into the tasks above using `xTaskCreatePinnedToCore` (all APIs available today); `loop()` shrinks to nothing.
3. Re-home the project as an ESP-IDF project with Arduino as a component; take over `sdkconfig` (tick 1000 Hz, TWDT config, IRAM flags).
4. Replace Arduino-layer drivers subsystem-by-subsystem (ESP32Servo → LEDC/MCPWM driver, RF24 → thin SPI driver or ESP-NOW consolidation), keeping logic modules host-testable throughout.

---

## 8. Source Index

**RTOS landscape:** [IDF FreeRTOS (SMP) guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos_idf.html) · [idf_changes.md kernel diff](https://github.com/espressif/esp-idf/blob/12f36a02/components/freertos/FreeRTOS-Kernel/idf_changes.md) · [v4.2 SMP changes (PRO/APP CPU)](https://docs.espressif.com/projects/esp-idf/en/v4.2.5/esp32/api-guides/freertos-smp.html) · [Arduino as ESP-IDF component](https://docs.espressif.com/projects/arduino-esp32/en/latest/esp-idf_component.html) · [Hubble: ESP-IDF vs Arduino](https://hubble.com/community/comparisons/esp-idf-vs-arduino-framework-for-esp32-when-to-switch-and-why/) · [Espressif Zephyr support status](https://developer.espressif.com/software/zephyr-support-status/) · [Hubble: Zephyr on ESP32](https://hubble.com/community/guides/getting-started-with-zephyr-rtos-on-esp32-why-it-might-replace-esp-idf-for-your-next-product/) · [Zephyr ESP32 dev overview issue](https://github.com/zephyrproject-rtos/zephyr/issues/29394) · [Zephyr ESP32-C5 support PR](https://github.com/zephyrproject-rtos/zephyr/pull/105674) · [Espressif: new OS support (NuttX/Zephyr)](https://www.espressif.com/en/news/new_operating_systems_in_ESP32) · [NuttX getting started](https://developer.espressif.com/blog/2020/11/nuttx-getting-started/) · [NuttX motor control (2025)](https://developer.espressif.com/blog/2025/05/nuttx-motor-control-and-sensing/) · [Eclipse ThreadX](https://threadx.io/) · [Azure RTOS → open source](https://techcommunity.microsoft.com/blog/iotblog/azure-rtos-transition-to-open-source-is-now-complete/4105027) · [mongoose-os vs zephyr activity](https://www.libhunt.com/compare-mongoose-os-vs-zephyr)

**Real-time patterns:** [ESP-Techpedia timer comparison](https://docs.espressif.com/projects/esp-techpedia/en/latest/esp-friends/advanced-development/system/timer.html) · [esp_timer docs](https://github.com/espressif/esp-idf/blob/master/docs/en/api-reference/system/esp_timer.rst) · [GPTimer docs](https://docs.espressif.com/projects/esp-idf/en/v5.1.7/esp32/api-reference/peripherals/gptimer.html) · [esp32.com timing/latency thread](https://esp32.com/viewtopic.php?t=38644) · [FreeRTOS task notifications](https://www.freertos.org/Documentation/02-Kernel/02-Kernel-features/03-Direct-to-task-notifications/01-Task-notifications) · [Mastering FreeRTOS ch.7 (interrupts)](https://freertos.gitbook.io/mastering-the-freertos-tm-real-time-kernel/mastering.ch07) · [ch.10 (notifications)](https://freertos.gitbook.io/mastering-the-freertos-tm-real-time-kernel/mastering.ch10) · [AWS FreeRTOS intertask coordination](https://docs.aws.amazon.com/freertos/latest/userguide/inter-task-coordination.html) · [SPI flash concurrency / cache disable](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_flash/spi_flash_concurrency.html) · [Memory types / IRAM_ATTR](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/memory-types.html) · [esp_attr.h](https://github.com/espressif/esp-idf/blob/master/components/esp_common/include/esp_attr.h) · [Watchdogs guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/wdts.html)

**Case studies:** [Crazyflie architecture wiki](https://wiki.bitcraze.io/projects:crazyflie:firmware:arch) · [Crazyflie config.h priorities](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/config/config.h) · [stabilizer.c](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/stabilizer.c) · [Bitcraze system-task guide](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/development/systemtask/) · [ESP-Drone repo](https://github.com/espressif/esp-drone) · [ESP-Drone flight control system](https://docs.espressif.com/projects/espressif-esp-drone/en/latest/system.html) · [ESP-Drone comms](https://docs.espressif.com/projects/espressif-esp-drone/en/latest/communication.html) · [Betaflight scheduler (DeepWiki)](https://deepwiki.com/betaflight/betaflight/8.1-task-scheduler) · [Betaflight deep dive](https://betaflight.com/docs/wiki/guides/current/Deep-Dive) · [Betaflight 4.3 tuning notes](https://betaflight.com/docs/wiki/tuning/4-3-Tuning-Notes) · [Betaflight RTOS discussion #1681](https://github.com/betaflight/betaflight/issues/1681) · [Scheduler PR #11354](https://github.com/betaflight/betaflight/pull/11354)

**Comms:** [ESP-NOW API docs](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/network/esp_now.html) · [espnow example (queue pattern)](https://github.com/espressif/esp-idf/blob/master/examples/wifi/espnow/main/espnow_example_main.c) · [esp-now component](https://github.com/espressif/esp-now/blob/master/src/espnow/src/espnow.c) · [Wi-Fi-task deadlock report](https://github.com/esp-rs/esp-idf-svc/issues/315) · [RF24 class reference](https://nrf24.github.io/RF24/classRF24.html) · [RF24 IRQ example](https://rf24.readthedocs.io/en/stable/examples_2InterruptConfigure_2InterruptConfigure_8ino-example.html) · [RF24 FreeRTOS deferred-IRQ example](https://github.com/maniacbug/FreeRTOS/blob/master/examples/pingpair/radio.cpp) · [RF24 ESP-IDF support issue](https://github.com/nRF24/RF24/issues/925)

**Tooling:** [platformio/platform-espressif32#1225](https://github.com/platformio/platform-espressif32/issues/1225) · [pioarduino README](https://github.com/pioarduino/platform-espressif32/blob/main/README.md) · [pioarduino version map](https://github.com/sivar2311/platform-espressif32-versions) · [app-trace / SystemView guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/app_trace.html) · [sysview example](https://github.com/espressif/esp-idf/blob/master/examples/system/sysview_tracing/README.md) · [JTAG debugging guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/jtag-debugging/) · [Other-JTAG (ESP-Prog pinout)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/jtag-debugging/configure-other-jtag.html) · [PlatformIO ESP32 debugging guide](https://community.platformio.org/t/esp32-debugging-setup-and-troubleshooting/28648) · [ESP-IDF unit testing](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/unit-tests.html) · [Linux-host testing + CMock](https://docs.espressif.com/projects/esp-idf/en/v5.0.9/esp32/api-guides/linux-host-testing.html)
