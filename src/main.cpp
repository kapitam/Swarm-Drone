// Swarm Drone firmware — entry point.
//
// Everything is a FreeRTOS task (research doc 04 architecture); setup() only
// initializes safe-state hardware and spawns tasks, then loop() retires.
// Fork selection is entirely in platformio.ini build flags; see
// docs/HANDBOOK.md for the fork matrix and safety procedures.
//
// SAFETY: attitude gains are UNTUNED. First power-up: PROPS OFF.

#include <Arduino.h>

#include "config/config.h"
#include "config/config_store.h"
#include "state_bus.h"
#include "platform/motors.h"
#include "platform/rc_link.h"
#include "platform/tof_vl53l5cx.h"
#include "vision/vision.h"
#include "tasks/tasks.h"

StateBus g_bus;

static const char* buildDescription() {
  return
#if defined(BOARD_XIAO_S3)
      "XIAO-S3 Vision (Build B)"
#elif defined(MOTORS_BRUSHED_PWM)
      "DevKit v1 brushed (Build A')"
#else
      "DevKit v1 ESC (Build A)"
#endif
      ;
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // Motors first, into safe state, before anything can fail.
  motors::init();
  motors::stop();

  config_store::begin();
  g_bus.begin();

  Serial.println();
  Serial.println("==== Swarm Drone firmware ====");
  Serial.printf("build: %s | robot id %u\n", buildDescription(),
                config_store::robotId());
  Serial.printf("forks: perception[%s%s] motors[%s] imu[%s] rc[%s] vision[%s]\n",
#if defined(PERCEPTION_V1_TOF)
                "V1-ToF",
#else
                "-",
#endif
#if defined(PERCEPTION_V2_VISION)
                "+V2-Vision",
#else
                "",
#endif
#if defined(MOTORS_ESC_PWM)
                "ESC-PWM 50Hz",
#else
                "brushed 20kHz",
#endif
#if defined(IMU_MPU6050)
                "MPU6050",
#elif defined(IMU_ICM42688)
                "ICM42688",
#else
                "MOCK",
#endif
#if defined(RC_LINK_NRF24)
                "nRF24",
#else
                "ESP-NOW",
#endif
#if defined(VISION_BACKEND_TFLM)
                "TFLM"
#elif defined(VISION_BACKEND_STUB)
                "stub"
#else
                "none"
#endif
  );
  Serial.println("SAFETY: gains untuned - props off for first power-up.");

  // Core-0 services first (radio/sensors), then the core-1 real-time pair.
  task_swarm::start();     // ESP-NOW beacons + commands (+ RC on ESP-NOW fork)
  rc_link::start();        // nRF24 fork spawns its poll task; ESP-NOW is fed
  tof::start();            // V1 perception (no-op when not compiled in)
  vision::start();         // V2 perception (no-op when not compiled in)
  task_telem::start();

  task_avoid::start();     // core 1, 50 Hz behavior/avoidance
  task_control::start();   // core 1, 500 Hz control loop

  Serial.println("[main] all tasks started");
}

void loop() {
  // Everything lives in tasks; the Arduino loopTask retires.
  vTaskDelete(nullptr);
}
