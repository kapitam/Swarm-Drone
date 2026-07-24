// tCtrl — 500 Hz hard-real-time control loop, core 1, priority 20 (research
// doc 04 s7.2). Paced by an esp_timer ISR via direct-to-task notification.
// IMU read -> Mahony fusion -> arming -> attitude PIDs -> mixer -> motors,
// then publishes the state snapshot. Never blocks on comms; consumes the
// latest setpoint via a length-1 overwrite queue.
//
// SAFETY: gains are UNTUNED placeholders. Props off for first power-up.

#include <Arduino.h>
#include "driver/gptimer.h"
#include "esp_task_wdt.h"

#include "../config/config.h"
#include "../state_bus.h"
#include "../platform/motors.h"
#include "../platform/imu.h"
#include "swarmcore/mahony.h"
#include "swarmcore/attitude.h"
#include "swarmcore/mixer.h"
#include "swarmcore/arming.h"
#include "swarmcore/estimator.h"
#include "swarmcore/rc.h"

namespace task_control {

static TaskHandle_t handle = nullptr;

// GPTimer alarm ISR -> direct-to-task notification (doc 04 s3.3/s3.4).
// (esp_timer's ISR dispatch isn't enabled in the Arduino sdkconfig, so the
// hardware-timer route is GPTimer.)
static bool IRAM_ATTR onTick(gptimer_handle_t, const gptimer_alarm_event_data_t*,
                             void*) {
  BaseType_t hpw = pdFALSE;
  vTaskNotifyGiveFromISR(handle, &hpw);
  return hpw == pdTRUE;  // request yield if a higher-prio task woke
}

static float readBatteryV() {
#if PIN_BATTERY_ADC >= 0
  static float filt = 0.0f;
  const float v = analogReadMilliVolts(PIN_BATTERY_ADC) * 1e-3f * BATT_DIVIDER;
  filt = (filt <= 0.01f) ? v : filt * 0.98f + v * 0.02f;
  return filt;
#else
  return 0.0f;  // no divider on this board: battery checks disabled
#endif
}

static void taskFn(void*) {
  // Subscribe to the task watchdog: a stalled control loop must reset us.
  esp_task_wdt_add(nullptr);

  sc::Mahony ahrs;
  sc::AttitudeController att;
  sc::Arming arming(sc::ArmingParams{1000, RC_TIMEOUT_MS, 0.05f, 0.85f,
                                     LOW_BATT_PER_CELL});
  sc::TiltOdometry odom;
  odom.zero(millis());

  const bool imuOk = imu::init();
  if (!imuOk) Serial.println("[ctrl] ERROR: IMU init failed - staying disarmed");
  Serial.printf("[ctrl] IMU: %s, loop %d Hz\n", imu::name(), CONTROL_HZ);

  // Hardware-timer pacing (doc 04 s3.3): 1 MHz GPTimer, auto-reload alarm.
  gptimer_handle_t timer = nullptr;
  gptimer_config_t tcfg = {};
  tcfg.clk_src = GPTIMER_CLK_SRC_DEFAULT;
  tcfg.direction = GPTIMER_COUNT_UP;
  tcfg.resolution_hz = 1000000;
  gptimer_new_timer(&tcfg, &timer);
  gptimer_event_callbacks_t cbs = {};
  cbs.on_alarm = onTick;
  gptimer_register_event_callbacks(timer, &cbs, nullptr);
  gptimer_alarm_config_t alarm = {};
  alarm.alarm_count = 1000000 / CONTROL_HZ;
  alarm.reload_count = 0;
  alarm.flags.auto_reload_on_alarm = true;
  gptimer_set_alarm_action(timer, &alarm);
  gptimer_enable(timer);
  gptimer_start(timer);

  ControlSnapshot snap;
  float gyro[3] = {0, 0, 0}, accel[3] = {0, 0, 1};
  const float dt = 1.0f / CONTROL_HZ;
  uint32_t batteryDivider = 0;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    const uint32_t t0 = micros();
    esp_task_wdt_reset();
    const uint32_t nowMs = millis();

    // ---- Sense + fuse ----
    if (imuOk) imu::read(gyro, accel);
    ahrs.update(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], dt);
    const float roll = ahrs.roll(), pitch = ahrs.pitch(), yaw = ahrs.yaw();

    // ---- Inputs ----
    const RcSnapshot rc = g_bus.rc();
    const sc::RcNorm rcN = sc::normalizeRc(rc.raw);
    const uint32_t rcAge = rc.lastMs ? (nowMs - rc.lastMs) : 0xFFFFFFFF;
    if (++batteryDivider % 50 == 0) snap.state.batteryV = readBatteryV();

    // ---- Arming / failsafe (e-stop latches) ----
    if (g_bus.estop()) arming.latchEStop();
    const sc::ArmState as =
        arming.update(rcN.throttle, rcN.yaw, rcAge,
                      snap.state.batteryV / BATT_CELLS, nowMs);
    const bool armed = arming.motorsAllowed() && imuOk;

    // ---- Dead-reckoned pose (placeholder tilt odometry; see estimator.h) ----
    if (g_bus.takeZeroPoseRequest()) odom.zero(nowMs);
    sc::AttitudeSetpoint sp;  // zero-safe default
    const bool haveSp = g_bus.peekSetpoint(sp);
    odom.update(roll, pitch, yaw, haveSp ? sp.throttle : 0.0f, armed, dt, nowMs);

    // ---- Act ----
    if (armed && haveSp) {
      const sc::TorqueCommand tc =
          att.update(sp, roll, pitch, gyro[0], gyro[1], gyro[2], dt);
      const sc::MotorOutputs m =
          sc::mixQuadX(tc.throttle, tc.roll, tc.pitch, tc.yaw);
      motors::write(m.m);
    } else {
      motors::stop();
      att.reset();
    }

    // ---- Publish snapshot ----
    snap.state.pose = odom.pose();
    snap.state.vel = odom.velocity();
    snap.state.roll = roll;
    snap.state.pitch = pitch;
    snap.state.yawRate = gyro[2];
    snap.state.poseValid = odom.poseValid() && imuOk;
    snap.state.armed = armed;
    snap.state.tMs = nowMs;
    snap.armState = uint8_t(as);
    snap.gyro[0] = gyro[0]; snap.gyro[1] = gyro[1]; snap.gyro[2] = gyro[2];
    ++snap.loopCount;
    const uint32_t el = micros() - t0;
    if (el > snap.maxLoopUs) snap.maxLoopUs = el;
    g_bus.publishControl(snap);
  }
}

void start() {
  xTaskCreatePinnedToCore(taskFn, "tCtrl", STACK_CONTROL, nullptr,
                          PRIO_CONTROL, &handle, CORE_RT);
}

}  // namespace task_control
