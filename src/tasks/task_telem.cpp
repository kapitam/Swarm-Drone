// tTelem — telemetry + health supervision, core 0, priority 10.
// One JSON line per tick on the USB serial (greppable, plottable), plus the
// rate-supervisor check on the control loop (doc 04 s3.6 folds tHealth's
// checks in here to keep the task count down; documented in HANDBOOK).

#include <Arduino.h>
#include "../config/config.h"
#include "../config/config_store.h"
#include "../state_bus.h"
#include "../platform/espnow_link.h"
#include "../platform/tof_vl53l5cx.h"
#include "../vision/vision.h"

namespace task_telem {

static const char* armName(uint8_t s) {
  switch (s) {
    case 0: return "DISARMED";
    case 1: return "ARMING";
    case 2: return "ARMED";
    case 3: return "FAILSAFE";
    case 4: return "ESTOP";
    default: return "?";
  }
}

static void taskFn(void*) {
  uint32_t lastLoopCount = 0;
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000 / TELEM_HZ));
    const uint32_t nowMs = millis();

    const ControlSnapshot c = g_bus.control();
    const RcSnapshot rc = g_bus.rc();
    const sc::SectorArray tof = g_bus.tofSectors();
    const sc::BehaviorOutput b = g_bus.behavior();
    SwarmSnapshot sw;
    g_bus.swarmView(sw);

    // Health: control loop rate supervision.
    const uint32_t loops = c.loopCount - lastLoopCount;
    lastLoopCount = c.loopCount;
    const uint32_t expected = CONTROL_HZ / TELEM_HZ;
    if (c.loopCount > 0 && loops < expected * 9 / 10)
      Serial.printf("[health] WARNING: control loop %u/%u ticks\n", unsigned(loops),
                    unsigned(expected));

    const uint16_t minFwd = tof.minForwardMm();
    Serial.printf(
        "{\"id\":%u,\"t\":%u,\"arm\":\"%s\",\"mode\":%u,\"reflex\":%d,"
        "\"rc_age\":%d,\"batt\":%.2f,\"rpy\":[%.2f,%.2f,%.2f],"
        "\"pos\":[%.2f,%.2f],\"vel\":[%.2f,%.2f],\"pose_ok\":%d,"
        "\"min_fwd_mm\":%d,\"tof_ok\":%d,\"nbrs\":%d,\"grad\":%u,"
        "\"espnow\":[%u,%u,%u],\"cam\":%d,\"model\":%d,\"inf\":[%u,%u],"
        "\"loop_max_us\":%u,\"heap\":%u}\n",
        config_store::robotId(), unsigned(nowMs), armName(c.armState),
        sw.mode, int(b.reflex),
        rc.lastMs ? int(nowMs - rc.lastMs) : -1, c.state.batteryV,
        c.state.roll, c.state.pitch, c.state.pose.yaw, c.state.pose.p.x,
        c.state.pose.p.y, c.state.vel.x, c.state.vel.y,
        c.state.poseValid ? 1 : 0,
        minFwd == sc::SectorArray::kUnknownMm ? -1 : int(minFwd),
        tof::healthy() ? 1 : 0, sw.table.count(), sw.gradient,
        unsigned(espnow_link::txCount()), unsigned(espnow_link::rxCount()),
        unsigned(espnow_link::rxDropCount()), vision::cameraHealthy() ? 1 : 0,
        vision::backendReady() ? 1 : 0, unsigned(vision::inferenceCount()),
        unsigned(vision::lastInferenceUs()), unsigned(c.maxLoopUs),
        unsigned(ESP.getFreeHeap()));
  }
}

void start() {
  xTaskCreatePinnedToCore(taskFn, "tTelem", STACK_TELEM, nullptr, PRIO_TELEM,
                          nullptr, CORE_COMM);
}

}  // namespace task_telem
