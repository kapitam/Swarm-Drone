// tAvoid — 50 Hz behavior + avoidance pipeline, core 1, priority 15
// (research doc 04 s7.2). Gathers RC / swarm / perception views, runs the
// portable BehaviorPipeline (mode source -> BVC -> VFH-lite -> governor) and
// publishes the AttitudeSetpoint through the overwrite queue.

#include <Arduino.h>
#include "../config/config.h"
#include "../state_bus.h"
#include "swarmcore/behavior.h"

namespace task_avoid {

// Steering view: elementwise min of ToF and vision sectors — vision may only
// make the picture MORE conservative for steering. Reflex stays ToF-only.
static sc::SectorArray fuse(const sc::SectorArray& tof,
                            const sc::SectorArray& vis) {
  sc::SectorArray out = tof;
  const uint32_t age = millis() - vis.stampMs;
  if (age > 500) return out;  // stale vision: ignore
  for (int i = 0; i < sc::kSectors; ++i) {
    if (!vis.known(i)) continue;
    if (!out.known(i) || vis.distMm[i] < out.distMm[i]) {
      out.distMm[i] = vis.distMm[i];
      out.validZones[i] = vis.validZones[i];
    }
  }
  if (vis.stampMs > out.stampMs) out.stampMs = vis.stampMs;
  return out;
}

static void taskFn(void*) {
  sc::BehaviorPipeline pipeline;
  SwarmSnapshot swarm;
  TickType_t wake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000 / AVOID_HZ));
    const uint32_t nowMs = millis();

    const ControlSnapshot ctrl = g_bus.control();
    const RcSnapshot rc = g_bus.rc();
    g_bus.swarmView(swarm);
    const sc::SectorArray tof = g_bus.tofSectors();

    sc::BehaviorInputs in;
    in.mode = swarm.estop ? sc::BehaviorMode::kHold
                          : sc::BehaviorMode(swarm.mode);
    in.rc = sc::normalizeRc(rc.raw);
    in.rcAgeMs = rc.lastMs ? (nowMs - rc.lastMs) : 0xFFFFFFFF;
    in.self = ctrl.state;
    in.table = &swarm.table;
    in.nowMs = nowMs;
    in.goal = swarm.goal;
    in.hasGoal = swarm.hasGoal;

#if defined(PERCEPTION_V2_VISION)
    in.sectors = fuse(tof, g_bus.visionSectors());
    in.reflexSectors = tof;      // reflex authority: direct sensor only
    in.hasReflexSectors = true;
#else
    in.sectors = tof;
#endif

    pipeline.setLeader(swarm.leaderId);
    const sc::BehaviorOutput out = pipeline.update(in);
    g_bus.publishSetpoint(out.setpoint);
    g_bus.publishBehavior(out);
  }
}

void start() {
  xTaskCreatePinnedToCore(taskFn, "tAvoid", STACK_AVOID, nullptr, PRIO_AVOID,
                          nullptr, CORE_RT);
}

}  // namespace task_avoid
