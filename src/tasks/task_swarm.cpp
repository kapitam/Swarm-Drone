// tSwarm — swarm coordination, core 0, priority 17 (research doc 04 s7.2).
// Owns the ESP-NOW RX queue: neighbor beacons -> NeighborTable, operator
// commands -> mode/e-stop/zero-pose/params, RC-over-ESP-NOW -> rc_link feed.
// Broadcasts our StatePacket at BEACON_HZ with per-robot phase jitter
// (research doc 01 s6.2) and publishes the SwarmSnapshot for core 1.

#include <Arduino.h>
#include "../config/config.h"
#include "../config/config_store.h"
#include "../state_bus.h"
#include "../platform/espnow_link.h"
#include "../platform/rc_link.h"
#include "swarmcore/packets.h"
#include "swarmcore/neighbor_table.h"
#include "swarmcore/gradient.h"
#include "swarmcore/behavior.h"

namespace task_swarm {

static sc::NeighborTable table;
static sc::Gradient gradient;
static SwarmSnapshot snap;

static void handleCmd(const sc::CmdPacket& c) {
  if (c.targetId != 0xFF && c.targetId != config_store::robotId()) return;
  switch (c.type) {
    case sc::kCmdEStop:
      g_bus.latchEStop();
      snap.estop = true;
      Serial.println("[swarm] E-STOP received");
      break;
    case sc::kCmdSetMode:
      if (c.arg0 <= sc::kBehaviorModeMax) snap.mode = c.arg0;
      break;
    case sc::kCmdZeroPose:
      g_bus.requestZeroPose();
      break;
    case sc::kCmdSetLeader:
      snap.leaderId = c.arg0;
      break;
    case sc::kCmdSetGoal:
      // GPS/computer seam: goal in shared-frame meters (a ground computer,
      // or later the GPS-equipped leader, streams these). arg0=0 clears.
      snap.hasGoal = (c.arg0 != 0);
      snap.goal = {c.argF, c.argF2};
      break;
    case sc::kCmdSetParam:
      // Param 1 = robot id; persisted only while disarmed (doc 04: no NVS
      // writes while armed — flash writes stall the other core's cache).
      if (c.arg0 == 1 && !g_bus.control().state.armed)
        config_store::saveRobotId(uint8_t(c.argF));
      break;
    default:
      break;
  }
}

static void dispatch(const espnow_link::RxItem& item) {
  if (item.len == sizeof(sc::StatePacket) &&
      item.data[0] == sc::kMagicState) {
    sc::StatePacket p;
    memcpy(&p, item.data, sizeof(p));
    table.update(p, item.tMs, config_store::robotId());
  } else if (item.len == sizeof(sc::CmdPacket) &&
             item.data[0] == sc::kMagicCmd) {
    sc::CmdPacket c;
    memcpy(&c, item.data, sizeof(c));
    if (sc::checkCmd(c)) handleCmd(c);
  } else if (item.data[0] == sc::kMagicRc) {
    rc_link::feedEspNow(item.data, item.len, item.tMs);
  }
}

static void sendBeacon(uint16_t& seq) {
  const ControlSnapshot ctrl = g_bus.control();
  const sc::BehaviorOutput behav = g_bus.behavior();
  sc::StatePacket p{};
  p.id = config_store::robotId();
  p.seq = seq++;
  p.xCm = int16_t(ctrl.state.pose.p.x * 100.0f);
  p.yCm = int16_t(ctrl.state.pose.p.y * 100.0f);
  p.zCm = int16_t(ctrl.state.z * 100.0f);
  p.vxCmS = int16_t(ctrl.state.vel.x * 100.0f);
  p.vyCmS = int16_t(ctrl.state.vel.y * 100.0f);
  // Commanded velocity: on a manually flown leader this is the stick intent;
  // MIMIC followers track it (CONOPS: leader-led flock over ESP-NOW).
  p.cmdVxCmS = int16_t(behav.commandedVel.x * 100.0f);
  p.cmdVyCmS = int16_t(behav.commandedVel.y * 100.0f);
  p.headingMrad = int16_t(sc::wrapPi(ctrl.state.pose.yaw) * 1000.0f);
  p.mode = snap.mode;
  p.gradient = gradient.value();
  p.flags = (ctrl.state.armed ? sc::kFlagArmed : 0) |
            (ctrl.state.poseValid ? sc::kFlagPoseValid : 0);
  p.batteryDv = uint8_t(sc::clampf(ctrl.state.batteryV * 10.0f, 0.0f, 255.0f));
  p.tMs = millis();
  sc::sealState(p);
  espnow_link::send(&p, sizeof(p));
}

static void taskFn(void*) {
  snap.mode = DEFAULT_BEHAVIOR_MODE;
  snap.leaderId = LEADER_ID_DEFAULT;
  if (!espnow_link::init())
    Serial.println("[swarm] ERROR: ESP-NOW init failed");
  else
    Serial.printf("[swarm] ESP-NOW up, channel %d, robot id %u, mode %u, "
                  "leader %u\n", ESPNOW_CHANNEL, config_store::robotId(),
                  snap.mode, snap.leaderId);

  // Per-robot beacon phase jitter breaks fleet-wide TX synchronization on
  // the CSMA-weak broadcast channel (doc 01 s6.2).
  const uint32_t beaconPeriod = 1000 / BEACON_HZ;
  uint32_t nextBeacon = millis() + (config_store::robotId() * 17) % beaconPeriod;
  uint16_t seq = 0;

  for (;;) {
    espnow_link::RxItem item;
    // Block up to 10 ms on RX; this also paces the housekeeping below.
    while (espnow_link::receive(item, 10)) dispatch(item);

    const uint32_t now = millis();
    if (int32_t(now - nextBeacon) >= 0) {
      nextBeacon += beaconPeriod;
      table.expire(now, NEIGHBOR_TTL_MS);
      gradient.setSeed(config_store::robotId() == GRADIENT_SEED_ID);
      gradient.update(table.minNeighborGradient(now, NEIGHBOR_TTL_MS));
      sendBeacon(seq);

      snap.table = table;  // struct copy (~1 KB at 10 Hz: negligible)
      snap.gradient = gradient.value();
      g_bus.publishSwarm(snap);
    }
  }
}

void start() {
  xTaskCreatePinnedToCore(taskFn, "tSwarm", STACK_SWARM, nullptr, PRIO_SWARM,
                          nullptr, CORE_COMM);
}

}  // namespace task_swarm
