#pragma once
// StateBus — the only data path between tasks (research doc 04 s7):
// single-writer snapshots guarded by short spinlock critical sections plus a
// length-1 overwrite queue for the attitude setpoint. The control loop never
// blocks on comms: every accessor is a bounded copy.

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "swarmcore/types.h"
#include "swarmcore/packets.h"
#include "swarmcore/neighbor_table.h"
#include "swarmcore/behavior.h"

struct RcSnapshot {
  sc::RcPacket raw{ {1000, 1500, 1500, 1500} };
  uint32_t lastMs = 0;      // 0 = never received
  uint32_t count = 0;
};

struct ControlSnapshot {
  sc::VehicleState state;
  float gyro[3] = {0, 0, 0};
  uint8_t armState = 0;     // sc::ArmState
  uint32_t loopCount = 0;   // rate supervision (health checks)
  uint32_t maxLoopUs = 0;
};

struct SwarmSnapshot {
  sc::NeighborTable table;  // copy; ~1 KB, copied at 50 Hz = negligible
  uint8_t gradient = 0xFF;
  uint8_t mode = 0;         // sc::BehaviorMode commanded by operator
  uint8_t leaderId = 0;
  bool estop = false;
  sc::Vec2 goal;
  bool hasGoal = false;
};

// Raw VL53L5CX frame for the dataset logger (train-time label derivation
// happens offline from raw zones — research doc 09 s4.4).
struct TofGrid {
  uint16_t distMm[64] = {0};
  uint8_t status[64] = {0};
  uint32_t stampMs = 0;
};

class StateBus {
 public:
  void begin() {
    setpointQ_ = xQueueCreate(1, sizeof(sc::AttitudeSetpoint));
  }

  // ---- RC (writer: rc link task) ----
  void publishRc(const sc::RcPacket& p, uint32_t nowMs) {
    portENTER_CRITICAL(&mux_);
    rc_.raw = p;
    rc_.lastMs = nowMs;
    ++rc_.count;
    portEXIT_CRITICAL(&mux_);
  }
  RcSnapshot rc() {
    portENTER_CRITICAL(&mux_);
    RcSnapshot c = rc_;
    portEXIT_CRITICAL(&mux_);
    return c;
  }

  // ---- Control state (writer: control task) ----
  void publishControl(const ControlSnapshot& s) {
    portENTER_CRITICAL(&mux_);
    ctrl_ = s;
    portEXIT_CRITICAL(&mux_);
  }
  ControlSnapshot control() {
    portENTER_CRITICAL(&mux_);
    ControlSnapshot c = ctrl_;
    portEXIT_CRITICAL(&mux_);
    return c;
  }

  // ---- Perception (writers: tof task / vision task) ----
  void publishTofSectors(const sc::SectorArray& s) {
    portENTER_CRITICAL(&mux_);
    tof_ = s;
    portEXIT_CRITICAL(&mux_);
  }
  sc::SectorArray tofSectors() {
    portENTER_CRITICAL(&mux_);
    sc::SectorArray c = tof_;
    portEXIT_CRITICAL(&mux_);
    return c;
  }
  void publishVisionSectors(const sc::SectorArray& s) {
    portENTER_CRITICAL(&mux_);
    vision_ = s;
    portEXIT_CRITICAL(&mux_);
  }
  sc::SectorArray visionSectors() {
    portENTER_CRITICAL(&mux_);
    sc::SectorArray c = vision_;
    portEXIT_CRITICAL(&mux_);
    return c;
  }
  void publishTofGrid(const TofGrid& g) {
    portENTER_CRITICAL(&gridMux_);
    grid_ = g;
    portEXIT_CRITICAL(&gridMux_);
  }
  void tofGrid(TofGrid& out) {
    portENTER_CRITICAL(&gridMux_);
    out = grid_;
    portEXIT_CRITICAL(&gridMux_);
  }

  // ---- Swarm view (writer: swarm task) ----
  void publishSwarm(const SwarmSnapshot& s) {
    portENTER_CRITICAL(&swarmMux_);
    swarm_ = s;
    portEXIT_CRITICAL(&swarmMux_);
  }
  void swarmView(SwarmSnapshot& out) {
    portENTER_CRITICAL(&swarmMux_);
    out = swarm_;
    portEXIT_CRITICAL(&swarmMux_);
  }
  void latchEStop() {
    portENTER_CRITICAL(&swarmMux_);
    swarm_.estop = true;
    portEXIT_CRITICAL(&swarmMux_);
  }
  bool estop() {
    portENTER_CRITICAL(&swarmMux_);
    const bool e = swarm_.estop;
    portEXIT_CRITICAL(&swarmMux_);
    return e;
  }

  // ---- Zero-pose request (writer: swarm task; consumer: control task) ----
  void requestZeroPose() { zeroPoseReq_ = true; }
  bool takeZeroPoseRequest() {
    const bool r = zeroPoseReq_;
    zeroPoseReq_ = false;
    return r;
  }

  // ---- Attitude setpoint (writer: avoid task; reader: control task) ----
  void publishSetpoint(const sc::AttitudeSetpoint& sp) {
    xQueueOverwrite(setpointQ_, &sp);
  }
  bool peekSetpoint(sc::AttitudeSetpoint& sp) {
    return xQueuePeek(setpointQ_, &sp, 0) == pdTRUE;
  }

  // ---- Behavior telemetry (writer: avoid task) ----
  void publishBehavior(const sc::BehaviorOutput& o) {
    portENTER_CRITICAL(&mux_);
    behav_ = o;
    portEXIT_CRITICAL(&mux_);
  }
  sc::BehaviorOutput behavior() {
    portENTER_CRITICAL(&mux_);
    sc::BehaviorOutput c = behav_;
    portEXIT_CRITICAL(&mux_);
    return c;
  }

 private:
  portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
  portMUX_TYPE swarmMux_ = portMUX_INITIALIZER_UNLOCKED;
  portMUX_TYPE gridMux_ = portMUX_INITIALIZER_UNLOCKED;
  TofGrid grid_;
  RcSnapshot rc_;
  ControlSnapshot ctrl_;
  sc::SectorArray tof_, vision_;
  SwarmSnapshot swarm_;
  sc::BehaviorOutput behav_;
  QueueHandle_t setpointQ_ = nullptr;
  volatile bool zeroPoseReq_ = false;
};

extern StateBus g_bus;
