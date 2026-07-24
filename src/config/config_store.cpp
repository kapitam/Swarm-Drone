#include "config_store.h"

#include <Arduino.h>
#include <Preferences.h>
#include "config.h"

namespace config_store {

static uint8_t id_ = ROBOT_ID_DEFAULT;

void begin() {
  Preferences prefs;
  if (prefs.begin("swarm", /*readOnly=*/true)) {
    id_ = prefs.getUChar("robot.id", ROBOT_ID_DEFAULT);
    prefs.end();
  }
}

uint8_t robotId() { return id_; }

void saveRobotId(uint8_t id) {
  Preferences prefs;
  if (prefs.begin("swarm", /*readOnly=*/false)) {
    prefs.putUChar("robot.id", id);
    prefs.end();
    id_ = id;
    Serial.printf("[config] robot id -> %u (persisted)\n", id);
  }
}

}  // namespace config_store
