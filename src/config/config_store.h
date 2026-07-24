#pragma once
// Persistent settings (NVS via Preferences). Writes ONLY while disarmed
// (flash writes suspend the cache — research docs 03/04).

#include <stdint.h>

namespace config_store {

void begin();                 // load persisted values (call before tasks)
uint8_t robotId();
void saveRobotId(uint8_t id);

}  // namespace config_store
