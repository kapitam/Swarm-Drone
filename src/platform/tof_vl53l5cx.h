#pragma once
// V1 perception: VL53L5CX 8x8 ToF -> SectorArray (research doc 08 recipe).
// Runs as a core-0 task (frame read is blocking Wire traffic). Publishes to
// g_bus.publishTofSectors(); the governor treats a dead sensor as BLIND.

namespace tof {

void start();       // spawns the sensor task (init inside the task: begin()
                    // blocks ~2-3 s for the 86 KB firmware upload, doc 08)
bool healthy();     // true once ranging and fresh
}  // namespace tof
