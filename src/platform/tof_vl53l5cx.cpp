#include "tof_vl53l5cx.h"

#include <Arduino.h>
#include "../config/config.h"
#include "../state_bus.h"

#if defined(PERCEPTION_V1_TOF)

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>
#include "swarmcore/sectors.h"

namespace tof {

static SparkFun_VL53L5CX imager;
static volatile bool healthy_ = false;
static volatile uint32_t lastFrameMs = 0;

static TwoWire& bus() {
#if TOF_USE_WIRE1
  return Wire1;
#else
  return Wire;
#endif
}

static bool initSensor() {
#if TOF_USE_WIRE1
  bus().begin(PIN_TOF_SDA, PIN_TOF_SCL, TOF_I2C_HZ);
#else
  // Shared bus (XIAO): Wire already begun by imu::init(); just set clock.
  bus().setClock(TOF_I2C_HZ);
#endif
  // Doc 08 checklist: 128 B transfers BEFORE begin() so the ~86 KB firmware
  // upload uses full chunks (2.8 s @ 400 kHz instead of 9.4 s @ default).
  imager.setWireMaxPacketSize(128);
  Serial.println("[tof] uploading sensor firmware (blocks 2-3 s)...");
  if (!imager.begin(0x29, bus())) return false;
  // Order matters: resolution BEFORE frequency (valid range depends on it).
  if (!imager.setResolution(8 * 8)) return false;
  if (!imager.setRangingFrequency(15)) return false;
  imager.setRangingMode(SF_VL53L5CX_RANGING_MODE::CONTINUOUS);
  imager.setTargetOrder(SF_VL53L5CX_TARGET_ORDER::CLOSEST);
  // Sharpener stays at silicon default 14% (doc 08: UM2884 Rev 7).
  return imager.startRanging();
}

static void taskFn(void*) {
  bool ok = initSensor();
  if (!ok) Serial.println("[tof] ERROR: VL53L5CX init failed (governor -> BLIND)");
  else Serial.println("[tof] ranging 8x8 @ 15 Hz");

  sc::SectorFilter filter;   // doc 08 defaults: rows 2-5, clamp 2 m, hold 3
  VL53L5CX_ResultsData results;
  uint16_t dist[64];
  uint8_t status[64], nb[64];

  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));  // poll ~100 Hz for 15 Hz data
    if (!ok) {
      vTaskDelay(pdMS_TO_TICKS(2000));  // periodic re-init attempts
      ok = initSensor();
      continue;
    }
    if (!imager.isDataReady()) continue;
    if (!imager.getRangingData(&results)) continue;
    for (int i = 0; i < 64; ++i) {
      dist[i] = uint16_t(results.distance_mm[i] < 0 ? 0 : results.distance_mm[i]);
      status[i] = results.target_status[i];
      nb[i] = results.nb_target_detected[i];
    }
    const uint32_t now = millis();
    g_bus.publishTofSectors(filter.update(dist, status, nb, now));
    lastFrameMs = now;
    healthy_ = true;
  }
}

void start() {
  xTaskCreatePinnedToCore(taskFn, "tTof", STACK_TOF, nullptr, PRIO_TOF,
                          nullptr, CORE_COMM);
}

bool healthy() { return healthy_ && (millis() - lastFrameMs) < 500; }

}  // namespace tof

#else  // !PERCEPTION_V1_TOF

namespace tof {
void start() {}
bool healthy() { return false; }
}  // namespace tof

#endif
