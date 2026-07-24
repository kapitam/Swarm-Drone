#include "vision.h"

#if defined(PERCEPTION_V2_VISION)

#include <Arduino.h>
#include "esp_camera.h"
#include "../config/config.h"
#include "../state_bus.h"
#include "backend.h"
#include "model_data.h"

#if defined(DATALOGGER_SD)
#include <SPI.h>
#include <SD.h>
#endif

namespace vision {

static volatile bool camOk = false;
static volatile bool backendOk = false;
static volatile uint32_t infCount = 0;
static uint8_t predBins[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ---------------------------------------------------------------- camera --
static bool initCamera(int xclkHz) {
  camera_config_t cfg = {};
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer = LEDC_TIMER_0;
  cfg.pin_pwdn = CAM_PIN_PWDN;
  cfg.pin_reset = CAM_PIN_RESET;
  cfg.pin_xclk = CAM_PIN_XCLK;
  cfg.pin_sccb_sda = CAM_PIN_SIOD;
  cfg.pin_sccb_scl = CAM_PIN_SIOC;
  cfg.pin_d7 = CAM_PIN_Y9;
  cfg.pin_d6 = CAM_PIN_Y8;
  cfg.pin_d5 = CAM_PIN_Y7;
  cfg.pin_d4 = CAM_PIN_Y6;
  cfg.pin_d3 = CAM_PIN_Y5;
  cfg.pin_d2 = CAM_PIN_Y4;
  cfg.pin_d1 = CAM_PIN_Y3;
  cfg.pin_d0 = CAM_PIN_Y2;
  cfg.pin_vsync = CAM_PIN_VSYNC;
  cfg.pin_href = CAM_PIN_HREF;
  cfg.pin_pclk = CAM_PIN_PCLK;
  cfg.xclk_freq_hz = xclkHz;
  // Doc 09 s4.1: grayscale 96x96 direct on OV2640; fb in PSRAM, always
  // grab the newest frame.
  cfg.pixel_format = PIXFORMAT_GRAYSCALE;
  cfg.frame_size = FRAMESIZE_96X96;
  cfg.fb_count = 2;
  cfg.fb_location = CAMERA_FB_IN_PSRAM;
  cfg.grab_mode = CAMERA_GRAB_LATEST;
  return esp_camera_init(&cfg) == ESP_OK;
}

// -------------------------------------------------------------- SD logger --
#if defined(DATALOGGER_SD)
// Binary record, little-endian ('SEC1' format, research doc 09 s4.4 with the
// attitude fields swapped in for AEC/AGC — 9,439 B, parsed by ml/dataset.py).
// RAW ToF zones on disk so bin edges can be re-tuned without re-flying.
#pragma pack(push, 1)
struct LogRecord {
  uint32_t magic;        // 0x53454331 'SEC1'
  uint16_t seq;
  uint32_t tCapMs;       // frame capture time
  uint32_t tTofMs;       // paired ToF frame stamp (skew gate at train time)
  uint8_t  img[VISION_IMG_W * VISION_IMG_H];
  uint16_t tofMm[64];    // raw VL53L5CX 8x8 distances, row-major
  uint8_t  tofStatus[64];// per-zone target_status (5/9 = valid)
  int16_t  rollMrad, pitchMrad, yawMrad;
  uint8_t  batteryDv;
  uint8_t  modelVer;
  uint8_t  predBins[8];  // live model shadow-mode output (0xFF = none)
  uint8_t  crc;          // crc8 over post-img tail (img excluded, doc 09)
};
#pragma pack(pop)
static_assert(sizeof(LogRecord) ==
                  4 + 2 + 4 + 4 + 9216 + 128 + 64 + 6 + 1 + 1 + 8 + 1,
              "LogRecord layout (9439 B)");

static QueueHandle_t logQ = nullptr;      // indices into the PSRAM ring
static LogRecord* ring = nullptr;         // 4 records in PSRAM
static constexpr int kRing = 4;
static File logFile;
static bool sdOk = false;
static uint16_t logSeq = 0;

static void loggerTask(void*) {
  for (;;) {
    int idx;
    if (xQueueReceive(logQ, &idx, portMAX_DELAY) != pdTRUE) continue;
    if (!sdOk || !logFile) continue;
    LogRecord& r = ring[idx];
    // Seal: crc over everything after img (cheap; img excluded per doc 09).
    const uint8_t* tail = reinterpret_cast<const uint8_t*>(r.tofMm);
    const size_t tailLen = sizeof(LogRecord) - offsetof(LogRecord, tofMm) - 1;
    r.crc = sc::crc8(tail, tailLen);
    logFile.write(reinterpret_cast<const uint8_t*>(&r), sizeof(r));
    static int sinceFlush = 0;
    if (++sinceFlush >= 8) { logFile.flush(); sinceFlush = 0; }
  }
}

static void initLogger() {
  // XIAO Sense SD: SPI on GPIO7/8/9, CS 21 (doc 09 s4.3). SD owns this bus.
  SPI.begin(7, 8, 9, PIN_SD_CS);
  sdOk = SD.begin(PIN_SD_CS);
  if (!sdOk) {
    Serial.println("[logger] no SD card - dataset logging disabled");
    return;
  }
  char name[32];
  for (int i = 0; i < 1000; ++i) {
    snprintf(name, sizeof(name), "/sess_%03d_m%u.bin", i, kModelVersion);
    if (!SD.exists(name)) break;
  }
  logFile = SD.open(name, FILE_WRITE);
  ring = static_cast<LogRecord*>(
      heap_caps_malloc(sizeof(LogRecord) * kRing, MALLOC_CAP_SPIRAM));
  logQ = xQueueCreate(kRing, sizeof(int));
  if (logFile && ring && logQ) {
    Serial.printf("[logger] logging to %s (%u B/record)\n", name,
                  unsigned(sizeof(LogRecord)));
    xTaskCreatePinnedToCore(loggerTask, "tLog", STACK_LOGGER, nullptr,
                            PRIO_LOGGER, nullptr, CORE_COMM);
  } else {
    sdOk = false;
    Serial.println("[logger] init failed (PSRAM/queue/file)");
  }
}

static void logFrame(const uint8_t* img, uint32_t tCap) {
  if (!sdOk || !ring) return;
  static int nextIdx = 0;
  // Skip if the ring slot is still queued (logger behind): drop, never block.
  if (uxQueueMessagesWaiting(logQ) >= kRing) return;
  LogRecord& r = ring[nextIdx];
  r.magic = 0x53454331;
  r.seq = logSeq++;
  r.tCapMs = tCap;
  TofGrid grid;
  g_bus.tofGrid(grid);
  r.tTofMs = grid.stampMs;
  memcpy(r.img, img, sizeof(r.img));
  memcpy(r.tofMm, grid.distMm, sizeof(r.tofMm));
  memcpy(r.tofStatus, grid.status, sizeof(r.tofStatus));
  for (int i = 0; i < 8; ++i) r.predBins[i] = predBins[i];
  const ControlSnapshot cs = g_bus.control();
  r.rollMrad = int16_t(cs.state.roll * 1000.0f);
  r.pitchMrad = int16_t(cs.state.pitch * 1000.0f);
  r.yawMrad = int16_t(cs.state.pose.yaw * 1000.0f);
  r.batteryDv = uint8_t(sc::clampf(cs.state.batteryV * 10.0f, 0.0f, 255.0f));
  r.modelVer = kModelVersion;
  const int idx = nextIdx;
  nextIdx = (nextIdx + 1) % kRing;
  xQueueSend(logQ, &idx, 0);
}
#endif  // DATALOGGER_SD

// ------------------------------------------------------------ vision task --
static void taskFn(void*) {
  // XCLK 20 MHz with the documented 10 MHz fallback for the S3 EV-VSYNC-OVF
  // bug (doc 09 s4.2).
  camOk = initCamera(20000000);
  if (!camOk) {
    Serial.println("[vision] 20 MHz XCLK failed, retrying at 10 MHz");
    camOk = initCamera(10000000);
  }
  Serial.println(camOk ? "[vision] camera up (96x96 gray)"
                       : "[vision] ERROR: camera init failed");
  backendOk = vision_backend::init();
#if defined(DATALOGGER_SD)
  if (camOk) initLogger();
#endif

  TickType_t wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / VISION_TARGET_HZ);
  sc::SectorArray sectors;
  for (;;) {
    vTaskDelayUntil(&wake, period);
    if (!camOk) continue;
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) continue;
    const uint32_t tCap = millis();
    if (fb->len >= VISION_IMG_W * VISION_IMG_H) {
      if (backendOk &&
          vision_backend::run(fb->buf, sectors, predBins, tCap)) {
        g_bus.publishVisionSectors(sectors);
        ++infCount;
      }
#if defined(DATALOGGER_SD)
      const ControlSnapshot cs = g_bus.control();
      if (cs.state.armed) logFrame(fb->buf, tCap);  // log while flying
#endif
    }
    esp_camera_fb_return(fb);
  }
}

void start() {
  xTaskCreatePinnedToCore(taskFn, "tVision", STACK_VISION, nullptr,
                          PRIO_VISION, nullptr, CORE_COMM);
}

bool cameraHealthy() { return camOk; }
bool backendReady() { return backendOk; }
uint32_t inferenceCount() { return infCount; }
uint32_t lastInferenceUs() { return vision_backend::lastRunUs(); }
const uint8_t* lastPredBins() { return predBins; }

}  // namespace vision

#else  // !PERCEPTION_V2_VISION

namespace vision {
void start() {}
bool cameraHealthy() { return false; }
bool backendReady() { return false; }
uint32_t inferenceCount() { return 0; }
uint32_t lastInferenceUs() { return 0; }
static const uint8_t kNone[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const uint8_t* lastPredBins() { return kNone; }
}  // namespace vision

#endif
