#include "rc_link.h"

#include <Arduino.h>
#include "../config/config.h"
#include "../state_bus.h"
#include "swarmcore/packets.h"

#if defined(RC_LINK_NRF24)
#include <SPI.h>
#include <RF24.h>
#endif

namespace rc_link {

#if defined(RC_LINK_NRF24)

static RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);
static SPIClass vspi(VSPI);

// Poll at 250 Hz — the nRF24 FIFO holds 3 frames, so nothing is lost between
// polls at the transmitter's ~50 Hz rate. (IRQ wiring is a PCB option; the
// devkit harness has no IRQ pin connected. See doc 04 s5.1.)
static void taskFn(void*) {
  vspi.begin(PIN_VSPI_SCK, PIN_VSPI_MISO, PIN_VSPI_MOSI, PIN_NRF_CSN);
  bool ok = radio.begin(&vspi);
  if (ok) {
    radio.setDataRate(RF24_1MBPS);
    radio.setPALevel(RF24_PA_LOW);
    radio.setChannel(NRF_CHANNEL);
    radio.openReadingPipe(0, reinterpret_cast<const uint8_t*>(NRF_PIPE_ADDR));
    radio.startListening();
    Serial.println("[rc] nRF24 listening");
  } else {
    Serial.println("[rc] ERROR: nRF24 not responding");
  }
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(4));
    if (!ok) continue;
    while (radio.available()) {
      sc::RcPacket p{};
      radio.read(&p, sizeof(p));
      // Plausibility gate (unvalidated radio bytes): channels in 900..2100.
      bool sane = true;
      for (int i = 0; i < 4; ++i)
        if (p.ch[i] < 900 || p.ch[i] > 2100) sane = false;
      if (sane) g_bus.publishRc(p, millis());
    }
  }
}

void start() {
  xTaskCreatePinnedToCore(taskFn, "tRadioNrf", 3072, nullptr, 18, nullptr,
                          CORE_COMM);
}
const char* name() { return "nRF24"; }
void feedEspNow(const uint8_t*, int, unsigned long) {}

#elif defined(RC_LINK_ESPNOW)

// No dedicated task: the swarm task owns the ESP-NOW RX queue and forwards
// RC frames here.
void start() { Serial.println("[rc] RC over ESP-NOW (fed by swarm task)"); }
const char* name() { return "ESP-NOW"; }

void feedEspNow(const uint8_t* data, int len, unsigned long nowMs) {
  if (len != int(sizeof(sc::RcEspNowPacket))) return;
  sc::RcEspNowPacket p;
  memcpy(&p, data, sizeof(p));
  if (!sc::checkRcEspNow(p)) return;
  g_bus.publishRc(p.rc, nowMs);
}

#else
#error "Define RC_LINK_NRF24 or RC_LINK_ESPNOW"
#endif

}  // namespace rc_link
