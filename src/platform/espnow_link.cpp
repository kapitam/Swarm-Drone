#include "espnow_link.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "../config/config.h"

namespace espnow_link {

static QueueHandle_t rxQ = nullptr;
static uint32_t txCnt = 0, rxCnt = 0, rxDrop = 0;
static const uint8_t kBroadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Arduino core 3.x (IDF 5.x) receive callback signature.
static void onRecv(const esp_now_recv_info_t* info, const uint8_t* data,
                   int len) {
  if (!rxQ || len <= 0 || len > int(sizeof(RxItem::data))) return;
  RxItem item;
  memcpy(item.data, data, len);
  item.len = uint8_t(len);
  item.rssi = info->rx_ctrl ? info->rx_ctrl->rssi : 0;
  item.tMs = millis();
  // Never block in the Wi-Fi task (doc 04 s5.2): drop when full.
  if (xQueueSend(rxQ, &item, 0) != pdTRUE) ++rxDrop;
  else ++rxCnt;
}

bool init() {
  rxQ = xQueueCreate(24, sizeof(RxItem));
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(onRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, kBroadcast, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
}

bool send(const void* data, size_t len) {
  const bool ok =
      esp_now_send(kBroadcast, static_cast<const uint8_t*>(data), len) == ESP_OK;
  if (ok) ++txCnt;
  return ok;
}

bool receive(RxItem& out, uint32_t waitMs) {
  return rxQ && xQueueReceive(rxQ, &out, pdMS_TO_TICKS(waitMs)) == pdTRUE;
}

uint32_t txCount() { return txCnt; }
uint32_t rxCount() { return rxCnt; }
uint32_t rxDropCount() { return rxDrop; }

}  // namespace espnow_link
