#pragma once
// ESP-NOW broadcast link (research docs 01 s6, 04 s5.2): callbacks run inside
// the high-priority Wi-Fi task -> they only copy + enqueue; the swarm task
// drains the queue. TX is broadcast to FF:FF:FF:FF:FF:FF on a fixed channel.

#include <stdint.h>
#include <stddef.h>

namespace espnow_link {

struct RxItem {
  uint8_t data[32];   // all our packets are <= 25 B
  uint8_t len = 0;
  int8_t rssi = 0;
  uint32_t tMs = 0;
};

bool init();  // starts Wi-Fi STA on ESPNOW_CHANNEL + registers callbacks
bool send(const void* data, size_t len);
// Drain one item; returns false if queue empty.
bool receive(RxItem& out, uint32_t waitMs);
uint32_t txCount();
uint32_t rxCount();
uint32_t rxDropCount();

}  // namespace espnow_link
