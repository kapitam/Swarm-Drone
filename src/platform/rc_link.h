#pragma once
// RC input fork (HANDBOOK "Forks"):
//   RC_LINK_NRF24  original transmitter hardware: 8-byte RcPacket on an
//                  nRF24L01 (VSPI, channel 108, pipe "00001") — byte-
//                  compatible with the pre-existing sketch/transmitter.
//   RC_LINK_ESPNOW RcEspNowPacket over the swarm broadcast channel: one
//                  radio total (mandatory on XIAO S3, where SD owns SPI).
// Either way the task publishes into g_bus; consumers only see RcSnapshot.

#include <stdint.h>

namespace rc_link {

// Spawns the service task appropriate to the fork (core 0).
void start();
const char* name();

// RC_LINK_ESPNOW only: called by the swarm task when an RC packet arrives on
// the shared ESP-NOW queue (the swarm task owns the RX queue).
void feedEspNow(const uint8_t* data, int len, unsigned long nowMs);

}  // namespace rc_link
