#pragma once
// Firmware-wide configuration: task layout (research doc 04 s7.2), rates,
// radio constants, defaults. One documented priority ladder, one place.

#include <stdint.h>
#include "pins.h"

// ---- Identity -------------------------------------------------------------
#ifndef ROBOT_ID_DEFAULT
#define ROBOT_ID_DEFAULT 1      // overridden by NVS ("robot.id"), set via CmdPacket
#endif

// ---- Task ladder (doc 04 s7.2). Higher = more urgent; app tasks stay below
// the Wi-Fi driver task on core 0. Real-time tasks: unique priorities. ----
#define CORE_RT   1   // hard real-time domain
#define CORE_COMM 0   // Wi-Fi/radio/telemetry domain

#define PRIO_CONTROL 20
#define PRIO_AVOID   15
#define PRIO_TOF     14   // sensor service (core 0, blocking I2C)
#define PRIO_VISION  12
#define PRIO_SWARM   17   // above telem: beacons and cmd handling
#define PRIO_LOGGER  4
#define PRIO_TELEM   10

#define STACK_CONTROL 4096
#define STACK_AVOID   4096
#define STACK_TOF     4096
#define STACK_VISION  6144
#define STACK_SWARM   4096
#define STACK_TELEM   4096
#define STACK_LOGGER  4096

// ---- Rates ------------------------------------------------------------------
#define CONTROL_HZ        500
#define AVOID_HZ          50
#define BEACON_HZ         10
#define TELEM_HZ          5
#define NEIGHBOR_TTL_MS   1500

// ---- Radio ------------------------------------------------------------------
#define ESPNOW_CHANNEL    6
#define NRF_CHANNEL       108        // from the original sketch
#define NRF_PIPE_ADDR     "00001"
#define GRADIENT_SEED_ID  0          // robot id that anchors the hop-count
                                     // gradient (0 = operator ground station)

// ---- Battery sense ------------------------------------------------------------
#define BATT_DIVIDER      2.0f       // ADC divider ratio (PCB TBD)

// ---- Safety -----------------------------------------------------------------
#define RC_TIMEOUT_MS     150        // original sketch's failsafe budget
#define LOW_BATT_PER_CELL 3.4f
#define BATT_CELLS        1          // PCB drone: 1S assumed until hardware lands

// ---- Vision (Build B) ---------------------------------------------------------
#define VISION_IMG_W      96
#define VISION_IMG_H      96
#define VISION_TARGET_HZ  10
