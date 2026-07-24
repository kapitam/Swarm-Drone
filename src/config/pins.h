#pragma once
// Pin maps per board. The PCB drone's final map is NOT decided — when the
// PCB lands, add a BOARD_PCB_V1 block here and nothing else changes.
// Selected by exactly one BOARD_* build flag (platformio.ini env).

#if defined(BOARD_DEVKIT_V1)
// ESP32 DOIT DevKit v1 (current bench hardware; matches the original sketch).
#define PIN_MOTOR_1 25   // front-right (kept from original ESC pin)
#define PIN_MOTOR_2 26   // rear-right
#define PIN_MOTOR_3 27   // rear-left
#define PIN_MOTOR_4 14   // front-left
// nRF24L01 on VSPI (original wiring).
#define PIN_NRF_CE 4
#define PIN_NRF_CSN 5
#define PIN_VSPI_SCK 18
#define PIN_VSPI_MISO 19
#define PIN_VSPI_MOSI 23
// IMU on Wire (400 kHz).
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22
// VL53L5CX on its own bus (Wire1) so its 1 MHz clock doesn't constrain the
// IMU (research doc 08 s2.4: design for 1 MHz, bring up at 400 kHz).
#define TOF_USE_WIRE1 1
#define PIN_TOF_SDA 33
#define PIN_TOF_SCL 32
#define TOF_I2C_HZ 400000
#define PIN_BATTERY_ADC 34   // ADC1 channel (ADC2 unusable with Wi-Fi, doc 03)

#elif defined(BOARD_XIAO_S3)
// Seeed XIAO ESP32S3 Sense (Build B "Vision").
// SD (expansion board) owns the only exposed SPI bus -> no nRF24 here
// (research doc 09 s4.3); RC + swarm both run over ESP-NOW.
#define PIN_MOTOR_1 1    // D0..D3
#define PIN_MOTOR_2 2
#define PIN_MOTOR_3 3
#define PIN_MOTOR_4 4
// IMU + VL53L5CX teacher share the exposed I2C header (D4/D5), 400 kHz
// (camera SCCB is a separate internal bus on GPIO39/40).
#define PIN_I2C_SDA 5
#define PIN_I2C_SCL 6
#define TOF_USE_WIRE1 0
#define TOF_I2C_HZ 400000
#define PIN_SD_CS 21     // internal, Sense expansion board
#define PIN_BATTERY_ADC -1  // no divider on the bare board

// XIAO ESP32S3 Sense OV2640 camera pin map (Seeed wiki / doc 09 s4.2).
#define CAM_PIN_PWDN -1
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK 10
#define CAM_PIN_SIOD 40
#define CAM_PIN_SIOC 39
#define CAM_PIN_Y9 48
#define CAM_PIN_Y8 11
#define CAM_PIN_Y7 12
#define CAM_PIN_Y6 14
#define CAM_PIN_Y5 16
#define CAM_PIN_Y4 18
#define CAM_PIN_Y3 17
#define CAM_PIN_Y2 15
#define CAM_PIN_VSYNC 38
#define CAM_PIN_HREF 47
#define CAM_PIN_PCLK 13

#else
#error "Define exactly one BOARD_* flag (BOARD_DEVKIT_V1 or BOARD_XIAO_S3)"
#endif
