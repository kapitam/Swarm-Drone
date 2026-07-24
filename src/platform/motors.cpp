#include "motors.h"

#include <Arduino.h>
#include "../config/config.h"
#include "swarmcore/types.h"

namespace motors {

static const uint8_t kPins[4] = {PIN_MOTOR_1, PIN_MOTOR_2, PIN_MOTOR_3,
                                 PIN_MOTOR_4};

#if defined(MOTORS_ESC_PWM)
// 50 Hz servo-style pulses, 14-bit resolution: 1 us = 0.8192 counts.
static constexpr uint32_t kFreq = 50;
static constexpr uint8_t kRes = 14;
static constexpr float kUsToDuty = float(1 << kRes) * kFreq / 1000000.0f;
static constexpr uint16_t kMinUs = 1000, kMaxUs = 2000;

static void writeUs(uint8_t pin, uint16_t us) {
  ledcWrite(pin, uint32_t(us * kUsToDuty));
}

void init() {
  for (uint8_t p : kPins) {
    ledcAttach(p, kFreq, kRes);   // Arduino core 3.x LEDC API
    writeUs(p, kMinUs);           // ESCs must see min pulse at boot to arm
  }
}

void write(const float m[4]) {
  for (int i = 0; i < 4; ++i) {
    const float v = sc::clampf(m[i], 0.0f, 1.0f);
    writeUs(kPins[i], uint16_t(kMinUs + v * (kMaxUs - kMinUs)));
  }
}

void stop() {
  for (uint8_t p : kPins) writeUs(p, kMinUs);
}

#elif defined(MOTORS_BRUSHED_PWM)
// 20 kHz above audible, 10-bit duty directly into the gate drivers.
static constexpr uint32_t kFreq = 20000;
static constexpr uint8_t kRes = 10;
static constexpr uint32_t kMaxDuty = (1 << kRes) - 1;

void init() {
  for (uint8_t p : kPins) {
    ledcAttach(p, kFreq, kRes);
    ledcWrite(p, 0);
  }
}

void write(const float m[4]) {
  for (int i = 0; i < 4; ++i) {
    const float v = sc::clampf(m[i], 0.0f, 1.0f);
    ledcWrite(kPins[i], uint32_t(v * kMaxDuty));
  }
}

void stop() {
  for (uint8_t p : kPins) ledcWrite(p, 0);
}

#else
#error "Define MOTORS_ESC_PWM or MOTORS_BRUSHED_PWM"
#endif

}  // namespace motors
