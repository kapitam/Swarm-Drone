#pragma once
// Kilobot-style hop-count gradient (research doc 01 s4.2): the positioning-free
// coordination fallback. Seed (operator station / designated robot) holds 0;
// everyone else adopts min(heard) + 1. Self-heals as beacons arrive.

#include <stdint.h>

namespace sc {

class Gradient {
 public:
  void setSeed(bool isSeed) { isSeed_ = isSeed; }

  // minNeighborGradient: from NeighborTable (255 if no live neighbor).
  uint8_t update(uint8_t minNeighborGradient) {
    if (isSeed_) { value_ = 0; return value_; }
    value_ = (minNeighborGradient == 0xFF || minNeighborGradient >= 0xFE)
                 ? 0xFF
                 : uint8_t(minNeighborGradient + 1);
    return value_;
  }

  uint8_t value() const { return value_; }
  bool connected() const { return value_ != 0xFF; }

 private:
  bool isSeed_ = false;
  uint8_t value_ = 0xFF;
};

}  // namespace sc
