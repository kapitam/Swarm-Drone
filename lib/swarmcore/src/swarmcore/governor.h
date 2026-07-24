#pragma once
// Speed governor + stop reflex (research doc 08 s7): piecewise-linear forward
// speed cap from the minimum forward sector distance, sized for 15 Hz sensor
// updates + ~100 ms latency. The reflex layer is the safety authority under
// every behavior — learned perception never owns it (doc 05/07 rule).
//
//   >= farMm (2000): vMax
//   reactMm (1400):  vReact (0.5)   — linear in between
//   slowMm  (700):   vSlow  (0.15)  — linear in between
//   stopMm  (400):   0, STOP flag
//   <  fearMm (150): back away at vBack

#include "types.h"

namespace sc {

struct GovernorParams {
  uint16_t farMm   = 2000;
  uint16_t reactMm = 1400;
  uint16_t slowMm  = 700;
  uint16_t stopMm  = 400;
  uint16_t fearMm  = 150;
  float vMax   = 1.0f;
  float vReact = 0.5f;
  float vSlow  = 0.15f;
  float vBack  = 0.2f;   // retreat speed inside fearMm
  // Unknown forward distance (sensor dead): crawl, don't fly blind.
  float vUnknown = 0.15f;
};

enum class ReflexState : uint8_t { kCruise, kSlow, kStop, kFear, kBlind };

struct GovernorOutput {
  float speedCap = 0.0f;   // max forward speed [m/s]
  float backoff  = 0.0f;   // commanded retreat speed [m/s] (kFear only)
  ReflexState state = ReflexState::kCruise;
};

class Governor {
 public:
  explicit Governor(const GovernorParams& p = {}) : p_(p) {}
  void setParams(const GovernorParams& p) { p_ = p; }
  const GovernorParams& params() const { return p_; }

  GovernorOutput update(const SectorArray& s) const {
    return fromDistance(s.minForwardMm());
  }
  GovernorOutput fromDistance(uint16_t minForwardMm) const;

 private:
  GovernorParams p_;
};

}  // namespace sc
