#include "governor.h"

namespace sc {

static float lerp(float a, float b, float t) { return a + (b - a) * t; }

GovernorOutput Governor::fromDistance(uint16_t dMm) const {
  GovernorOutput o;
  if (dMm == SectorArray::kUnknownMm) {
    o.speedCap = p_.vUnknown;
    o.state = ReflexState::kBlind;
    return o;
  }
  if (dMm < p_.fearMm) {
    o.speedCap = 0.0f;
    o.backoff = p_.vBack;
    o.state = ReflexState::kFear;
    return o;
  }
  if (dMm < p_.stopMm) {
    o.speedCap = 0.0f;
    o.state = ReflexState::kStop;
    return o;
  }
  if (dMm < p_.slowMm) {
    const float t = float(dMm - p_.stopMm) / float(p_.slowMm - p_.stopMm);
    o.speedCap = lerp(0.0f, p_.vSlow, t);
    o.state = ReflexState::kSlow;
    return o;
  }
  if (dMm < p_.reactMm) {
    const float t = float(dMm - p_.slowMm) / float(p_.reactMm - p_.slowMm);
    o.speedCap = lerp(p_.vSlow, p_.vReact, t);
    o.state = ReflexState::kSlow;
    return o;
  }
  if (dMm < p_.farMm) {
    const float t = float(dMm - p_.reactMm) / float(p_.farMm - p_.reactMm);
    o.speedCap = lerp(p_.vReact, p_.vMax, t);
    o.state = ReflexState::kCruise;
    return o;
  }
  o.speedCap = p_.vMax;
  o.state = ReflexState::kCruise;
  return o;
}

}  // namespace sc
