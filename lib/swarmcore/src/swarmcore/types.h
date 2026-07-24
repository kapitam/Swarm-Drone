#pragma once
// Portable base types for the swarm drone core. No Arduino/ESP dependencies.
// Units: SI floats internally (m, m/s, rad, s); packets use cm / mrad / ms.

#include <stdint.h>
#include <math.h>

namespace sc {

constexpr int   kSectors        = 8;      // forward polar sectors (V1/V2 contract)
constexpr float kSectorFovDeg   = 63.0f;  // VL53L5CX diagonal-limited forward FoV
constexpr float kPi             = 3.14159265358979f;
constexpr float kGravity        = 9.80665f;

inline float deg2rad(float d) { return d * kPi / 180.0f; }
inline float rad2deg(float r) { return r * 180.0f / kPi; }
inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
// Wrap angle to (-pi, pi]
inline float wrapPi(float a) {
  while (a > kPi) a -= 2.0f * kPi;
  while (a <= -kPi) a += 2.0f * kPi;
  return a;
}

struct Vec2 {
  float x = 0.0f, y = 0.0f;
  Vec2() = default;
  Vec2(float x_, float y_) : x(x_), y(y_) {}
  Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
  Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
  Vec2 operator*(float s) const { return {x * s, y * s}; }
  Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
  float dot(const Vec2& o) const { return x * o.x + y * o.y; }
  float norm() const { return sqrtf(x * x + y * y); }
  Vec2 normalized() const {
    float n = norm();
    return n > 1e-6f ? Vec2{x / n, y / n} : Vec2{0.0f, 0.0f};
  }
  Vec2 limited(float maxNorm) const {
    float n = norm();
    return (n > maxNorm && n > 1e-6f) ? Vec2{x * maxNorm / n, y * maxNorm / n} : *this;
  }
};

struct Pose2D {
  Vec2  p;          // position in shared frame [m]
  float yaw = 0.0f; // heading [rad], 0 = +x, CCW positive
};

// Own-vehicle state snapshot exchanged between tasks (single writer: control task).
struct VehicleState {
  Pose2D   pose;
  Vec2     vel;               // horizontal velocity estimate [m/s], shared frame
  float    z = 0.0f;          // altitude estimate [m] (0 until a height sensor lands)
  float    roll = 0.0f, pitch = 0.0f;   // [rad]
  float    yawRate = 0.0f;    // [rad/s]
  float    batteryV = 0.0f;
  bool     poseValid = false; // dead-reckoning quality flag
  bool     armed = false;
  uint32_t tMs = 0;
};

// The shared perception contract: forward polar sectors, nearest obstacle per
// sector. Sector 0 = leftmost, kSectors-1 = rightmost within the forward FoV.
struct SectorArray {
  static constexpr uint16_t kMaxMm      = 2000;  // clamp (doc 08: trust <=2 m)
  static constexpr uint16_t kUnknownMm  = 0xFFFF;
  uint16_t distMm[kSectors];
  uint8_t  validZones[kSectors];  // contributing valid zones (0 => distMm unknown)
  uint32_t stampMs = 0;
  SectorArray() {
    for (int i = 0; i < kSectors; ++i) { distMm[i] = kUnknownMm; validZones[i] = 0; }
  }
  bool known(int i) const { return validZones[i] > 0 && distMm[i] != kUnknownMm; }
  // Minimum distance across the central sectors (used by the speed governor).
  uint16_t minForwardMm(int halfWidth = kSectors / 2) const {
    uint16_t m = kUnknownMm;
    int lo = kSectors / 2 - halfWidth, hi = kSectors / 2 + halfWidth - 1;
    if (lo < 0) lo = 0;
    if (hi >= kSectors) hi = kSectors - 1;
    for (int i = lo; i <= hi; ++i)
      if (known(i) && distMm[i] < m) m = distMm[i];
    return m;
  }
  static float sectorCenterRad(int i) {
    const float span = deg2rad(kSectorFovDeg);
    return (float(i) - (kSectors - 1) / 2.0f) * (span / kSectors);
  }
};

}  // namespace sc
