#pragma once
// Wire formats. Little-endian packed structs, <=32 B so every packet fits a
// single nRF24L01 frame as well as ESP-NOW (250 B limit is never a concern).
// CRC8 (poly 0x31, init 0xFF) over all bytes preceding the crc field.

#include <stdint.h>
#include <string.h>

namespace sc {

inline uint8_t crc8(const uint8_t* d, size_t n) {
  uint8_t crc = 0xFF;
  for (size_t i = 0; i < n; ++i) {
    crc ^= d[i];
    for (int b = 0; b < 8; ++b)
      crc = (crc & 0x80) ? uint8_t((crc << 1) ^ 0x31) : uint8_t(crc << 1);
  }
  return crc;
}

enum PacketMagic : uint8_t {
  kMagicState = 0xA5,
  kMagicCmd   = 0xC3,
  kMagicRc    = 0xB4,
};

// Legacy manual-control packet — byte-compatible with the original sketch's
// transmitter (4x uint16 channel values, 1000..2000 us convention).
#pragma pack(push, 1)
struct RcPacket {
  uint16_t ch[4];  // 0: throttle, 1: roll, 2: pitch, 3: yaw
};

// Swarm state beacon, broadcast at ~10 Hz (doc 01 s2: id+pose+vel+timestamp).
struct StatePacket {
  uint8_t  magic;        // kMagicState
  uint8_t  id;           // robot id (unique per fleet)
  uint16_t seq;          // wraps; used for loss estimation
  int16_t  xCm, yCm;     // position in shared frame [cm]
  int16_t  zCm;          // altitude [cm]
  int16_t  vxCmS, vyCmS; // velocity estimate [cm/s]
  // Commanded/intended planar velocity [cm/s], world frame. For a manually
  // flown leader this is the stick intent — followers in MIMIC mode track it
  // without needing a shared position frame (CONOPS: leader-led flock).
  int16_t  cmdVxCmS, cmdVyCmS;
  int16_t  headingMrad;  // yaw [milliradians], wrapped (-3142..3142]
  uint8_t  mode;         // BehaviorMode
  uint8_t  gradient;     // hop count to seed (255 = unknown/infinity)
  uint8_t  flags;        // bit0 armed, bit1 poseValid, bit2 lowBattery
  uint8_t  batteryDv;    // battery [deciVolt] (e.g. 3.8 V -> 38)
  uint32_t tMs;          // sender millis at send time (coarse latency estimate)
  uint8_t  crc;
};  // 29 bytes (still fits a 32 B nRF24 frame if ever relayed)

enum StateFlags : uint8_t {
  kFlagArmed      = 1 << 0,
  kFlagPoseValid  = 1 << 1,
  kFlagLowBattery = 1 << 2,
};

enum CmdType : uint8_t {
  kCmdSetMode  = 1,   // arg0 = BehaviorMode
  kCmdEStop    = 2,   // fleet-wide motor cut (latched until disarm+rearm)
  kCmdZeroPose = 3,   // re-zero dead-reckoned pose at current location
  kCmdSetParam = 4,   // arg0 = param id, argF = value
  kCmdSetLeader= 5,   // arg0 = leader robot id
  kCmdSetGoal  = 6,   // argF = x [m], argF2 = y [m] in the shared frame.
                      // THE GPS/computer seam: a ground computer (or a
                      // GPS-equipped leader) streams goals here; FLOCK's
                      // goal term consumes them. arg0 = 0 clears the goal.
};

// Operator command, broadcast; targetId 0xFF = whole fleet.
struct CmdPacket {
  uint8_t  magic;     // kMagicCmd
  uint8_t  targetId;  // robot id or 0xFF broadcast
  uint8_t  type;      // CmdType
  uint8_t  arg0;
  float    argF;
  float    argF2;
  uint32_t tMs;
  uint8_t  crc;
};  // 17 bytes

// RC over ESP-NOW (RC_LINK_ESPNOW fork): the legacy 4-channel payload wrapped
// with magic + target + crc so it can share the broadcast channel.
struct RcEspNowPacket {
  uint8_t  magic;     // kMagicRc
  uint8_t  targetId;  // robot id or 0xFF
  RcPacket rc;
  uint8_t  crc;
};  // 11 bytes
#pragma pack(pop)

static_assert(sizeof(RcPacket) == 8, "RcPacket layout");
static_assert(sizeof(StatePacket) == 29, "StatePacket layout");
static_assert(sizeof(CmdPacket) == 17, "CmdPacket layout");
static_assert(sizeof(RcEspNowPacket) == 11, "RcEspNowPacket layout");

inline void sealState(StatePacket& p) {
  p.magic = kMagicState;
  p.crc = crc8(reinterpret_cast<const uint8_t*>(&p), sizeof(p) - 1);
}
inline bool checkState(const StatePacket& p) {
  return p.magic == kMagicState &&
         p.crc == crc8(reinterpret_cast<const uint8_t*>(&p), sizeof(p) - 1);
}
inline void sealCmd(CmdPacket& p) {
  p.magic = kMagicCmd;
  p.crc = crc8(reinterpret_cast<const uint8_t*>(&p), sizeof(p) - 1);
}
inline bool checkCmd(const CmdPacket& p) {
  return p.magic == kMagicCmd &&
         p.crc == crc8(reinterpret_cast<const uint8_t*>(&p), sizeof(p) - 1);
}
inline void sealRcEspNow(RcEspNowPacket& p) {
  p.magic = kMagicRc;
  p.crc = crc8(reinterpret_cast<const uint8_t*>(&p), sizeof(p) - 1);
}
inline bool checkRcEspNow(const RcEspNowPacket& p) {
  return p.magic == kMagicRc &&
         p.crc == crc8(reinterpret_cast<const uint8_t*>(&p), sizeof(p) - 1);
}

}  // namespace sc
