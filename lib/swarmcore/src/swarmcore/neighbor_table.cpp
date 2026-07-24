#include "neighbor_table.h"

namespace sc {

bool NeighborTable::update(const StatePacket& p, uint32_t nowMs, uint8_t selfId) {
  if (!checkState(p) || p.id == selfId) return false;

  Neighbor* slot = nullptr;
  Neighbor* freeSlot = nullptr;
  Neighbor* oldest = nullptr;
  for (auto& e : n_) {
    if (e.used && e.id == p.id) { slot = &e; break; }
    if (!e.used && !freeSlot) freeSlot = &e;
    if (e.used && (!oldest || e.lastHeardMs < oldest->lastHeardMs)) oldest = &e;
  }
  if (!slot) slot = freeSlot ? freeSlot : oldest;  // evict oldest when full
  if (!slot) return false;

  if (slot->used && slot->id == p.id) {
    uint16_t gap = uint16_t(p.seq - slot->lastSeq);
    if (gap > 1 && gap < 1000) slot->lossEstimate += gap - 1;
  } else {
    *slot = Neighbor{};
    slot->id = p.id;
  }

  slot->pose.p = {p.xCm * 0.01f, p.yCm * 0.01f};
  slot->pose.yaw = p.headingMrad * 1e-3f;
  slot->z = p.zCm * 0.01f;
  slot->vel = {p.vxCmS * 0.01f, p.vyCmS * 0.01f};
  slot->cmdVel = {p.cmdVxCmS * 0.01f, p.cmdVyCmS * 0.01f};
  slot->mode = p.mode;
  slot->gradient = p.gradient;
  slot->flags = p.flags;
  slot->batteryV = p.batteryDv * 0.1f;
  slot->lastSeq = p.seq;
  slot->lastHeardMs = nowMs;
  if (slot->rxCount < 0xFFFF) ++slot->rxCount;
  slot->used = true;
  return true;
}

int NeighborTable::expire(uint32_t nowMs, uint32_t ttlMs) {
  int live = 0;
  for (auto& e : n_) {
    if (!e.used) continue;
    if (nowMs - e.lastHeardMs > ttlMs) e.used = false;
    else ++live;
  }
  return live;
}

int NeighborTable::count() const {
  int c = 0;
  for (const auto& e : n_) c += e.used ? 1 : 0;
  return c;
}

const Neighbor* NeighborTable::byId(uint8_t id) const {
  for (const auto& e : n_)
    if (e.used && e.id == id) return &e;
  return nullptr;
}

uint8_t NeighborTable::minNeighborGradient(uint32_t nowMs, uint32_t ttlMs) const {
  uint8_t g = 0xFF;
  for (const auto& e : n_) {
    if (!e.used || nowMs - e.lastHeardMs > ttlMs) continue;
    if (e.gradient < g) g = e.gradient;
  }
  return g;
}

}  // namespace sc
