// Host-native unit tests for lib/swarmcore.  Run:  pio test -e native
#include <unity.h>
#include <string.h>

#include "swarmcore/types.h"
#include "swarmcore/packets.h"
#include "swarmcore/sectors.h"
#include "swarmcore/neighbor_table.h"
#include "swarmcore/gradient.h"
#include "swarmcore/boids.h"
#include "swarmcore/leader_follower.h"
#include "swarmcore/vfh_lite.h"
#include "swarmcore/governor.h"
#include "swarmcore/bvc.h"
#include "swarmcore/mahony.h"
#include "swarmcore/mixer.h"
#include "swarmcore/arming.h"
#include "swarmcore/behavior.h"

using namespace sc;

// ---------------------------------------------------------------- packets --
static void test_state_packet_roundtrip() {
  StatePacket p{};
  p.id = 7; p.seq = 42; p.xCm = -123; p.yCm = 456; p.zCm = 78;
  p.vxCmS = 100; p.vyCmS = -50; p.headingMrad = 1571;
  p.mode = 3; p.gradient = 2; p.flags = kFlagArmed | kFlagPoseValid;
  p.batteryDv = 38; p.tMs = 123456;
  sealState(p);
  TEST_ASSERT_TRUE(checkState(p));
  p.xCm += 1;  // corrupt
  TEST_ASSERT_FALSE(checkState(p));
}

static void test_cmd_packet_roundtrip() {
  CmdPacket c{};
  c.targetId = 0xFF; c.type = kCmdEStop; c.arg0 = 0; c.argF = 0.0f; c.tMs = 1;
  sealCmd(c);
  TEST_ASSERT_TRUE(checkCmd(c));
  c.type = kCmdSetMode;
  TEST_ASSERT_FALSE(checkCmd(c));
}

// ---------------------------------------------------------------- sectors --
static void fillGrid(uint16_t* d, uint8_t* st, uint8_t* nb, uint16_t dist) {
  for (int i = 0; i < 64; ++i) { d[i] = dist; st[i] = 5; nb[i] = 1; }
}

static void test_sector_min_pool_middle_rows() {
  uint16_t d[64]; uint8_t st[64], nb[64];
  fillGrid(d, st, nb, 1800);
  // Obstacle at 600 mm in column 3, row 4 (middle band) -> sector 3.
  d[4 * 8 + 3] = 600;
  // A closer return in row 7 (ground band) must be ignored.
  d[7 * 8 + 3] = 200;
  SectorFilter f;
  const SectorArray& s = f.update(d, st, nb, 1000);
  TEST_ASSERT_EQUAL_UINT16(600, s.distMm[3]);
  TEST_ASSERT_EQUAL_UINT16(1800, s.distMm[2]);
  TEST_ASSERT_EQUAL_UINT16(600, s.minForwardMm());
}

static void test_sector_invalid_not_free_and_hold() {
  uint16_t d[64]; uint8_t st[64], nb[64];
  fillGrid(d, st, nb, 500);
  SectorFilter f;
  f.update(d, st, nb, 0);
  // Now all zones go invalid (status 255): sectors must HOLD 500 for 3 frames.
  for (int i = 0; i < 64; ++i) st[i] = 255;
  for (int k = 1; k <= 3; ++k) {
    const SectorArray& s = f.update(d, st, nb, k * 66);
    TEST_ASSERT_TRUE(s.known(4));
    TEST_ASSERT_EQUAL_UINT16(500, s.distMm[4]);
  }
  const SectorArray& s4 = f.update(d, st, nb, 4 * 66);
  TEST_ASSERT_FALSE(s4.known(4));  // degraded to unknown after holdFrames
  TEST_ASSERT_EQUAL_UINT16(SectorArray::kUnknownMm, s4.minForwardMm());
}

static void test_sector_clamp_and_status_filter() {
  uint16_t d[64]; uint8_t st[64], nb[64];
  fillGrid(d, st, nb, 3500);           // beyond clamp
  st[3 * 8 + 0] = 4;                   // invalid status in sector 0 band
  SectorFilter f;
  const SectorArray& s = f.update(d, st, nb, 10);
  TEST_ASSERT_EQUAL_UINT16(SectorArray::kMaxMm, s.distMm[0]);  // other rows fill it
  TEST_ASSERT_EQUAL_UINT16(SectorArray::kMaxMm, s.distMm[7]);
}

// ---------------------------------------------------------- neighbor table --
static StatePacket mkState(uint8_t id, uint16_t seq, float x, float y,
                           float vx, float vy) {
  StatePacket p{};
  p.id = id; p.seq = seq;
  p.xCm = int16_t(x * 100); p.yCm = int16_t(y * 100);
  p.vxCmS = int16_t(vx * 100); p.vyCmS = int16_t(vy * 100);
  p.flags = kFlagPoseValid;
  sealState(p);
  return p;
}

static void test_neighbor_update_extrapolate_expire() {
  NeighborTable t;
  TEST_ASSERT_TRUE(t.update(mkState(2, 1, 1.0f, 0.0f, 0.5f, 0.0f), 1000, /*self*/ 1));
  TEST_ASSERT_FALSE(t.update(mkState(1, 1, 0, 0, 0, 0), 1000, 1));  // own echo
  TEST_ASSERT_EQUAL_INT(1, t.count());
  const Neighbor* nb = t.byId(2);
  TEST_ASSERT_NOT_NULL(nb);
  // 500 ms later at vx=0.5 -> x should extrapolate to 1.25.
  Vec2 pred = nb->predictedPos(1500);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.25f, pred.x);
  // Expiry.
  TEST_ASSERT_EQUAL_INT(1, t.expire(2000, 1500));
  TEST_ASSERT_EQUAL_INT(0, t.expire(3000, 1500));
  TEST_ASSERT_EQUAL_INT(0, t.count());
}

static void test_neighbor_loss_estimate_and_eviction() {
  NeighborTable t;
  t.update(mkState(2, 1, 0, 0, 0, 0), 0, 1);
  t.update(mkState(2, 5, 0, 0, 0, 0), 100, 1);  // 3 lost
  TEST_ASSERT_EQUAL_UINT16(3, t.byId(2)->lossEstimate);
  // Fill beyond capacity: oldest must be evicted, newest present.
  for (uint8_t id = 3; id < 3 + NeighborTable::kMax; ++id)
    t.update(mkState(id, 1, 0, 0, 0, 0), 200 + id, 1);
  TEST_ASSERT_EQUAL_INT(NeighborTable::kMax, t.count());
  TEST_ASSERT_NULL(t.byId(2));  // evicted (oldest)
}

// --------------------------------------------------------------- gradient --
static void test_gradient_hopcount() {
  Gradient g;
  g.setSeed(false);
  TEST_ASSERT_EQUAL_UINT8(0xFF, g.update(0xFF));  // disconnected
  TEST_ASSERT_EQUAL_UINT8(3, g.update(2));
  g.setSeed(true);
  TEST_ASSERT_EQUAL_UINT8(0, g.update(0xFF));
}

// ------------------------------------------------------------------ boids --
static void test_boids_separation_pushes_away() {
  NeighborTable t;
  t.update(mkState(2, 1, 0.5f, 0.0f, 0.0f, 0.0f), 1000, 1);  // 0.5 m ahead +x
  VehicleState self;
  self.pose.p = {0, 0};
  BoidsParams bp;
  bp.fenceXMin = -100; bp.fenceXMax = 100; bp.fenceYMin = -100; bp.fenceYMax = 100;
  Boids b(bp);
  Vec2 v = b.compute(self, t, 1000, nullptr, /*separationOnly*/ true);
  TEST_ASSERT_TRUE(v.x < -0.05f);  // pushed -x, away from neighbor
}

static void test_boids_cohesion_pulls_toward_far_neighbor() {
  NeighborTable t;
  t.update(mkState(2, 1, 3.0f, 0.0f, 0.0f, 0.0f), 1000, 1);  // 3 m away: cohesion zone
  VehicleState self;
  self.pose.p = {0, 0};
  self.pose.yaw = kPi / 2;  // facing +y so persistence doesn't mask +x pull
  BoidsParams bp;
  bp.fenceXMin = -100; bp.fenceXMax = 100; bp.fenceYMin = -100; bp.fenceYMax = 100;
  Boids b(bp);
  Vec2 v = b.compute(self, t, 1000);
  TEST_ASSERT_TRUE(v.x > 0.02f);
}

static void test_boids_geofence_repels() {
  NeighborTable t;  // empty
  VehicleState self;
  BoidsParams bp;   // default fence +-10, margin 1.5
  self.pose.p = {9.5f, 0.0f};  // 0.5 m from +x fence
  Boids b(bp);
  Vec2 v = b.compute(self, t, 0);
  TEST_ASSERT_TRUE(v.x < -0.05f);
}

// -------------------------------------------------------- leader-follower --
static void test_follow_tracking_coast_lost() {
  NeighborTable t;
  StatePacket lp = mkState(0, 1, 2.0f, 0.0f, 0.5f, 0.0f);  // leader at (2,0)
  t.update(lp, 1000, 5);
  VehicleState self;
  self.pose.p = {0.8f, 0.0f};  // exactly at slot (leader-1.2, 0)
  self.vel = {0.5f, 0.0f};     // matching leader speed -> pure feedforward
  LeaderFollowerParams fp;     // leaderId 0, offset (-1.2, 0)
  LeaderFollower f(fp);

  FollowOutput o = f.update(self, t, 1000);
  TEST_ASSERT_EQUAL_INT(int(FollowStatus::kTracking), int(o.status));
  // At-slot, matched velocity: command == leader velocity (feedforward).
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.5f, o.velocity.x);

  o = f.update(self, t, 1000 + 800);   // > coastMs (400), < lostMs (1200)
  TEST_ASSERT_EQUAL_INT(int(FollowStatus::kCoasting), int(o.status));
  o = f.update(self, t, 1000 + 1500);  // > lostMs
  TEST_ASSERT_EQUAL_INT(int(FollowStatus::kLost), int(o.status));
}

// -------------------------------------------------------------- vfh-lite ---
static SectorArray mkSectors(uint16_t all) {
  SectorArray s;
  for (int i = 0; i < kSectors; ++i) { s.distMm[i] = all; s.validZones[i] = 4; }
  return s;
}

static void test_vfh_free_passthrough_and_steer() {
  VfhLite v;
  SectorArray s = mkSectors(1900);
  VfhResult r = v.update(s, 0.0f);
  TEST_ASSERT_FALSE(r.steered);
  TEST_ASSERT_FALSE(r.allBlocked);

  // Block the center; sides free. With +-1 inflation, sectors 2..5 inflate
  // from blocked 3,4 -> nearest free are 1 (left) or 6 (right).
  s = mkSectors(1900);
  s.distMm[3] = 800; s.distMm[4] = 800;
  r = v.update(s, 0.0f);
  TEST_ASSERT_TRUE(r.steered);
  TEST_ASSERT_TRUE(fabsf(r.headingRad) > deg2rad(10.0f));
  TEST_ASSERT_TRUE(fabsf(r.headingRad) < deg2rad(32.0f));  // within FoV
}

static void test_vfh_hysteresis() {
  VfhLite v;
  SectorArray s = mkSectors(1900);
  s.distMm[4] = 1300;               // below block (1400) -> blocked
  v.update(s, 0.0f);
  s.distMm[4] = 1500;               // above block, below release (1600)
  VfhResult r = v.update(s, 0.0f);
  TEST_ASSERT_TRUE(r.blockedMask & (1 << 4));  // still blocked (hysteresis)
  s.distMm[4] = 1700;               // above release
  r = v.update(s, 0.0f);
  TEST_ASSERT_FALSE(r.blockedMask & (1 << 4));
}

static void test_vfh_all_blocked() {
  VfhLite v;
  SectorArray s = mkSectors(300);
  VfhResult r = v.update(s, 0.0f);
  TEST_ASSERT_TRUE(r.allBlocked);
}

// -------------------------------------------------------------- governor ---
static void test_governor_curve() {
  Governor g;
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f, g.fromDistance(2000).speedCap);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.5f, g.fromDistance(1400).speedCap);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.15f, g.fromDistance(700).speedCap);
  GovernorOutput o = g.fromDistance(399);
  TEST_ASSERT_EQUAL_INT(int(ReflexState::kStop), int(o.state));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, o.speedCap);
  o = g.fromDistance(100);
  TEST_ASSERT_EQUAL_INT(int(ReflexState::kFear), int(o.state));
  TEST_ASSERT_TRUE(o.backoff > 0.1f);
  o = g.fromDistance(SectorArray::kUnknownMm);
  TEST_ASSERT_EQUAL_INT(int(ReflexState::kBlind), int(o.state));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.15f, o.speedCap);
  // Monotonic between knots.
  TEST_ASSERT_TRUE(g.fromDistance(1000).speedCap > g.fromDistance(800).speedCap);
  TEST_ASSERT_TRUE(g.fromDistance(1800).speedCap > g.fromDistance(1500).speedCap);
}

// ------------------------------------------------------------------- bvc ---
static void test_bvc_blocks_head_on_and_allows_lateral() {
  NeighborTable t;
  t.update(mkState(2, 1, 1.0f, 0.0f, 0.0f, 0.0f), 1000, 1);  // 1 m ahead +x
  Bvc bvc;  // safety 0.5 -> maxAlong = 0.5*1.0-0.5 = 0 -> no approach allowed
  Vec2 v = bvc.constrainVelocity({1.0f, 0.0f}, {0, 0}, t, 1000);
  TEST_ASSERT_TRUE(v.x <= 1e-3f);          // approach removed
  v = bvc.constrainVelocity({0.0f, 1.0f}, {0, 0}, t, 1000);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.0f, v.y);  // lateral untouched
}

static void test_bvc_far_neighbor_no_constraint() {
  NeighborTable t;
  t.update(mkState(2, 1, 3.0f, 0.0f, 0.0f, 0.0f), 1000, 1);
  Bvc bvc;  // maxAlong = 1.0; step = 1.0*0.5 = 0.5 < 1.0 -> untouched
  Vec2 v = bvc.constrainVelocity({1.0f, 0.0f}, {0, 0}, t, 1000);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.0f, v.x);
}

// ---------------------------------------------------------------- mahony ---
static void test_mahony_converges_to_level() {
  Mahony m;
  // Static, level: accel = (0,0,1g), no rotation. Start from a tilted quat by
  // feeding a roll rate first.
  for (int i = 0; i < 100; ++i) m.update(0.5f, 0, 0, 0, 0, 1.0f, 0.005f);
  TEST_ASSERT_TRUE(fabsf(m.roll()) > 0.1f);  // got tilted
  for (int i = 0; i < 4000; ++i) m.update(0, 0, 0, 0, 0, 1.0f, 0.005f);
  TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.0f, m.roll());   // pulled back level
  TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.0f, m.pitch());
}

// ----------------------------------------------------------------- mixer ---
static void test_mixer_neutral_and_saturation() {
  MixerParams mp;
  MotorOutputs o = mixQuadX(0.5f, 0, 0, 0, mp);
  for (int i = 0; i < 4; ++i) TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.5f, o.m[i]);

  // Full throttle + roll demand: no output exceeds 1.0 and differential kept.
  o = mixQuadX(1.0f, 0.3f, 0, 0, mp);
  float hi = 0, lo = 1;
  for (int i = 0; i < 4; ++i) {
    TEST_ASSERT_TRUE(o.m[i] <= 1.0f + 1e-3f);
    TEST_ASSERT_TRUE(o.m[i] >= 0.0f - 1e-3f);
    if (o.m[i] > hi) hi = o.m[i];
    if (o.m[i] < lo) lo = o.m[i];
  }
  TEST_ASSERT_TRUE(hi - lo > 0.2f);  // differential survives saturation

  // Idle floor while armed with throttle.
  o = mixQuadX(0.06f, 0, 0, 0, mp);
  for (int i = 0; i < 4; ++i) TEST_ASSERT_TRUE(o.m[i] >= mp.idleThrottle - 1e-3f);
}

static void test_mixer_roll_sign_convention() {
  // roll > 0 -> right motors (m0, m1) decrease, left motors (m2, m3) increase.
  MotorOutputs o = mixQuadX(0.5f, 0.2f, 0, 0);
  TEST_ASSERT_TRUE(o.m[2] > o.m[0]);
  TEST_ASSERT_TRUE(o.m[3] > o.m[1]);
}

// ---------------------------------------------------------------- arming ---
static void test_arming_sequence_and_failsafe() {
  Arming a;
  uint32_t t = 0;
  // Idle sticks: stays disarmed.
  TEST_ASSERT_EQUAL_INT(int(ArmState::kDisarmed), int(a.update(0.0f, 0.0f, 10, 4.0f, t)));
  // Hold throttle low + yaw right for 1 s.
  for (t = 100; t <= 1300; t += 100) a.update(0.02f, 1.0f, 10, 4.0f, t);
  TEST_ASSERT_EQUAL_INT(int(ArmState::kArmed), int(a.state()));
  TEST_ASSERT_TRUE(a.motorsAllowed());
  // RC loss -> failsafe.
  a.update(0.5f, 0.0f, 500, 4.0f, t);
  TEST_ASSERT_EQUAL_INT(int(ArmState::kFailsafe), int(a.state()));
  TEST_ASSERT_FALSE(a.motorsAllowed());
  // Link back + throttle low -> disarmed.
  a.update(0.0f, 0.0f, 10, 4.0f, t + 100);
  TEST_ASSERT_EQUAL_INT(int(ArmState::kDisarmed), int(a.state()));
}

static void test_arming_estop_latch() {
  Arming a;
  uint32_t t = 0;
  for (t = 100; t <= 1300; t += 100) a.update(0.02f, 1.0f, 10, 4.0f, t);
  TEST_ASSERT_TRUE(a.motorsAllowed());
  a.latchEStop();
  TEST_ASSERT_FALSE(a.motorsAllowed());
  // Normal stick input does not release the latch.
  a.update(0.5f, 0.0f, 10, 4.0f, t += 100);
  TEST_ASSERT_EQUAL_INT(int(ArmState::kEStop), int(a.state()));
  // Disarm gesture (throttle low + yaw left held) releases to disarmed.
  for (int i = 0; i <= 12; ++i) a.update(0.02f, -1.0f, 10, 4.0f, t += 100);
  TEST_ASSERT_EQUAL_INT(int(ArmState::kDisarmed), int(a.state()));
}

// ------------------------------------------------------- behavior pipeline --
static void test_behavior_manual_reflex_brake() {
  BehaviorPipeline bp;
  BehaviorInputs in;
  in.mode = BehaviorMode::kManual;
  in.rc.pitch = 1.0f;      // full forward stick
  in.rc.throttle = 0.5f;
  in.sectors = mkSectors(300);  // obstacle at 30 cm -> STOP zone
  BehaviorOutput o = bp.update(in);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, o.setpoint.pitchAngle);  // braked
  // Roll/yaw authority preserved.
  in.rc.roll = 0.5f;
  o = bp.update(in);
  TEST_ASSERT_TRUE(o.setpoint.rollAngle > 0.1f);
  // Free space: forward passes through.
  in.sectors = mkSectors(1900);
  o = bp.update(in);
  TEST_ASSERT_TRUE(o.setpoint.pitchAngle > 0.3f);
}

static void test_behavior_hold_zero_velocity() {
  BehaviorPipeline bp;
  NeighborTable t;
  BehaviorInputs in;
  in.mode = BehaviorMode::kHold;
  in.table = &t;
  in.sectors = mkSectors(1900);
  in.self.vel = {0, 0};
  BehaviorOutput o = bp.update(in);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, o.commandedVel.norm());
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, o.setpoint.rollAngle);
}

static void test_behavior_fear_backs_away() {
  BehaviorPipeline bp;
  NeighborTable t;
  BehaviorInputs in;
  in.mode = BehaviorMode::kHold;
  in.table = &t;
  in.sectors = mkSectors(100);  // inside fear zone
  in.self.pose.yaw = 0.0f;
  BehaviorOutput o = bp.update(in);
  TEST_ASSERT_TRUE(o.commandedVel.x < -0.1f);  // backing up in world frame
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_state_packet_roundtrip);
  RUN_TEST(test_cmd_packet_roundtrip);
  RUN_TEST(test_sector_min_pool_middle_rows);
  RUN_TEST(test_sector_invalid_not_free_and_hold);
  RUN_TEST(test_sector_clamp_and_status_filter);
  RUN_TEST(test_neighbor_update_extrapolate_expire);
  RUN_TEST(test_neighbor_loss_estimate_and_eviction);
  RUN_TEST(test_gradient_hopcount);
  RUN_TEST(test_boids_separation_pushes_away);
  RUN_TEST(test_boids_cohesion_pulls_toward_far_neighbor);
  RUN_TEST(test_boids_geofence_repels);
  RUN_TEST(test_follow_tracking_coast_lost);
  RUN_TEST(test_vfh_free_passthrough_and_steer);
  RUN_TEST(test_vfh_hysteresis);
  RUN_TEST(test_vfh_all_blocked);
  RUN_TEST(test_governor_curve);
  RUN_TEST(test_bvc_blocks_head_on_and_allows_lateral);
  RUN_TEST(test_bvc_far_neighbor_no_constraint);
  RUN_TEST(test_mahony_converges_to_level);
  RUN_TEST(test_mixer_neutral_and_saturation);
  RUN_TEST(test_mixer_roll_sign_convention);
  RUN_TEST(test_arming_sequence_and_failsafe);
  RUN_TEST(test_arming_estop_latch);
  RUN_TEST(test_behavior_manual_reflex_brake);
  RUN_TEST(test_behavior_hold_zero_velocity);
  RUN_TEST(test_behavior_fear_backs_away);
  return UNITY_END();
}
