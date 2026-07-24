# Native unit tests

```bash
pio test -e native        # 26 tests, <1 s, no hardware needed
```

`test_swarmcore/test_main.cpp` covers `lib/swarmcore`: packet CRC round-trips,
sector reduction (middle-row min-pool, "invalid is not free" hold, clamp),
neighbor table (extrapolation, expiry, loss estimate, eviction), gradient,
boids force directions + geofence, leader–follower tracking/coast/lost,
VFH (steer, hysteresis, all-blocked), governor curve knots, BVC head-on vs
lateral, Mahony convergence, mixer neutrality/saturation/sign convention,
arming sequence + failsafe + E-stop latch, and behavior-pipeline end-to-end
cases (manual reflex brake, hold, fear back-away).

Guidelines:
- Anything that changes control/safety behavior in `swarmcore` gets a test
  here first — this suite is what lets weaker agents refactor safely.
- Tests must stay Arduino-free (the `native` env has no framework).
- Device-side code (`src/`) is validated by compiling all four envs
  (`pio run -e devkit_v1_tof -e devkit_v1_brushed -e xiao_s3_vision -e
  xiao_s3_vision_stub`) plus the bench procedures in `docs/HANDBOOK.md` §5.2.
