# 08 — V1 ToF Integration Deep-Dive: VL53L5CX on ESP32

**Scope:** Implementation-ready integration research for the **V1 perception sensor** selected in [02-obstacle-avoidance.md](02-obstacle-avoidance.md): the ST **VL53L5CX** 8×8 multizone ToF, forward-mounted on a PCB drone (ESP32 DOIT DevKit v1 today; same stack applies to the ESP32-S3 vision variant of [07-camera-depth-version-spec.md](07-camera-depth-version-spec.md)). Covers library choice, sensor configuration, 8-sector reduction, multi-sensor/swarm interference, electrical/mounting design, and concrete starting parameters for the VFH-lite steering layer, speed governor, and stop reflex. Written for the firmware agent implementing the driver *right now*.

**Status:** Research phase. All cited URLs were retrieved and verified during research (July 2026). Primary sources: ST datasheet DS13754, ULD manual UM2884 (both the current Rev 7 and older Rev 2 — **they differ**, see §3.4), AN5657, ST community answers from ST staff, the ETH Zürich Matrix-ToF-Drones flight-proven firmware (closest published precedent to our V1), and a 2026 independent characterization paper.

---

## 1. Executive summary

- **Library: SparkFun VL53L5CX Arduino Library v1.0.3** (PlatformIO id `sparkfun/SparkFun VL53L5CX Arduino Library`). It wraps ST's ULD, ships all three plugins, works on ESP32/ESP32-S3 with plain `Wire`, and exposes every setter we need (`setResolution`, `setRangingFrequency`, `setRangingMode`, `setTargetOrder`, `startRanging`, `isDataReady`, `getRangingData`). It bundles an old ULD core (1.1.1, 2021) — acceptable for V1; the fallback if we ever need ULD 2.0 features is wrapping ST's current ULD ourselves using the same `platform.cpp` shape (§2.4).
- **The ~86 KB sensor firmware lives in MCU flash** (`const … PROGMEM` array), not heap, and is pushed to the sensor over I²C on *every* power-up: measured 9.4 s at 100 kHz, **2.8 s at 400 kHz, 1.4 s at 1 MHz with 128-byte transfers**. On ESP32 always call `setWireMaxPacketSize(128)` (ESP32's Wire buffer is 128 B; the library defaults to 32 B chunks) and run the bus at 400 kHz minimum.
- **Configuration for V1** (flight-proven verbatim by ETH's Crazyflie deck): **8×8 @ 15 Hz, ranging mode CONTINUOUS, target order CLOSEST**, sharpener left at default, one target per zone. Three defaults will bite if unset: the sensor boots in **4×4 @ 1 Hz**, resolution must be set **before** frequency, and **no reconfiguration is possible while ranging** (stop → set → restart).
- **Sector reduction:** min-pool each of the 8 columns over the 4 middle rows (mask floor/ceiling rows like ETH), accepting only zones with `target_status ∈ {5, 9}` **and** `nb_target_detected == 1` — exactly the filter ETH flew with. ST's own confidence rating: status 5 = 100 %, status 6/9 = 50 %, everything else < 50 %. Invalid zones are "free" for steering but counted, and a sector with too few valid zones degrades the governor.
- **Trim the I²C frame.** With default output config a 8×8 frame is ~1.4 KB ≈ 35–40 ms of *blocking* Wire traffic at 400 kHz. Disabling unneeded outputs via `build_flags` (`-D VL53L5CX_DISABLE_AMBIENT_PER_SPAD …`) cuts it to ~300–500 B ≈ 8–13 ms. Keep `nb_target_detected` + `target_status` always (ST requirement).
- **Control starting points** (anchored to ETH's flown values + our doc-02 kinematics): stop reflex at **0.40 m**, back-away below **0.15 m**, governor bands 0.4/0.7/1.4 m, cruise **0.5 m/s** (hard cap 1.0 m/s indoor — ETH measured 100 % success at 0.5 m/s, 80 % at 1.0, 40 % at 1.5). VFH-lite: sector blocked < 1.4 m, free again > 1.6 m, 2-frame debounce, ±2-sector width inflation.
- **Multi-sensor & swarm:** 2–4 sensors share one bus via the LPn address-change dance (each sensor needs its own LPn GPIO; an 8-bit I²C GPIO expander like ETH's is the PCB-friendly way). Sensor-to-sensor IR interference is *mild by design* — ST states the histogram flags an interferer's return as an extra target with **invalid status**, which our status filter already drops; keep 1 target/zone so the valid target is promoted. No published drone-swarm-scale study exists; treat it as a bench-validation item.
- **Electrical:** both rails 3.3 V is legal (AVDD 2.8/3.3 V, IOVDD 1.8/2.8/3.3 V). Budget **~95 mA average, ~150 mA worst-case peak** while ranging; 100 nF + 4.7 µF per rail at the pins; IOVDD must come up with-or-after AVDD; thermal pad B4 to ground copper (module dissipates up to 430 mW at 3.3/3.3 V). No cover glass in V1 — a recessed bezel instead; sunlight is the hard limit (grey-target range collapses to ~0.65–0.8 m at 5 klux and worse in direct sun): V1 flies indoors.

---

## 2. Library selection

### 2.1 The candidates

| Library | PlatformIO id / source | Latest | ULD core | ESP32 story | Verdict |
|---|---|---|---|---|---|
| **SparkFun VL53L5CX Arduino Library** | `sparkfun/SparkFun VL53L5CX Arduino Library` | **1.0.3** (2022-11-14) | 1.1.1 (2021) | Explicitly supports ESP32 (hookup guide recommends ESP32 Thing Plus class); `setWireMaxPacketSize(128)` designed for ESP32's 128 B Wire buffer; highest popularity of the L5CX libs on the PlatformIO registry (rank ~438) | **Primary pick** |
| **STM32duino VL53L5CX** | `stm32duino/STM32duino VL53L5CX` | 1.2.3 (2023-08-23) | 1.x | 1.2.3 release is literally titled "Fix issue with ESP32 platforms" (adds a STOP between chunked writes); chunking hardcoded to `DEFAULT_I2C_BUFFER_LEN` | Works, but API is clunkier (begin/init split), BSD-3, STM32-first maintenance |
| **Adafruit VL53L5CX** | `adafruit/Adafruit VL53L5CX` (repo `adafruit/Adafruit_VL53L5`) | 1.0.1 (2026-04-09) | **1.3.9** (2023) | Brand new (created 2026-03-14, "with assistance from Claude Code"), BusIO transport, ships detection-thresholds + motion wrappers, `begin(addr, &Wire, i2c_clock)` | Promising but unproven (0 stars, no field history). Re-evaluate once it accumulates users/issues |
| **RJRP44 VL53L5CX** (ESP-IDF component) | `rjrp44/vl53l5cx` 4.0.1 on the ESP Component Registry | 4.0.x (2025) | **2.0.1** (current) | Developed on ESP32-S3, uses the new IDF I²C driver, examples run at 1 MHz with 2.2 kΩ pullups; requires main task stack ≥ 7168 B | **IDF-only** — not usable under Arduino/PlatformIO `framework = arduino`. The fallback path if we migrate to IDF-with-Arduino-component |
| ST ULD direct (STSW-IMG023) | st.com download, wrap `platform.cpp` yourself | ULD 2.0.x | 2.0.x | The stm32duino/SparkFun/Adafruit libs are all thin wrappers over this; porting = 5 I/O functions | Escape hatch, not a starting point |
| Simon Levy VL53L5 | github `simondlevy/VL53L5` | — | 1.x | The ancestor of SparkFun's port (credited in SparkFun's header) | Superseded |

**Choice: SparkFun 1.0.3.** Rationale: proven on ESP32 for four years, the exact API surface we need, one-line PlatformIO install, and — decisive for this project — it's the library the SparkFun Qwiic boards in our BOM are documented against. Its weaknesses are known and manageable: the bundled ULD core is 1.1.1 (missing the ULD 2.0 `vl53l5cx_set_VHV_repeat_count()` periodic temperature recalibration, §6.5) and upstream is quiet (last release 2022, but the sensor's register interface is frozen so there is little to maintain).

```ini
; platformio.ini
lib_deps =
    sparkfun/SparkFun VL53L5CX Arduino Library@^1.0.3
build_flags =
    ; trim the per-frame I2C payload (see §2.3); keep NB_TARGET_DETECTED and TARGET_STATUS!
    -D VL53L5CX_DISABLE_AMBIENT_PER_SPAD
    -D VL53L5CX_DISABLE_NB_SPADS_ENABLED
    -D VL53L5CX_DISABLE_SIGNAL_PER_SPAD
    -D VL53L5CX_DISABLE_MOTION_INDICATOR
    ; keep RANGE_SIGMA_MM + REFLECTANCE_PERCENT during bring-up; disable later for speed
```

API surface (verified from `SparkFun_VL53L5CX_Library.h`): `begin(addr=0x29, Wire)`, `setAddress`, `setResolution(16|64)`, `setRangingFrequency(hz)`, `setRangingMode(CONTINUOUS|AUTONOMOUS)`, `setIntegrationTime(ms)`, `setSharpenerPercent(0..99)`, `setTargetOrder(CLOSEST|STRONGEST)`, `setPowerMode`, `startRanging()`, `stopRanging()`, `isDataReady()`, `getRangingData(VL53L5CX_ResultsData*)`, `setWireMaxPacketSize(bytes)`, plus an error callback. All three ST plugins (xtalk calibration, detection thresholds, motion indicator) are included as sources.

### 2.2 The ~86 KB firmware upload — where it lives and what it costs

The VL53L5CX has **no NVM for its own firmware**: the host uploads it over I²C after every power-on ("the function copies the firmware (~84 kbytes) to the module" — UM2884 §4.1). In the SparkFun library the blob is declared (`src/vl53l5cx_buffers.h`, line 84) as

```c
const uint8_t VL53L5CX_FIRMWARE[] PROGMEM = {
```

i.e. **`const` + `PROGMEM` → it is linked into flash (.rodata)**, which on ESP32 is memory-mapped — zero RAM cost, ~86 KB of the 4 MB flash (the array is 0x15000 = 86,016 bytes, written to the sensor in three chunks of 0x8000/0x8000/0x5000). RAM cost at runtime is separate and small: `begin()` heap-allocates the `VL53L5CX_Configuration` struct (~3.3 KB: 1440 B temp buffer + 776 B xtalk + 488 B offset + misc) and your `VL53L5CX_ResultsData` is ~1.35 KB with default outputs ("1356 bytes of RAM" per the SparkFun example comment). Total library footprint: ~95 KB flash, ~5 KB RAM.

**Upload time** (SparkFun's own measurements in `Example2_FastStartup`, confirmed on the SparkFun forum):

| I²C clock | 32 B transfers (default) | 128 B transfers |
|---|---|---|
| 100 kHz | 9.4 s | — |
| 400 kHz | 2.8 s | 2.5 s |
| 1 MHz | 1.65 s | **1.4 s** |

Note this corrects doc 02, which paraphrased it as "~1.4 s @ 400 kHz": 400 kHz gives 2.5–2.8 s; 1.4 s requires 1 MHz + 128 B transfers. Design consequence: `begin()` blocks for seconds — run it during boot/pre-arm (with the status LED blinking), never mid-flight; a watchdog around the sensor task must allow for it.

### 2.3 ESP32 gotchas (the ones that will actually bite)

1. **32-byte default chunking.** The library chunks all I²C into `wireMaxPacketSize` transactions, default 32 B (`I2C_BUFFER_SIZE = 32` in `SparkFun_VL53L5CX_Library_Constants.h`). The ESP32 Arduino `Wire` buffer is **128 bytes** — call `myImager.setWireMaxPacketSize(128)` *before* `begin()` so the firmware upload benefits too (the setter only writes a member variable, so it is safe to call at any time). The parameter is a `uint8_t`, so 128 is effectively the max useful value.
2. **1 MHz is sensor-legal but ESP32-out-of-spec.** The sensor's I²C runs "400 kHz to 1 MHz" (DS13754 Table 1) and UM2884 says "capability of operating up to 1 MHz". Espressif's current I²C documentation for the classic ESP32 and S3 specs standard (100 kHz) and fast mode (400 kHz) only, while the legacy driver accepts clocks "no higher than 1 MHz for now". In practice 1 MHz works with strong pullups and short traces (SparkFun measured their 1.4 s number that way; RJRP44's S3 examples run 1 MHz with 2.2 kΩ pullups). **Recommendation: design the PCB for 1 MHz (2.2 kΩ pullups, short traces), bring up at 400 kHz, then try 1 MHz; keep 400 kHz as the qualified fallback.**
3. **The per-frame read is big and blocking.** With the default output configuration (everything enabled) an 8×8 frame transfers **~1.4 KB → ~35–40 ms of blocking `Wire` time at 400 kHz** (~16 ms at 1 MHz). Arduino `Wire` on ESP32 has no async API, so this *must* live in the core-0 sensor task (per doc 02 §6 / doc 04 architecture), never in the 100 Hz control task. Better: trim outputs with the `VL53L5CX_DISABLE_*` build flags (UM2884 §5.2). Keeping only distance + target status + nb-targets(+ sigma + reflectance during tuning) cuts the frame to ~300–500 B ≈ **8–13 ms at 400 kHz, ~3–5 ms at 1 MHz**. ST: "always keep 'number of targets detected' and 'target status' enabled." These are compile-time flags used to size structs — set them in `build_flags` so library and application sources agree.
4. **Don't reconfigure while ranging.** "The get/set functions cannot be used during a ranging session, and 'on-the-fly' programming is not supported" (UM2884 §3.3). A runtime 8×8→4×4 profile switch = `stopRanging() → setResolution → setRangingFrequency → startRanging()`.
5. **Surprising factory defaults**: boots in **4×4** at **1 Hz**, autonomous mode, target order *strongest*. If you forget `setRangingFrequency(15)` you'll get a 1 Hz sensor and a very slow drone. Set resolution **before** frequency (the valid frequency range depends on resolution).
6. **Task stack:** the ULD keeps its big buffers in the heap-allocated `Dev` struct, but give the sensor task ≥ 8 KB stack anyway — the IDF port documents ≥ 7168 B required for init.
7. **ESP32-S3:** nothing sensor-specific changes — same `Wire` API, same 128 B buffer; the actively-maintained ESP-IDF port is developed on S3. (The problematic Arduino platform historically was ESP8266, which needed PROGMEM workarounds — not us.)
8. **Address semantics trap:** ST's C ULD uses the **8-bit** address (0x52); the SparkFun wrapper uses the **7-bit** form everywhere (`begin()` default = 0x52>>1 = **0x29**, and `setAddress(newAddr)` takes 7-bit too). Mixing the two conventions is the classic multi-sensor bug (see §5.1).

### 2.4 Fallback path

If SparkFun 1.0.3 hits a wall (e.g. we need ULD 2.0's periodic VHV temperature recalibration, §6.5): ST's current ULD (STSW-IMG023, now 2.0.x — the IDF port tracks 2.0.1) is ~10 files + a 5-function platform layer; the stm32duino `platform.cpp` (with its ESP32 STOP-between-chunks fix) is a working template for the Wire implementation. This keeps our exact driver API and swaps the core underneath.

---

## 3. Sensor configuration for a forward-mounted drone sensor

### 3.1 Resolution × rate: 8×8 @ 15 Hz vs 4×4 @ 60 Hz

Allowed: 4×4 at 1–60 Hz, 8×8 at 1–15 Hz (UM2884 Table 2). What the datasheet says the modes actually *range* (continuous mode, inner zones, typical):

| Target | 4×4 @ 30 Hz | 8×8 @ 15 Hz |
|---|---|---|
| White 88 %, dark | 4000 mm | 3500 mm |
| White 88 %, 5 klux | 1700 mm | 1100 mm |
| Grey 17 %, dark | 2400 mm | 1300 mm |
| Grey 17 %, 5 klux | 1000 mm | 800 mm |

4×4 concentrates 4× the SPADs per zone → it sees **farther** and tolerates ambient light better (ST staff confirm "4x4 works better than 8x8 in high sunlight"), and at 60 Hz the frame age drops from ≤ 67 ms to ≤ 17 ms. What 8×8 buys is **angular resolution: 5.6°/zone vs 11.25°/zone** (the FoV is 45°×45°; the 63–65° figure in doc 02 is the *diagonal*). With 4×4 our polar interface collapses to 4 forward sectors — too coarse to steer around a chair leg or door frame; with 8×8 we get the full 8 columns → 8 sectors that doc 02's VFH-lite layer was designed for.

**Recommendation for 0.5–1 m/s: 8×8 @ 15 Hz.** Justification: (a) it is the exact configuration the ETH Matrix-ToF-Drones deck flew to 100 % reliability at 0.5 m/s indoors (TRO 2023); (b) our doc-02 latency math shows 15 Hz supports ~1 m/s with the stop margins in §7; (c) grey-target range 1.3 m ≥ our 1.4 m react distance only marginally, but the governor treats "no return" conservatively. Keep **4×4 @ 60 Hz as a defined profile** (one stop/reconfig/start away) for two future cases: flight > 1.5 m/s, and high-ambient environments. V1 firmware should implement the profile switch but not use it in flight.

### 3.2 Ranging mode: CONTINUOUS (not the default autonomous)

UM2884 §4.5: continuous keeps the VCSEL on for the whole frame → "maximum ranging distance and ambient immunity are better… advised for fast ranging measurements or high performances"; autonomous pulses the VCSEL for `integration_time` per frame to save power. The datasheet quantifies the cost of autonomous at the default 5 ms integration, 8×8 @ 15 Hz, grey target, inner zone: **800 mm (dark) / 500 mm (5 klux) vs 1300/800 mm continuous — a ~40 % range loss.** The power saved (order 100 mW) is irrelevant next to drone motors (ETH measured the whole ToF deck at ~3 % of their 10 W platform, and *carrying* the deck's 2.5 g cost twice as much as operating it). ETH's flight code sets `VL53L5CX_RANGING_MODE_CONTINUOUS`. **V1: CONTINUOUS.**

### 3.3 Integration time: not applicable (continuous), 5 ms if autonomous

`vl53l5cx_set_integration_time_ms()` **only affects autonomous mode** — "changing integration time if ranging mode is set to continuous has no effect" (UM2884 §4.6). If the 4×4@60 Hz profile ever runs autonomous for power reasons: default 5 ms; constraint = sum of integrations (×4 for 8×8) + 1 ms < frame period; first frame may arrive early (UM2884 Rev 6 note). V1 sets nothing here.

### 3.4 Sharpener: leave at default — and note the default is 14 %, not 5 %

The sharpener removes veiling-glare bleed between adjacent zones; 0 % = off, 99 % = max (UM2884 §4.8). **Documentation discrepancy we verified:** UM2884 Rev 2 (the PDF most tutorials mirror, e.g. Pololu's) says "the default is 5 %"; the current UM2884 **Rev 7 (26-Aug-2025) explicitly corrects this: "Modified default sharpener value from 5% to 14%"**, matching the datasheet's FoV-measurement note ("14 % sharpener (default value)"). So a fresh sensor runs 14 %. Higher sharpener sharpens zone boundaries but can *split* a close obstacle into fewer zones and creates status-12 pixels ("target blurred by another one, due to sharpener"). For obstacle avoidance the datasheet-characterized default is the right starting point; revisit only with bench evidence (e.g. a thin pole smearing across 3+ columns → try 30–50 %).

### 3.5 Target order: CLOSEST

Default is *strongest*, which reports a bright far wall in preference to a dark near obstacle — exactly wrong for avoidance. **Set `SF_VL53L5CX_TARGET_ORDER::CLOSEST`** (UM2884 §4.9). ETH's flight code does. Keep `VL53L5CX_NB_TARGET_PER_ZONE = 1` (platform.h default in both SparkFun and Adafruit): one target/zone means the sensor itself promotes the best valid target, which is also ST's stated mitigation for sensor-to-sensor interference (§5.3).

### 3.6 Which zones to trust

- **Corner/edge vignetting is real and characterized:** at 8×8 continuous, grey target, dark: inner zones 1300 mm typ / 900 min, corner zones 1100 typ / **600 min** (DS13754 Table 18) — corners range 15–35 % shorter. Treat edge-column "free" readings beyond ~1 m with suspicion; the min-pool over 4 rows already averages some of this out.
- **Row masking (floor/ceiling):** a level-mounted 45° FoV at 0.5 m altitude sees the floor from ~1.2 m ahead; pitched −10° in forward flight, from ~0.8 m — inside our react distance, so unmasked bottom rows cause phantom braking. ETH's flown solution: process only middle rows, treating row ≥ 6 as ground-zone and row < 2 as ceiling-zone (`GROUND_BORDER 6`, `CELLING_BORDER 2`), with separate too-close checks (`DIS_GROUND_MIN 400`, `DIS_CEILING_MIN 600` mm). **V1: min-pool rows 2–5 into the sector array; keep rows 0–1/6–7 for optional ceiling/floor proximity flags.** (§6.6 for the mechanical-tilt alternative.)
- **Warm-up drift:** an independent 2026 characterization (three sensors, robot-arm ground truth) found a startup transient (~15 min to full thermal equilibrium, near-linear range bias, stable once the module is warm) plus an offset bias ≤ 3 cm, σ ≈ 1 % of range on white targets and up to ~6 % on black. At our 0.4–1.4 m thresholds a ≤ 3 cm bias is absorbed by margins — no compensation needed for V1, but log `ResultsData.silicon_temp_degc` per frame so we can see it. (ULD 2.0's `vl53l5cx_set_VHV_repeat_count()` periodic re-calibration is the proper fix if we ever chase centimeters; not exposed by SparkFun 1.0.3.)

---

## 4. Sector reduction: 8×8 grid → 8-sector polar array

### 4.1 Validity filtering — which target_status codes to accept

UM2884 Table 4 (verified current in Rev 7), with ST's confidence guidance: *"a target with status 5 is considered as 100 % valid. A status of 6 or 9 can be considered with a confidence value of 50 %. All other statuses are below 50 %."*

| status | meaning | accept? |
|---|---|---|
| 5 | Range valid | **yes** |
| 9 | Range valid with large pulse (possibly merged target) | **yes** |
| 6 | Wraparound not performed (typically the first range) | optional (transient at stream start; harmless to accept for *presence*) |
| 10 | Range valid but no target at previous range | no (flicker-prone) |
| 12 | Target blurred by another, due to sharpener | no |
| 0–4, 7, 8, 11, 13 | not updated / SNR / sigma / consistency failures | no |
| 255 | No target detected | no — but see §4.3: *absence of return ≠ absence of obstacle* |

**The flight-proven rule** (ETH `ToF_process.c`, verbatim): a pixel is valid iff `(status == 5 || status == 9) && nb_target_detected == 1`. Adopt it unchanged. This answers the spec question: yes, **{5, 9}** is the tested-in-the-wild set (plus the nb-targets guard); 6 is defensible to accept during the first frames after `startRanging()`.

### 4.2 Reduction algorithm (column min-pool with validity)

With the sensor mounted so grid columns ≙ azimuth (verify on the bench — the ST zone map is transposed relative to what the SparkFun library returns, and breakout orientation adds a rotation; wave a hand at the left edge and check which columns respond):

```
sector[c]      = min over rows 2..5 of distance_mm[zone(r,c)]  where pixel valid
valid_zones[c] = count of valid pixels in rows 2..5             (0..4)
if valid_zones[c] == 0 → sector[c] = SECTOR_FREE (4000)         // "no return"
```

Each sector spans 5.6° of the 45° horizontal FoV. Cost: one pass over 64 zones — microseconds (ETH's full pipeline including clustering: 210 µs on a 168 MHz M4).

- **Reflectance/sigma filters:** optional, not load-bearing. The status codes already encode sigma-too-high (3) and low signal (1, 8), which is why ETH shipped without extra filters. During bring-up, keep `range_sigma_mm` and `reflectance_percent` enabled and log them: sigma spikes near sector flicker tell you whether to add a `sigma ≤ 15 mm` gate (datasheet accuracy: ±15 mm below 200 mm, ±5–11 % above); chronically low reflectance across many zones indicates a dirty aperture (§6.4). In the flight build, disable both outputs for I²C speed.
- **Outlier debounce:** ETH's paper additionally ignores single-pixel obstacles (min cluster size 2 neighbors) to kill speckle; our min-pool equivalent is the 2-frame temporal debounce in §7.3. Both are cheap; start with temporal only, add the 2-of-4-rows spatial vote if bench flicker demands it.

### 4.3 The "invalid = free?" policy (important safety nuance)

ETH binarizes invalid pixels as no-obstacle, which flew fine at 0.5 m/s — but the 2026 characterization shows *why care is needed*: on **black vinyl** targets only 19–34 % of measurements at 25–60 cm were flagged reliable (the obstacle is there; the sensor mostly refuses to answer), and ETH's own paper documents a chromed chair support that was invisible except below ~0.5 m. Policy for V1:

- Steering (VFH-lite): invalid zones don't block sectors (else the drone freezes in open space — beyond ~2.6 m validity naturally drops below ~50 %).
- Governor: **confidence-gate it.** If the *forward* sectors (3, 4) have `valid_zones == 0` for > 0.5 s while commanded speed > v_min, cap speed at 0.3 m/s. Rationale: persistent no-return dead-ahead is either open space (fine, we're barely slower) or an absorptive/specular obstacle (we just saved the drone).
- The full-FoV validity statistics from ETH (> 95 % valid ≤ 2 m indoors, < 50 % at 2.6 m) justify **clamping the useful range to 2.0 m** for control purposes (their `MAX_DISTANCE_TO_PROCESS = 2000` mm).

---

## 5. Multi-sensor and swarm interference

### 5.1 2–4 sensors on one ESP32 I²C bus (the LPn procedure)

All VL53L5CX power up at address 0x52 (8-bit). Per-sensor addresses are assigned at every boot using **LPn** (a.k.a. the I²C-enable pin) — UM2884 §2.3:

1. Power everything up; hold **all LPn low except sensor A**.
2. `begin(0x29)` → `setAddress(0x2A)` (SparkFun API = 7-bit! 0x2A/0x2B/0x2C… keep them spaced and clear of the PCA9554 at 0x20 if using the expander).
3. Raise the next LPn, repeat with a new address; finally raise all LPn.

Practical notes from a documented multi-L5CX build: allow ~100 ms settle after each address change before touching the next sensor (StartRanging on the second sensor returns 255 otherwise), and the 7-bit/8-bit confusion is the #1 cause of "two sensors answer on one address". Each sensor still needs its ~86 KB firmware upload → 4 sensors ≈ 6–10 s boot at 400 kHz; upload sequentially.

**Pin budget:** one GPIO per LPn (+ optionally per INT). On our pin-starved PCB, copy ETH's deck: a **PCA9554-class 8-bit I²C GPIO expander** (theirs sits at 0x20) drives LPn/I2C_RST per sensor and senses INT — 2–4 sensors cost zero extra MCU GPIOs. An **I²C mux (TCA9548A)** is the alternative (no address changes needed, isolates a hung sensor) at the cost of a mux hop on every transaction and another part; with only 2–4 devices the LPn dance is simpler and is what ST documents.

### 5.2 Same-robot sensor-to-sensor interference

ST staff, on the record: multiple L5CX at different angles for a wider FoV showed "minimal" effect; the correct mosaic is tilting adjacent sensors by ~one FoV (22.5–45°) — "the physical separation you get by tilting them is the only thing you need to do (it's also the only thing you can do)" — there is no modulation/channel coding to configure. All sensors share the same 940 nm band.

### 5.3 Robot-to-robot (drone-to-drone) interference — what's known

**No published study of VL53L5CX-vs-VL53L5CX interference at swarm scale was found** (July 2026) — flag this honestly as a bench-validation item. What ST states (community answers by ST ToF staff):

- Cross-illumination mostly appears as **ambient light** to the other sensor and is discounted; heavy mutual illumination degrades range like any ambient (§6.3).
- Where an interferer's pulse *does* land in the histogram, "the sensor will return an **extra target**, but this false target will have an **invalid status** … if you ask for [one] target per zone, the valid target will be promoted and returned. Please check the target status before accepting it." The interferer's histogram "footprint" is recognized as invalid.
- Direct face-to-face at close range is the worst case ("unless you place them looking directly at each other at close distances it should be OK").

Consequences for the swarm: our §4.1 status filter + 1 target/zone is **already the ST-recommended mitigation**; expect transient invalid pixels (not silent wrong distances) when drones face each other — the 2-frame debounce and the invalid-zone governor rule absorb that. Defense in depth comes from the doc-02 architecture: inter-agent avoidance runs on radio-shared positions (BVC), not on the ToF, and per-agent altitude offsets reduce face-to-face geometry. **Bench test to schedule:** two sensors head-on at 0.5–3 m, log status-histogram vs distance; repeat with 15 Hz streams free-running (unsynchronized frame phases drift past each other, so any collision is intermittent by construction).

---

## 6. Electrical & mounting on a PCB drone

### 6.1 Supplies and current

- Rails: **AVDD 2.8 or 3.3 V; IOVDD 1.8, 2.8 or 3.3 V** (DS13754). Single 3.3 V for both is the simple PCB answer.
- Current (DS13754 Table 12, "active ranging", resolution-independent): AVDD **45 mA typ / 50 mA max**, IOVDD **50 mA typ / 80 mA max**; **peak = average + 10 mA on each rail** → budget ~95 mA average, **~150 mA worst-case peak** per sensor on the 3.3 V rail. Power ≈ 216 mW typical continuous-mode (AN5657), up to 430 mW max at 3.3/3.3 V.
- Decoupling: "capacitors on the external supplies (AVDD and IOVDD) should be placed as close as possible to the module pins" (DS13754 §1.4 note); the reference schematic pairs a **4.7 µF bulk + 100 nF ceramic per rail**. On a shared drone 3.3 V rail (nRF24 bursts + servo/ESC noise), give the sensor its own local 4.7 µF and consider a ferrite bead from the main rail.
- **Power-up sequencing:** IOVDD must rise **with or after** AVDD, fall with or before it; "avoid powering IOVDD while AVDD is unpowered to prevent increased leakage" (DS13754 §2.3). Same 3.3 V net for both trivially satisfies this.
- Reset: full sensor reset = AVDD+IOVDD+LPn low ≥ 10 ms, then high (UM2884 §4.2) — if you can't cut the rail, at least route **LPn and I2C_RST to GPIOs/expander** (I2C_RST toggling resets just the I²C machine; the recovery ladder is: I2C_RST → LPn+re-init → power-cycle). Wire **INT (pin A3)** to a GPIO: it fires per frame and auto-clears in ~100 µs; interrupt-driven readout beats polling by up to one poll interval of latency.
- Thermal (AN5657): θ_module = 40 °C/W, junction ≤ 100 °C (shutdown 110 °C) — solder the **thermal pad B4 to ground copper** and keep total system thermal resistance ≤ ~70 °C/W; on a small PCB flying at 3.3/3.3 V this is the difference between a warm and a throttling sensor. Expect a warm-up range-bias transient until thermal equilibrium (§3.6).
- I²C pullups: 2.2 kΩ to 3.3 V for 1 MHz operation (IDF-port-proven); Espressif's general guidance is 1–10 kΩ, 2–5 kΩ preferred at higher speeds. The DS reference circuit shows a 47 kΩ pullup on INT (open drain).

### 6.2 Cover glass, prop wash and dust

- **V1 recommendation: no cover glass.** Every datasheet characterization is "without coverglass"; glass adds crosstalk that must be calibrated (xtalk plugin, 600 mm white-target procedure, UM2884 §3.2) even though the histogram gives inherent xtalk immunity beyond 60 cm. Instead mount the module behind a **recessed bezel/shroud** for crash and prop-wash protection, with the opening sized to the **collector exclusion zone (55.5° × 61°, 82° diagonal — wider than the 45°×45° FoV!)** so the bezel itself doesn't vignette (DS13754 §1.2).
- If a window becomes necessary (outdoor dust): ST's cover-glass rules — > 90 % transmission at 940 nm, ~1 mm thick, parallel, **air gap < 0.5 mm or an opaque-at-940 nm gasket separating Tx/Rx paths**, no AR coating tricks, and run xtalk calibration with the window fitted (calibration data can be captured once and reused across identical units, with `xtalk margin` headroom).
- **Dust/dirt on the aperture** behaves like growing crosstalk + signal loss: watch two firmware-visible signals — rising invalid-pixel fraction and falling `reflectance` on known targets. A pre-arm self-check ("with props off, ≥ 90 % of zones valid against the floor at known height") catches a dirtied or obstructed aperture before flight.

### 6.3 Sunlight / outdoor limits

Quantified by the datasheet at 5 klux (≈ overcast daylight; direct sun is 50–100 klux): 8×8 continuous grey-target range drops from 1300 mm (dark) to **800 mm typ / 600 min**, corners to 650/400. ST staff add: ambient beyond roughly **1.2 Mcps/SPAD → zones return status 255** ("couldn't range due to excessive ambient"); the sensor works best with the sun ~90° off boresight; 4×4 outperforms 8×8 in high ambient; and a 500 W halogen lamp near the FoV (incandescent = strong 940 nm) produced ~45 % measurement rejection in the 2026 characterization. **V1 envelope: indoors / heavy overcast.** Outdoor sun ⇒ the doc-02 fallbacks (lower speed cap, mmWave complement) — do not chase it with this sensor alone.

### 6.4 Vibration and motion

The module has no moving parts, and the strongest evidence it tolerates quadrotor vibration is that the ETH deck flew it on a 35 g Crazyflie (brushed motors ≈ worst-case vibration per gram) with 100 % avoidance reliability at 0.5 m/s and the sensor characterized "in nano-UAV flying conditions … precise and reliable". Two motion-related realities to design for instead: (a) at 15 Hz and 1 m/s a frame is up to 67 ms stale → ~7 cm of unmodeled ego-motion, folded into the stop margins (§7); (b) hard braking pitches the airframe and swings the FoV up/down — the row-masking of §3.6 (or IMU-indexed row masks later) prevents floor-flash panic during exactly the maneuver the reflex commands.

### 6.5 Temperature

Range offset drifts ~0.05 mm/°C typical (DS13754 §5.4) — negligible at our thresholds; the warm-up transient (§3.6) is the visible effect. If we later want it gone: ULD 2.0's `vl53l5cx_set_VHV_repeat_count(N)` re-runs the internal temperature calibration every N frames (UM2884 Rev 7 §4.14; costs a few ms; not exposed in SparkFun 1.0.3).

### 6.6 Mounting geometry for pitched-forward flight

Level-mounted at h = 0.5 m altitude the lower FoV edge (−22.5°) meets the floor `0.5/tan(22.5°) ≈ 1.2 m` ahead; add a −10° forward-flight pitch and it's `0.5/tan(32.5°) ≈ 0.8 m` — inside the 1.4 m react distance. Options, in order of preference for V1:

1. **Mount flat + software row-mask** (rows 0–1/6–7 excluded from steering; the ETH pattern) — zero mechanical complexity, and the masked bottom rows double as a "floor too close" check.
2. **Fixed upward tilt ≈ +10°** (≈ the cruise pitch magnitude at 0.5–1 m/s) so the FoV is level *in flight* — buys back the two bottom rows as obstacle rows; costs a wedge on the PCB mount and slightly degraded hover-time floor clearance below the FoV.
3. IMU-driven dynamic row masking — later, only if flight logs show phantom ground braking during aggressive maneuvers.

---

## 7. Control-side starting parameters

All anchored to the only published flight-tested VL53L5CX avoidance stack (ETH TRO 2023: thresholds from their shipped source) and cross-checked against doc-02 kinematics with our latency budget (frame ≤ 67 ms + readout ~10 ms + control tick ≤ 10 ms + actuation ≈ 20 ms ⇒ **t_lat ≈ 110 ms worst-case**).

### 7.1 Stop reflex (lowest layer, overrides everything, runs every control tick)

| Parameter | Value | Why |
|---|---|---|
| `D_STOP` | **0.40 m** | ETH `DIS_STOP = 400 mm`. Kinematic check at 1.0 m/s: latency travel 0.11 m + braking v²/2a = 1.0/(2·2.0) = 0.25 m ⇒ 0.36 m < 0.40 ✓ (at 0.5 m/s: 0.12 m, huge margin) |
| `D_FEAR` (back-away) | **0.15 m**, command −0.2 m/s | ETH `DIS_FEAR = 150 mm`, `VEL_FEAR = −0.2` |
| Trigger condition | any *valid* forward-sector (sectors 2–5) distance < `D_STOP` for **1 frame** | one 15 Hz frame = 67 ms; debouncing the *stop* reflex would spend 40 % of the margin |
| Brake command | zero forward setpoint (hover); rely on attitude controller | ETH commanded up to −20 m/s² braking and measured stop 0.2–0.5 m short of walls from 1.1–1.9 m/s peaks; they also showed aggressive braking corrupts optical-flow velocity estimates near walls — command the stop, don't stunt it |

### 7.2 Speed governor (`v_max = f(d_min_ahead)`)

Piecewise bands for V1 (cheap, testable — direct scaling of ETH's flown `DIS_REACT/DIS_SLOW` = 1400/700 mm bands to our 0.5–1 m/s envelope):

| forward min distance d | commanded cap |
|---|---|
| d < 0.40 m | 0 (reflex) |
| 0.40–0.70 m | **0.3 m/s** |
| 0.70–1.40 m | **0.7 × v_cruise** |
| > 1.40 m or no return | **v_cruise** |
| forward sectors all-invalid > 0.5 s | 0.3 m/s (see §4.3) |
| sensor frame older than 150 ms | 0 — treat sensor as failed (doc-02 staleness rule) |

with **v_cruise = 0.5 m/s default, 1.0 m/s hard cap indoors**. Empirical basis: ETH office-environment reliability was **100 % at 0.5 m/s, 80 % at 1.0, 40 % at 1.5, 10 % at 2.0** over 20 flights each — 0.5 cruise / 1.0 cap is the defensible V1 envelope, matching doc 02 §6. The continuous-form upgrade (drop-in later): `v_cmd ≤ sqrt(2·a_brake·max(0, d_min − D_STOP − v·t_lat))` with a_brake = 1.5–2 m/s² (ETH's forward accel limit was 1.5 m/s²), which reproduces the bands within ~10 %.

### 7.3 VFH-lite steering bins

Published MCU VFH implementations tune two thresholds with hysteresis on the polar histogram (VFH+, Ulrich & Borenstein 1998 — the τ_low/τ_high binary histogram prevents valley flapping); our metric-distance variant:

| Parameter | Start value | Why |
|---|---|---|
| Sectors | 8 × 5.6° over 45° | sensor columns 1:1 (§4.2); doc 02's "8 forward sectors" (its "over 63°" was the diagonal — horizontal span is 45°) |
| Block threshold `τ_block` | sector min < **1.4 m** | ETH `DIS_REACT = 1400 mm`; also ≈ the grey-target 8×8 typical range (1.3 m) — beyond it the sensor can't vouch for the sector anyway |
| Free threshold `τ_free` | sector min > **1.6 m** | hysteresis gap 0.2 m ≈ 3× sensor σ at 1.4 m (5 % ⇒ 7 cm) — VFH+ two-threshold pattern |
| State debounce | **2 consecutive frames** (133 ms) to flip blocked↔free | kills single-frame speckle; the *reflex* (§7.1) stays 1-frame |
| Width inflation | blocked sectors dilate by **±2 sectors** | drone half-width + margin ≈ 0.25 m at 1.4 m ⇒ atan(0.25/1.4) ≈ 10° ≈ 2 sectors (VFH+ robot-width compensation) |
| Min admissible valley | **3 free sectors** (≈ 17° ⇒ 0.41 m opening at 1.4 m) | must physically fit the inflated drone |
| Valley selection hysteresis | keep previous heading while its valley stays free; else valley nearest goal heading | VFH+ anti-dither rule |
| Escape | all sectors blocked > 1 s ⇒ yaw-in-place scan (±45°), then doc-02 bug/wall-follow mode | 45° FoV is narrow; yawing *is* the cheap wider scan |

---

## 8. Driver checklist (for the firmware agent)

### 8.1 platformio.ini

```ini
lib_deps =
    sparkfun/SparkFun VL53L5CX Arduino Library@^1.0.3
build_flags =
    -D VL53L5CX_DISABLE_AMBIENT_PER_SPAD
    -D VL53L5CX_DISABLE_NB_SPADS_ENABLED
    -D VL53L5CX_DISABLE_SIGNAL_PER_SPAD
    -D VL53L5CX_DISABLE_MOTION_INDICATOR
    ; flight build (after bring-up/tuning) may add:
    ; -D VL53L5CX_DISABLE_RANGE_SIGMA_MM
    ; -D VL53L5CX_DISABLE_REFLECTANCE_PERCENT
```

Never disable `NB_TARGET_DETECTED` or `TARGET_STATUS` (ST requirement + our filter needs them).

### 8.2 Exact init sequence (order matters)

```cpp
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

SparkFun_VL53L5CX tof;
VL53L5CX_ResultsData frame;          // ~1 KB with trimmed outputs

void tof_init() {                    // core-0 sensor task, stack >= 8 KB
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);             // qualify 1 MHz on the bench, then bump
  tof.setWireMaxPacketSize(128);     // ESP32 Wire buffer is 128 B (default lib chunk = 32 B)

  // (multi-sensor only: LPn dance + setAddress(0x2A..) here, 7-bit addresses, 100 ms settle)

  uint32_t t0 = millis();
  if (!tof.begin()) fail();          // uploads 86 KB firmware: expect 2.5-2.8 s @400 kHz
  log("fw upload %lu ms", millis() - t0);

  if (!tof.setResolution(8 * 8)) fail();                            // BEFORE frequency!
  if (!tof.setRangingFrequency(15)) fail();                         // boot default is 1 Hz
  if (!tof.setRangingMode(SF_VL53L5CX_RANGING_MODE::CONTINUOUS)) fail();  // default is autonomous
  if (!tof.setTargetOrder(SF_VL53L5CX_TARGET_ORDER::CLOSEST)) fail();     // default is strongest
  // sharpener: leave at factory default (14%)
  // integration time: no effect in continuous mode - don't set

  if (!tof.startRanging()) fail();
  // NOTE: any reconfig from here on requires stopRanging() first (no on-the-fly changes)
}
```

Pre-arm gate: init complete + first frame received + ≥ 90 % zones valid against a known scene; expose a "sensor ready" flag to the control task.

### 8.3 Per-frame processing (15 Hz, core-0 sensor task)

```cpp
// Preferred: INT pin (A3) -> GPIO falling-edge -> semaphore. Fallback: poll isDataReady() every 5 ms.
if (tof.isDataReady() && tof.getRangingData(&frame)) {       // ~8-13 ms blocking @400 kHz (trimmed)
  uint32_t t_frame = millis();
  uint16_t sector[8];  uint8_t valid[8];
  for (int c = 0; c < 8; c++) { sector[c] = 4000; valid[c] = 0; }

  for (int r = ROW_MIN; r <= ROW_MAX; r++) {                 // rows 2..5 (floor/ceiling mask, §3.6)
    for (int c = 0; c < 8; c++) {
      int z = zone_index(r, c);        // fix row/col orientation ON THE BENCH (transposed vs datasheet!)
      if (frame.nb_target_detected[z] == 1 &&
          (frame.target_status[z] == 5 || frame.target_status[z] == 9)) {
        valid[c]++;
        if (frame.distance_mm[z] < sector[c]) sector[c] = frame.distance_mm[z];
      }
    }
  }
  // optional floor/ceiling proximity from masked rows (ETH: floor < 400 mm, ceiling < 600 mm -> flag)

  publish_sectors(sector, valid, t_frame, frame.silicon_temp_degc);   // lock-free double buffer
}
// Control task (core 1, 100 Hz): stop reflex (§7.1) -> governor (§7.2) -> VFH-lite (§7.3),
// treating data older than 150 ms as sensor-failed.
```

### 8.4 Bring-up measurements to log (once, on real hardware)

1. Firmware upload time at 400 kHz vs 1 MHz, 32 B vs 128 B (validate table §2.2 on our PCB).
2. `getRangingData` wall time with default vs trimmed outputs (validate §2.3 numbers).
3. Zone-map orientation (hand-sweep test) → hardcode `zone_index()`.
4. Validity-vs-distance curve on a matte dark target (expect ETH's shape: > 95 % ≤ 2 m).
5. Two-sensor face-to-face status histogram, 0.5–3 m (§5.3 open question).
6. 20-minute warm-up drift log of a fixed target + `silicon_temp_degc` (§3.6).

---

## 9. Recommended defaults table

| Parameter | Value | Why (source) |
|---|---|---|
| Library | SparkFun VL53L5CX Arduino Library **1.0.3** | ESP32-proven ULD wrapper, full API, our BOM's board (§2.1) |
| I²C clock | **400 kHz** (qualify 1 MHz on bench) | sensor: 400 k–1 MHz (DS); ESP32 officially Fm=400 kHz, 1 MHz works in practice (§2.3) |
| `setWireMaxPacketSize` | **128** | ESP32 Wire buffer 128 B; default 32 B chunks slow everything (SparkFun hookup guide) |
| I²C pullups | **2.2 kΩ** to 3.3 V | 1 MHz-capable per IDF-port wiring + Espressif 2–5 kΩ guidance |
| Resolution | **8×8** (set *before* frequency) | 8 columns = 8 sectors @ 5.6°; ETH flight config (§3.1) |
| Ranging frequency | **15 Hz** | max at 8×8; boot default is 1 Hz — must set (UM2884) |
| Ranging mode | **CONTINUOUS** | +40 % grey-target range vs autonomous\@5 ms; ambient immunity; ETH flight config (§3.2) |
| Integration time | untouched | no effect in continuous mode (UM2884 §4.6) |
| Sharpener | untouched (**default = 14 %**, not 5 % — UM2884 Rev 7 correction) | datasheet characterization baseline (§3.4) |
| Target order | **CLOSEST** | avoidance wants nearest, not brightest; ETH flight config (§3.5) |
| Targets per zone | **1** (`VL53L5CX_NB_TARGET_PER_ZONE`, platform default) | valid target auto-promoted; ST's anti-interference guidance (§5.3) |
| Outputs enabled | distance, target_status, nb_target_detected (+ sigma & reflectance during bring-up) | ST minimum + our filter; frame 1.4 KB → ~0.3–0.5 KB (§2.3) |
| Data readout | INT pin preferred, else 5 ms polling; core-0 task, stack ≥ 8 KB | blocking Wire reads; INT auto-clears ~100 µs (UM2884 §5.3) |
| Validity filter | `status ∈ {5, 9} && nb_targets == 1` (status 6 tolerated at stream start) | ST confidence table + ETH flown filter (§4.1) |
| Rows used for sectors | **2–5** (mask 0–1 ceiling, 6–7 floor) | ETH `GROUND_BORDER/CELLING_BORDER`; pitched-flight floor intrusion (§3.6, §6.6) |
| Useful range clamp | **2.0 m** | validity > 95 % below 2 m indoors, collapses beyond (ETH characterization) |
| Sector reduction | column min-pool + per-column valid count | §4.2 |
| Mounting | flat, recessed bezel ≥ exclusion zone (55.5°×61°), no cover glass, thermal pad to GND pour | DS FoV/exclusion + AN5657 (§6.2, §6.1) |
| Supply | 3.3 V both rails, 100 nF + 4.7 µF each at pins, IOVDD ≥ AVDD ordering, ~150 mA peak budget | DS13754 (§6.1) |
| LPn / I2C_RST / INT | route to GPIOs or PCA9554-class expander | reset ladder + multi-sensor + interrupt readout (§5.1, §6.1) |
| `D_STOP` / `D_FEAR` | **0.40 m / 0.15 m** (back-away −0.2 m/s) | ETH flown values + kinematic check at 1 m/s (§7.1) |
| Governor bands | 0.4 / 0.7 / 1.4 m → 0 / 0.3 / 0.7·cruise / cruise | ETH `DIS_SLOW/DIS_REACT` (§7.2) |
| Cruise / cap | **0.5 m/s / 1.0 m/s** indoor | ETH reliability: 100 %@0.5, 80 %@1.0, 40 %@1.5 (§7.2) |
| VFH-lite | block < 1.4 m, free > 1.6 m, 2-frame debounce, ±2-sector inflation, min valley 3 | VFH+ hysteresis pattern + sensor σ (§7.3) |
| Staleness | frame > 150 ms old ⇒ sensor failed ⇒ stop | doc-02 rule; ~2 frame periods |
| Multi-sensor addresses | 7-bit 0x2A+ via LPn dance, 100 ms settle between sensors | UM2884 §2.3 + field reports (§5.1) |
| Outdoor use | not in V1 (indoor/overcast only) | 5 klux halves-to-quarters range; >~1.2 Mcps/SPAD ambient ⇒ status 255 (§6.3) |

---

## 10. Corrections/deltas to earlier docs

- Doc 02 §2.2: "boot takes ~1.4 s at 400 kHz+" → **1.4 s is 1 MHz + 128 B; 400 kHz is 2.5–2.8 s** (§2.2).
- Doc 02 §3.2/§6: "8 forward sectors over 63°" and "I²C readout ~2–5 ms" → horizontal FoV is **45°** (63–65° is diagonal), and the *default-config* readout is ~35–40 ms at 400 kHz until outputs are trimmed (§2.3).
- Doc 02 §7.2's VFH layer gets its first concrete parameter set here (§7.3); doc 07's teacher-logging record can reuse the §8.3 filter for its `tof_status` field.

## 11. References

**ST official**
- VL53L5CX datasheet DS13754 (SparkFun mirror, Rev 2) — https://cdn.sparkfun.com/assets/6/e/3/0/6/vl53l5cx-datasheet.pdf
- VL53L5CX datasheet (current, st.com) — https://www.st.com/resource/en/datasheet/vl53l5cx.pdf
- UM2884 ULD user manual (current Rev 7 — sharpener default 14 %, VHV periodic temp compensation) — https://www.st.com/resource/en/user_manual/um2884-a-guide-to-using-the-vl53l5cx-multizone-timeofflight-ranging-sensor-with-a-wide-field-of-view-ultra-lite-driver-uld-stmicroelectronics.pdf
- UM2884 Rev 2 (Pololu mirror; the widely-linked older revision) — https://www.pololu.com/file/0J1885/um2884-a-guide-to-using-the-vl53l5cx-multizone-timeofflight-ranging-sensor-with-wide-field-of-view-ultra-lite-driver-uld-stmicroelectronics.pdf
- AN5657 PCB thermal guidelines — https://www.st.com/resource/en/application_note/an5657-vl53l5cx-module-integration-guide--hardware-integration-and-customer-training-stmicroelectronics.pdf
- STSW-IMG023 ULD product page — https://www.st.com/en/embedded-software/stsw-img023.html
- ST BSP component (ULD version history, 1.3.9 @ 2023) — https://github.com/STMicroelectronics/stm32-vl53l5cx ; release notes — https://github.com/STMicroelectronics/stm32-vl53l5cx/blob/main/Release_Notes.html

**ST community (ST ToF staff answers)**
- Mutual influence of multiple sensors (invalid-status false target, footprint detection) — https://stcommunity.st.com/t5/imaging-sensors/vl53l5cx-mutual-influence-of-multiple-sensors/td-p/197217
- Multiple ToF in proximity ("seen as ambient light … discounted") — https://stcommunity.st.com/t5/imaging-sensors/will-multiple-tof-sensors-vl53l-series-specifically-in-close/td-p/61089
- FoV extension by 22.5° tilt; LPn per sensor — https://community.st.com/t5/imaging-sensors/interference-between-tof-sensors-vl53l5cx/td-p/76049
- Multi-sensor LPn address-change debugging (Jetson) — https://community.st.com/imaging-sensors-49/how-to-use-multiple-vl53l5cx-sensors-129984
- Indirect sunlight / ~1.2 Mcps/SPAD ambient limit / 4×4-better-in-sun — https://community.st.com/t5/imaging-sensors/how-to-use-the-vl53l5cx-with-indirect-sunlight/td-p/116356
- Sunlight range (ST provides 0 and 5 klux references only) — https://stcommunity.st.com/t5/imaging-sensors/vl53l5cx-sunlight-range/td-p/602218
- Cover glass behind window (crosstalk options) — https://community.st.com/t5/imaging-sensors/vl53l5cx-how-can-we-place-a-vl53l5cx-behind-a-glass-or-plastic/td-p/100994
- Cover glass basics (clear/close/parallel/thin/clean) — https://community.st.com/mems-and-sensors-62/time-of-flight-cover-glass-4
- Gasket required for air gap > 0.5 mm — https://community.st.com/t5/imaging-sensors/vl53l5cx-gasket-requirements-with-cover-window-and-large-air-gap/td-p/142460

**Libraries**
- SparkFun VL53L5CX Arduino Library (v1.0.3; API, IO layer, `PROGMEM` firmware, FastStartup timings) — https://github.com/sparkfun/SparkFun_VL53L5CX_Arduino_Library ; PlatformIO registry — https://registry.platformio.org/libraries/sparkfun/SparkFun%20VL53L5CX%20Arduino%20Library
- SparkFun hookup guide (ESP32 recommendation, 128 B buffer note) — https://learn.sparkfun.com/tutorials/qwiic-tof-imager---vl53l5cx-hookup-guide/all
- SparkFun forum: firmware upload at every power-on, 9.4 s→1.4 s — https://community.sparkfun.com/t/vl53l5cx-firmware/63818
- stm32duino VL53L5CX (1.2.3 "Fix issue with ESP32 platforms") — https://github.com/stm32duino/VL53L5CX/releases
- Adafruit VL53L5CX (new 2026; ULD 1.3.9) — https://github.com/adafruit/Adafruit_VL53L5 ; https://www.arduinolibraries.info/libraries/adafruit-vl53-l5-cx
- RJRP44 ESP-IDF component (ULD 2.0.1, S3, ≥ 7168 B stack, 2.2 kΩ pullups) — https://github.com/RJRP44/VL53L5CX-Library/releases ; https://components.espressif.com/components/rjrp44/vl53l5cx/versions/4.0.0/readme
- Multi-sensor Arduino field notes (LPn dance, 100 ms settle, 7-bit trap) — https://davespace.xyz/blog/interfacing-multiple-vl53l5cx-sensors

**ESP32 platform**
- ESP-IDF I²C docs, classic ESP32 (legacy driver "no higher than 1 MHz"; new docs Sm/Fm 100/400 kHz; 2–5 kΩ pullups) — https://docs.espressif.com/projects/esp-idf/en/v5.1/esp32/api-reference/peripherals/i2c.html ; https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html

**Flight-proven precedent & characterization**
- Müller, Niculescu, Polonelli, Magno, Benini, "Robust and Efficient Depth-Based Obstacle Avoidance for Autonomous Miniaturized UAVs", IEEE T-RO 2023 — https://doi.org/10.1109/tro.2023.3315710 ; open version — https://ar5iv.labs.arxiv.org/html/2208.12624
- ETH-PBL Matrix_ToF_Drones (deck hardware + flown firmware: config calls, {5,9} filter, 1400/700/400/150 mm thresholds, row masks, PCA9554 LPn expander) — https://github.com/ETH-PBL/Matrix_ToF_Drones
- Bitcraze blog on the deck (≥ 2 m measurements incomplete; validity-based filtering) — https://www.bitcraze.io/tag/obstacle-avoidance/
- "On the Characterisation of the Time-of-Flight VL53L5CX Sensor … for Indoor Robotics", Sensors 26(5):1639, 2026 (warm-up transient, ≤ 3 cm bias, black-vinyl validity collapse, halogen 45 % rejection) — https://www.mdpi.com/1424-8220/26/5/1639
- Niculescu et al., "Towards a Multi-Pixel Time-of-Flight Indoor Navigation System for Nano-Drone Applications", I2MTC 2022 — https://doi.org/10.1109/i2mtc48687.2022.9806701 (cited via the above)
- BatDeck (ultrasonic complement for glass/reflective surfaces) — https://arxiv.org/html/2412.10048v1

**Control algorithms**
- Ulrich & Borenstein, "VFH+: Reliable Obstacle Avoidance for Fast Mobile Robots", ICRA 1998 (τ_low/τ_high hysteresis, width compensation) — http://www.cs.cmu.edu/~iwan/papers/vfh+.pdf ; https://doi.org/10.1109/robot.1998.677362
