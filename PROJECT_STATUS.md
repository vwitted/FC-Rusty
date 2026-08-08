# FC-Rusty — Project Status

A Rust flight controller targeting the **DAKEFPV H743** (STM32H743VIT6,
Cortex-M7F @ 480 MHz). The north-star is **stable, high-authority
attitude control via MPC**; everything else — estimation, sensors,
arming, comms — exists to feed or protect that loop.

Status is a snapshot, not a log. Update this document whenever a
material hardware or design change lands (see `CLAUDE.md`).

---

## Current Hardware — DAKEFPV H743

### Peripherals

| Peripheral             | Role                              | Status                                        |
|------------------------|-----------------------------------|-----------------------------------------------|
| USART6 TX (PC6)        | defmt logger (115200, T6 pad)     | ✅ verified                                    |
| UART5 RX (PB5)         | CRSF RC receiver (R5, 416666)     | ✅ verified — 6 channels parsing               |
| USART1 (PA9/PA10)      | GPS (NMEA, 9600, T1/R1)           | ✅ verified — 3D fix, home latches             |
| SPI1 + PA4             | ICM-42688P IMU1 (onboard)         | ✅ verified — 8 kHz reads, MEKF fusing         |
| SPI4 + PB1             | ICM-42688P IMU2 (onboard)         | ✅ verified — 8 kHz reads, averaged with IMU1  |
| I2C2 (PB10/PB11)       | SPL06 barometer (onboard)         | ✅ proper driver — 128 Hz, correct calibration              |
| PA0/PA1/PA2/PA3 (GPIO) | DShot300 bit-banged, motors M1–M4 | ✅ bidir verified 2026-08-08 — decoded eRPM on M1/M2/M3; M4 returns no reply (off-chip fault, open). TIM1 is a pacer only; DMA2_CH2 drives BSRR / samples IDR |
| PD10                   | Status LED (active low)           | ✅ heartbeat blink task                        |
| USB-C                  | DFU flashing                      | ✅ verified — no SWD on this board             |
| UART4 (PD1/PD0)        | DisplayPort / VTX (T4/R4)         | ⚪ not wired                                   |
| USART3                 | ESC telemetry (T3/R3)             | ⚪ not wired                                   |
| UART7 / UART8          | General purpose (T7/R7, T8/R8)   | ⚪ available                                   |
| SPI2 (PB13/14/15) + PB12 | AT7456E OSD (MAX7456-compatible) | ⚪ no driver yet                               |

### Why this board

- **Radiolink F722** (STM32F722RET6): prior board. DShot was unresolvable
  via Embassy's `waveform_up` / `waveform_up_multi_channel` APIs — scope
  traces showed malformed waveforms from DMAR contention. Fix: write to
  DMAR directly. Opportunity taken to move to a more capable board.
- **DAKEFPV H743**: STM32H743VIT6 at 480 MHz, 1 MB SRAM, dual onboard
  ICM-42688P (√2 noise reduction), all four motor outputs on a single
  TIM2 (no multi-timer synchronisation problem), USB-C DFU flashing.
  The DShot fix (direct DMAR burst write) carries straight over and
  works cleanly on the H743.

---

## What's Verified on Hardware

(The specifics here are largely validated on older hardware, but the overall functionality has now been implemented and enhanced for the STM32H743).

- **Attitude**: Dual ICM-42688P sensors read at 8 kHz via SPI. MEKF fusing accel (100 Hz update)
  and gyro (8 kHz predict). Gyro bias bounded to 0.3–0.5 dps; innovation
  gate rejects ~0% at rest and ~25% under aggressive motion. Sensor
  frame mapped to NED via `BODY_SIGN=[+1,-1,-1]`.
- **Position**: 6-state linear PosKF (pn, pe, pz, vn, ve, vz) running
  at 100 Hz predict. Baro and GPS fuse as independent measurement
  paths — pilot decides what to fly with via the soft arm gate
  (`baro_ready || gps_home_latched`).
  - **GPS position**: home latches on the first fix with
    `FIX3D && sats ≥ 5 && HDOP < 3.5` (relaxed for Alpha; see backlog).
    Subsequent good fixes fuse as local NED. σ_gps_h = 2 m,
    σ_gps_v = 5 m.
  - **GPS velocity** (NMEA RMC ground speed × course) fuses
    independently of home latching at σ = 0.3 m/s, via the new
    `PosKf::update_gps_velocity(vn, ve)` method. This caps DR
    position drift from O(t²) to O(t) and means PosHold without GPS
    home is best-effort but useful for tens of seconds rather than
    diverging immediately. Below 0.3 m/s ground speed the path
    fuses (0, 0) since RMC course is undefined at low speeds —
    actively damps drift while stationary.
  - **Baro** is *not* fused before arm. On the Disarmed→Armed event
    the `ARM_LATCH` signal makes pos_kf_task latch `p_ref = current
    pressure` and zero the KF's vertical state, so altitude reads
    ~0 at arm. From arm forward baro fuses every sample at σ_baro =
    0.3 m, which dominates GPS altitude over the short term.
  - **Readiness flags**: `altitude_ready` (= `baro_calibrated ||
    home_latched`, gates AltHold and PosHold) and `home_latched`
    (gates GpsRescue / GpsHome / RTH).
  - Outdoor verification 2026-04-20 (under the prior GPS-anchored
    design): clean ready transition, baro 26 reads/s with 0 errors,
    post-home-latch IMU-only drift (~1500 m) corrected in one GPS
    tick. Re-verification of the new pilot-discretion design plus
    velocity fusion is on the post-Alpha checklist.

- **Failsafe descent**: RC loss never auto-disarms. The control loop
  picks one of three failsafe modes based on what's still alive:
  - `GpsRescue` (existing): home_latched → climb to safe alt + RTH
    + auto-land at home.
  - `FailsafeLand` (new): altitude_ready, no home → closed-loop
    descent at 0.7 m/s, level attitude, auto-disarm when
    `altitude_up < 0.3 m`.
  - `FailsafeBlind` (new): no altitude, no home → open-loop throttle
    at 90 % of hover, level attitude. **No auto-disarm** — without
    altitude data there's no safe stop criterion. Descent runs until
    pilot regains RC or battery cuts. Impact-signature disarm is a
    Beta backlog item.

  IMU loss is the only mid-flight auto-disarm path: there's no safe
  recovery from losing attitude.
- **Comms**: CRSF RC (6 channels), NMEA GPS, defmt over USART3.
- **Control loop**: Asynchronous dual-loop architecture communicating via lock-free `Watch` channel:
  - **Outer Loop (100 Hz)**: `navigation_task` handles Attitude MPC, altitude hold, position hold, and RC processing.
  - **Inner Loop (8 kHz)**: `control_loop` executes the rate PID and DShot output, fully synchronized to the MEKF gyro predicts without arbitrary timers.

---

## What's Next

1. ~~**Flash + outdoor verify `a7feea0`.**~~ Flashed 2026-04-21.
   GPS-latch thresholds relaxed to 5 sats / HDOP < 3.5 for Alpha
   testing. Push once verified.
   Tighten GPS thresholds post Alpha for more reliable prearm (7 sats / HDOP < 2.5)
   **~~Motor bring-up on F722~~**
   ~~blocked, under investigation~~
   Resolved: Record kept for reference as of around 25-04-26:
   Driver or peripheral config is implicated, not the ESCs. Three
   of the four ESCs are proven healthy on TIM2's signal; a fourth
   is unproven. Arm attempts at 29 % thrust did not produce clean
   spin on any motor. Full session observations and hypotheses in
   `docs/motor-bringup-log.md`. Captured
   oscilloscope waveforms and eventually traced malformed bitstream to various embassy issues populating DMAR. We implemented this functionality directly to avoid the issue (src/drivers/dshot_hw.rs) so the ESCs now receive the correct bitstream and the motors spin up on arm.
  
2. **Close the loop on hardware.** Rate-PID-only hover first, then
   enable the MPC outer loop and tune for transfer from sim.

   *(Status Verification: This point remains **TRUE**. While the logic and loop decoupling have been heavily validated in the simulation environments (`sim_gps_rescue`, `sim_hover`), we are still awaiting the delivery of the final target hardware to begin the physical motor bring-up and physical PID tuning.)*

## ****Alpha Complete 03-05-2026****

### post-Alpha tweaks

> **Branch note (`dakefpv-h743-post-alpha`, 2026-06-05):** this branch
> ports the post-Alpha work that was developed on `geprc-taker-h743`
> back onto the DAKEFPV H743 (the GEPRC TAKER board stopped responding
> on the bench). Integrated here so far, all building clean with
> 125/125 host tests: the `RawImu` public-orientation refactor, the
> 2nd-order Butterworth IMU LPF, LIS2MDL → MEKF yaw fusion, the
> per-fuse GPS log, the baro-only arming deadlock fix, and the PosKF
> pre-arm fusion + HDOP-only home latch + arm-reorigin rewrite. Plus
> the BF-style bidir DShot driver retargeted to TIM2 (see below).

- [ ] GPS thresholds tightened to 7 sats / HDOP < 2.0
- [x] Re-enable arming on baro only, but if GPS fix is available set home co-ords. Implemented 2026-05-08: arming gate is now soft (`baro_ready || gps_home_latched`); baro p_ref latches on the Disarmed→Armed transition via `ARM_LATCH` signal; `PosEstimate` exposes split `altitude_ready` / `home_latched` flags. `arming::ArmingStateMachine.require_gps` renamed to `require_altitude_ref`.
- [x] GPS home latch + pre-arm fusion overhaul (ported 2026-06-05, orig. 2026-05-15): HDOP-only per-fix gate (`sats >= 4 && hdop < 2.5`, 3-streak to latch), per-fix HDOP-scaled `update_gps_scaled`, always-fusing baro with a provisional `p_ref` seeded from the first sample, and arm-time frame re-origin that zeroes the KF and re-anchors p_ref + home.
- [x] Assign CRSF channels for user-initiated GPS Rescue, pos-hold and alt-hold functionality. (Implemented: CH5 for mode, CH6 for RTH trigger)
- [~] **ESC Bidirectional DShot.** Driver implemented: BF-style
      per-channel CC DMA on TIM2, bidir polarity + input-capture RX +
      GCR/CRC/eRPM decode (`dshot_hw.rs` + `dshot_frame.rs` +
      `dshot_telemetry.rs`). Includes the CCR-reset + EGR.UG fix for
      the residual-capture glitch on the first post-RX TX cell.
      **Unverified on hardware** — needs a working board to confirm
      the ESC decodes our frames and that `DShot RX` logs non-zero
      eRPM. `uf-dshot` dependency fully removed.
- [x] **Motor-test bench firmware** (`--features motor-test`, 2026-06-21):
      a fully decoupled DShot bench driver for verifying the bidir work
      above — no arming, RC, PID, or flight stack, just the DShot driver.
      Per-motor throttle, bidir, and loop-freq are set at build time
      (`M1_PCT`..`M4_PCT`, `BIDIR`, `LOOP_KHZ` 2–8 kHz), clamped to 25%,
      with a 5 s props-off countdown. e.g.
      `M1_PCT=6 cargo build --release --features motor-test`. Config parse
      is host-tested; the run loop is bench-verified. Spec + plan in
      `docs/superpowers/{specs,plans}/2026-06-21-dshot-motor-test*`. This is
      the clean path to test motor spin-up, superseding the throttle hacks
      noted below.
      **Bench session 2026-07-25 — motors spin at commanded throttle.**
      Three fixes out of that session: (1) `run()` now streams zero-throttle
      MotorStop frames for 3 s after the countdown so ESCs arm (they lock
      out on a nonzero first frame — this was why nothing spun); (2) unset
      `Mx_PCT` now defaults to 5% instead of 0% so a bare motor-test flash
      actually spins motors (explicit `Mx_PCT=0` still stops; **props-off is
      now load-bearing on every motor-test flash**); (3) recognised env vars
      with unparseable values (`BIDIR=false`, `M1_PCT=ten`) are a *compile
      error* via `const` asserts — unset still defaults, so stray env junk
      stays harmless. Misspelt var *names* remain undetectable; the startup
      banner printing the resolved config is the safety net for those.
      `scripts/flash-motor-test.sh` added (DFU flash of the motor-test
      build; env vars must be passed to the script since it rebuilds).
      Open bench observation: rare intermittent single-motor spin while
      receiving MotorStop — possible signal-integrity/decode issue, parked.
- [x] **SPL06 baro fixed on bench** (2026-07-25): two stacked bugs.
      (1) The calibration block was read from 0x18 on a wrong "SPL06
      differs from DPS310" comment; it is 0x10..=0x21, same as DPS310
      (Betaflight/iNav agree) — wrong registers parsed as coefficients
      gave P≈647 kPa / T≈-168 °C. Now reads 0x10; bench shows sane cal,
      P≈99.6 kPa at 1 atm. (2) Debug trap: with `DEFMT_LOG=trace`,
      embassy's per-byte I2C trace logging inflates the 18-byte cal
      read past the deliberate 5 ms bus timeout (`BARO_TIMEOUT_MS`) —
      init fails every attempt at ~3 bytes. Trace-level logging and the
      baro bus timeout are incompatible; use `info` for baro work.
      `Spl06Error::I2c` now carries the underlying embassy error kind
      (Timeout vs Nack) so this is diagnosable from the log next time.
- [~] **Persist flash config store** (`src/persist/`, 2026-06-22):
      first non-volatile storage in the repo — a versioned, CRC-checked
      32-byte record in the last 128 KB flash sector (bank 2, `0x081E0000`,
      reserved in `memory.x`; `FLASH` shrunk to 1920K). Pure `record`
      module (host-tested: CRC, encode/decode, version/magic/CRC/blank
      rejection) + firmware `flash` wrapper (`read`/`write`, disarmed-only,
      erase-then-write one record). Boot reads it into an uncalibrated
      default; an uncalibrated board behaves exactly as before. This is
      **sub-project A** — the foundation for the magnetometer-calibration /
      yaw fix (sub-project B). Spec + plan in
      `docs/superpowers/{specs,plans}/2026-06-22-persist-flash-config*`.
      Code committed; **bench round-trip not yet verified** (a `[~]`, not a
      `[x]`): build with `--features persist-selftest`, flash, and confirm
      the marker survives a power cycle on the USART6 console.
- [ ] revert throttle changes implemented for bench motor testing (posssibly this is done by adding a stick scaling factor in the mixer)

### Items for Beta build

- **Impact-signature disarm.** Currently `FailsafeBlind` (RC lost,
  no altitude reference) has no auto-disarm — the descent runs
  until pilot recovery or battery cut. With ICM accel data we can
  detect a hard-landing impulse (peak accel magnitude ≫ 1 g for
  short duration) and disarm on that signature. Bonus feature:
  generalise to a non-failsafe "crash detected → cut motors" gate
  for any flight mode, which would save props on a botched landing.
  - Design: rolling-window peak detection on body-frame accel
    magnitude, threshold ≈ 4–5 g for ≥10 ms. Disable while throttle
    is high (avoid thrust-axis noise during aggressive manoeuvres).
  - **Schedule: with ICM telemetry features in Beta.**

- **Accel bias estimation in PosKF.** The 6-state filter predicts
  kinematics from raw body specific force with no accel-bias state.
  Outdoor testing showed ~0.4 m/s/s drift between GPS fixes. Benign
  while GPS σ=2 m dominates, but will degrade GPS-rescue accuracy and
  any loiter/precision features.
  - Design: extend to 9-state (pn pe pz vn ve vz bax bay baz), subtract
    estimated bias from predict, random-walk process noise with τ ≈
    hundreds of seconds.
  - **Schedule: before any precision autonomous feature.**

- **Bidirectional DShot + eRPM into the MPC.** Measured eRPM from the
  ESCs closes the loop on real motor behaviour, with several payoff layers:
  1. **RPM-based notch filter** on the gyro signal — removes motor/prop
     resonances that currently pass through to the attitude estimate.
  2. **Adaptive thrust mapping** — runtime DShot→RPM→thrust lookup
     linearises the actuator response and directly improves MPC accuracy.
     The MPC's B matrix currently uses a fixed model; substituting
     measured data makes it adaptive.
  3. **Motor failure detection** — diverging RPM from commanded value
     detected in real time; MPC's constraint handling makes mixer
     reconfiguration theoretically possible.
  USART3 (ESC telemetry pad) is free; needs wiring and a bidirectional
  DShot + telemetry-decode driver.

- **Magnetometer calibration + true-north yaw.** (mag was already fused
  in the MEKF since 2026-05-13; the `ImuData.mag` slot is log-only.)
  Sub-project B, 2026-06-23, `[~]` bench-pending:
  - `control/mag_cal.rs` — `MagCalibrator`: online least-squares **sphere
    fit** for the hard-iron offset + **bin-coverage** completion (24 bins =
    8 azimuth × 3 elevation, not a fixed timer). Host-tested.
  - MEKF (`attitude_mekf.rs`): `set_hard_iron` (subtract offset before
    fusion — the offset was leaking into yaw), `anchor_heading` (tilt-
    compensated magnetic heading + declination → true-north reference),
    `update_yaw_reference` (GPS course-over-ground as a scalar yaw update).
  - Orchestration (`main.rs`): **AUX4** (channel index 7), disarmed-only
    spin-cal; result persisted via sub-project A; COG fused only when
    `groundspeed > 2 m/s` AND forward-stick (a quad's COG = heading only in
    forward flight). Declination is a compile-time `const DECLINATION_DEG`
    (default 0.3°). Soft-iron deliberately out of scope (hard-iron only).
  - Spec + plan in `docs/superpowers/{specs,plans}/2026-06-22-mag-cal-yaw-fix*`.
  - **LED feedback** (`control/cal_led.rs`, 2026-06-23): the onboard LED
    (PD10) signals the cal lifecycle for no-laptop field use —
    accelerating blink (duty → near-solid) while calibrating, **blackout**
    held at coverage-complete until you hold level, triple-burst on anchor,
    and a held 5 s/5 s slow-flash on a degenerate fit until you revert
    AUX4. Pure pattern fn host-tested; `mekf_task` publishes the phase, the
    reworked `blink_task` renders it. Spec + plan in
    `docs/superpowers/{specs,plans}/2026-06-23-cal-led-feedback*`. CRSF
    FC→TX telemetry (the richer status channel) is banked under Post-Alpha
    Directions.
  - **Not yet bench-verified:** spin-cal coverage/anchor, COG convergence
    (and the forward-stick sign), uncalibrated regression, LED patterns.

- ~~**SPL06 barometer driver.**~~ Done. `drivers/baro.rs` now has a
  proper `Spl06` struct reading calibration from the correct register
  start (0x18, not the DPS310's 0x10). Running at 128 Hz / 1× OSR;
  baro task ticks at 125 Hz. DPS310 code retained for reference.

- **Position hold.** `control/position.rs` is written and unit-tested but
  not yet wired into the control loop. Gates on reliable baro + GPS
  fusion and sufficient PID tuning in flight.

- **ISM6HG256X breakout (Beta).** STMicro 6-axis IMU (LGA-14) — driver
  `drivers/ism6hg256x.rs` is written, compiles, and is intentionally
  unreferenced from `main.rs` until the breakout is built and wired.
  Configured for ±16 g / ±4000 dps / 7.68 kHz ODR / high-perf mode,
  push-pull pulsed DRDY on INT1 — same control-loop shape as the
  ICM-42688P so it can substitute or supplement. ±4000 dps (vs the
  ICM's ±2000 max) chosen because the gyro is already noise-floor-
  limited at ±2000, so the wider FS is strictly more saturation
  headroom for crash/recovery scenarios. Fancy features (high-g 256 g
  channel, FSM, MLC, SFLP, OIS, EIS, sensor hub, FIFO) deliberately
  left out.
  - **Bring-up gotchas** (worth re-reading before flashing):
    - Output registers are **little-endian** (L then H) — opposite of
      the ICM-42688P's big-endian. Don't blindly cross-reference
      decoding logic between the two drivers.
    - `WHO_AM_I` returns `0x73` (vs `0x47` on the ICM); the init path
      will hard-fail with `WhoAmIMismatch` if the wrong driver is
      pointed at the wrong chip.
    - `Orientation` parameter must match how the breakout is soldered
      into the airframe — `Identity` is the safe default for bench work
      but won't be right in flight.
    - Temperature uses the standard ST formula `raw/256 + 25` (1 LSB =
      1/256 °C). Diagnostic only, not on the control path.
    - Chip max ODR is 7.68 kHz (not 8 kHz like the ICM); decimation
      math in any task that fuses both must account for that.
    - Gyro LSB is 140 mdps (not 70 mdps as at ±2000 dps); any code
      cross-checking against ICM samples needs to scale, not compare
      raw counts.

- **LIS2MDL magnetometer breakout (Beta).** STMicro 3-axis mag (LGA-12)
  — driver `drivers/lis2mdl.rs` is written, compiles, and is
  intentionally unreferenced from `main.rs` until the breakout
  arrives. I2C-only path (fixed addr `0x1E`); intended to share the
  same I2C peripheral as the SPL06 baro. Configured for 100 Hz
  continuous mode in high-resolution power mode with COMP_TEMP_EN,
  OFF_CANC, BDU and the digital LPF (BW = 25 Hz) all on. WHO_AM_I =
  `0x40`; output registers are **little-endian**. Sensitivity is
  1.5 mgauss/LSB (`SENS_UT_PER_LSB = 0.15`) — diagnostic temperature
  is the internal die sensor at 8 LSB/°C with no documented zero
  offset. No interrupt pin wired; `data_ready()` polls
  `STATUS_REG.Zyxda`. Hard-iron calibration / world-frame heading
  fusion is still a separate post-Alpha task — see
  "Magnetometer calibration" in the backlog above.

---

## Implemented Modules

All host-tested (`cargo test --lib --no-default-features --target x86_64-unknown-linux-gnu`).

| Module           | File                     | Description                                                   |
|------------------|--------------------------|---------------------------------------------------------------|
| Rate PID         | `control/pid.rs`         | 3-axis, derivative-on-measurement, D-term LPF, anti-windup   |
| Attitude MPC     | `control/mpc.rs`         | 6-state roll/pitch/yaw+rates, tinympc ADMM, 100 Hz           |
| Altitude hold    | `control/altitude.rs`    | PID + hover feedforward, anti-windup, gated on PosKF.ready   |
| Position PD      | `control/position.rs`    | Horizontal hold, world→body rotation, tilt-limited — **written, not yet wired** |
| Quad-X mixer     | `control/mixer.rs`       | Airmode + no-airmode paths, phantom-thrust prevention         |
| Arming FSM       | `control/arming.rs`      | Pre-arm (thr/lvl/imu/rc/altitude_ref), soft baro-or-GPS gate, failsafe-on-RC-loss (never disarms; control loop chooses descent), IMU-loss-only auto-disarm |
| PosKF            | `estimation.rs`          | 6-state linear KF; GPS position + GPS velocity + baro + IMU predict |
| MEKF             | `attitude_mekf.rs`       | Quaternion MEKF with gyro-bias state, 8 kHz predict          |
| CRSF parser      | `drivers/crsf.rs`        | Byte streaming, 11-bit unpack, link stats, CRC8              |
| NMEA parser      | `drivers/nmea.rs`        | GGA/RMC/GSA/VTG, 3D fix detection, checksum                  |
| UBX parser       | `drivers/ubx.rs`         | u-blox binary protocol — written, not yet active             |
| WT901B parser    | `drivers/wt901b.rs`      | All packet types; `ImuData` type still used internally        |
| SPL06 driver     | `drivers/baro.rs`        | 128 Hz / 1× OSR, correct cal from 0x18; DPS310 retained     |
| ISM6HG256X driver | `drivers/ism6hg256x.rs` | ±16 g / ±4000 dps / 7.68 kHz, SPI; written for Beta breakout, unreferenced |
| LIS2MDL driver   | `drivers/lis2mdl.rs`     | 3-axis mag, I2C addr 0x1E, 100 Hz HR + LPF + OFF_CANC; wired into baro_task + fused in MEKF for yaw |
| IMU LPF          | `imu_filter.rs`          | 2nd-order Butterworth biquad bank on the fused dual-IMU stream; 150 Hz gyro / 25 Hz accel default |
| DShot driver     | `drivers/dshot_bitbang.rs` | TIM1 as pacer only; DMA2_CH2 writes BSRR to GPIOA, reads IDR 3× oversampled. DShot300, bidir working on hardware. Replaced the timer-output-compare driver on 2026-08-08 |
| DShot BSRR frame | `drivers/dshot_bb_frame.rs` | Pure builder: 51 BSRR words per frame (16 bits × 3 states + 3 hold); inversion is a half-word swap |
| DShot GCR decode | `drivers/dshot_bb_decode.rs` | Pure decoder: samples → 21 GCR bits → quintets → eRPM period. No EDT frame-type discrimination yet |
| DShot frame      | `drivers/dshot_frame.rs` | 16-bit frame encoder, bidir CRC inversion; MSB-first wire unpack |
| DShot telemetry  | `drivers/dshot_telemetry.rs` | GCR 5→4 decode + EDT/eRPM payload parse + period→RPM. **Orphaned** since the cutover — no callers |
| Physics sim      | `sim/sim.rs`             | 6DOF rigid body, τ=30ms motor lag, NED, ground collision     |
| Sensor sim       | `sim/sensors.rs`         | GPS (10 Hz + noise), baro (50 Hz + noise/drift), xorshift64  |
| TinyMPC solver   | `control/tinympc-rs/`    | ADMM, no_std, const-generic dimensions                        |

### Control cascade

```
PosKF (100 Hz) ← GPS (1 Hz NMEA) + baro (25 Hz) + IMU predict
      │
      ▼
Position PD (5 Hz)  ← [written, not yet wired]
      │
      ▼
Attitude MPC (100 Hz) → rate setpoints
      │
      ▼
Rate PID (8 kHz) → torque demands → mixer → DShot → motors
```

### Simulation examples

`cargo run --example <name> --no-default-features`:

| Example          | Description                                          | Status                             |
|------------------|------------------------------------------------------|------------------------------------|
| `sim_hover`      | PID-only hover at 5 m                                | Stable, ±0.02 m altitude           |
| `sim_mpc_hover`  | MPC+PID hover at 5 m                                 | MPC converges in 3–5 iterations    |
| `sim_kf_hover`   | Full stack, noisy GPS+baro → KF → altitude hold      | KF altitude within ~30 mm of truth |
| `sim_gps_rescue` | Fly (20, 10) m → home at 5 m altitude                | Arrives within 0.23 m of home      |

---

## Post-Alpha Directions

Parked here so they aren't forgotten. **None of these should be started
until the post-Alpha tweaks above are done.**

### H743 headroom

The H743 is significantly more capable than the F722 it replaced:
480 MHz vs 216 MHz, 1 MB SRAM, ITCM/DTCM zero-wait-state memories,
dual FPU. Ideas ordered by payoff/risk ratio:

1. **MPC hot loop in ITCM.** TinyMPC's ADMM iteration is our tightest
   inner loop; zero-wait ITCM should measurably cut solve time. Low risk.
2. **MPC rate → 200 Hz.** 100 Hz landed 2026-06-21: the outer loop was
   mislabelled 50 Hz but actually ran at 25 Hz (gated `cycle_count % 4`
   on a 100 Hz task) while `MPC_DT` assumed 50 Hz — a 2× model/loop
   mismatch. Now runs every navigation cycle, with `MPC_DT`/`MPC_PERIOD_US`
   the single source of truth shared by `main.rs` and `mpc.rs`. Next step
   200 Hz is gated on (1) for solve headroom.
3. **Longer MPC horizon.** At `MPC_DT = 0.01` the 10-step horizon previews
   only 0.1 s (halving `MPC_DT` to reach 100 Hz halved the preview from
   0.2 s). Bumping `HX` restores look-ahead at the cost of RAM + solve
   time; helps on aggressive manoeuvres.
4. **True NMPC (12-state, SQP / RTI).** Biggest payoff, biggest work.
   Needs a nonlinear solver (acados-style). Gate on (1)+(2) showing us
   the solve-time budget.
5. **Bidirectional DShot + adaptive B matrix.** See Backlog above for the
   full rationale. The MPC aspect specifically: feed measured eRPM through
   a thrust model (eRPM → force) and use it to update the B matrix in
   the MPC's linear model at runtime. This makes the controller
   self-correcting as battery voltage sags, props wear, and ESC
   characteristics vary motor to motor. Phases:
   - Bidirectional DShot capture (timer input capture + DMA)
   - eRPM → thrust estimation
   - Online B-matrix update in the MPC solve loop
   - Disturbance observer for unmodelled dynamics

### OSD / VTX

- **Analog OSD (AT7456E, onboard):** SPI character-overlay chip, register-
  compatible with the MAX7456. Verified from ArduPilot docs. Driver would
  follow the MAX7456 protocol; datasheet is the AT7456E.
- **Digital OSD:** MSP DisplayPort over UART4 (already labelled for this).
- **VTX control:** SmartAudio or IRC Tramp over a spare UART.

### Blackbox logging

Record ~200 Hz flight data to onboard flash. Key fields: IMU raw+fused,
RC input, control demands, motor outputs, MPC solve time, GPS fix
quality. Format could be Betaflight-compatible or custom.

### Current sensing

Battery voltage and motor current feedback to the FC and pilot.
The board has current-sensing hardware; a driver is needed.

### Configuration interface

- **MSP over USB** — Betaflight configurator compatibility.
- **MAVLink over UART** — Mission Planner / QGC.
- **Parameter storage in flash** — persist tuning across reboots. (A
  general versioned flash store now exists — `src/persist/`, sub-project A
  — currently holding only the mag calibration; extend its `Config` for
  PID/tuning.)

### Pilot telemetry (CRSF FC→TX)

A general channel for surfacing FC state back to the EdgeTX transmitter
(and its voice callouts) — far more useful than the onboard LED for field
feedback (cal progress, arming-blocker reasons, battery, GPS, attitude,
sensor health). Deferred as its own subsystem; notes for when we return:

- We are currently **RX-only** on CRSF (UART5 RX, PB5). CRSF is half-duplex
  single-wire, so telemetry FC→TX needs the **TX direction on that line**
  plus a **telemetry-frame scheduler**. That's the bulk of the work.
- Once present, **native CRSF telemetry frames** are decoded by EdgeTX
  automatically (battery `0x08`, GPS `0x02`, attitude `0x1E`, **flight-mode
  text `0x21`**, vario) and can be **voice-announced** via Special
  Functions — no Lua. The flight-mode text frame is a low-effort way to
  speak short status strings ("CAL 60", "CAL OK").
- **MSP-over-CRSF** (`0x7A`/`0x7C`) carries arbitrary data but needs a
  **Lua script on the TX** (like the Betaflight configurator) — more work
  both ends.
- Right-sized for general status/voice; the per-feature LED stays as the
  no-radio fallback.

---

## Journal — 2026-07-26: bidirectional DShot bring-up (unresolved)

Long bench session on the DAKEFPV H743 chasing why bidirectional DShot
does not work while plain DShot does. **Not resolved.** Bidir still fails:
the ESC does not accept frames (motors do not spin) and no telemetry is
ever decoded (`NoEdge` on every frame). Plain DShot is unaffected and
works end to end, including in the flight firmware.

### Fixed and confirmed on hardware

Three real defects, all in the bidir-only RX→TX path (`dshot_hw.rs`):

1. **Stale compare register** left the line asserted for ~8 µs before
   every frame, swallowing bit 0's falling edge — the edge BLHeli syncs
   on. The old guard wrote `CCRn` with `OCxPE=1`, so the zero landed in
   the preload register and never reached the active one.
2. **The update event needed `CCxE=1`** to take effect. Isolated with a
   bench probe: writing `CCR1` alone changed nothing, a single `EGR.UG`
   released all four pads at once.
3. **A GPIO glitch guard in the output direction that BF does not have.**
   BF's H7 guard exists only in `pwmDshotSetDirectionInput`;
   `pwmDshotSetDirectionOutput` touches no GPIO. Ours was symmetric.

Also: the direction switch back to output now happens immediately before
transmit (as BF does from `pwmTelemetryDecode`) rather than when the
response window closes, and the transmit path follows BF's register
order with the DMA streams armed last. That shortened the stuck-LOW
first bit from ~8 µs to 3.4 µs — it moved the symptom, not the cause.

The transmit reorder sits in the path **shared with non-bidir**, so it
was re-verified on hardware afterwards: plain DShot still spins the
motors normally. No regression to the working protocol.

### The open problem

The transmit-setup pad trace localises it exactly:

    after switch=0000 | after ARR/CNT=0000 | after CCxDE=0000
    | after DMA armed=1111 | after frame=1111

The line is LOW from the moment the direction switch returns, and only
goes HIGH once the DMA writes a cell value. The idle probe narrows it
further: `OCM=FORCE_INACTIVE` gives idle-high, but PWM mode 1 with what
should be `CCR=0` gives an **active** output. So the active compare
register still holds an RX capture value, and with `ARR` at `0xFFFFFFFF`
the `CNT < CCR` condition stays true for a long time.

**Unexplained:** why the active compare register cannot be cleared by
writing it with preload disabled — which is all BF's `LL_TIM_OC_Init`
does, and BF works on this exact board and ESC.

### 2026-08-02 — the reference was the wrong file

`dshot_bitbang = AUTO` (the BF default) resolves to **bit-banging** on
H7. Verified verbatim in `src/platform/common/stm32/dshot_bitbang_shared.c`:

    bool isDshotBitbangActive(const motorDevConfig_t *motorDevConfig)
    {
    #if defined(STM32F4) || defined(APM32F4)
        return useDshotBitbang == ON ||
            (useDshotBitbang == AUTO && useDshotTelemetry
             && motorProtocol != PROSHOT1000);
    #else
        return useDshotBitbang == ON ||
            (useDshotBitbang == AUTO && motorProtocol != PROSHOT1000);
    #endif
    }

H7 takes the `#else` branch: AUTO means bitbang for any protocol except
ProShot1000, regardless of whether telemetry is enabled. Only F4
additionally requires `useDshotTelemetry`.

So the working Betaflight on this board is **bit-banging**, for both
bidir and plain DShot, and `pwm_output_dshot_hal.c` — the file this
driver claims to port and which the 2026-07-26 session transliterated
against — is not the code producing that waveform.

**This voids the open question above** rather than answering it. "Why
can we not clear the active compare register when BF's `LL_TIM_OC_Init`
can?" assumed BF does so successfully on this hardware. It does not do
it at all. There was no working counter-example.

It does not prove the timer-DMA path cannot work on H7 — BF still ships
it for when bitbang is off — only that we have no evidence it does, and
that BF defaults away from it on every family after F4.

### Next steps

- **Reference capture.** Flash BF (known working bidir on this hardware)
  and capture one full frame period at ~10 µs/div. Gives the ESC's real
  reply timing and edge spacing — which calibrates `DEADTIME_US` and the
  GCR decoder — and settles whether BF's idle line is clean.
- **Finish the port properly.** The header claims a direct port of BF's
  H7 driver; it is not. The DMA lifecycle is the substituted piece: BF
  tears down and reconfigures the stream *inside* the direction switches
  with an explicit `Direction` field, while we construct and drop Embassy
  `Transfer` objects per frame. Needing an `EGR.UG` that BF does not need
  is itself evidence the port diverges structurally.
- **Confirm `UDE` vs `CCxDE`** for H7 specifically. Our port assumes
  per-channel compare DMA; one BF source read suggested update-event DMA
  ("exactly one transfer per TIM cycle"), but that fetch mixed in H5/N6
  detail and was not confirmed.
- **Recalibrate the RX self-test** before trusting it. It reports 2 of 8
  self-driven edges captured, but the capture timestamps show ~145 ticks
  between them rather than the ~24 expected, so the pulse generator's
  timing is wrong, not necessarily the capture path. Register dump
  confirms the RX config is correct (`CCS=1 ICPSC=0`, both-edge `CCER`).

### Bench tooling added

- Build stamp (`<epoch>-<sha>[-dirty]`) logged at DShot init and echoed
  by the flash scripts with the binary's SHA-256, so "is this the
  firmware I just built" is answerable.
- `rerun-if-env-changed` for the motor-test env vars. Without it,
  `LOOP_KHZ=2 ./scripts/flash-motor-test.sh` recompiled nothing and
  flashed the previous config — very likely the cause of the
  "motor bidir setting not responding to code changes" note from
  2026-07-25.
- `DEADTIME_US=<n>` build-time override. Moving it moves the direction
  switch, which is how the stray pulse was pinned to our code rather
  than the ESC.
- Idle probe, transmit-setup pad trace, and RX loopback self-test, all
  gated to single frames inside the MotorStop arming window.

### 2026-08-08 — bidirectional DShot works on the bit-banged driver

Bidirectional DShot now works on the bit-banged driver
(`src/drivers/dshot_bitbang.rs` + `src/drivers/dshot_bb_decode.rs`). The
older timer-DMA driver (`dshot_hw.rs`), the subject of the investigation
above, never achieved it and remains unfixed; plain (non-bidir) DShot
does work there.

The rewrite happened because of the 2026-08-02 finding above: Betaflight
resolves `dshot_bitbang = AUTO` to bit-banging on everything after F4,
including H7, so the whole timer-DMA port was built against the wrong
reference. In the bitbang design the timer is only a pacer; DMA writes
BSRR words to GPIOA to produce the waveform and reads IDR with 3×
oversampling to capture the reply.

Measured/derived timing actually in the code: TX pacer ARR=265 (240 MHz
/ 266 = 902 kHz state rate, 3 states per bit = 3.325 µs/bit = DShot300,
+0.25% fast). RX pacer ARR=212 (1.125 MHz = reply rate 5/4 × 300 kHz,
oversampled 3×). One frame is 51 states = 56.5 µs; the RX window is 140
samples = 124 µs.

Bench result 2026-08-08 with `BIDIR=1 LOOP_KHZ=2` (at the time this also
took `DRIVER=bitbang`; the timer driver was retired later the same day and
that variable no longer exists): motors
ran and reply data resembling eRPM appeared on the scope after the
frame. Caveat: one of the four motors was physically unsoldered on the
bench rig, so this was a 3-of-4 result, not a clean sweep. Decoded
telemetry has **not** yet been confirmed in the logs — that remains the
pending bench gate. This entry adds the decode wrapper
(`DshotBitbang::send_and_decode`, commit `6f5cb61`) and wires it into the
bitbang drive loop in `motor_test.rs`, logging
`motor-test RX [bitbang]: M1=… M2=… M3=… M4=…` at ~10 Hz; the arming
loop still calls the undecoded `send_and_receive`, mirroring the timer
path's arming loop, which also doesn't decode or log. Bench-verifying
that log line is still open.

Three defects were found in the implementation plan document itself
during execution, worth recording as a caution about that document: a
GCR test-encoder helper that drove the line HIGH at frame start when a
real ESC pulls it LOW; PAC type errors (TIM1 needs `ArrCore`/`CntCore`
newtypes, TIM2 does not); and `Transfer::new_read` requiring
`peri_addr: *mut W`, not `*const W`.

One code-review finding was raised as Critical and then downgraded to
Minor after the bench disproved it: a predicted stray pulse in the idle
gap from the input→output MODER switch. It did not appear. The reviewer
had cited the Betaflight hold-states rationale as its mechanism, but
that rationale is explicitly about the transition *to* an input, which
the hold states already cover — not the transition back to output. That
distinction has now misled one reviewer; worth checking carefully before
citing it again.

Still outstanding: the driver is bench-only (`motor-test` feature), not
yet wired into the flight path — that's Task 6.

### 2026-08-08 (later) — cutover: timer-DMA DShot retired

The bit-banged driver replaced `dshot_hw.rs` on the flight path.
`dshot_hw.rs` and `dshot_diag.rs` are deleted; `dshot_frame.rs` stays
(shared encoder). Work moved to branch `dakefpv-h743-bitbang-dshot`;
the pre-cutover tree is preserved on `archive/dakefpv-h743-timer-dma-dshot`
and, more portably, at commit `bbf2d2b`.

Justification for deleting rather than keeping a fallback: the bitbang
driver covers *both* modes. Plain DShot (`BIDIR=0`) was the Task 3 bench
gate and motors spun; bidirectional was verified with decoded eRPM on
three channels. So the timer driver was redundant, not a safety net.

Bench state at cutover: M1/M2/M3 return stable eRPM ~3900–4100 µs at 5%
throttle (≈15,200 eRPM ≈ 2,170 mechanical RPM on a 14-pole motor). M4
returns `NoSignal` — its line never goes low in any of the 140 samples,
across every probe burst. Since all four pins are read from one `IDR`
word in a single DMA transfer with `MODER` written for all four together,
there is no per-channel code path that could single out M4, so this is
an off-chip fault. Two candidates were considered and both weakened:
ESC-side bidir config (there is none — ESCs auto-detect the inverted
signalling) and a broken signal wire (M4 spins at the correct frequency,
so TX arrives intact). The asymmetry worth probing is that the MCU drives
push-pull both ways while the ESC only pulls *low* against our internal
pull-up, so a degraded path can pass TX and still fail RX. Unresolved;
scope the M4 pad during the receive window.

**Open, and load-bearing for flight:** `control_loop` hardcodes
`dt = 0.000125` (8 kHz), but a bidirectional DShot300 frame is ~181 µs
(TX 56.5 + RX 124.2) against a 125 µs period. The loop cannot hold 8 kHz;
because `IMU_DATA` is a latest-value `Signal` it free-runs at ~5.5 kHz and
drops gyro samples rather than lagging. The rate PID then sees
non-uniformly sampled gyro *and* a `dt` wrong by ~45%. Fix under
consideration is DShot600 (`TX_ARR` 265→132, `RX_ARR` 212→106, total
~91 µs) plus a measured rather than hardcoded `dt`. **Do not read
anything into 8 kHz inner-loop behaviour until this is settled.**

A code review of the cutover also caught that the bench RX probe had
followed the driver into the armed flight loop. It emits eight
`defmt::info!` lines, and `logger::putc` busy-waits on USART6 TXE at
115200 baud inside a global `critical_section` — milliseconds of
interrupts-off, repeating, in flight. Now gated behind the `motor-test`
feature. The 10 Hz telemetry log in `control_loop` has the same blocking
property and predates the cutover; it should get the same treatment.
