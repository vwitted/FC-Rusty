# Sim and tuning direction, 2026-09-12

Notes from the session that followed the first plant capture. Not a
specification; PROJECT_STATUS.md holds current state.

## Prerequisites

- **Magnetometer.** The SE100 V2 compass (IST8310) reads on the bench but
  has no hard-iron calibration and an unverified mounting orientation.
  Both are needed before heading is trusted, and before sim work that
  depends on real sensor figures.
- **Airframe figures.** All-up weight with the flight battery,
  motor-to-motor diagonal, cell count, motor size and KV, prop, and hover
  throttle if known. The sim defaults describe a 5in racer; the airframe
  is a 7in. Only `motor_tau` (36 ms) is measured.

## Sim roadmap

1. Replace the 5in plant defaults with the 7in figures, and re-bless the
   sweep baseline once.
2. Run the PosKF and MEKF in the harness loop by default, with baro and
   GPS noise models. The PosKF is not in the harness today: altitude and
   position control run on true state, so the harness cannot show what
   the estimators contribute to position stability.
3. Model vibration locked to rotor speed, seeded from the measured eRPM.
   The current model is a fixed-frequency sine, which cannot decide
   whether the gyro filter is needed.
4. Extend the tuner beyond its seven genes (rate and yaw gains, gyro
   cutoff, D-term filter) to the rate-loop rate, MPC rate and MPC horizon.
5. Validate against the first blackbox flight by comparing the sim's
   predicted response with the measured one.

Progress 2026-09-13: `AttitudeMpc` takes its timestep and rate-lag constant
at runtime (`MpcModel`) and is generic over its horizons; the harness builds
it for each rate preset. `scripts/mpc-bench.sh` times the solver on the
board at prediction horizons 4 to 30. The tuner genes are not yet added.

## Decisions

- 2026-09-13: the MPC horizon is a tuner variable, not held at a fixed
  look-ahead time. The horizon lengths are const-generic parameters of the
  vendored solver (`Solver<f32, MpcPolicy, NX, NU, HX, HU>` in
  `src/control/mpc.rs`), so one build cannot vary them. The sim will
  instantiate a small set of horizons and let the tuner choose among them;
  the firmware keeps one. The MPC's rate-response constant (`TAU_MOTOR`,
  30 ms) models the closed rate loop, so it depends on the rate gains and
  belongs in the same search.
- 2026-09-13: horizon and MPC rate are chosen without regard to compute
  first, then cut to what the board can run. For each horizon and rate, the
  remaining genes are re-tuned and the best cost recorded over several
  seeds, giving a cost surface. The feasible region is where the MPC solve,
  timed on the board, fits inside the MPC period with headroom for the rest
  of the navigation task. The choice is the lowest-cost point in that
  region; a smaller horizon is preferred when its cost is within noise of
  that minimum. The cost need not fall monotonically with horizon: model
  error compounds over a longer look-ahead, and the solver is capped at
  10 iterations per solve.

## Known defects

Found during the 2026-09-12/13 sessions and not yet fixed. Each is either
folded into the work above or needs its own fix; none should be dropped.

Harness and filters:
- The gyro low-pass filter accepts a cutoff at or above Nyquist.
  `new_lowpass_butterworth` in `src/imu_filter.rs` prewarps with
  `tan(pi * fc / fs)`, which turns negative past fs/2, and has no guard.
  The legacy preset samples at 200 Hz with the firmware's 150 Hz cutoff,
  so every legacy row went non-finite (104 of 104 failures). On the
  undegraded legacy trace: non-finite at 2.5 s with the default cutoff; a
  crash at the 5 s drop with a 40 or 90 Hz cutoff; flies with the filter
  off. It must be guarded before loop rate becomes a tuner gene, or the
  tuner will generate invalid filters. The firmware's own 150 Hz at 8 kHz
  is unaffected.

Firmware:
- The rate gains put the sim's rate loop into a saturated 41 Hz limit
  cycle. Recorded as a pre-flight item in PROJECT_STATUS.md.
- `TAU_MOTOR` (30 ms) in `src/control/mpc.rs` is a pre-measurement guess at
  the closed rate-loop constant. Folded into the tuner search above.
- PosKF drifts at rest without GPS: on the 2026-09-09 bench log
  (`docs/log_09-09-2026.log`) north position walked 1.2 m in 8 s and
  altitude wandered 0.2-0.6 m while the board warmed from 27 to 29.5 degC.
  Not investigated.

Fixed 2026-09-13: the `main.rs` comment calling the navigation loop 50 Hz
(it runs at 100 Hz), and PROJECT_STATUS.md listing defmt on USART3 (it is
USART6) and NMEA-only GPS (UBX is preferred). Also the legacy preset
solving a 10 ms MPC model every 20 ms: the harness now discretises the MPC
for each preset's own period and gives the firmware preset the firmware
model exactly. That was not the cause of the legacy failures; the filter
entry above is.

Pending hardware verification:
- IST8310 hard-iron calibration, mounting orientation, and the handedness
  choice (Z flip, as PX4 and ArduPilot; Betaflight flips Y).
- ESC re-arming between plant-capture runs.

## Tuning approach

Defaults should be discovered per hardware rather than tuned to hold
across a range of airframes. When the firmware moves to new hardware, the
discovery is rerun against that hardware's measured plant and yields a
variant or branch. Scoring across a range remains useful only while a
figure on one airframe is still unmeasured, such as inertia before a
flight log exists; the range collapses as each figure is measured.

## Waypoint flight scenario

A sim scenario that flies a realistic waypoint mission with mock CRSF
pilot commands injected, exercising the MPC and sensor fusion together
rather than regulation about level. Its purpose is to tune the MPC loop
rate and horizon, which the current sweep cannot reach.

## DShot rate

DShot300 was recalled as the diagnostic choice, with DShot600 to follow
once bidirectional telemetry worked. The driver already runs DShot600
(`src/drivers/dshot_bitbang.rs`, `TX_ARR = 132`; bench logs report
`ARR=132`): a DShot300 frame took about 181 us and could not fit the
8 kHz loop's 125 us period. Two comments still stating DShot300 were
corrected on this date.
