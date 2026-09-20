# Sim and tuning direction, 2026-09-12

Notes from the session that followed the first plant capture. Not a
specification; PROJECT_STATUS.md holds current state.

## Prerequisites

- **Magnetometer.** RESOLVED, 2026-09-20. The fitted compass is a
  QMC5883P alongside an M10 GPS, now running at 10 Hz.
- **Airframe figures.** SUPPLIED, 2026-09-20, in
  `docs/motor_body_measurements.md`. Carried into the code as
  `QuadParams::deadcat_7in()` and `mixer::DEADCAT_7IN`. Two numbers are
  still open, and both block the retune below:
  - **Thrust.** 25 N per motor, 100 N total, is 13.6:1 on 750 g where a
    7in build is usually 4-6:1. Loop gain scales with it. Hover throttle
    on the first flight separates the cases: 27% means 100 N is right,
    54% means it is about 4x high.
  - **Fore/aft centre of gravity.** Given twice in the measurements, and
    the two disagree in DIRECTION. The motor-to-CoG arm lengths put it
    aft of the motor centroid; the separate "1:1.6 rear:front" figure
    puts it forward. The aft reading is used, as the one taken to the
    motors, but it is a reconciliation rather than a measurement. One
    balance test settles it: balance the airframe fore/aft on an edge and
    measure from the rear motor axis to the balance line.

## Sim roadmap

1. Replace the 5in plant defaults with the 7in figures, and re-bless the
   sweep baseline once. STARTED 2026-09-20: the figures are in the code
   as `QuadParams::deadcat_7in()`, but the default still describes the
   5in quad because adopting them requires retuning the whole cascade.
   See "Retuning against the real plant" below.
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
board at prediction horizons 4 to 30. The iteration cap is a model
parameter too (`MPC_MAX_ITER` in the sweep), and the bench times caps 5,
10, 20 and 50 at every horizon. The tuner genes are not yet added.

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
model exactly. That was not the cause of the legacy failures. The
gyro low-pass filter was: `new_lowpass_butterworth` accepted a cutoff at or
above Nyquist, where the bilinear prewarp turns negative and the filter
unstable. It now clamps the cutoff to 0.45 of the sample rate and treats
zero as disabled. Every legacy resonance row now flies (104 failures to
none); the sweep baseline was re-blessed for those 13 rows only.

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

## Frame geometry and the mixer, 2026-09-20

The airframe is a deadcat: rear motors 200 mm apart, front pair 300 mm,
sides 230 mm. Those separations are over-determined and agree — they
predict a 336 mm diagonal against 330 mm measured — which fixes the
lateral half-spans at 100/150 mm and the longitudinal motor span at
224.5 mm.

**Only the mixer's thrust column should change.** The expected change
was to scale the roll and pitch columns by lever arm, which is what
minimum-effort allocation gives. Measured on this frame, that is worse
on both counts that matter: torque is limited by the first motor to hit
a rail rather than by total effort, and arm-scaling under-drives the
short arms, giving 14% less roll moment per unit of saturation and 7%
less pitch. It also quadruples the roll-to-pitch cross term. `+/-1` is
the right answer here, and `mixer.rs` has tests recording why.

What is a real defect is the thrust column. Four equal thrusts about a
centre of gravity that is not their centroid do not balance: at hover the
residual is 0.07 N.m, about 1100 deg/s^2, enough to carry the aircraft
through 90 degrees of pitch in under a second open-loop. In the sweep it
costs a factor of 27 in attitude RMS (1.10 deg against 0.041). The
derived column removes it, up to a collective of 0.96 where the rear pair
clamps.

`mixer::DEADCAT_7IN` is built and tested but NOT wired into flight;
`main.rs` still uses `QUAD_X`. Switching it is a one-line change, held
back until the balance test fixes the sign of the CoG offset — applied
backwards it would double the trim rather than remove it.

## Retuning against the real plant

`QuadParams::default()` still describes the 5in quad. Sweeping the
measured figures in makes every case a flyaway, on every axis and seed.
Isolated:

- The cause is `max_thrust`, not the geometry. At the old 20 N the real
  deadcat geometry flies; at 100 N even a symmetric frame flies away.
- Scaling the rate gains recovers attitude at about 0.2x (attitude RMS
  0.415) but not altitude. Hover throttle moves from 0.54 to 0.27, so the
  altitude and position loops are mistuned by the same factor: altitude
  RMS 13.6 m, airborne 40% of the time.

So adopting the real plant means retuning the whole cascade, not just the
rate loop, and it is worth doing only once the thrust figure is
confirmed. Sequence: confirm thrust, settle the CoG by balance test, wire
the derived mixer, then run the GA jointly over rate gains, filters,
altitude and position gains, and re-bless.
