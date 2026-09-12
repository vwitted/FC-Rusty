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
