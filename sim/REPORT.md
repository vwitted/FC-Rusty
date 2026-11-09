# Estimator tuning and hardening: report

Scope: the MEKF attitude estimator, the 6-state PosKF and the GPS-derived
acceleration compensation, evaluated in closed loop by `sim-eval`
(src/sim/est_eval.rs, examples/sim_eval.rs). 22 iterations were run; 6 were
accepted and committed. Every candidate is logged in sim/ITERATIONS.md.

**All results are host simulation. None of these changes has flown.**

## Method

- **Loop.** The firmware's estimator wiring runs end to end: ICM-42688P clip at ±16 g / ±2000 dps, the
  ImuFilter front end (gyro 150 Hz, accel 25 Hz), MEKF predict at 8 kHz and accel update at 100 Hz, GPS
  compensation through `gps_accel`, PosKF at 100 Hz with 10 Hz GPS and 25 Hz baro. Altitude and position
  loops close on the estimates. Estimator constants are shared with main.rs through the library
  (c7859c5, 56b5f3b), so a value accepted here is the value that flies.
- **Controller.** The exam flies the GA-fitted rate gains (kp 0.009, kd 0.00015), declared in every
  scenario file. The firmware's 0.020/0.001 put this plant in the saturated 41 Hz limit cycle recorded in
  PROJECT_STATUS item 3: motors were on a rail for 99.9 % of steps, which would make every estimator
  number a property of that pathology.
- **Metrics** (pooled over seeds 1–4; definitions in examples/sim_eval.rs):
  - `rmse_attitude`: MEKF tilt error, in degrees. Yaw is excluded because the sim has no magnetometer.
  - `rmse_altitude`: PosKF altitude error, in metres.
  - `max_overshoot`: the true response past a commanded step, as a fraction of the step.
  - `divergence_count`: seeds that crashed, tumbled or drifted, or whose estimator error exceeded 30°,
    10 m or 50 m.
- **Acceptance.** A candidate needed at least one metric to improve, and no metric to regress against
  either the current best or the frozen `sim/baseline.json`. The noise floor was max(1 %, 0.002), fixed
  before the baseline was taken.
- **Held-out check.** From iteration 9 every candidate also had to survive seeds 5–12. It was rejected if
  its gains did not replicate there, if any divergence count rose, or if any scenario's survivor-matched
  median per-seed RMSE worsened by more than 5 %. This criterion was formalised at iteration 17 and
  applied retroactively.

## Results

Exam (seeds 1–4), baseline → final. Against the baseline, 21 metrics improve and none regresses.

| Scenario | tilt RMSE (deg) | alt RMSE (m) | max overshoot | diverged (of 4) | pass |
|---|---|---|---|---|---|
| accel_saturation | 15.61 → 0.94 | 4.32 → 0.32 | 0.000 → 0.000 | 2 → 0 | no → yes |
| accel_sign_x | 5.59 → 5.54 | 0.20 → 0.20 | 0.000 → 0.000 | 0 → 0 | no → no |
| accel_sign_y | 5.81 → 5.78 | 0.21 → 0.21 | 0.000 → 0.000 | 0 → 0 | no → no |
| accel_sign_z | — → 2.82 | — → 0.22 | — → 0.000 | 4 → 0 | no → no |
| airmode_transitions | 4.77 → 3.41 | 0.21 → 0.19 | 0.091 → 0.073 | 0 → 0 | no → no |
| altitude_steps | 6.16 → 2.02 | 0.58 → 0.25 | 0.306 → 0.122 | 0 → 0 | no → no |
| gps_dropout_10s | 7.17 → 5.25 | 0.49 → 0.32 | 0.000 → 0.000 | 3 → 1 | no → no |
| gyro_sign_x | — → — | — → — | — → — | 4 → 4 | no → no |
| gyro_sign_y | — → — | — → — | — → — | 4 → 4 | no → no |
| gyro_sign_z | — → — | — → — | — → — | 4 → 4 | no → no |
| nominal_hover | 2.43 → 1.35 | 0.25 → 0.21 | 0.000 → 0.000 | 0 → 0 | no → yes |

"—" means every seed failed, so there is nothing to average. `altitude_steps` misses its 2.0° threshold
by 0.02°.

Held-out seeds 5–12, baseline-equivalent parameters → final:

| Scenario | tilt RMSE (deg) | alt RMSE (m) | diverged (of 8) | pass |
|---|---|---|---|---|
| accel_saturation | 20.39 → 0.99 | 4.18 → 0.34 | 7 → 0 | no → yes |
| accel_sign_x | 5.69 → 5.98 | 0.18 → 0.21 | 3 → 0 | no → no |
| accel_sign_y | 5.44 → 6.17 | 0.18 → 0.24 | 1 → 0 | no → no |
| accel_sign_z | — → 2.91 | — → 0.22 | 8 → 0 | no → no |
| airmode_transitions | 4.35 → 3.37 | 0.20 → 0.20 | 0 → 0 | no → no |
| altitude_steps | 6.54 → 2.37 | 0.51 → 0.21 | 0 → 0 | no → no |
| gps_dropout_10s | — → 8.99 | — → 0.45 | 8 → 6 | no → no |
| gyro_sign_x | — → — | — → — | 8 → 8 | no → no |
| gyro_sign_y | — → — | — → — | 8 → 8 | no → no |
| gyro_sign_z | — → — | — → — | 8 → 8 | no → no |
| nominal_hover | 2.36 → 1.42 | 0.21 → 0.20 | 0 → 0 | no → yes |

## Accepted changes

| Commit | Iteration | Change | Mechanism |
|---|---|---|---|
| 5a8cbd1 | 3 | Skip the accel update, and coast the PosKF at σ_a 16 m/s², on a window holding a clipped raw sample | Vibration riding on the 1 g offset clips asymmetrically, which biases the window mean |
| d7f9dc6 | 8 | Exclude the accelerometer when the body-z specific force exceeds +0.5 g for 30 ms | Thrust is along body −z in every orientation, so a positive f_z is a sensor fault, not flight |
| 85a5c83 | 19 | While the accel is excluded, aid tilt with the GPS-implied thrust axis R·e₃ = (g·e₃ − a)/‖·‖ | A tilt reference independent of the IMU |
| e910a3e | 20 | Fuse that axis weakly (σ 2.0) beside valid accel updates | In manoeuvres the accelerometer reads only the thrust axis |
| 58e7255 | 21 | Accel-update σ grows with body rate, gain 3 per rad/s | Rotation adds specific force that the gravity model lacks; the term is 0 in hover |
| 4580b05 | — | Removed the firmware-compiled code of rejected iterations | No behaviour change |

The firmware raises a `defmt::warn!` when the specific-force invariant excludes the accelerometer, so a
mis-mapped IMU axis is reported on the bench.

## Step 5: circularity audit of acceleration references

**No acceleration reference used in a firmware update step is derived from the IMU prediction.**
Traced in the code, not taken from its comments:

- **MEKF accel update** (main.rs, `mekf_task`).
  - The measured specific force has a GPS acceleration subtracted
    (`gps_accel::compensate_specific_force_g`).
  - That acceleration comes from `GpsAccelEstimator`, which is fed receiver ground speed and course
    through `estimation::gps_velocity_ned` in `pos_kf_task`, never the PosKF velocity state.
  - The correction is rotated into the body frame by the MEKF's own attitude. That is not circular: for an
    attitude error δθ it adds a gain error of order ‖a‖/g·δθ and does not cancel the gravity term.
  - The Euler rotation it uses matches the MEKF quaternion
    (`gps_accel::tests::euler_rotation_matches_the_mekf_quaternion`).
- **PosKF updates.** GPS position (geodetic → NED), GPS velocity (receiver speed and course) and baro.
  The IMU enters only the predict input, R(q)·f + g, which is a prediction, not a reference.
- **COG yaw update.** Receiver course over ground, gated on armed, 3D fix, GPS ground speed and stick.
- **Thrust-axis aiding** (added in this work). Built from `GpsAccelEstimator` only; no IMU term.

The only algebraically circular reference in the tree is the harness's opt-in demonstration
(`harness.rs` `compensate_from_estimate`, `sim_sweep --compensate-circular`). There,
f − R̂ᵀ(R̂f + g)/g = −R̂ᵀe₃, which is exactly the predicted gravity, so the innovation is zero.
`gps_accel::tests::imu_derived_reference_is_circular_and_gps_derived_is_not` now pins this: the circular
reference leaves the quaternion unchanged to within 1e-6, while the GPS-derived one corrects a 15° tilt
error by more than 80 %. The GPS-derived alternative it calls for already exists, and is what the firmware
uses.

**A sim/firmware mismatch, not circularity.** `harness::run_case` rotates its GPS compensation by the
TRUE attitude (`sim.state.q`), has no clamp or stale handling, and closes altitude on truth. The
"GPS-derived" figures in the `HarnessCfg::compensate_from_gps` comments, such as att_rms 0.546 against
2.775, were measured that way and are optimistic. `est_eval` uses the firmware path instead.

## Regressions

- **Against the frozen baseline: none on the exam.**
- **Held-out `accel_sign_x` / `accel_sign_y` tilt RMSE** rises from 5.69 to 5.98° and from 5.44 to
  6.17°, while their divergences fall from 3/8 to 0 and from 1/8 to 0. Part of this is survivorship: seeds
  that used to diverge, and so were excluded, are now included. The survivor-matched median at acceptance
  was about +2 % for each, below the 5 % held-out bar.
- **Nominal hover** carries about +0.5 % from iteration 20, inside the noise floor, which iteration 21 then
  more than recovered (2.45 → 1.35°).

## Blocked by the rule

**The largest gain found was not committed.** It is a joint MEKF retune, iteration 11:

```
est.accel_window_mean = true
gps_accel.tau_s = 0.35
mekf.sigma_a = 0.3
```

Applied to the baseline estimator:

- **Gains:** nominal tilt 2.43 → 0.69°, altitude-step tilt 6.16 → 0.97°, GPS-dropout divergences 3/4 → 0.
  Held-out, dropout divergences fall 8/8 → 0.
- **Cost:** tilt RMSE on the mirrored-accelerometer scenarios rises 8–18 %.

The mechanism: trusting the gyro more slightly worsens how the MEKF settles on the mirror image of a
sign-inverted accel axis. Whether that trade is acceptable is your call. Re-tested against the final
estimator (iteration 21), it is still blocked by `accel_sign_x/y` alone. The same trade blocks
`mekf.sigma_a` alone and the GPS-accel τ; see iterations 15 and 22.

## Unresolved

1. **Gyro sign inversion (`gyro_sign_*`): 4/4 diverge, before and after.**
   - In this firmware the rate PID and the MEKF read the same gyro signal (`gyro_corr_dps` derives from
     the MEKF's `gyro_rad`), so a real inversion also inverts the rate loop. That is positive feedback
     that no estimator can survive.
   - The scenarios invert only the estimator's copy. Even then the MEKF tumbles within 3–5 s, faster than
     a bias-magnitude watchdog can react (iteration 14).
   - These scenarios are not physically realisable on this firmware. A rate-loop plausibility check is a
     control-side task.
2. **Accelerometer x/y sign inversion.** A static inversion is unobservable. A multirotor's horizontal
   body specific force is drag only, so a mirrored axis is self-consistent with a mirrored tilt. The MEKF
   settles about 2× the wind-holding tilt away, and no innovation gate fires (iteration 2). Detecting it
   needs a dynamic GPS/IMU correlation monitor, which was not built.
3. **GPS dropout in wind: 1/4 (exam) and 6/8 (held-out) still diverge.**
   - Once the GPS compensation decays, the accel update pulls the estimate toward the thrust axis. A
     multirotor accelerometer reads thrust plus drag, not gravity. Traced (`sim-eval --trace`): the truth
     reaches −10° roll / −19° pitch while the estimate reads about 0°.
   - Tilt is not observable from the IMU there. R inflation and zero-velocity aiding did not replicate
     (iterations 4 and 5).
   - Separately, and on the control side: the firmware has **no GPS-staleness fallback for position
     hold**. `home_latched` stays true through an outage, so the position loop keeps flying a
     dead-reckoned estimate (control/modes.rs).
4. **Sustained clipping without GPS.** Every window is excluded, the thrust-axis aid needs fresh GPS, and
   attitude falls back to gyro-only and drifts on residual bias.
5. **Thrust-axis aiding is biased by drag.** Holding station in wind needs tilt with a = 0, which the model
   reads as level. That is why its σ is loose beside valid updates, and it limits the `accel_sign_z`
   residual (2.8°).
6. **Measurement limits of this suite.**
   - Four seeds cannot resolve effects of a few percent. Several exam passes did not replicate on
     held-out seeds (iterations 4, 9, 15 and 17).
   - RMSE over surviving seeds is not monotone in survival: more survivors can read as a regression. A
     future metric should score failed seeds rather than drop them.
   - The pass thresholds were set before the baseline and were not changed. No scenario passed at
     baseline; 2 pass now (accel_saturation, nominal_hover).
7. **Not verified on hardware.**
   - The firmware changes compile (`cargo build --release`), host tests pass (447) and sim-baseline is
     unchanged.
   - The clip flag, the invariant warning and the thrust-axis path have not run on the board.
   - The 16 g saturation test (`i16::MAX`/`MIN`) assumes the driver's ±16 g configuration
     (drivers/icm42688.rs).

## Reproduce

```
cargo sim-eval --suite sim/scenarios                     # -> sim/latest.json
cargo sim-eval --compare sim/baseline.json sim/latest.json
cargo sim-eval sim/scenarios/gps_dropout_10s.toml --trace 2> trace.csv
scripts\sim-eval.cmd ...  /  scripts/sim-eval.sh ...    # same, from PowerShell / bash
```

The baseline is reproduced exactly by turning the accepted mechanisms off through overrides:

```
est.clip_skip_update = false
est.clip_kf_coast = false
est.sf_invariant_check = false
est.thrust_aid_sigma = 0
est.thrust_aid_always_sigma = 0
mekf.accel_r_rate_gain = 0
```
