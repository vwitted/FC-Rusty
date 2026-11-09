# Estimator tuning log

Every candidate, accepted or not. Each row is a full-suite run of
`cargo sim-eval --suite sim/scenarios` compared with
`--compare sim/latest.json <candidate> --baseline sim/baseline.json`
(rule in examples/sim_eval.rs: at least one metric improves beyond
max(1 %, abs eps), none regresses beyond it against either the current best
or the frozen baseline). Accepted candidates are committed one per commit and
`sim/latest.json` is updated; `sim/baseline.json` is never rewritten.

| # | Candidate | Verdict | Notes |
|---|-----------|---------|-------|
| 1 | Accel window mean (boxcar anti-alias over the 80-sample decimation window), MEKF + PosKF | REJECT | sat att 15.6->11.5, airmode att 4.77->3.86; regressed gps_dropout att 7.17->7.37, alt 0.485->0.504 and sat alt 4.32->4.41. Dropout score is one surviving seed of four: noise-dominated. |
| 2 | MEKF accel NIS gate (y'S^-1y > G rejects, release after N), scanned G 9..100, N 50..300 | REJECT | Never fires on accel_sign_x/y (a static inversion is tracked as a consistent mirror image, innovation stays small); at G<=50 it starves the update and diverges saturation/dropout. Best (G=100) only regressed sat alt 4.32->4.41. |
| 3 | Clip-aware estimation: skip the MEKF accel update and coast the PosKF (zero accel, sigma_a 16) when any raw sample in the window hit full scale. Coast sigma scanned 2..24 | **ACCEPT** | accel_saturation div 2->0, att 15.61->6.72, alt 4.32->0.32. Plateau 12..24 all 0/4 diverged; 2 and 6 diverge (coasting too confident). Only fires under clipping, so no other scenario moves. |
| 4 | Inflate MEKF accel R while GPS reference is stale: (a) whenever !is_fresh -- (b) only after GPS was fresh then lost. sigma 0.12..2.0 | REJECT | (a) moved airmode/sign scenarios via the 0.2 s before the first two fixes. (b) passes the rule at 0.12/0.16 but is non-monotonic in sigma; held-out seeds 5-16: divergence 10/12 -> 8..9/12 with no trend. Rejected as noise, not mechanism. |
| 5 | While GPS is lost in flight, fuse a zero-velocity pseudo-measurement into the PosKF at 10 Hz, sigma 0.5..4 m/s | REJECT | Exam: passes at 1/2/4 m/s (small). Held-out seeds 5-16: divergence 8..10/12 vs 10/12, no trend. Trace (--trace) shows why: after the GPS compensation decays the accel update pulls the MEKF to the THRUST axis (a multirotor accelerometer reads thrust+drag, not gravity), truth reaches -10/-19 deg while the estimate reads ~0, and the loop flies away. Tilt is unobservable from the accel here; velocity aiding cannot fix it. |
| 6 | Multirotor specific-force invariant: window-mean body-z > +0.5 g is a fault (thrust is along body -z in every orientation); skip + coast as for clipping | REJECT (deferred) | accel_sign_z 4/4 diverged -> 0/4, att null -> 6.72, alt null -> 0.22. Blocked by accel_saturation alt 0.317 -> 0.342: the invariant also fires on 1 window/seed with no raw clip, because the 25 Hz front-end LPF carries clipped bias into the next window -- the clip flag misses it. Retried after iteration 7. |
| 7 | Clip hold-off: treat 1/2/4 windows after a clipped one as clipped (front-end LPF memory) | REJECT | Bit-identical suite: no scenario has an unclipped window following a clipped one. The stray implausible window of iteration 6 is the FIRST sample (LPF primed on a +2.7 g vibration peak), not LPF carry-over. |
| 8 | Specific-force invariant with persistence: exclude the accel (skip + coast) after N consecutive windows with body-z mean > +0.5 g. N scanned 2/3/5/10 | **ACCEPT** (N=3) | accel_sign_z div 4->0, att null->6.73, alt null->0.22. N=2 similar, 5 worse att, 10 recovers late (38 deg). Persistence removes the startup-sample trigger that blocked iteration 6. Firmware logs a defmt warning when the fault is declared. |
