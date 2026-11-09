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
