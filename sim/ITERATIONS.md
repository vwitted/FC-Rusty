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
