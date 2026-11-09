## Unit Tests
 Host unit tests (strips firmware feature so cortex-m isn't pulled in).
# The tests run as a native binary, so they need the host triple, not the
# thumbv7em default from .cargo/config.toml. These wrappers read it from
# `rustc -vV`, so the same command works on Debian and on Windows.
scripts/test-host.sh                    # Debian, or Git Bash on Windows
scripts\test-host.cmd                   # Windows PowerShell / cmd
scripts/test-host.sh persist::record    # filter to one module or test

##Sim Tests
# Simulation examples. Examples are HOST binaries, so they need the host
# triple for the same reason the tests do -- .cargo/config.toml pins
# [build] target = thumbv7em, and without --target you get
# "can't find crate for `std`".
TRIPLE=$(rustc -vV | sed -n 's/^host: //p')
cargo run --release --example sim_mpc_hover --no-default-features --target $TRIPLE
cargo run --release --example sim_sweep    --no-default-features --target $TRIPLE
cargo run --release --example sim_sweep    --no-default-features --target $TRIPLE -- --csv

# sim_sweep runs the full cascade across a grid of sensor degradations
# (gyro noise, bias, vibration, intermittency, motor asymmetry), 8 seeds per
# case, and reports att_rms / att_max / alt_rms / air% / failures. The whole
# sweep is well under a second -- it is the host sim, not the emulator, and
# is the right tool for degradation statistics. See src/sim/degrade.rs.
```

## Estimator evaluation (sim-eval)
```
cargo sim-eval sim/scenarios/nominal_hover.toml     # one scenario -> one JSON object
cargo sim-eval --suite sim/scenarios --detail       # all scenarios; fault evidence on stderr
cargo sim-eval --compare sim/latest.json new.json --baseline sim/baseline.json
scripts/sim-eval.sh ... / scripts\sim-eval.cmd ...  # same, from bash / PowerShell
```
`cargo sim-eval` is an alias (.cargo/config.toml) using `--target host-tuple`,
so it is identical in both shells. The loop (src/sim/est_eval.rs) runs the
firmware estimator wiring end to end -- MEKF, PosKF, gps_accel, IMU front-end
filters -- and closes altitude and position on the estimates. Metric
definitions and the acceptance rule are in examples/sim_eval.rs; results and
history in sim/.
