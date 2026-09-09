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
