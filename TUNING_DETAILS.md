
## Plant calibration and the flight log

If details are required, see TUNING_DETAILS.md
```
# Bench: motor step response -> motor_tau. PROPS ON, AIRCRAFT SECURED --
# the lag being measured is dominated by aerodynamic load, so a props-off
# capture measures the wrong system. Different warning text from the
# ordinary motor test, which wants props OFF.
scripts/plant-capture.sh              # build + flash; PLANT_RUNS=10 by default.
                                      # Opens a defmt reader in a new terminal,
                                      # logging to logs/plant_capture/.
scripts/plant-fit.sh logs/plant_capture/<time>.log   # fit; pools every run
                                      # (scripts\plant-fit.cmd on Windows)

# Flight: log to flash bank 2, survives reflashing (DFU writes bank 1).
scripts/blackbox-dump.sh              # RUN THIS FIRST on a new board: bank 2
                                      # has never been erased and its contents
                                      # are undefined until it has been.
scripts/blackbox-record.sh            # then fly this
scripts/blackbox-dump.sh              # then read it back, and it erases
scripts/plant-fit.sh flight.log
```

The capture repeats `PLANT_RUNS` times (1-50, default 10). Each dump
leaves the ESCs disarmed, so every later run re-arms them behind a 3 s
counted-down zero-throttle stream. After the last run the build stops DShot
output and parks; the ESCs stay disarmed until the board is power-cycled.

The reader is `scripts/defmt-log.sh`: USART6 at 115200 baud through a
USB-UART adapter, auto-detected or set with `SERIAL_DEV`. It runs
`defmt-print -e <elf> serial --path <dev> --baud <baud>`, the command
verified on the bench, so it needs a `defmt-print` with serial support
(`cargo install defmt-print`). Beside each log it keeps a copy of the ELF
it decoded with. `DEFMT_LOG=<subdir>` enables it for any flash script.

First capture: 2026-09-12, `docs/plant-capture-2026-09-12.log`, `motor_tau`
36 ms.

The capture build is selected by `PLANT_CAPTURE=1`, deliberately not
`PROFILE`: `flash-motor-test.sh` has a shell variable of that name for the
cargo profile, and assigning to an already-exported name keeps it
exported, so `PROFILE=1 scripts/flash-motor-test.sh` reached cargo as
`PROFILE=release` and silently turned the capture off.

Flashing needs `dfu-util` and `lsusb`, so it is a Debian job; only
`plant-fit` has a `.cmd` twin.

---

## MPC solve-time bench

```
scripts/mpc-bench.sh                  # build + flash; no motors driven
```

Times `AttitudeMpc` at prediction horizons 4 to 30, each at iteration caps
5, 10, 20 and 50, with the DWT cycle counter and the flight entry's core
configuration. One line per horizon and cap:

    MPC_BENCH,ph,ch,cap,construct_us,solve_min_us,solve_mean_us,solve_max_us,max_iters,unconverged,solves

Solves that never converge run to the cap, so size the period against the
cap's `solve_max_us`, not a typical solve.

A horizon is usable at a given MPC period only if `solve_max_us` fits inside
it with headroom for the rest of the navigation task. The bench runs with
nothing else active, so flight solves will be slower. Timings read zero in
Renode, which does not model the cycle counter.
