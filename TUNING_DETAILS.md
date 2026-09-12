
## Plant calibration and the flight log

If details are required, see TUNING_DETAILS.md
```
# Bench: motor step response -> motor_tau. PROPS ON, AIRCRAFT SECURED --
# the lag being measured is dominated by aerodynamic load, so a props-off
# capture measures the wrong system. Different warning text from the
# ordinary motor test, which wants props OFF.
scripts/plant-capture.sh              # build + flash the capture firmware
scripts/plant-fit.sh capture.log      # fit it (scripts\plant-fit.cmd on Windows)

# Flight: log to flash bank 2, survives reflashing (DFU writes bank 1).
scripts/blackbox-dump.sh              # RUN THIS FIRST on a new board: bank 2
                                      # has never been erased and its contents
                                      # are undefined until it has been.
scripts/blackbox-record.sh            # then fly this
scripts/blackbox-dump.sh              # then read it back, and it erases
scripts/plant-fit.sh flight.log
```

After the dump the capture build stops DShot output and parks; the ESCs
stay disarmed until the board is power-cycled. First capture: 2026-09-12,
`docs/plant-capture-2026-09-12.log`, `motor_tau` 36 ms.

The capture build is selected by `PLANT_CAPTURE=1`, deliberately not
`PROFILE`: `flash-motor-test.sh` has a shell variable of that name for the
cargo profile, and assigning to an already-exported name keeps it
exported, so `PROFILE=1 scripts/flash-motor-test.sh` reached cargo as
`PROFILE=release` and silently turned the capture off.

Flashing needs `dfu-util` and `lsusb`, so it is a Debian job; only
`plant-fit` has a `.cmd` twin.

---