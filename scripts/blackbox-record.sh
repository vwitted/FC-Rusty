#!/usr/bin/env bash
# blackbox-record.sh — build and flash the flight firmware with the
# blackbox enabled, so the next flight is logged to flash.
#
#   scripts/blackbox-record.sh
#
# This is the ordinary flight firmware plus logging: 200 Hz of motor
# commands, eRPM telemetry and gyro into flash bank 2, WHILE ARMED only.
# 143 s of log; it stops when full rather than wrapping.
#
# Read it back afterwards with scripts/blackbox-dump.sh. The log survives
# reflashing, because DFU writes bank 1 and the log lives in bank 2 --
# that is what makes "fly, then flash the dump build" work at all.
#
# BEFORE THE FIRST EVER RECORDING, run scripts/blackbox-dump.sh once. Bank
# 2 has never been erased on a new board, so its contents are undefined:
# if it is not blank, the append-point search finds a bogus offset or
# decides the log is already full, and the flight records nothing. The
# dump build erases as its last act, which leaves the region clean.
#
# Two things to watch in the log once it is running:
#
#   "control loop NNNN Hz measured"  -- the flash writes happen on the
#       same cooperative executor as the 8 kHz control loop, so they
#       block it for the duration of each write. This number is how you
#       find out whether that matters; it was never measured on hardware.
#
#   "blackbox: N samples dropped"    -- the staging channel overflowed,
#       meaning flash is not keeping up. The log has holes in it.

set -euo pipefail

cd "$(dirname "$0")/.."

echo "==> flight firmware WITH blackbox logging"
echo "    first time on this board? run scripts/blackbox-dump.sh first,"
echo "    to erase a bank that has never been erased."
echo

FEATURES=blackbox exec scripts/flash-dfu.sh "$@"
