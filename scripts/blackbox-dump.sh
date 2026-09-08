#!/usr/bin/env bash
# blackbox-dump.sh — build and flash the firmware that reads the flight
# log back out over defmt and then erases it.
#
#   scripts/blackbox-dump.sh
#
# This build DOES NOT FLY. It dumps at boot and halts, because the dump
# writes tens of thousands of lines through a logger that busy-waits
# inside a critical section, and an aircraft must not be armable while
# interrupts are held off for minutes. It links to ~27 KB, because the
# whole flight stack drops out.
#
# Flashing it does not destroy the log: DFU writes bank 1 and the log
# lives in bank 2. That is the entire point of the arrangement -- fly a
# recording build, then flash this and read back the flight you just did.
#
# Sequence:
#   1. scripts/blackbox-dump.sh          (flash this build)
#   2. capture the defmt output, e.g.
#        <your serial reader> | defmt-print -e \
#          target/thumbv7em-none-eabihf/release/fc-firmware | tee flight.log
#   3. scripts/plant-fit.sh flight.log
#   4. scripts/blackbox-record.sh        (back to a flying build)
#
# The erase happens as the LAST act, after defmt::flush(), so the log is
# not cleared until the dump has actually left the wire. Which means: do
# not power-cycle the board mid-dump if you want the data.
#
# ALSO RUN THIS ONCE BEFORE THE FIRST EVER RECORDING. Bank 2 has never
# been erased on a new board and its contents are undefined; if it is not
# blank, the append-point search finds a bogus offset or decides the log
# is full, and the flight records nothing. This build's erase leaves it
# clean. The dump beforehand will be empty or nonsense -- that is fine and
# expected.

set -euo pipefail

cd "$(dirname "$0")/.."

echo "==> blackbox DUMP build (does not fly; dumps, erases, halts)"
echo
echo "    After flashing, capture the defmt stream and then run:"
echo "      scripts/plant-fit.sh <log>"
echo "    Do not power-cycle mid-dump; the erase is last but the data is"
echo "    only yours once it is off the wire."
echo

FEATURES=blackbox-dump exec scripts/flash-dfu.sh "$@"
