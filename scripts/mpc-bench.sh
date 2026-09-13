#!/usr/bin/env bash
# mpc-bench.sh — build and flash the MPC solve-time bench.
#
#   scripts/mpc-bench.sh
#
# Times AttitudeMpc at a menu of horizons (4 to 30 prediction steps), each
# at iteration caps 5, 10, 20 and 50, on the board with the DWT cycle
# counter. Logs one MPC_BENCH line per horizon and cap, then halts. It
# never initialises DShot, so no motor can spin and props do not matter.
#
# The table is the feasibility data for choosing the MPC horizon and rate:
# a horizon is usable at a given MPC period only if solve_max_us fits inside
# that period with headroom for the rest of the navigation task. See
# docs/2026-09-12-sim-direction.md.
#
# A defmt reader opens in a new terminal once DFU is confirmed, logging to
# logs/mpc_bench/ (DEFMT_LOG= skips it). Reflash a flight or motor-test
# build afterwards; this build does nothing else.

set -euo pipefail

cd "$(dirname "$0")/.."

export DEFMT_LOG="${DEFMT_LOG-mpc_bench}"

echo "==> MPC solve-time bench (no motors driven; halts after the table)"
echo

FEATURES=mpc-bench exec scripts/flash-dfu.sh "$@"
