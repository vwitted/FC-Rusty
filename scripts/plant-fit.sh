#!/usr/bin/env bash
# plant-fit.sh — turn a capture log into a motor_tau.
#
#   scripts/plant-fit.sh capture.log
#   cat capture.log | scripts/plant-fit.sh
#   POLE_PAIRS=6 scripts/plant-fit.sh capture.log
#
# `capture.log` is whatever defmt-print emitted while the board dumped its
# capture -- either the bench profile (scripts/plant-capture.sh) or a
# flight blackbox dump (scripts/blackbox-dump.sh). Lines that are not
# samples are ignored, so piping a whole session in is fine.
#
# Exists because the invocation is not guessable. Examples are HOST
# binaries, so they need --no-default-features (to drop the firmware
# feature and its Embassy/cortex-m deps) and an explicit --target (because
# .cargo/config.toml pins thumbv7em, and without it you get "can't find
# crate for `std`"). Same reasoning as test-host.sh, which reads the
# triple from rustc for the same reason.
#
# POLE_PAIRS defaults to 7, which is right for the 12N14P outrunners a 5"
# quad almost always uses. It CANCELS in a time constant, so getting it
# wrong costs nothing in motor_tau -- it only rescales the reported RPM
# columns.

set -euo pipefail

cd "$(dirname "$0")/.."

TRIPLE="$(rustc -vV | sed -n 's/^host: //p')"
if [ -z "$TRIPLE" ]; then
    echo "!! could not read the host triple from rustc -vV" >&2
    exit 2
fi

if [ $# -ge 1 ] && [ ! -f "$1" ]; then
    echo "!! no such log: $1" >&2
    echo "   Capture one first — see scripts/plant-capture.sh" >&2
    exit 2
fi

# Build quietly first so a compile error is not interleaved with the
# report, then run. `--` separates cargo's arguments from the tool's.
cargo build --release --example fit_plant --no-default-features \
    --target "$TRIPLE" -q

exec cargo run --release --example fit_plant --no-default-features \
    --target "$TRIPLE" -q -- "$@"
