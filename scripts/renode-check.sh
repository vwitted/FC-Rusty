#!/usr/bin/env bash
# renode-check.sh — run one check from the Renode harness by name.
#
# Shared by the per-check scripts beside it, so the path resolution exists
# once rather than once per check.
#
#   scripts/renode-check.sh boot-smoke
#   scripts/renode-check.sh sim-baseline
#
# The harness is renode/ in this repository; see renode/README.md. The
# ELF-dependent checks need scripts/stage-firmware.sh to have run.
#
# Note for Windows users: run this from GIT BASH. `bash` on PATH under
# PowerShell is WSL, a separate Linux environment with no Renode and no
# Rust toolchain. The .cmd scripts beside these exist to find Git Bash for
# you.

set -uo pipefail

if [ $# -lt 1 ]; then
    echo "usage: renode-check.sh <check-name>"
    echo "  boot-smoke  logger-uart  flight-bringup  sim-baseline  dshot-gpio  esc-model"
    exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNNER="$SCRIPT_DIR/../renode/run-tests.sh"

exec bash "$RUNNER" "$1"
