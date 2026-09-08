#!/usr/bin/env bash
# renode-check.sh — run one check from the Renode harness by name.
#
# Shared by the per-check scripts beside it, so the path resolution and
# the "harness is missing" message exist once rather than five times.
#
#   scripts/renode-check.sh boot-smoke
#   scripts/renode-check.sh sim-baseline
#
# The harness is NOT in this repository. It lives in the Renode workspace
# that contains this checkout, one level up, and is not version-controlled
# — see the workspace CLAUDE.md. So a standalone clone of FC-Rusty-Code
# cannot run these, and saying so plainly beats a confused "no such file"
# from somewhere three scripts deep.
#
# Note for Windows users: run this from GIT BASH. `bash` on PATH under
# PowerShell is WSL, a separate Linux environment with no Renode and no
# Rust toolchain. The .cmd scripts beside these exist to find Git Bash for
# you.

set -uo pipefail

if [ $# -lt 1 ]; then
    echo "usage: renode-check.sh <check-name>"
    echo "  boot-smoke  logger-uart  sim-baseline  dshot-gpio  esc-model"
    exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "$SCRIPT_DIR/../.." && pwd)"
RUNNER="$WORKSPACE/tests/run-tests.sh"

if [ ! -f "$RUNNER" ]; then
    echo "!! Renode harness not found at $RUNNER"
    echo "   These checks need the Renode workspace that contains this"
    echo "   checkout; it is not part of this repository. A standalone"
    echo "   clone can still run the host tests: scripts/test-host.sh"
    exit 2
fi

exec bash "$RUNNER" "$1"
