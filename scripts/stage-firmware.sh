#!/usr/bin/env bash
# stage-firmware.sh — build both firmware variants and put them where the
# Renode harness reads them.
#
# Four of the five Renode checks need a staged ELF and none of them build
# it, so this was a manual build-and-copy repeated on every firmware
# change. It is exactly the step that gets skipped: on 2026-09-08 the
# staged motor-test ELF was two weeks old, so both DShot checks had been
# passing against a build that predated the plant-capture work entirely.
#
# A stale ELF does not fail. It reports on the wrong firmware — the same
# hazard FC_BUILD_STAMP exists to catch on the bench.
#
#   scripts/stage-firmware.sh
#
# On Windows run this from GIT BASH, or use stage-firmware.cmd, which
# finds Git Bash and calls this. `bash` on PATH under PowerShell is WSL,
# which has no Rust toolchain.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE="$(cd "$REPO/.." && pwd)"
OUT="$REPO/target/thumbv7em-none-eabihf/release/fc-firmware"
DEST="$WORKSPACE/target/thumbv7em-none-eabihf/release"

if [ ! -d "$DEST" ]; then
    echo "!! Renode harness staging directory not found:"
    echo "   $DEST"
    echo "   These builds are only needed by the Renode checks, which live in"
    echo "   the workspace containing this checkout and are not part of this"
    echo "   repository. Nothing to stage."
    exit 2
fi

cd "$REPO"

# Order matters. Both variants write the same cargo output path, so
# motor-test is built FIRST and flight SECOND: finishing on flight leaves
# the repo's own target/ holding the build someone is most likely to copy
# by hand.
echo "==> building motor-test firmware"
cargo build --release --features motor-test
cp "$OUT" "$DEST/fc-firmware-motortest"

echo "==> building flight firmware"
cargo build --release
cp "$OUT" "$DEST/fc-firmware"

# The workspace root keeps a second copy of the flight ELF and its
# CLAUDE.md states the two are byte-identical. Keep that true here rather
# than leaving it to be noticed later.
cp "$OUT" "$WORKSPACE/fc-firmware"

echo
echo "staged: fc-firmware (flight), fc-firmware-motortest (bench)"
echo "run the checks with scripts/test-<name>.sh, or tests/run-tests.sh"
