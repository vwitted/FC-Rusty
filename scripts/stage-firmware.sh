#!/usr/bin/env bash
# stage-firmware.sh — build both firmware variants and put them where the
# Renode harness reads them (target/renode/, gitignored with target/).
#
# Five of the Renode checks need a staged ELF and none of them build it.
# It is exactly the step that gets skipped: on 2026-09-08 the staged
# motor-test ELF was two weeks old, so both DShot checks had been passing
# against a build that predated the plant-capture work entirely.
#
# A stale ELF does not fail. It reports on the wrong firmware — the same
# hazard FC_BUILD_STAMP exists to catch on the bench.
#
# Staged copies rather than cargo's own output path, because both variants
# write that path: a later `cargo build --features motor-test` would
# otherwise silently replace the flight ELF under test.
#
#   scripts/stage-firmware.sh
#
# On Windows run this from GIT BASH, or use stage-firmware.cmd, which
# finds Git Bash and calls this. `bash` on PATH under PowerShell is WSL,
# which has no Rust toolchain.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/.." && pwd)"
OUT="$REPO/target/thumbv7em-none-eabihf/release/fc-firmware"
DEST="$REPO/target/renode"

cd "$REPO"
mkdir -p "$DEST"

# Motor-test first and flight second, so cargo's own output path is left
# holding the flight build, the one most likely to be flashed by hand.
echo "==> building motor-test firmware"
cargo build --release --features motor-test
cp "$OUT" "$DEST/fc-firmware-motortest"

echo "==> building flight firmware"
cargo build --release
cp "$OUT" "$DEST/fc-firmware"

echo
echo "staged in target/renode/: fc-firmware (flight), fc-firmware-motortest (bench)"
echo "run the checks with scripts/test-<name>.sh, or renode/run-tests.sh"
