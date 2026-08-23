#!/usr/bin/env bash
# test-host.sh — run the host-side unit tests on whichever machine you're on.
#
# These tests run as a *native* binary, so they must be built for the host,
# not for the firmware target. `.cargo/config.toml` sets
# `[build] target = "thumbv7em-none-eabihf"`, so without an explicit
# `--target` cargo happily builds ARM test binaries and then fails trying to
# execute them. The correct triple differs per machine —
# x86_64-unknown-linux-gnu on the Debian box, x86_64-pc-windows-msvc on the
# Windows one — so ask rustc instead of hardcoding it.
#
# `--no-default-features` drops the `firmware` feature, keeping the Embassy /
# cortex-m / defmt deps out of a host build.
#
# Usage:
#   scripts/test-host.sh                    # all host tests
#   scripts/test-host.sh persist::record    # only tests matching a filter
#   scripts/test-host.sh -- --nocapture     # pass flags to the test harness
#
# On Windows use scripts/test-host.cmd — it locates Git Bash and calls this
# script. Do not use `bash` from PowerShell: that resolves to WSL, a separate
# Linux environment that cannot see the Windows Rust toolchain.

set -euo pipefail

cd "$(dirname "$0")/.."

HOST_TRIPLE="$(rustc -vV | sed -n 's/^host: //p')"

if [ -z "${HOST_TRIPLE}" ]; then
  echo "!! could not determine the host triple from 'rustc -vV'" >&2
  exit 1
fi

echo "==> cargo test --lib on ${HOST_TRIPLE}"
cargo test --lib --no-default-features --target "${HOST_TRIPLE}" "$@"
