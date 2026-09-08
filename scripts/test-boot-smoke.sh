#!/usr/bin/env bash
# test-boot-smoke.sh — Flight firmware survives H743 bring-up.
#
# Asserts defmt bytes leave USART6 (proving the clock tree came up,
# not merely that the PC is somewhere in flash) and that the PC is in
# the flash window. Needs the flight ELF staged: scripts/stage-firmware.sh
#
# Tier: fast.  Included in tests/run-tests.sh --quick.

exec bash "$(dirname "${BASH_SOURCE[0]}")/renode-check.sh" boot-smoke
