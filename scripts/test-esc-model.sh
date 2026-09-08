#!/usr/bin/env bash
# test-esc-model.sh — A simulated ESC decodes the emulated DShot wire.
#
# tests/esc_model.py decodes DShot off the BSRR write stream and checks
# all four motors produce a CRC-valid zero-throttle arming frame. ~100 s.
# Decodes from write ORDER, never timestamps -- Renode runs the whole DMA
# transfer in zero virtual time, so intervals here are meaningless.
# Needs the MOTOR-TEST ELF staged.
#
# Tier: slow.  Skipped by --quick.

exec bash "$(dirname "${BASH_SOURCE[0]}")/renode-check.sh" esc-model
