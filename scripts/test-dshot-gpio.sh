#!/usr/bin/env bash
# test-dshot-gpio.sh — DShot programs PA0..PA3 for bidirectional output.
#
# Reads GPIOA back after DshotBitbang::new and asserts OUTPUT /
# PUSH_PULL / LOW_SPEED / PULL_UP. ~3 min: the motor-test firmware runs
# its 2.5 s REMOVE PROPS countdown first, which is not skippable.
# Needs the MOTOR-TEST ELF staged: scripts/stage-firmware.sh
#
# Tier: slow.  Skipped by --quick.

exec bash "$(dirname "${BASH_SOURCE[0]}")/renode-check.sh" dshot-gpio
