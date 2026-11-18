#!/usr/bin/env bash
# test-flight-bringup.sh — Flight firmware gets through sensor bring-up.
#
# Runs the flight ELF on the workspace's local overlay (DMA that clears EN
# on completion, a real SPI1) with fixed-value ICM-42688-P models on SPI1
# and SPI4 at level rest. Asserts, from decoded defmt, that both IMUs
# initialise, the MEKF seeds at roll = pitch = 0, and the control loop
# starts. The level seed guards the chip->body mounting convention in
# src/drivers/orientation.rs. Catches what boot-smoke cannot: main()
# parked forever on an await, with the PC idling in flash.
#
# Needs the flight ELF staged (scripts/stage-firmware.sh) and defmt-print.
# Tier: fast.  Included in tests/run-tests.sh --quick.

exec bash "$(dirname "${BASH_SOURCE[0]}")/renode-check.sh" flight-bringup
