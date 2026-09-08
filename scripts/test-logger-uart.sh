#!/usr/bin/env bash
# test-logger-uart.sh — defmt UART matches the clock tree it assumes.
#
# logger.rs hard-codes APB2 = 120 MHz, so USART6 BRR is the load-bearing
# value: if the clock tree changes underneath it the log silently becomes
# garbage at the wrong baud. Needs the flight ELF staged.
#
# Tier: fast.  Included in tests/run-tests.sh --quick.

exec bash "$(dirname "${BASH_SOURCE[0]}")/renode-check.sh" logger-uart
