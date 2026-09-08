#!/usr/bin/env bash
# test-sim-baseline.sh — Host sim still produces the committed numbers.
#
# Not Renode: runs sim_sweep --csv and diffs it against
# tests/sim-baseline.csv. Catches unintended changes to the sim or the
# controllers -- a 17% change to motor_tau moves 252 values.
#
# When a change IS intended, accept it in the same commit with:
#     BLESS=1 tests/run-tests.sh sim-baseline
#
# Tier: fast.  Included in tests/run-tests.sh --quick.

exec bash "$(dirname "${BASH_SOURCE[0]}")/renode-check.sh" sim-baseline
