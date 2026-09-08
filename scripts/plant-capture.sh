#!/usr/bin/env bash
# plant-capture.sh — build and flash the motor step-response capture.
#
# This is the bench measurement that identifies `motor_tau`: the firmware
# drives all four motors through a scripted profile of steps, records the
# eRPM reply from bidirectional DShot into RAM, stops the motors, and then
# dumps the capture over defmt. Feed that dump to scripts/plant-fit.sh.
#
#   scripts/plant-capture.sh
#
# ---------------------------------------------------------------------
# PROPS ON. This is the one bench build that needs them.
#
# The ordinary motor test says REMOVE PROPS. This one is the opposite,
# and the reason is not convenience: the lag being measured is dominated
# by the aerodynamic load on the prop, so a props-off capture measures a
# real time constant for the wrong system and the number is useless.
#
# So this run spins four loaded props to 25% on a script, with no
# operator in the loop once it starts. SECURE THE AIRCRAFT. The firmware
# prints "PLANT CAPTURE — PROPS ON, AIRCRAFT SECURED" and counts down
# five seconds before the first frame; if you see the motor-test wording
# instead ("REMOVE PROPS"), the capture is NOT enabled and something in
# this script has gone wrong -- stop and check rather than proceeding.
# ---------------------------------------------------------------------
#
# Note the variable is PLANT_CAPTURE, not PROFILE. flash-motor-test.sh
# has a shell variable named PROFILE for the cargo profile, and assigning
# to an already-exported name keeps it exported -- so `PROFILE=1` reached
# cargo as PROFILE=release and silently disabled the capture while the
# operator had already fitted props for it.

set -euo pipefail

cd "$(dirname "$0")/.."

echo "=============================================================="
echo " PLANT CAPTURE — this build spins PROPS. Secure the aircraft."
echo "=============================================================="
echo

# BIDIR=1 is not optional here: the whole measurement is the eRPM reply.
# Without it the firmware still flies the profile and records nothing,
# and says so, but that is a wasted props-on run.
export PLANT_CAPTURE=1
export BIDIR=1

exec scripts/flash-motor-test.sh "$@"
