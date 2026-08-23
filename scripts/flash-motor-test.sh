#!/usr/bin/env bash
# flash-dfu.sh — build the firmware and flash it to the DAKEFPVH743
# via USB DFU. The board has no SWD pads, so this is the intended flow.
#
# Prerequisites (one-off per machine):
#   rustup component add llvm-tools
#   cargo install cargo-binutils
#   sudo apt install dfu-util         # Debian/Ubuntu
#   # or:   brew install dfu-util    # macOS
#
# Usage:
#   scripts/flash-dfu.sh              # release build, then flash
#   scripts/flash-dfu.sh --debug      # debug build (larger, slower)
#
# Bench knobs (compile-time, so set them on this script's invocation):
#   MOTOR_PIN_ORDER=NNNN scripts/flash-motor-test.sh
#       Motor -> pad mapping, one digit per motor; 0 means that motor is not
#       driven at all (its pad stays high-Z on its pull-up, so that ESC sees
#       no command and never replies). Must not use a pad twice, and must
#       leave at least one motor driven; anything malformed is a compile
#       error, not a silent fallback.
#           1234  default, all four
#           4231  swap M1 and M4 -- pair this with a physical ESC lead swap
#           0004  drive M4 alone, to tell "this ESC never answers" apart
#                 from "its answer is swamped by its neighbours"
#   ONLY_MOTOR=4 scripts/flash-motor-test.sh
#       Shorthand for the isolation case above; exactly MOTOR_PIN_ORDER=0004.
#       Setting both is an error.
#   RX_SAMPLES=400 LOOP_KHZ=2 scripts/flash-motor-test.sh
#       Widen the telemetry capture window to catch a reply that lands after
#       the default 140 samples. Never for flight: 400 samples is 178 us and
#       blows the 125 us loop period, hence the paired LOOP_KHZ.
#
# To put the board in DFU mode: hold BOOT, plug USB-C, release BOOT.
# Verify with `lsusb | grep STMicroelectronics` — you should see the
# "STM Device in DFU Mode" VID:PID 0483:df11 enumerate.
#
# After flashing, `:leave` auto-reboots the MCU into the freshly-
# written firmware, so you do NOT need to power-cycle.

set -euo pipefail

cd "$(dirname "$0")/.."

PROFILE="release"
PROFILE_FLAG="--release"
for arg in "$@"; do
  case "$arg" in
    --debug) PROFILE="debug";   PROFILE_FLAG="" ;;
    --release) PROFILE="release"; PROFILE_FLAG="--release" ;;
    *) echo "unknown arg: $arg" >&2; exit 2 ;;
  esac
done

ELF="target/thumbv7em-none-eabihf/${PROFILE}/fc-firmware"
BIN="target/thumbv7em-none-eabihf/${PROFILE}/fc-firmware.bin"

echo "==> cargo objcopy (${PROFILE})"
cargo objcopy ${PROFILE_FLAG} --features motor-test --bin fc-firmware -- -O binary "${BIN}"

if ! lsusb | grep -q "0483:df11"; then
  echo
  echo "!! STM32 DFU device (0483:df11) not seen on USB."
  echo "   Hold BOOT on the FC, plug in USB-C, release BOOT, and re-run."
  exit 1
fi

STAMP="$(cat target/build-stamp.txt 2>/dev/null || echo unknown)"
SHA="$(sha256sum "${BIN}" 2>/dev/null | cut -c1-16 || shasum -a 256 "${BIN}" | cut -c1-16)"

echo "==> dfu-util flashing ${BIN} (size: $(stat -c %s "${BIN}" 2>/dev/null || stat -f %z "${BIN}") bytes)"
dfu-util -a 0 -s 0x08000000:leave -D "${BIN}"

echo "==> flashed; MCU should have rebooted into the new firmware"
echo
echo "    build stamp : ${STAMP}"
echo "    binary sha  : ${SHA}"
echo
echo "    The firmware logs 'DShot build: [<stamp>]' at init. If that does not"
echo "    match the stamp above, the board is running older firmware."
