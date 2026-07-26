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
