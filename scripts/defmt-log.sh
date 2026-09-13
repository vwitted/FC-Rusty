#!/usr/bin/env bash
# defmt-log.sh — read the board's defmt log from a USB-UART adapter,
# decode it, show it and save it.
#
#   scripts/defmt-log.sh <elf> <log-file>
#       Read in this terminal until Ctrl-C.
#   scripts/defmt-log.sh --launch <elf> <subdir>
#       Choose logs/<subdir>/<timestamp>.log, open a new terminal running
#       the mode above, and print the log path on stdout. The flash scripts
#       call this when DEFMT_LOG is set, after DFU is confirmed and before
#       the write, so the reader is listening when the board reboots.
#
# The firmware logs on USART6 TX (the T6 pad) at 115200 baud, 8N1. The
# stream is binary defmt, decoded against the ELF that was flashed; a
# stale ELF produces plausible-looking nonsense rather than an error. So
# each log keeps a copy of the ELF it was decoded with and the raw bytes
# (<log>.elf, <log>.raw), which is enough to decode it again later.
#
# Environment:
#   SERIAL_DEV   adapter device. Default: the only /dev/ttyUSB* or
#                /dev/ttyACM* present; none or several is an error.
#   DEFMT_BAUD   default 115200.
#
# Requires defmt-print (cargo install defmt-print). Debian: the flash
# scripts that call this need dfu-util and lsusb.

set -euo pipefail

cd "$(dirname "$0")/.."

BAUD="${DEFMT_BAUD:-115200}"

resolve_dev() {
  if [ -n "${SERIAL_DEV:-}" ]; then
    if [ ! -e "${SERIAL_DEV}" ]; then
      echo "!! SERIAL_DEV=${SERIAL_DEV} does not exist" >&2
      return 1
    fi
    echo "${SERIAL_DEV}"
    return 0
  fi
  local devs=() d
  for d in /dev/ttyUSB* /dev/ttyACM*; do
    [ -e "$d" ] && devs+=("$d")
  done
  if [ "${#devs[@]}" -ne 1 ]; then
    echo "!! found ${#devs[@]} serial adapters (${devs[*]:-none}); set SERIAL_DEV" >&2
    return 1
  fi
  echo "${devs[0]}"
}

need_defmt_print() {
  if ! command -v defmt-print >/dev/null; then
    echo "!! defmt-print not found; install it with: cargo install defmt-print" >&2
    return 1
  fi
}

# ---- --launch: open the reader in a new terminal -------------------------
if [ "${1:-}" = "--launch" ]; then
  ELF="${2:?usage: scripts/defmt-log.sh --launch <elf> <subdir>}"
  SUBDIR="${3:?usage: scripts/defmt-log.sh --launch <elf> <subdir>}"
  need_defmt_print
  DEV="$(resolve_dev)"
  [ -f "${ELF}" ] || { echo "!! no ELF at ${ELF}" >&2; exit 2; }
  mkdir -p "logs/${SUBDIR}"
  LOG="logs/${SUBDIR}/$(date +%Y%m%d-%H%M%S).log"

  RUN="cd $(printf %q "$PWD") && SERIAL_DEV=$(printf %q "$DEV") DEFMT_BAUD=$(printf %q "$BAUD") scripts/defmt-log.sh $(printf %q "$ELF") $(printf %q "$LOG")"
  # Trapped rather than ignored: children reset a trapped signal to its
  # default, so Ctrl-C still stops the reader while this shell survives to
  # keep the window open for reading.
  INNER="trap 'true' INT; ${RUN}; echo; read -r -p 'Reader stopped. Enter closes this window. ' _"

  launched=""
  if [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    if command -v gnome-terminal >/dev/null; then
      gnome-terminal -- bash -c "${INNER}" >/dev/null 2>&1 && launched=gnome-terminal
    elif command -v konsole >/dev/null; then
      konsole -e bash -c "${INNER}" >/dev/null 2>&1 & launched=konsole
    elif command -v xfce4-terminal >/dev/null; then
      xfce4-terminal -x bash -c "${INNER}" >/dev/null 2>&1 & launched=xfce4-terminal
    elif command -v x-terminal-emulator >/dev/null; then
      x-terminal-emulator -e bash -c "${INNER}" >/dev/null 2>&1 & launched=x-terminal-emulator
    elif command -v xterm >/dev/null; then
      xterm -e bash -c "${INNER}" >/dev/null 2>&1 & launched=xterm
    fi
  fi

  if [ -n "${launched}" ]; then
    echo "==> defmt reader opened in a new ${launched} window: ${DEV} -> ${LOG}" >&2
  else
    # No desktop terminal (for example over SSH): log in the background.
    nohup bash -c "${RUN}" >/dev/null 2>&1 &
    echo "==> no terminal emulator found; defmt reader running in the background" >&2
    echo "    follow it with: tail -f ${LOG}" >&2
  fi
  # Let the reader configure the port before the board reboots.
  sleep 1
  echo "${LOG}"
  exit 0
fi

# ---- foreground: read until Ctrl-C ---------------------------------------
ELF="${1:?usage: scripts/defmt-log.sh <elf> <log-file>}"
LOG="${2:?usage: scripts/defmt-log.sh <elf> <log-file>}"
need_defmt_print
DEV="$(resolve_dev)"
[ -f "${ELF}" ] || { echo "!! no ELF at ${ELF}" >&2; exit 2; }
mkdir -p "$(dirname "${LOG}")"

BASE="${LOG%.log}"
cp "${ELF}" "${BASE}.elf"

stty -F "${DEV}" "${BAUD}" raw -echo -ixon -ixoff cs8 -cstopb -parenb

{
  echo "# defmt log $(date -Iseconds)"
  echo "# device ${DEV} at ${BAUD} baud"
  echo "# build stamp $(cat target/build-stamp.txt 2>/dev/null || echo unknown)"
  echo "# decoded with ${BASE}.elf; raw bytes in ${BASE}.raw"
} | tee "${LOG}"
echo "==> reading ${DEV}; Ctrl-C to stop"

cat "${DEV}" | tee "${BASE}.raw" | defmt-print -e "${BASE}.elf" | tee -a "${LOG}"
