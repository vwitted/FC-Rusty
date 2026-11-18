#!/usr/bin/env bash
# run-tests.sh — Renode-based checks for the FC-Rusty firmware.
#
# Renode's Monitor has no assertion language, so each .resc prints its
# findings between RESULT-BEGIN/RESULT-END markers and this script does the
# checking. Non-zero exit on any failure, so it drops into CI or a pre-flash
# habit unchanged.
#
# Usage:
#   renode/run-tests.sh              # everything (~5 min)
#   renode/run-tests.sh --quick      # fast checks only (~45 s) -- per-commit
#   renode/run-tests.sh boot         # only checks whose name matches
#   BLESS=1 renode/run-tests.sh sim-baseline   # accept new sim numbers
#
# The slow checks wait out the motor-test firmware's 2.5 s countdown, which is
# a safety feature (REMOVE PROPS) and is deliberately not made skippable.
#
# --console is not optional. Without it Renode hides the Monitor and swallows
# command errors, so a platform that failed to load looks exactly like one
# that booted and printed nothing.

set -uo pipefail
# The repo root is the CWD for every check: @-paths below (@renode/...,
# @target/renode/...) fall back to it, since the Renode install has neither.
cd "$(dirname "$0")/.."

# Renode binary. Defaults to the current nightly installed outside OneDrive
# (a 300 MB tree inside it would sync to the cloud). Override with RENODE=.
#
# That install MUST contain the dotfile `.renode-root`: Renode resolves @-paths
# relative to whichever directory holds that marker, and without it falls back
# to the CWD -- which silently changes which platform file you get. `cp -r dir/*`
# does not copy it.
RENODE="${RENODE:-$HOME/renode-current/renode.exe}"
if [ ! -x "$RENODE" ]; then
    echo "!! Renode not found at $RENODE (set RENODE=... to override)"
    exit 2
fi

QUICK=0
if [ "${1:-}" = "--quick" ]; then QUICK=1; shift; fi
FILTER="${1:-}"
PASS=0
FAIL=0
LOG=""

# Full Renode output is kept in $LOG so a check can look beyond the RESULT
# block. $LOG must be set by the CALLER: run_resc is invoked through process
# substitution, which runs in a subshell, so anything it assigns there is lost.
run_resc() {                       # run_resc <script> -> prints result values
    timeout 600 "$RENODE" --disable-xwt --console --plain "$1" 2>&1 \
        | head -c 400000 > "$LOG"
    sed -n '/RESULT-BEGIN/,/RESULT-END/p' "$LOG" | grep -oE '0x[0-9a-fA-F]+'
}

ok()   { printf '    ok    %-26s %s\n' "$1" "${2:-}"; PASS=$((PASS + 1)); }
bad()  { printf '    FAIL  %-26s %s\n' "$1" "${2:-}"; FAIL=$((FAIL + 1)); }

check() {                          # check <name> <actual> <expected>
    if [ "$2" = "$3" ]; then ok "$1" "$2"; else bad "$1" "got $2, want $3"; fi
}

want() {                           # want <elf> — is it staged for Renode?
    if [ ! -f "$1" ]; then
        echo "!! missing $1"
        echo "   stage it with scripts/stage-firmware.sh; see renode/README.md"
        exit 2
    fi
}

# matches <name> <tier>; tier "slow" is skipped under --quick.
matches() {
    if [ "${2:-fast}" = "slow" ] && [ "$QUICK" = "1" ]; then return 1; fi
    [ -z "$FILTER" ] || [[ "$1" == *"$FILTER"* ]]
}

# ------------------------------------------------------------- sim-baseline
# The host sim, not Renode. It lives here because this script is the
# documented per-commit gate and the check belongs where people already
# run things -- not in a build hook, which would either block every build
# for 10 s or report into a void nobody reads.
#
# Compares sim_sweep --csv against a committed baseline. See
# tests/check-sim-baseline.py for why a baseline rather than
# just running the sweep.
#
# Re-bless in the same commit as the change that moved the numbers -- that
# is the whole discipline the check exists to enforce.
if matches sim-baseline; then
    echo "==> sim-baseline: host sim still produces the committed numbers"
    if ! command -v cargo >/dev/null 2>&1; then
        echo "    skip  cargo not on PATH"
    else
        SIM_CSV="$(mktemp)"
        TRIPLE="$(rustc -vV | sed -n 's/^host: //p')"
        # --no-default-features drops the firmware feature so this builds
        # for the host; see CLAUDE.md.
        if (cargo run --release --example sim_sweep \
                --no-default-features --target "$TRIPLE" -q -- --csv) \
                > "$SIM_CSV" 2>/dev/null && [ -s "$SIM_CSV" ]; then
            if BLESS="${BLESS:-}" python tests/check-sim-baseline.py \
                    "$SIM_CSV" tests/sim-baseline.csv; then
                PASS=$((PASS + 1))
            else
                FAIL=$((FAIL + 1))
            fi
        else
            bad "sweep ran" "sim_sweep produced no CSV -- build failure?"
        fi
        rm -f "$SIM_CSV"
    fi
fi

# ---------------------------------------------------------------- boot-smoke
if matches boot-smoke; then
    echo "==> boot-smoke: flight firmware survives H743 bring-up"
    want target/renode/fc-firmware
    LOG="$(mktemp)"
    mapfile -t V < <(run_resc renode/checks/boot-smoke.resc)
    PC="${V[0]:-}"

    # The load-bearing assertion. A PC inside flash is NOT enough on its own:
    # the classic failure is the firmware spinning forever on PWR ACTVOSRDY
    # inside embassy_stm32::init(), which is also "in flash". defmt bytes
    # leaving USART6 prove logger::init_usart6() ran, so the clock tree came up.
    if grep -q "usart6:" "$LOG"; then
        ok "reached logger init" "defmt on the wire"
    else
        bad "reached logger init" "no UART output — stalled before logger init"
    fi

    if [ -z "$PC" ]; then
        bad "PC readable" "no result — platform or ELF failed to load"
    else
        DEC=$((PC))
        if [ "$DEC" -ge $((0x08000000)) ] && [ "$DEC" -lt $((0x08200000)) ]; then
            ok "PC in flash" "$PC"
        else
            bad "PC in flash" "$PC outside flash"
        fi
    fi
    rm -f "$LOG"
fi

# -------------------------------------------------------------- logger-uart
if matches logger-uart; then
    echo "==> logger-uart: defmt UART matches the clock tree it assumes"
    want target/renode/fc-firmware
    LOG="$(mktemp)"
    mapfile -t V < <(run_resc renode/checks/logger-uart.resc)
    if [ "${#V[@]}" -lt 2 ]; then
        bad "USART6 readable" "expected 2 values, got ${#V[@]}"
    else
        # BRR is the load-bearing one: logger.rs hard-codes APB2 = 120 MHz.
        check "USART6 BRR = 115200 baud" "$(printf '0x%x' $((${V[0]})))" "0x412"
        # Bit 7 (TXEIE) is masked off: logger.rs sets it while bytes are
        # queued for the drain interrupt, so its value depends on timing.
        check "USART6 CR1 = UE|TE"       "$(printf '0x%x' $((${V[1]} & ~0x80)))" "0x9"
    fi
    rm -f "$LOG"
fi

# ----------------------------------------------------------- flight-bringup
# boot-smoke passes even when main() is parked forever on a DMA await (the
# executor's WFE loop is in flash). This runs on platforms/local-overlay.resc
# (DMA that clears EN on completion, a real SPI1) and local-imu.resc
# (fixed-value ICM-42688-P on both buses, level rest), and asserts on the
# decoded defmt. The level MEKF seed guards the chip->body mounting
# convention: a flipped axis seeds at +-180 deg.
if matches flight-bringup; then
    echo "==> flight-bringup: flight firmware gets past sensor bring-up"
    want target/renode/fc-firmware
    if ! command -v defmt-print >/dev/null; then
        bad "defmt-print available" "cargo install defmt-print"
    else
        LOG="$(mktemp)"
        UART="$(mktemp)"
        # The UART file backend needs an absolute host path, and @-paths do not
        # expand $CWD, so the path is passed in as $uart. -e runs after any
        # script argument, so the .resc is included from -e, not passed.
        timeout 600 "$RENODE" --disable-xwt --console --plain \
            -e "\$uart=@$(cygpath -m "$UART"); include @renode/checks/flight-bringup.resc" 2>&1 \
            | head -c 400000 > "$LOG"
        TEXT="$(defmt-print -e target/renode/fc-firmware < "$UART" 2>&1)"
        if grep -q "There was an error" "$LOG"; then
            bad "overlay loaded" "$(grep -m1 -A1 'There was an error' "$LOG" | tail -1)"
        fi
        for line in "IMU1 \(SPI1, Roll180\) initialised OK" \
                    "IMU2 \(SPI4, Pitch180\) initialised OK" \
                    "MEKF seeded: euler=\[-?0\.0,-?0\.0," \
                    "Control loop starting"; do
            label="${line//\\/}"        # patterns are ERE; print them unescaped
            if grep -qE "$line" <<<"$TEXT"; then
                ok "reached: $label"
            else
                bad "reached: $label" "absent from defmt output"
            fi
        done
        rm -f "$LOG" "$UART"
    fi
fi

# --------------------------------------------------------------- dshot-gpio
if matches dshot-gpio slow; then
    echo "==> dshot-gpio: DShot programs PA0..PA3 for bidirectional output"
    echo "    (~3 min — the firmware's bench countdown runs first)"
    want target/renode/fc-firmware-motortest
    LOG="$(mktemp)"
    mapfile -t V < <(run_resc renode/checks/dshot-gpio.resc)
    if [ "${#V[@]}" -lt 4 ]; then
        bad "GPIOA readable" "expected 4 register values, got ${#V[@]}"
    else
        # Low byte only: bits above pin 3 belong to other peripherals.
        low() { printf '0x%02x' $(( $1 & 0xFF )); }
        check "MODER = OUTPUT"     "$(low "${V[0]}")" "0x55"
        check "OTYPER = PUSH_PULL" "$(low "${V[1]}")" "0x00"
        check "OSPEEDR = LOW"      "$(low "${V[2]}")" "0x00"
        check "PUPDR = PULL_UP"    "$(low "${V[3]}")" "0x55"
    fi
    rm -f "$LOG"
fi

# ------------------------------------------------------------------ esc-model
if matches esc-model slow; then
    echo "==> esc-model: a simulated ESC decodes the emulated DShot wire"
    echo "    (~100 s — runs to the firmware's arming phase)"
    want target/renode/fc-firmware-motortest
    LOG="$(mktemp)"
    timeout 600 "$RENODE" --disable-xwt --console --plain renode/checks/esc-sim.resc 2>&1         | head -c 400000 > "$LOG"
    # esc_model.py exits non-zero unless every motor yields a CRC-valid frame.
    if python renode/checks/esc_model.py --decode-log "$LOG" | sed 's/^/    /'; then
        PASS=$((PASS + 1))
    else
        FAIL=$((FAIL + 1))
    fi
    rm -f "$LOG"
fi

echo
echo "passed: $PASS   failed: $FAIL"
[ "$FAIL" -eq 0 ] || exit 1
