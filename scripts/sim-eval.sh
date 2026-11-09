#!/usr/bin/env bash
# sim-eval.sh — headless estimator evaluation (examples/sim_eval.rs).
#
#   scripts/sim-eval.sh sim/scenarios/nominal_hover.toml
#   scripts/sim-eval.sh --suite sim/scenarios [--params cand.toml] [--detail]
#   scripts/sim-eval.sh --compare sim/latest.json new.json --baseline sim/baseline.json
#
# Thin wrapper over the `cargo sim-eval` alias (.cargo/config.toml), which
# selects the host target with `host-tuple`. scripts/sim-eval.cmd is the
# PowerShell / cmd twin; it needs no Git Bash.
set -euo pipefail
cd "$(dirname "$0")/.."
exec cargo sim-eval "$@"
