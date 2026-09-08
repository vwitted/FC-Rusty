#!/usr/bin/env python3
"""Compare a sim_sweep --csv run against the committed baseline.

Why a baseline rather than "run the sims on build".

Running the sweep produces a 97-row table. Nobody reads a 97-row table on
every build, and a check nobody reads is worse than no check -- it makes
"green" mean "I did not look". The useful signal is not the table, it is
the DIFFERENCE from last time: which rows moved, and by how much.

That turns the sweep into a regression test for control and sim
behaviour, which is what it is actually good for. Changing the motor
model on 2026-09-08 moved every altitude number in the file by 10-90x;
that was found by running the sweep by hand before and after, which is
exactly the manual work this replaces.

Tolerance: the sweep is bit-identical run-to-run on one host (verified),
so exact comparison would work there. It is NOT guaranteed identical
across hosts -- this project builds on both Windows and Debian, and libm
differs in the last ulp. 0.1% relative is far tighter than any
behavioural change worth catching (the smallest real one so far was a
factor of two) and far looser than float noise.

Usage:
    check-sim-baseline.py <new.csv> <baseline.csv> [--tol 0.001]
    BLESS=1 ...   rewrites the baseline instead of comparing
"""

import csv
import os
import sys

# Columns compared with a relative tolerance. `value` is part of the key.
FLOAT_COLS = ["att_rms", "att_max", "alt_rms", "pos_rms", "air_frac"]
# Counts must match exactly: a run that used to fail 0/8 and now fails 1/8
# is a real change no matter how small the float columns moved.
INT_COLS = ["failures", "seeds", "diverged", "crashed", "flyaway", "nonfinite"]


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    keyed = {}
    for r in rows:
        keyed[(r["axis"], r["value"])] = r
    if len(keyed) != len(rows):
        sys.exit(f"!! {path}: duplicate (axis, value) keys -- cannot compare")
    return keyed


def differs(col, a, b, tol):
    """True if the two values differ enough to report."""
    if a == b:
        return False
    if col in INT_COLS:
        return True
    try:
        fa, fb = float(a), float(b)
    except ValueError:
        return True
    # Absolute floor as well as relative: a column that reads 0.0000 in the
    # baseline has no scale to be relative to, and 0 -> 0.0004 is rounding,
    # not a regression.
    if abs(fa - fb) <= 1e-4:
        return False
    scale = max(abs(fa), abs(fb))
    return abs(fa - fb) > tol * scale


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    tol = 0.001
    for a in sys.argv[1:]:
        if a.startswith("--tol"):
            tol = float(a.split("=", 1)[1])
    if len(args) != 2:
        sys.exit(__doc__)
    new_path, base_path = args

    if os.environ.get("BLESS") == "1":
        with open(new_path, newline="") as src, open(base_path, "w", newline="") as dst:
            dst.write(src.read())
        print(f"    blessed {base_path} from this run")
        return 0

    if not os.path.exists(base_path):
        sys.exit(
            f"!! no baseline at {base_path}\n"
            f"   create it with: BLESS=1 tests/run-tests.sh sim-baseline"
        )

    new = load(new_path)
    base = load(base_path)

    added = sorted(set(new) - set(base))
    removed = sorted(set(base) - set(new))
    changed = []
    for key in sorted(set(new) & set(base)):
        for col in FLOAT_COLS + INT_COLS:
            if col not in new[key] or col not in base[key]:
                continue
            if differs(col, base[key][col], new[key][col], tol):
                changed.append((key, col, base[key][col], new[key][col]))

    if not (added or removed or changed):
        print(f"    ok    sweep matches baseline    {len(new)} rows, tol {tol:g}")
        return 0

    print(f"    FAIL  sweep differs from baseline  {len(changed)} values,"
          f" {len(added)} added, {len(removed)} removed")
    for key in removed:
        print(f"          - row gone:  {key[0]} {key[1]}")
    for key in added:
        print(f"          + row new:   {key[0]} {key[1]}")
    # Cap the listing: a model change moves everything and 400 lines of it
    # helps nobody. The count above is the honest summary.
    for (axis, value), col, was, now in changed[:25]:
        print(f"          {axis} {value}  {col}: {was} -> {now}")
    if len(changed) > 25:
        print(f"          ... and {len(changed) - 25} more")
    print("          If this change is intended, re-bless in the SAME commit:")
    print("            BLESS=1 tests/run-tests.sh sim-baseline")
    return 1


if __name__ == "__main__":
    sys.exit(main())
