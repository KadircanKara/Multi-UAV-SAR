"""Backfill the -AllObjectives.pkl siblings for the seeded scenarios.

    PYTHONPATH="$(pwd)/backend:$(pwd)" python backend/scripts/backfill_all_objectives.py [--force] [--dry-run]

Walks Results/Objectives/*-ObjectiveValues.pkl, loads each scenario's solutions,
and writes the five cached objectives out beside them. Idempotent: a scenario
whose sibling is newer than its solutions file is skipped unless --force.

Deliberately sequential. Each -SolutionObjects.pkl is ~160 MB unpickled, so the
loop holds exactly one scenario's solutions at a time; parallelising it would
multiply peak RSS by the worker count for no useful gain (the cost is I/O and
unpickling, and the whole run is a one-off).

Aborts on the first solution missing a cached objective rather than recomputing
it — see app/all_objectives.py for why a recomputed Max Mean TBV is wrong.
"""
import glob
import os
import sys

import app.rootpath  # noqa: F401  (repo root on sys.path)
import pandas as pd

from app import settings
from app.all_objectives import all_objectives_path, write_all_objectives

_OBJ_SUFFIX = "-ObjectiveValues.pkl"


def _is_current(scenario: str, sol_path: str) -> bool:
    """True when a sibling already exists and is at least as new as the
    solutions it was derived from."""
    dst = all_objectives_path(scenario)
    if not os.path.isfile(dst):
        return False
    return os.path.getmtime(dst) >= os.path.getmtime(sol_path)


def main(force: bool, dry_run: bool) -> int:
    obj_dir = os.path.join(settings.RESULTS_ROOT, "Objectives")
    sol_dir = os.path.join(settings.RESULTS_ROOT, "Solutions")

    written = skipped = no_solutions = 0
    for path in sorted(glob.glob(os.path.join(obj_dir, f"*{_OBJ_SUFFIX}"))):
        scenario = os.path.basename(path)[: -len(_OBJ_SUFFIX)]
        sol_path = os.path.join(sol_dir, f"{scenario}-SolutionObjects.pkl")
        if not os.path.isfile(sol_path):
            no_solutions += 1
            continue
        if not force and _is_current(scenario, sol_path):
            skipped += 1
            continue
        if dry_run:
            written += 1
            continue
        solutions = list(pd.read_pickle(sol_path))
        # SolutionObjects rows can be 1-element numpy arrays (PathUnitTest.py).
        solutions = [s[0] if hasattr(s, "shape") and getattr(s, "shape", None) == (1,)
                     else s for s in solutions]
        n = write_all_objectives(scenario, solutions)
        print(f"  {scenario}: {n} rows")
        written += 1

    print(f"backfill: written={written} skipped={skipped} "
          f"no_solutions={no_solutions}")
    return written


if __name__ == "__main__":
    args = sys.argv[1:]
    main(force="--force" in args, dry_run="--dry-run" in args)
