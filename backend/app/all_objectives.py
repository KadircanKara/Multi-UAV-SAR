"""The ``<scenario>-AllObjectives.pkl`` sibling artifact.

Each seeded scenario's ``-ObjectiveValues.pkl`` holds only the objectives its own
model optimised — Mission Time alone for MTSP, a single weighted-sum column for
the WS models. Comparing scenarios across ALL five objectives therefore used to
mean loading the ~160 MB ``-SolutionObjects.pkl`` per scenario and reading the
values off the solutions, which is why the comparison endpoint had to cap how
many scenarios one request could touch.

This module writes those five values out ONCE, next to the file they are missing
from, so the comparison read path is a small DataFrame load.

Two invariants make the artifact trustworthy:

  * **Never recompute.** Every seeded solution already caches all five objectives
    (PathUnitTest.run back-fills whatever the run skipped, and calculate_tbv is
    hardcoded on in PathRepair._do). Recomputing Max Mean TBV under the CURRENT
    code diverges from the seeded value — get_visit_times now counts
    return-to-base fly-overs as visits — so a recomputed number would silently
    disagree with what the model pages show. A missing cached attribute is an
    error, not an invitation to compute.
  * **Natural/unsigned units.** ``-ObjectiveValues.pkl`` stores SIGNED values
    (polarity pre-applied by PathProblem, so Percentage Connectivity is
    negative); the values cached on the solutions are natural. This file stores
    the natural ones — the shape ``stats_from_objective_dicts`` consumes.

The filename deliberately avoids the ``-ObjectiveValues.pkl`` suffix: several
scans match on it with ``endswith`` and would otherwise count every scenario
twice.
"""
from __future__ import annotations

import math
import os
import tempfile
from typing import Optional

import app.rootpath  # noqa: F401  # side-effect: repo root on sys.path

import pandas as pd

from app import settings
from app.library_service import _is_safe_scenario_name

from PathOptimizationModel import obj_name_sol_attr_dict

SUFFIX = "-AllObjectives.pkl"

# Canonical order. obj_name_sol_attr_dict is the authoritative objective set;
# freezing the order here keeps the written frames stable across pandas versions.
COLUMNS: list[str] = list(obj_name_sol_attr_dict.keys())


class MissingCachedObjective(Exception):
    """A solution lacks a cached objective, so writing it would require a
    recompute — which is forbidden (see the module docstring)."""


def all_objectives_path(scenario: str) -> str:
    return os.path.join(settings.RESULTS_ROOT, "Objectives", f"{scenario}{SUFFIX}")


def rows_from_solutions(solutions: list) -> list[dict[str, Optional[float]]]:
    """Read all five objectives off each solution, strictly.

    Raises MissingCachedObjective if any objective is not already cached — the
    caller must never paper over that by computing it.
    """
    rows: list[dict[str, Optional[float]]] = []
    for i, sol in enumerate(solutions):
        missing = [name for name, attr in obj_name_sol_attr_dict.items()
                   if not hasattr(sol, attr)]
        if missing:
            raise MissingCachedObjective(
                f"solution #{i} has no cached value for {', '.join(missing)}; "
                f"refusing to recompute (recomputed values diverge from the "
                f"seeded ones)")
        # Safe now: every attribute is already cached, so this reads the
        # solution's own values directly instead of routing through
        # PathFuncDict.objective_values(), whose compute_all_objectives()
        # unconditionally touches sol.info even when nothing needs computing.
        row: dict[str, Optional[float]] = {}
        for name, attr in obj_name_sol_attr_dict.items():
            v = getattr(sol, attr)
            try:
                f = float(v)
            except (TypeError, ValueError):
                f = None
            else:
                if not math.isfinite(f):
                    f = None
            row[name] = f
        rows.append({name: row[name] for name in COLUMNS})
    return rows


def write_all_objectives(scenario: str, solutions: list) -> int:
    """Write the sibling for one scenario. Returns the row count.

    Atomic: writes a temp file in the destination directory and os.replace()s it
    in, so a concurrent reader sees either the old frame or the new one, never a
    half-written pickle.
    """
    rows = rows_from_solutions(solutions)
    df = pd.DataFrame(rows, columns=COLUMNS)
    dst = all_objectives_path(scenario)
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(dst), suffix=".tmp")
    os.close(fd)
    try:
        df.to_pickle(tmp)
        os.replace(tmp, dst)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise
    return int(df.shape[0])


def read_all_objectives(
    scenario: str,
    expected_rows: Optional[int] = None,
) -> Optional[list[dict[str, Optional[float]]]]:
    """Read one scenario's sibling as per-solution objective dicts.

    Returns None — never raises — when the file is absent, unreadable, has the
    wrong columns, or disagrees with *expected_rows*. Callers treat None as
    "skip this scenario", which is honest: a stale or truncated sibling must not
    be aggregated into a comparison as if it were complete.
    """
    if not _is_safe_scenario_name(scenario):
        return None
    path = all_objectives_path(scenario)
    if not os.path.isfile(path):
        return None
    try:
        df = pd.read_pickle(path)
    except Exception:
        return None
    if list(df.columns) != COLUMNS:
        return None
    if expected_rows is not None and int(df.shape[0]) != int(expected_rows):
        return None
    rows: list[dict[str, Optional[float]]] = []
    for values in df.itertuples(index=False, name=None):
        row: dict[str, Optional[float]] = {}
        for name, v in zip(COLUMNS, values):
            if v is None:
                row[name] = None
                continue
            f = float(v)
            row[name] = f if math.isfinite(f) else None
        rows.append(row)
    return rows
