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


def write_all_objectives(
    scenario: str,
    solutions: list,
    source_path: Optional[str] = None,
) -> int:
    """Write the sibling for one scenario. Returns the row count.

    Atomic: writes a temp file in the destination directory and os.replace()s it
    in, so a concurrent reader sees either the old frame or the new one, never a
    half-written pickle.

    When *source_path* is given and exists, its (size, mtime_ns) is stamped
    onto the written frame's ``.attrs["source"]`` (pandas preserves ``.attrs``
    through to_pickle/read_pickle). read_all_objectives can then detect a
    source file that was replaced after the sibling was derived from it — see
    that function's docstring. Pass the source path AFTER it has been written
    to its final location, so the stamp matches the bytes actually on disk.
    """
    rows = rows_from_solutions(solutions)
    df = pd.DataFrame(rows, columns=COLUMNS).astype("float64")
    if source_path is not None and os.path.isfile(source_path):
        st = os.stat(source_path)
        df.attrs["source"] = {"size": st.st_size, "mtime_ns": st.st_mtime_ns}
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
    source_path: Optional[str] = None,
) -> Optional[list[dict[str, Optional[float]]]]:
    """Read one scenario's sibling as per-solution objective dicts.

    Returns None — never raises — when the file is absent, unreadable, has the
    wrong columns, or disagrees with *expected_rows*. Callers treat None as
    "skip this scenario", which is honest: a stale or truncated sibling must not
    be aggregated into a comparison as if it were complete.

    When *source_path* is given and the frame carries a ``source`` stamp (see
    write_all_objectives), the stamp's (size, mtime_ns) is compared against
    *source_path*'s current stat; a mismatch means the sibling was derived from
    a since-replaced source and is treated as stale (returns None). This is a
    stat-only check — the source file itself is never opened or unpickled, so
    the whole point of the sibling (avoiding the ~160 MB solutions load) still
    holds.

    A frame with NO stamp — every sibling written before this check existed,
    including all pre-existing backfilled siblings — is accepted regardless of
    *source_path*. This keeps the check fully backward compatible: it detects
    staleness only for siblings written after this change, never invalidates an
    old unstamped one, and so requires no re-backfill.

    Likewise, when *source_path* is given but does not exist on disk, the stamp
    check is skipped (a missing source cannot contradict a stamp) — the sibling
    is accepted rather than invalidated.
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
    stamp = df.attrs.get("source")
    if source_path is not None and stamp is not None:
        try:
            st = os.stat(source_path)
        except OSError:
            pass
        else:
            if (int(stamp.get("size", -1)) != st.st_size
                    or int(stamp.get("mtime_ns", -1)) != st.st_mtime_ns):
                return None
    rows: list[dict[str, Optional[float]]] = []
    for values in df.itertuples(index=False, name=None):
        row: dict[str, Optional[float]] = {}
        for name, v in zip(COLUMNS, values):
            # The frame is float64 (write_all_objectives coerces via
            # .astype("float64")), so v is a float/NaN, not None; the guard
            # is defensive only, in case an older or hand-built pickle ever
            # carries an object-dtype column with a literal None in it.
            if v is None:
                row[name] = None
                continue
            f = float(v)
            row[name] = f if math.isfinite(f) else None
        rows.append(row)
    return rows
