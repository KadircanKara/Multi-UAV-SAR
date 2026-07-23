"""Behaviour lock for PathRepair.interpolate_path.

The repair operator dominates optimizer runtime (~89% of a generation), so it
is a target for optimisation — and an optimisation that changes the produced
path silently changes every objective value the project has ever computed.
These tests pin the exact output across a spread of scenarios so a rewrite has
to prove it is a pure speed-up.
"""
import json
import os
import random

import numpy as np
import pytest

from PathInfo import PathInfo
from PathRepair import PathRepair
from PathSolution import PathSolution

_GOLDEN_PATH = os.path.join(os.path.dirname(__file__), "data", "path_repair_golden.json")


@pytest.fixture(scope="session")
def path_repair_golden():
    """Recorded outputs of the pre-optimisation implementation.

    Regenerate deliberately (and review the diff!) with:
        SAR_UPDATE_GOLDEN=1 pytest tests/test_path_repair.py
    A changed golden means the repaired paths changed, which means every
    objective value changes too — never update it to make a test pass.
    """
    updating = os.environ.get("SAR_UPDATE_GOLDEN") == "1"
    try:
        with open(_GOLDEN_PATH) as fh:
            recorded = json.load(fh)
    except FileNotFoundError:
        if not updating:
            raise
        recorded = {}

    def check(key: str, actual: list) -> list:
        if updating:
            recorded[key] = [int(c) for c in actual]
        elif key not in recorded:
            pytest.fail(f"no golden recorded for {key!r}; "
                        f"regenerate with SAR_UPDATE_GOLDEN=1")
        return recorded[key]

    yield check

    if updating:
        os.makedirs(os.path.dirname(_GOLDEN_PATH), exist_ok=True)
        with open(_GOLDEN_PATH, "w") as fh:
            json.dump(recorded, fh, indent=1, sort_keys=True)


def _scenario(**over):
    sc = {
        "grid_size": 8,
        "cell_side_length": 50,
        "number_of_drones": 4,
        "max_drone_speed": 2.5,
        "comm_cell_range": 2,
        "n_visits": 1,
        "target_positions": [12],
        "th": 0.9,
        "detection_probability": 0.7,
    }
    sc.update(over)
    return sc


def _solution(seed: int, **over) -> PathSolution:
    """A raw (unrepaired) solution, exactly as PathSampling produces one."""
    info = PathInfo(_scenario(**over))
    rng = np.random.RandomState(seed)
    path = rng.permutation(info.n_visits * info.number_of_cells) % info.number_of_cells
    py_rng = random.Random(seed)
    start_points = sorted(py_rng.sample(range(1, len(path)), info.number_of_drones - 1))
    start_points.insert(0, 0)
    return PathSolution(path, start_points, info)


# Spread over the parameters that change the loop's shape: n_visits drives the
# per-cell visit ceiling, drones drives path length, grid drives cell count.
_CASES = [
    ("baseline", {}),
    ("n_visits_2", {"n_visits": 2}),
    ("n_visits_3", {"n_visits": 3}),
    ("drones_16", {"number_of_drones": 16}),
    ("drones_16_visits_3", {"number_of_drones": 16, "n_visits": 3}),
    ("grid_4", {"grid_size": 4}),
]


@pytest.mark.parametrize("label,over", _CASES, ids=[c[0] for c in _CASES])
@pytest.mark.parametrize("seed", [1, 2, 7])
def test_interpolate_path_output_is_stable(label, over, seed, path_repair_golden):
    """Output must match the recorded golden path exactly."""
    sol = _solution(seed, **over)
    result = PathRepair().interpolate_path(sol)

    assert result == path_repair_golden(f"{label}-{seed}", result)


@pytest.mark.parametrize("label,over", _CASES, ids=[c[0] for c in _CASES])
def test_no_cell_is_visited_more_than_n_visits(label, over):
    """The invariant the count-guard exists to enforce."""
    sol = _solution(1, **over)

    result = PathRepair().interpolate_path(sol)

    counts = {}
    for city in result:
        counts[city] = counts.get(city, 0) + 1
    assert max(counts.values()) <= sol.info.n_visits


@pytest.mark.parametrize("label,over", _CASES, ids=[c[0] for c in _CASES])
def test_path_length_reaches_the_loop_target(label, over):
    sol = _solution(1, **over)

    result = PathRepair().interpolate_path(sol)

    assert len(result) >= sol.info.number_of_cells * sol.info.n_visits


def test_interpolation_does_not_guarantee_adjacency():
    """Documents a real property, not an aspiration.

    interpolate_between_cities emits a full adjacent chain, but the caller drops
    any mid-cell already at its n_visits ceiling — so the surviving path can
    still jump. That is why the models carry "Path Speed Violations as
    Constraint" in H rather than relying on repair to guarantee a walkable path.
    Any rewrite of this loop must preserve the skipping, not "fix" it.
    """
    sol = _solution(1)

    result = PathRepair().interpolate_path(sol)

    steps = []
    for a, b in zip(result, result[1:]):
        ca, cb = sol.get_coords(a), sol.get_coords(b)
        steps.append(max(np.abs(np.array(cb) - np.array(ca))) / sol.info.cell_side_length)
    assert max(steps) > 1, "expected at least one non-adjacent jump"
