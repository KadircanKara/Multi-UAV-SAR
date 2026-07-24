"""Bounds on the realtime interpolation in Time.get_real_paths.

get_real_paths sub-samples every leg into ceil(leg_dist / speed) endpoint-inclusive
columns and accumulates them across the whole path. A pathological scenario (huge
cell / near-zero speed) — or simply a very long path — can balloon that into a
multi-million-column trajectory that the downstream
get_real_connectivity_matrix's zeros((cols, nodes, nodes)) allocation turns into
gigabytes and an OOM in the MAIN process. A cumulative ceiling caps the whole
timeline; the per-leg clamp only bounds a single leg.

These build their own PathSolution (no seeded pickle), so they run regardless of
whether the Results/ tree is provisioned.
"""
import numpy as np
import pytest

import Time
from PathInfo import PathInfo
from PathSolution import PathSolution


def _solution():
    """A deterministic full-coverage grid-8 solution (real params: cell 50, speed
    2.5), whose realtime trajectory is a few hundred columns."""
    info = PathInfo({'grid_size': 8, 'cell_side_length': 50, 'number_of_drones': 4,
                     'max_drone_speed': 2.5, 'comm_cell_range': 2, 'n_visits': 1,
                     'target_positions': [12], 'th': 0.9,
                     'detection_probability': 0.7})
    path = np.arange(info.number_of_cells)
    start_points = np.array([0, 16, 32, 48])
    return PathSolution(path, start_points, info,
                        calculate_pathplan=True, calculate_connectivity=True)


def test_default_ceiling_is_inert_for_a_legit_solution():
    """A normal grid-8 trajectory is a few hundred columns — orders of magnitude
    below the ceiling — so the guard never fires for valid input and the
    interpolated output is produced unchanged."""
    sol = _solution()
    x, y = Time.get_real_paths(sol)
    assert x.shape == y.shape
    assert 0 < x.shape[1] < Time._MAX_REALTIME_TOTAL_COLS
    # Far below, not merely under: legit runs top out near ~1.2k columns.
    assert x.shape[1] < Time._MAX_REALTIME_TOTAL_COLS // 100


def test_cumulative_ceiling_raises_valueerror(monkeypatch):
    """Once the accumulated column count crosses the ceiling, get_real_paths
    raises ValueError (mapped to 422 by the playground routers; a clean failed run
    in the optimizer pool) instead of allocating the oversized trajectory."""
    sol = _solution()
    # This solution's trajectory is a few hundred columns; a ceiling of 50 forces
    # the guard to trip partway through the leg loop.
    monkeypatch.setattr(Time, "_MAX_REALTIME_TOTAL_COLS", 50)
    with pytest.raises(ValueError, match="Realtime trajectory too long"):
        Time.get_real_paths(sol)
