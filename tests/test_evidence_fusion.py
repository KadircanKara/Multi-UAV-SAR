"""Integration tests for cross-drone evidence fusion (2026-07-06 fix).

Pre-fix, merge_maps copied only the latest-timestep observation record and
beliefs never fused across drones: a target visited 3 times by 3 DIFFERENT
drones capped every node's belief at the single-visit posterior 0.778 and
detection never fired (all time metrics inf). These tests pin the fix.
"""
import numpy as np
import pytest

from PathInfo import PathInfo
from PathSolution import PathSolution
from SensingReplay import SensingConfig, replay


def _scenario(n_visits, targets):
    return {'grid_size': 8, 'cell_side_length': 50, 'number_of_drones': 4,
            'max_drone_speed': 2.5, 'comm_cell_range': 2, 'n_visits': n_visits,
            'target_positions': list(targets), 'th': 0.9,
            'detection_probability': 0.7}


def _solution(n_visits, path, start_points, targets=(12,)):
    info = PathInfo(_scenario(n_visits, targets))
    return PathSolution(np.array(path), np.array(start_points), info,
                        calculate_pathplan=True, calculate_connectivity=True)


def _cfg(topo):
    return SensingConfig(merge_topology=topo, time_model="discrete",
                         target_locations=[12])   # defaults: p=0.7 q=0.2 B=0.9


@pytest.fixture(scope="module")
def split_visits_solution():
    """n_visits=3 full coverage; cell 12 is visited exactly once each by
    drones 0, 1, 2 (path indices 12, 76, 140) — never twice by the same one."""
    return _solution(3, np.arange(192), [0, 48, 96, 144])


def test_split_visits_detect_with_onboard_merging(split_visits_solution):
    r = replay(split_visits_solution, _cfg("onboard"))
    assert np.isfinite(r.detection_time), \
        "3 independent visits (odds 3.5^3 -> 0.977 > B=0.9) must trigger detection"
    assert np.isfinite(r.time_at_least_one_drone_knows_all)


def test_split_visits_stay_undetected_without_merging(split_visits_solution):
    r = replay(split_visits_solution, _cfg("none"))
    # single-visit posterior 0.778 < B=0.9 on every isolated node
    assert r.detection_time is None or not np.isfinite(r.detection_time)


def test_belief_series_monotone_for_target(split_visits_solution):
    r = replay(split_visits_solution, _cfg("onboard"))
    series = r.cell_occupancy_probabilities[12]
    assert all(b >= a - 1e-12 for a, b in zip(series, series[1:])), \
        "target-cell fused belief must never decrease (events only accumulate)"


def test_same_drone_repeat_visits_still_detect():
    """Control: drone 0 visits cell 12 twice itself (indices 12 and 76);
    two chained events -> 0.9245 > B, matching pre-fix single-drone behavior."""
    sol = _solution(2, np.arange(128), [0, 96, 112, 120])
    r = replay(sol, _cfg("onboard"))
    assert np.isfinite(r.detection_time)
    assert np.isfinite(r.inform_time)


def _cfg_rt(topo):
    return SensingConfig(merge_topology=topo, time_model="realtime",
                         target_locations=[12])


def test_realtime_detects_consecutive_repeat_visits():
    """RT-1 regression: nvisits_3 matrices encode re-visits as consecutive
    same-cell columns; those legs have distance 0 for every drone, so they add
    zero realtime columns. Matrix-column-driven sensing must still fire an
    event per visited matrix column instead of silently dropping them.

    Cell 12 is visited 3x consecutively by one drone, but the belief crosses
    B=0.9 after the 2nd event (odds 3.5^2 -> 0.9245), which correctly triggers
    early return before the 3rd visit — exact parity with the (untouched)
    discrete pipeline, verified independently below to match this value."""
    sol = _solution(3, np.repeat(np.arange(64), 3), [0, 48, 96, 144])
    r = replay(sol, _cfg_rt("onboard"))
    assert np.isfinite(r.detection_time)
    from Sensing import _fused_belief
    beliefs = [_fused_belief(r.search_map[row, 12], 0.7, 0.2)
               for row in range(r.search_map.shape[0])]
    assert max(beliefs) == pytest.approx(0.9245283018867925)
    # parity check: the discrete pipeline (frozen, ground truth) must agree
    discrete_cfg = SensingConfig(merge_topology="onboard", time_model="discrete",
                                 target_locations=[12])
    r_discrete = replay(sol, discrete_cfg)
    discrete_beliefs = [_fused_belief(r_discrete.search_map[row, 12], 0.7, 0.2)
                        for row in range(r_discrete.search_map.shape[0])]
    assert max(discrete_beliefs) == pytest.approx(max(beliefs))


def test_realtime_split_visits_detect_with_merging(split_visits_solution):
    """Cross-drone fusion must work through the realtime pipeline too."""
    r = replay(split_visits_solution, _cfg_rt("onboard"))
    assert np.isfinite(r.detection_time)


def test_realtime_mission_time_finite_after_early_return(split_visits_solution):
    """RT-2 regression: early return must never strand a drone off-grid;
    the all-home break must fire and mission time stay finite."""
    r = replay(split_visits_solution, _cfg_rt("onboard"))
    assert np.isfinite(r.effective_mission_time)
    assert r.effective_mission_time > 0
