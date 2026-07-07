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


from collections import Counter


def _event_multiset(search_map):
    """Multiset of sensing events (drone, matrix-column, cell, positive) across
    all nodes/cells, ignoring the prior sentinel (timestep < 0)."""
    c = Counter()
    for row in range(search_map.shape[0]):
        for cell in range(search_map.shape[1]):
            for e in search_map[row, cell]:
                if e["timestep"] >= 0:
                    c[(e["drone"], e["timestep"], cell, e["positive"])] += 1
    return c


def _cfg_parity(time_model, B, targets):
    # merge_topology="none" isolates the SENSING substrate: no cross-node event
    # copying, so the multiset holds each drone's own observations only. B=0.999
    # (unreachable) guarantees no early return, so both pipelines sense their
    # full tours and the only remaining difference would be a real sensing bug.
    return SensingConfig(merge_topology="none", time_model=time_model,
                         detection_prob=0.7, false_alarm_prob=0.2,
                         belief_threshold=B, target_locations=targets)


def test_realtime_discrete_sensing_parity_spread_visits(split_visits_solution):
    """No early return (B=0.999): realtime and discrete must produce the
    identical sensing-event multiset. Locks the invariant that the two
    pipelines share one sensing substrate and differ only in merge cadence."""
    rd = replay(split_visits_solution, _cfg_parity("discrete", 0.999, [12]))
    rr = replay(split_visits_solution, _cfg_parity("realtime", 0.999, [12]))
    assert _event_multiset(rr.search_map) == _event_multiset(rd.search_map)


def test_realtime_discrete_sensing_parity_consecutive_repeats():
    """dt=0 repeat-visit legs (consecutive same-cell columns) must sense in
    realtime exactly as in discrete (RT-1 + tour window together)."""
    sol = _solution(3, np.repeat(np.arange(64), 3), [0, 48, 96, 144])
    rd = replay(sol, _cfg_parity("discrete", 0.999, [12]))
    rr = replay(sol, _cfg_parity("realtime", 0.999, [12]))
    assert _event_multiset(rr.search_map) == _event_multiset(rd.search_map)


def test_realtime_no_sensing_during_hovering():
    """A drone with a short tour hovers on its last cell; that hover must NOT
    generate sensing events. Target sits on the hovered cell (drone 0's last
    tour cell), visited once for real. Realtime must match discrete's event
    count for that cell (pre-fix realtime counted the hover repeats)."""
    sol = _solution(1, np.arange(64), [0, 4, 8, 12], targets=(3,))
    rd = replay(sol, _cfg_parity("discrete", 0.9, [3]))
    rr = replay(sol, _cfg_parity("realtime", 0.9, [3]))
    de, re_ = _event_multiset(rd.search_map), _event_multiset(rr.search_map)
    cell3_d = sum(v for k, v in de.items() if k[2] == 3)
    cell3_r = sum(v for k, v in re_.items() if k[2] == 3)
    assert cell3_d >= 1               # the real tour visit is sensed
    assert cell3_r == cell3_d         # realtime adds no hover events
    # and full parity holds for this early-return-free scenario
    assert re_ == de
