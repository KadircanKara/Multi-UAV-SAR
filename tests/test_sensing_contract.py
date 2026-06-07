import numpy as np
import pytest

from Sensing import sensing_and_discrete_info_sharing, sensing_and_realtime_info_sharing
from SensingReplay import SensingConfig


def CFG(topo, B=0.7, time_model="realtime"):
    return SensingConfig(merge_topology=topo, time_model=time_model, target_locations=[12],
                         belief_threshold=B, detection_prob=0.7, false_alarm_prob=0.2)

EXPECTED_KEYS = {
    "cell occupancy probabilities", "search map", "occupancy status",
    "detection time", "inform time", "mission time",
    "time at least one drone knows all targets",
}


def test_realtime_runs_without_nameerror(small_solution):
    metrics, x = sensing_and_realtime_info_sharing(small_solution, CFG("onboard"))
    assert isinstance(metrics, dict)


def test_discrete_runs_without_nameerror(small_solution):
    metrics, x = sensing_and_discrete_info_sharing(small_solution, CFG("onboard", time_model="discrete"))
    assert isinstance(metrics, dict)


def test_both_pipelines_return_identical_keys(small_solution):
    m_d, _ = sensing_and_discrete_info_sharing(small_solution, CFG("onboard", time_model="discrete"))
    m_r, _ = sensing_and_realtime_info_sharing(small_solution, CFG("onboard"))
    assert set(m_d.keys()) == EXPECTED_KEYS
    assert set(m_r.keys()) == EXPECTED_KEYS


def test_realtime_mission_time_finite(small_solution):
    metrics, _ = sensing_and_realtime_info_sharing(small_solution, CFG("onboard"))
    assert np.isfinite(metrics["mission time"]) and metrics["mission time"] > 0


def test_realtime_early_return_shortens_mission(small_solution):
    # Favorable: low threshold + onboard merging -> drones learn fast, go home early.
    # Unfavorable: unreachable threshold + no merging -> full path is flown.
    favorable, _ = sensing_and_realtime_info_sharing(small_solution, CFG("onboard"))
    unfavorable, _ = sensing_and_realtime_info_sharing(small_solution, CFG("none", B=0.999))
    assert np.isfinite(favorable["mission time"])
    assert favorable["mission time"] < unfavorable["mission time"]


def test_realtime_detection_finite_when_detectable(small_solution):
    metrics, _ = sensing_and_realtime_info_sharing(small_solution, CFG("onboard"))
    assert np.isfinite(metrics["detection time"])


def test_realtime_detection_inf_when_threshold_unreachable(small_solution):
    metrics, _ = sensing_and_realtime_info_sharing(small_solution, CFG("onboard", B=0.999))
    assert metrics["detection time"] == np.inf


def test_onboard_le_none_detection_within_realtime(small_solution):
    onboard, _ = sensing_and_realtime_info_sharing(small_solution, CFG("onboard"))
    none_, _ = sensing_and_realtime_info_sharing(small_solution, CFG("none"))
    assert onboard["detection time"] <= none_["detection time"]


def test_onboard_le_none_detection_within_discrete(small_solution):
    onboard, _ = sensing_and_discrete_info_sharing(small_solution, CFG("onboard", time_model="discrete"))
    none_, _ = sensing_and_discrete_info_sharing(small_solution, CFG("none", time_model="discrete"))
    assert onboard["detection time"] <= none_["detection time"]


def test_realtime_merging_propagates_to_non_visiting_drones(small_solution):
    """Merged beliefs must reach drones that never visit the target themselves.

    Fixture geometry: only drone 0's subtour (cells 0-15) contains target cell
    12; drones 1-3 (search_map rows 2-4) never visit it, so ANY observation of
    cell 12 in their maps can only have arrived via merge_maps. With merging
    'none' those rows must hold exactly the initial default observation.
    B=0.999 disables early return so both runs fly identical full paths.

    Note: this pins belief PROPAGATION through the realtime pipeline's
    every-second merge cadence; it does not isolate strictly-between-cells
    merge events (that would require receive-time provenance the observation
    dicts don't carry).
    """
    onboard, _ = sensing_and_realtime_info_sharing(
        small_solution, CFG("onboard", B=0.999))
    none_, _ = sensing_and_realtime_info_sharing(
        small_solution, CFG("none", B=0.999))
    target = 12
    n_nodes = onboard["search map"].shape[0]
    non_visiting_rows = range(2, n_nodes)   # drones 1-3: subtours exclude cell 12
    for row in non_visiting_rows:
        assert len(none_["search map"][row, target]) == 1, \
            f"row {row}: 'none' must leave only the default observation"
        assert len(onboard["search map"][row, target]) > 1, \
            f"row {row}: 'onboard' must have delivered a merged observation"
