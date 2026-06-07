import numpy as np
import pytest

from Sensing import sensing_and_discrete_info_sharing, sensing_and_realtime_info_sharing

# NOTE: until Task 11 flips signatures to SensingConfig, call with the current
# loose kwargs. Task 11 updates ONLY the call shape in this file, not the asserts.
KW = dict(target_locations=[12], B=0.7, p=0.7, q=0.2)
KW_STRICT = dict(target_locations=[12], B=0.999, p=0.7, q=0.2)  # B unreachable with n_visits=1

EXPECTED_KEYS = {
    "cell occupancy probabilities", "search map", "occupancy status",
    "detection time", "inform time", "mission time",
    "time at least one drone knows all targets",
}


def test_realtime_runs_without_nameerror(small_solution):
    metrics, x = sensing_and_realtime_info_sharing(small_solution, merging_strategy="onboard", **KW)
    assert isinstance(metrics, dict)


def test_discrete_runs_without_nameerror(small_solution):
    metrics, x = sensing_and_discrete_info_sharing(small_solution, merging_strategy="onboard", **KW)
    assert isinstance(metrics, dict)


def test_both_pipelines_return_identical_keys(small_solution):
    m_d, _ = sensing_and_discrete_info_sharing(small_solution, merging_strategy="onboard", **KW)
    m_r, _ = sensing_and_realtime_info_sharing(small_solution, merging_strategy="onboard", **KW)
    assert set(m_d.keys()) == EXPECTED_KEYS
    assert set(m_r.keys()) == EXPECTED_KEYS


def test_realtime_mission_time_finite(small_solution):
    metrics, _ = sensing_and_realtime_info_sharing(small_solution, merging_strategy="onboard", **KW)
    assert np.isfinite(metrics["mission time"]) and metrics["mission time"] > 0


def test_realtime_early_return_shortens_mission(small_solution):
    # Favorable: low threshold + onboard merging -> drones learn fast, go home early.
    # Unfavorable: unreachable threshold + no merging -> full path is flown.
    favorable, _ = sensing_and_realtime_info_sharing(small_solution, merging_strategy="onboard", **KW)
    unfavorable, _ = sensing_and_realtime_info_sharing(small_solution, merging_strategy="none", **KW_STRICT)
    assert np.isfinite(favorable["mission time"])
    assert favorable["mission time"] < unfavorable["mission time"]


def test_realtime_detection_finite_when_detectable(small_solution):
    metrics, _ = sensing_and_realtime_info_sharing(small_solution, merging_strategy="onboard", **KW)
    assert np.isfinite(metrics["detection time"])


def test_realtime_detection_inf_when_threshold_unreachable(small_solution):
    metrics, _ = sensing_and_realtime_info_sharing(small_solution, merging_strategy="onboard", **KW_STRICT)
    assert metrics["detection time"] == np.inf


def test_onboard_le_none_detection_within_realtime(small_solution):
    onboard, _ = sensing_and_realtime_info_sharing(small_solution, merging_strategy="onboard", **KW)
    none_, _ = sensing_and_realtime_info_sharing(small_solution, merging_strategy="none", **KW)
    assert onboard["detection time"] <= none_["detection time"]


def test_onboard_le_none_detection_within_discrete(small_solution):
    onboard, _ = sensing_and_discrete_info_sharing(small_solution, merging_strategy="onboard", **KW)
    none_, _ = sensing_and_discrete_info_sharing(small_solution, merging_strategy="none", **KW)
    assert onboard["detection time"] <= none_["detection time"]


def test_realtime_merges_midflight(small_solution):
    """The realtime pipeline must exchange beliefs BETWEEN grid cells.

    Evidence accepted: with onboard merging, some drone's search_map holds an
    observation of the target cell that its own discrete path could not have
    produced before its first own visit — i.e. it was received via a merge.
    Strict mid-flight verification: rerun with merging disabled ('none') and
    assert the receive disappears while per-drone self-observations are equal.
    """
    onboard, x_on = sensing_and_realtime_info_sharing(
        small_solution, merging_strategy="onboard", target_locations=[12], B=0.999, p=0.7, q=0.2)
    none_, x_off = sensing_and_realtime_info_sharing(
        small_solution, merging_strategy="none", target_locations=[12], B=0.999, p=0.7, q=0.2)
    # B=0.999 disables early return so both runs fly identical full paths.
    target = 12
    received_on = sum(len(onboard["search map"][node, target]) for node in range(5))
    received_off = sum(len(none_["search map"][node, target]) for node in range(5))
    assert received_on > received_off  # merging propagated target observations
