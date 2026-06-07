import numpy as np
import pytest

from SensingReplay import SensingConfig, replay


def cfg(topo="onboard", time_model="discrete", B=0.7):
    return SensingConfig(merge_topology=topo, time_model=time_model,
                         target_locations=[12], belief_threshold=B,
                         detection_prob=0.7, false_alarm_prob=0.2)


def test_replay_dispatches_discrete(small_solution):
    r = replay(small_solution, cfg(time_model="discrete"))
    assert np.isfinite(r.effective_mission_time)
    assert r.label == "onboard"


def test_replay_dispatches_realtime(small_solution):
    r = replay(small_solution, cfg(time_model="realtime"))
    assert np.isfinite(r.effective_mission_time)
    assert len(r.cell_occupancy_probabilities) == small_solution.info.number_of_cells


def test_replay_result_carries_metrics_and_solution(small_solution):
    r = replay(small_solution, cfg())
    assert r.detection_time <= r.effective_mission_time
    assert r.solution is not small_solution          # deep copy, original untouched
    assert r.config.merge_topology == "onboard"


def test_replay_custom_label(small_solution):
    r = replay(small_solution, cfg(), label="my-label")
    assert r.label == "my-label"
