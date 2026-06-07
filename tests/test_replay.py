import os

import numpy as np
import pytest

from SensingReplay import SensingConfig, replay, compare


def cfg(topo="onboard", time_model="discrete", B=0.7):
    return SensingConfig(merge_topology=topo, time_model=time_model,
                         target_locations=[12], belief_threshold=B,
                         detection_prob=0.7, false_alarm_prob=0.2)


def test_replay_dispatches_discrete(small_solution):
    r = replay(small_solution, cfg(time_model="discrete"))
    assert np.isfinite(r.effective_mission_time)
    assert r.label == "onboard"


def test_replay_dispatches_realtime(small_solution):
    r_real = replay(small_solution, cfg(time_model="realtime"))
    r_disc = replay(small_solution, cfg(time_model="discrete"))
    assert np.isfinite(r_real.effective_mission_time)
    assert len(r_real.cell_occupancy_probabilities) == small_solution.info.number_of_cells
    # discrete sums exact leg times; realtime counts 1s columns — a dispatch bug
    # that ignores time_model would make these exactly equal
    assert r_real.effective_mission_time != r_disc.effective_mission_time


def test_replay_result_carries_metrics_and_solution(small_solution):
    r = replay(small_solution, cfg())
    assert r.detection_time <= r.effective_mission_time
    assert r.solution is not small_solution          # deep copy, original untouched
    assert r.config.merge_topology == "onboard"


def test_replay_custom_label(small_solution):
    r = replay(small_solution, cfg(), label="my-label")
    assert r.label == "my-label"


def test_compare_table_and_artifacts(small_solution, tmp_path):
    configs = [cfg("none"), cfg("onboard"), cfg("gcs")]
    result = compare(small_solution, configs, output_dir=str(tmp_path),
                     scenario_label="testscn", animations=False)
    assert list(result.table.index) == ["none", "onboard", "gcs"]
    assert "Effective Mission Time" in result.table.columns
    assert "Detection Time" in result.table.columns
    # onboard merging never detects later than none (append-only invariant)
    assert (result.table.loc["onboard", "Detection Time"]
            <= result.table.loc["none", "Detection Time"])
    assert (tmp_path / "testscn-comparison.csv").exists()
    assert len(result.plot_paths) == 2          # targets-over-time + belief evolution
    for p in result.plot_paths:
        assert os.path.exists(p)


def test_compare_duplicate_topology_labels_deduped(small_solution, tmp_path):
    configs = [cfg("onboard", B=0.7), cfg("onboard", B=0.85)]
    result = compare(small_solution, configs, output_dir=str(tmp_path),
                     scenario_label="dup", animations=False)
    assert list(result.table.index) == ["onboard-0", "onboard-1"]


@pytest.mark.slow
def test_compare_animations_render(small_solution, tmp_path):
    result = compare(small_solution, [cfg("onboard")], output_dir=str(tmp_path),
                     scenario_label="anim", animations=True)
    assert len(result.animation_paths) == 1
    assert os.path.exists(result.animation_paths[0])
    assert result.animation_paths[0].endswith(".gif")
