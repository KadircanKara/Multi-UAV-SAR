"""End-to-end: result set -> SolutionSelector -> SensingConfig -> compare()."""
import numpy as np
import pandas as pd
import pytest

from PathInfo import PathInfo
from PathSolution import PathSolution
from PathOptimizationModel import TC_MOO_NSGA2
from SolutionSelection import SolutionSelector
from SensingReplay import SensingConfig, compare
from conftest import make_scenario


@pytest.fixture(scope="module")
def mini_result_set():
    """Three hand-built solutions standing in for a small Pareto front."""
    info = PathInfo(make_scenario([12]))
    paths = [np.arange(64), np.roll(np.arange(64), 7), np.roll(np.arange(64), 31)]
    sols = [PathSolution(p, np.array([0, 16, 32, 48]), info,
                         calculate_pathplan=True, calculate_connectivity=True)
            for p in paths]
    F = pd.DataFrame({
        "Mission Time": [s.mission_time for s in sols],
        "Percentage Connectivity": [-abs(s.percentage_connectivity or 0.5) for s in sols],
    })
    return F, sols


def test_select_then_compare(mini_result_set, tmp_path):
    F, sols = mini_result_set
    selector = SolutionSelector(F, sols, TC_MOO_NSGA2)
    assert selector.result_kind == "front"
    idx, solution, label = selector.best("Mission Time")

    configs = [SensingConfig.from_info(solution.info, merge_topology=t,
                                       belief_threshold=0.7)
               for t in ("none", "onboard", "gcs")]
    result = compare(solution, configs, output_dir=str(tmp_path),
                     scenario_label="integration", animations=False)

    assert (result.table.loc["onboard", "Detection Time"]
            <= result.table.loc["none", "Detection Time"])
    assert np.isfinite(result.table.loc["onboard", "Effective Mission Time"])
    assert len(result.plot_paths) == 2


def test_realtime_compare_end_to_end(mini_result_set, tmp_path):
    F, sols = mini_result_set
    solution = SolutionSelector(F, sols, TC_MOO_NSGA2).balanced()[1]
    configs = [SensingConfig.from_info(solution.info, merge_topology=t,
                                       time_model="realtime", belief_threshold=0.7)
               for t in ("none", "onboard")]
    result = compare(solution, configs, output_dir=str(tmp_path),
                     scenario_label="rt-integration", animations=False)
    assert np.isfinite(result.table.loc["onboard", "Effective Mission Time"])
