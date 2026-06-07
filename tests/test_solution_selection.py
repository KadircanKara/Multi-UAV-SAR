import numpy as np
import pandas as pd
import pytest

from PathOptimizationModel import TC_MOO_NSGA2, TC_WS, MTSP
from SolutionSelection import SolutionSelector, StrategyUnavailableError


def front_selector(n=5):
    """5-point 2-objective front; F is SIGNED as stored on disk
    (Mission Time positive-minimize, Percentage Connectivity negative-minimize)."""
    F = pd.DataFrame({
        "Mission Time":            [100.0, 200.0, 300.0, 400.0, 500.0],
        "Percentage Connectivity": [-0.10, -0.30, -0.50, -0.70, -0.90],
    })
    solutions = [f"sol{i}" for i in range(n)]      # selector never inspects solutions
    return SolutionSelector(F, solutions, TC_MOO_NSGA2)


def single_selector(model):
    F = pd.DataFrame({"Mission Time": [123.0]}) if model is MTSP \
        else pd.DataFrame({"Mission Time & Percentage Connectivity Weighted Sum": [0.5]})
    return SolutionSelector(F, ["only"], model)


def test_result_kind():
    assert front_selector().result_kind == "front"
    assert single_selector(MTSP).result_kind == "single"
    assert single_selector(TC_WS).result_kind == "single"


def test_degenerate_moo_front_downgrades_with_warning():
    F = pd.DataFrame({"Mission Time": [100.0], "Percentage Connectivity": [-0.5]})
    with pytest.warns(UserWarning):
        sel = SolutionSelector(F, ["only"], TC_MOO_NSGA2)
    assert sel.result_kind == "single"


def test_capabilities_front():
    caps = front_selector().capabilities()
    assert caps["best"] == ["Mission Time", "Percentage Connectivity"]
    assert caps["balanced"] and caps["knee"] and caps["by_weights"]
    assert caps["by_index"] == 4 and caps["the_solution"] is False


def test_capabilities_single():
    caps = single_selector(MTSP).capabilities()
    assert caps["best"] == [] and not caps["balanced"] and not caps["knee"]
    assert not caps["by_weights"] and caps["the_solution"] is True


def test_the_solution_only_for_single():
    idx, sol, label = single_selector(MTSP).the_solution()
    assert idx == 0 and sol == "only"
    with pytest.raises(StrategyUnavailableError):
        front_selector().the_solution()


def test_by_index_bounds():
    sel = front_selector()
    idx, sol, label = sel.by_index(3)
    assert idx == 3 and sol == "sol3"
    with pytest.raises(StrategyUnavailableError):
        sel.by_index(99)


def test_ws_error_message_teaches():
    with pytest.raises(StrategyUnavailableError, match="MOO variant"):
        single_selector(TC_WS).by_weights({"Mission Time": 1.0})


def test_best_polarity_aware():
    sel = front_selector()
    idx, sol, label = sel.best("Mission Time")
    assert idx == 0                       # 100 s is fastest
    idx, sol, label = sel.best("Percentage Connectivity")
    assert idx == 4                       # -0.90 signed == 90% connectivity, the most


def test_best_rejects_non_model_objective():
    with pytest.raises(StrategyUnavailableError, match="Max Mean TBV"):
        front_selector().best("Max Mean TBV")   # TC never optimized TBV


def test_best_unavailable_for_single():
    with pytest.raises(StrategyUnavailableError):
        single_selector(MTSP).best("Mission Time")


def test_balanced_centroid_nearest():
    # normalized F: MT = [0,.25,.5,.75,1], PC = [1,.75,.5,.25,0] (after min-max);
    # centroid = (.5,.5) -> nearest is index 2 exactly.
    idx, sol, label = front_selector().balanced()
    assert idx == 2


def test_by_weights_one_hot_equals_best():
    sel = front_selector()
    idx_w, _, _ = sel.by_weights({"Mission Time": 1.0})
    idx_b, _, _ = sel.best("Mission Time")
    assert idx_w == idx_b


def test_by_weights_validates_keys_and_zeroes():
    sel = front_selector()
    with pytest.raises(StrategyUnavailableError):
        sel.by_weights({"Nonexistent Objective": 1.0})
    with pytest.raises(StrategyUnavailableError):
        sel.by_weights({"Mission Time": 0.0})


def test_knee_on_kneed_front():
    # Convex front with a pronounced knee at index 1.
    F = pd.DataFrame({"Mission Time": [100.0, 120.0, 300.0, 500.0],
                      "Percentage Connectivity": [-0.20, -0.80, -0.85, -0.90]})
    sel = SolutionSelector(F, list("abcd"), TC_MOO_NSGA2)
    idx, sol, label = sel.knee()
    assert idx == 1


def test_knee_falls_back_to_balanced_with_warning():
    # Perfectly linear front: HighTradeoffPoints().do returns None (verified
    # against pymoo 0.6.1.6 in this venv).
    F = pd.DataFrame({"Mission Time": [100.0, 200.0, 300.0],
                      "Percentage Connectivity": [-0.9, -0.5, -0.1]})
    sel = SolutionSelector(F, list("abc"), TC_MOO_NSGA2)
    with pytest.warns(UserWarning, match="balanced"):
        idx, sol, label = sel.knee()
    assert label == "Balanced (knee fallback)"
