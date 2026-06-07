"""Regression lock for the algorithm factory + Weighted-Sum model scoring.

Guards the removal of the dead WeightedSumGA parallel path: the _WS models run
on plain GA + PathProblem's WS branch (via calculate_ws_score_from_ws_objective),
NOT on the removed WeightedSumGA algorithm. These tests pass before and after
that removal.
"""
import numpy as np
import pytest

from PathOptimizationModel import (AVAILABLE_MODELS, TC_WS,
                                   calculate_ws_score_from_ws_objective)


def test_factory_builds_every_registry_algorithm():
    from PathAlgorithm import PathAlgorithm
    from pymoo.core.algorithm import Algorithm
    used_algs = sorted({m["Alg"] for m in AVAILABLE_MODELS.values()})
    for alg in used_algs:                       # GA, NSGA2, NSGA3
        built = PathAlgorithm(alg)()
        assert isinstance(built, Algorithm), f"{alg} did not build a pymoo algorithm"


def test_no_registry_model_uses_weighted_sum_ga():
    algs = {m["Alg"] for m in AVAILABLE_MODELS.values()}
    assert "Weighted Sum GA" not in algs        # the removed dead path


def test_weighted_sum_model_scoring_is_finite(small_solution):
    # WS models reduce their objectives to one normalized score here. Save/restore
    # the session fixture's model so we don't contaminate other tests.
    original = small_solution.info.model
    try:
        small_solution.info.model = TC_WS       # Mission Time & Percentage Connectivity
        score = calculate_ws_score_from_ws_objective(small_solution)
    finally:
        small_solution.info.model = original
    assert np.isfinite(score)
    assert score != 0
