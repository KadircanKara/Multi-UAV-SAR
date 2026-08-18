"""The comparison objectives path reads the precomputed -AllObjectives.pkl and
never touches SolutionObjects.

The point of the artifact is that one request can aggregate every seeded
scenario cheaply. These tests pin the three properties that makes that safe:
the values come from the sibling, a missing/stale sibling is skipped rather
than silently mis-aggregated, and no code path falls back to a selector load.
"""
import os

import pandas as pd
import pytest

from app import all_objectives, comparison_service, library_service, settings


@pytest.fixture
def seeded(tmp_path, monkeypatch):
    """A results tree with one scenario whose sibling is present."""
    monkeypatch.setattr(settings, "RESULTS_ROOT", str(tmp_path))
    os.makedirs(tmp_path / "Objectives")
    os.makedirs(tmp_path / "Solutions")
    scenario = "MOO_NSGA2_TCD_g_8_a_50_n_4_v_2.5_r_2_nvisits_2"
    pd.DataFrame({"Mission Time": [1.0, 2.0]}).to_pickle(
        tmp_path / "Objectives" / f"{scenario}-ObjectiveValues.pkl")
    pd.DataFrame(
        [[100.0, 0.5, 2.0, 1.0, 50.0], [200.0, 0.9, 4.0, 2.0, 70.0]],
        columns=all_objectives.COLUMNS,
    ).to_pickle(tmp_path / "Objectives" / f"{scenario}{all_objectives.SUFFIX}")
    library_service._bust_scenario_memos()
    return scenario


def test_stats_come_from_the_sibling(seeded):
    stats = comparison_service._scenario_stats(seeded)
    assert stats["n_solutions"] == 2
    mt = stats["objective_stats"]["Mission Time"]
    assert mt == {"min": 100.0, "max": 200.0, "mean": 150.0, "best": 100.0}


def test_maximize_objective_best_is_the_max(seeded):
    stats = comparison_service._scenario_stats(seeded)
    # Percentage Connectivity has polarity -1.
    assert stats["objective_stats"]["Percentage Connectivity"]["best"] == 0.9


def test_objectives_the_model_did_not_optimize_are_present(seeded):
    stats = comparison_service._scenario_stats(seeded)
    # TCD does not optimise Max Mean TBV, but the sibling carries it.
    assert stats["objective_stats"]["Max Mean TBV"]["min"] == 50.0


def test_no_selector_is_ever_loaded(seeded, monkeypatch):
    def _boom(*a, **kw):
        raise AssertionError("the objectives path must not load a selector")

    monkeypatch.setattr(comparison_service, "get_selector", _boom)
    assert comparison_service._scenario_stats(seeded) is not None


def test_missing_sibling_is_skipped(seeded):
    os.unlink(os.path.join(settings.RESULTS_ROOT, "Objectives",
                           f"{seeded}{all_objectives.SUFFIX}"))
    library_service._bust_scenario_memos()
    assert comparison_service._scenario_stats(seeded) is None


def test_row_count_mismatch_is_skipped(seeded):
    # Sibling says 3 rows, the objective table says 2 — stale artifact.
    pd.DataFrame(
        [[1.0, 0.5, 2.0, 1.0, 50.0]] * 3, columns=all_objectives.COLUMNS,
    ).to_pickle(os.path.join(settings.RESULTS_ROOT, "Objectives",
                             f"{seeded}{all_objectives.SUFFIX}"))
    library_service._bust_scenario_memos()
    assert comparison_service._scenario_stats(seeded) is None


def test_skipped_scenario_is_reported_not_dropped(seeded):
    result = comparison_service.compare_objectives([seeded, "NOT_A_SCENARIO"])
    assert [s["scenario"] for s in result["scenarios"]] == [seeded]
    assert result["skipped"] == ["NOT_A_SCENARIO"]


def test_repeat_calls_are_served_from_the_memo(seeded, monkeypatch):
    comparison_service._scenario_stats(seeded)

    def _boom(*a, **kw):
        raise AssertionError("second call should not re-read the pickle")

    monkeypatch.setattr(comparison_service, "read_all_objectives", _boom)
    assert comparison_service._scenario_stats(seeded)["n_solutions"] == 2


def test_bust_invalidates_the_memo(seeded):
    comparison_service._scenario_stats(seeded)
    os.unlink(os.path.join(settings.RESULTS_ROOT, "Objectives",
                           f"{seeded}{all_objectives.SUFFIX}"))
    library_service._bust_scenario_memos()
    assert comparison_service._scenario_stats(seeded) is None


def test_traversal_scenario_name_is_skipped_without_touching_the_path(seeded, monkeypatch):
    """The old path validated via get_selector; the sibling path must validate
    itself, or a request body can steer os.path.isfile/read_pickle out of the
    results root."""
    import app.comparison_service as cs

    def _boom(*a, **kw):
        raise AssertionError("must not build a path for an unsafe name")

    monkeypatch.setattr(cs, "_obj_path", _boom)
    evil = "MOO_NSGA2_TC_g_8_a_50_n_4_v_2.5_r_2_nvisits_2/../../../../etc/passwd"
    assert cs._scenario_stats(evil) is None


def test_traversal_name_is_reported_as_skipped(seeded):
    import app.comparison_service as cs
    evil = "MOO_NSGA2_TC_g_8_a_50_n_4_v_2.5_r_2_nvisits_2/../../../../etc/passwd"
    result = cs.compare_objectives([evil])
    assert result["scenarios"] == []
    assert result["skipped"] == [evil]


class _SiblingTestSolution:
    """Module-level (not local-to-test) so pandas/pickle can serialize it."""
    mission_time = 10.0
    percentage_connectivity = 0.8
    max_disconnected_time = 1.0
    mean_disconnected_time = 0.5
    max_mean_tbv = 20.0


def test_save_run_writes_the_sibling(tmp_path, monkeypatch):
    """A saved run that has no sibling would be invisible to Compare, since the
    objectives path reads nothing else."""
    from app import optimizer_service

    monkeypatch.setattr(settings, "RESULTS_ROOT", str(tmp_path))
    os.makedirs(tmp_path / "Objectives")
    os.makedirs(tmp_path / "Solutions")

    run_dir = tmp_path / "run"
    os.makedirs(run_dir)
    pd.DataFrame({"Mission Time": [10.0]}).to_pickle(run_dir / "Objectives.pkl")
    pd.to_pickle([_SiblingTestSolution()], run_dir / "Solutions.pkl")

    optimizer_service._write_run_sibling("SCEN_NEW", str(run_dir))

    rows = all_objectives.read_all_objectives("SCEN_NEW", expected_rows=1)
    assert rows == [{
        "Mission Time": 10.0,
        "Percentage Connectivity": 0.8,
        "Max Disconnected Time": 1.0,
        "Mean Disconnected Time": 0.5,
        "Max Mean TBV": 20.0,
    }]
