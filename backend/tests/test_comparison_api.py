"""Tests for POST /api/comparison — cross-model objective comparison."""
import pytest

ALL_OBJECTIVES = [
    "Mission Time",
    "Percentage Connectivity",
    "Max Disconnected Time",
    "Mean Disconnected Time",
    "Max Mean TBV",
]


# ---------------------------------------------------------------------------
# Helpers — discover real seeded scenario names via the grid endpoint
# ---------------------------------------------------------------------------

def _scenario_with(client, model_key: str, n_visits: int):
    """Return a seeded scenario name for *model_key* with the given n_visits."""
    grid = client.get(f"/api/models/{model_key}/grid").json()
    for row in grid["scenarios"]:
        if row.get("n_visits") == n_visits:
            return row["scenario"]
    return None


def _compare(client, scenarios):
    return client.post("/api/comparison", json={"scenarios": scenarios})


# ---------------------------------------------------------------------------
# Happy path
# ---------------------------------------------------------------------------

@pytest.mark.needs_seed_data
def test_compare_two_models_status_and_shape(client):
    a = _scenario_with(client, "TCD_MOO_NSGA2", 2)
    b = _scenario_with(client, "TCDT_MOO_NSGA2", 2)
    assert a and b, "expected seeded n_visits=2 scenarios for TCD and TCDT"

    resp = _compare(client, [a, b])
    assert resp.status_code == 200
    data = resp.json()

    assert data["objectives"] == ALL_OBJECTIVES
    assert data["polarities"]["Percentage Connectivity"] == -1
    assert data["polarities"]["Mission Time"] == 1
    assert data["skipped"] == []
    assert [s["scenario"] for s in data["scenarios"]] == [a, b]


@pytest.mark.needs_seed_data
def test_every_scenario_reports_all_objectives(client):
    a = _scenario_with(client, "TCD_MOO_NSGA2", 2)
    b = _scenario_with(client, "TCDT_MOO_NSGA2", 2)
    data = _compare(client, [a, b]).json()
    for s in data["scenarios"]:
        assert set(s["objective_stats"].keys()) == set(ALL_OBJECTIVES)
        for obj in ALL_OBJECTIVES:
            stat = s["objective_stats"][obj]
            assert stat is not None
            assert stat["best"] is not None


@pytest.mark.needs_seed_data
def test_unoptimized_objective_is_still_computed(client):
    """The core capability: TCD does NOT optimise Max Mean TBV, yet it must be
    reported (computed from the solution objects) so it can be compared to
    TCDT, which does optimise it."""
    tcd = _scenario_with(client, "TCD_MOO_NSGA2", 2)
    data = _compare(client, [tcd]).json()
    s = data["scenarios"][0]
    assert "Max Mean TBV" not in s["optimized_objectives"]
    tbv = s["objective_stats"]["Max Mean TBV"]
    assert tbv is not None and tbv["best"] is not None and tbv["best"] > 0


# ---------------------------------------------------------------------------
# n_visits == 1 → Max Mean TBV is undefined (null)
# ---------------------------------------------------------------------------

@pytest.mark.needs_seed_data
def test_tbv_is_null_at_n_visits_1(client):
    sc = _scenario_with(client, "TCD_MOO_NSGA2", 1)
    assert sc, "expected a seeded TCD n_visits=1 scenario"
    data = _compare(client, [sc]).json()
    s = data["scenarios"][0]
    assert s["n_visits"] == 1
    assert s["objective_stats"]["Max Mean TBV"] is None
    # other objectives are still present
    assert s["objective_stats"]["Mission Time"]["best"] is not None


# ---------------------------------------------------------------------------
# Skipping + error cases
# ---------------------------------------------------------------------------

@pytest.mark.needs_seed_data
def test_unknown_scenario_is_skipped_not_fatal(client):
    good = _scenario_with(client, "TCD_MOO_NSGA2", 2)
    data = _compare(client, [good, "totally_bogus_scenario"]).json()
    assert [s["scenario"] for s in data["scenarios"]] == [good]
    assert data["skipped"] == ["totally_bogus_scenario"]


def test_all_unknown_returns_404(client):
    resp = _compare(client, ["nope_one", "nope_two"])
    assert resp.status_code == 404


def test_empty_scenarios_returns_422(client):
    resp = _compare(client, [])
    assert resp.status_code == 422


def test_too_many_scenarios_returns_422(client):
    resp = _compare(client, [f"x_{i}" for i in range(361)])  # max_length = 360
    assert resp.status_code == 422


@pytest.mark.needs_seed_data
def test_duplicates_collapsed(client):
    sc = _scenario_with(client, "TCD_MOO_NSGA2", 2)
    data = _compare(client, [sc, sc]).json()
    assert len(data["scenarios"]) == 1


# ---------------------------------------------------------------------------
# POST /api/comparison/time — cross-model time-metric comparison
# ---------------------------------------------------------------------------

TIME_METRICS = [
    "Effective Mission Time",
    "Detection Time",
    "Inform Time",
    "Time At Least One Drone Knows All Targets",
]

_VALID_CONFIG = {
    "merge_topology": "onboard",
    "time_model": "discrete",
    "detection_prob": 0.8,
    "false_alarm_prob": 0.1,
    "belief_threshold": 0.9,
    "target_locations": [12],
}


def _compare_time(client, scenarios, config=None, strategy="balanced", **extra):
    body = {
        "scenarios": scenarios,
        "config": config if config is not None else _VALID_CONFIG,
        "strategy": strategy,
        **extra,
    }
    return client.post("/api/comparison/time", json=body)


@pytest.mark.needs_seed_data
def test_compare_time_happy_path(client):
    a = _scenario_with(client, "TCD_MOO_NSGA2", 2)
    b = _scenario_with(client, "TCDT_MOO_NSGA2", 2)
    assert a and b, "expected seeded n_visits=2 scenarios for TCD and TCDT"

    resp = _compare_time(client, [a, b], strategy="balanced")
    assert resp.status_code == 200
    data = resp.json()

    assert data["metrics"] == TIME_METRICS
    assert data["strategy"] == "balanced"
    assert data["skipped"] == []
    assert [s["scenario"] for s in data["scenarios"]] == [a, b]
    for s in data["scenarios"]:
        assert set(s["metric_values"].keys()) == set(TIME_METRICS)


@pytest.mark.needs_seed_data
def test_compare_time_best_without_objective_is_422(client):
    a = _scenario_with(client, "TCD_MOO_NSGA2", 2)
    assert a
    resp = _compare_time(client, [a], strategy="best")
    assert resp.status_code == 422


@pytest.mark.needs_seed_data
def test_compare_time_best_skips_model_without_that_objective(client):
    """'best' on an objective a given scenario's model did not optimize must skip
    that scenario, not fail the whole multi-scenario comparison."""
    tc = _scenario_with(client, "TCD_MOO_NSGA2", 2)     # does NOT optimize TBV
    tcdt = _scenario_with(client, "TCDT_MOO_NSGA2", 2)  # optimizes Max Mean TBV
    assert tc and tcdt
    resp = _compare_time(
        client, [tc, tcdt], strategy="best", objective_name="Max Mean TBV"
    )
    assert resp.status_code == 200, resp.text
    data = resp.json()
    assert tcdt in [s["scenario"] for s in data["scenarios"]]
    assert tc in data["skipped"]


@pytest.mark.needs_seed_data
def test_compare_time_bad_config_is_422(client):
    a = _scenario_with(client, "TCD_MOO_NSGA2", 2)
    assert a
    bad_config = {**_VALID_CONFIG, "detection_prob": 0.1, "false_alarm_prob": 0.2}
    resp = _compare_time(client, [a], config=bad_config)
    assert resp.status_code == 422


@pytest.mark.needs_seed_data
def test_compare_time_bogus_scenario_is_skipped(client):
    good = _scenario_with(client, "TCD_MOO_NSGA2", 2)
    assert good
    resp = _compare_time(client, [good, "totally_bogus_scenario"])
    assert resp.status_code == 200
    data = resp.json()
    assert [s["scenario"] for s in data["scenarios"]] == [good]
    assert data["skipped"] == ["totally_bogus_scenario"]


def test_compare_time_all_bogus_returns_404(client):
    resp = _compare_time(client, ["nope_one", "nope_two"])
    assert resp.status_code == 404


def test_compare_time_empty_scenarios_returns_422(client):
    resp = _compare_time(client, [])
    assert resp.status_code == 422


@pytest.mark.needs_seed_data
def test_compare_time_single_result_model_uses_lone_solution(client):
    """Single-solution models (MTSP) have no Pareto front, so a front strategy
    like 'balanced' must fall back to the one solution rather than 422."""
    mtsp = _scenario_with(client, "MTSP", 2)
    assert mtsp, "expected a seeded MTSP n_visits=2 scenario"
    resp = _compare_time(client, [mtsp], strategy="balanced")
    assert resp.status_code == 200
    data = resp.json()
    s = data["scenarios"][0]
    assert s["selected_index"] == 0
    assert all(s["metric_values"][m] is not None for m in data["metrics"])
