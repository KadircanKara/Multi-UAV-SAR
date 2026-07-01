"""Tests for GET /api/models/{model_key}/grid."""
import math
import time


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _get_grid(client, model_key: str):
    return client.get(f"/api/models/{model_key}/grid")


# ---------------------------------------------------------------------------
# 404 cases
# ---------------------------------------------------------------------------

def test_unknown_model_key_404(client):
    resp = _get_grid(client, "NOPE")
    assert resp.status_code == 404


def test_valid_model_no_seeded_scenarios_404(client):
    # TC_MOO_NSGA3 is in the registry but has no seeded results
    resp = _get_grid(client, "TC_MOO_NSGA3")
    assert resp.status_code == 404


# ---------------------------------------------------------------------------
# Happy-path: TCD_MOO_NSGA2
# ---------------------------------------------------------------------------

def test_tcd_moo_nsga2_grid_status(client):
    resp = _get_grid(client, "TCD_MOO_NSGA2")
    assert resp.status_code == 200


def test_tcd_moo_nsga2_grid_top_level_fields(client):
    data = _get_grid(client, "TCD_MOO_NSGA2").json()
    assert data["model_key"] == "TCD_MOO_NSGA2"
    assert data["type"] == "MOO"
    assert data["algorithm"] == "NSGA2"
    assert data["objectives"] == [
        "Mission Time", "Percentage Connectivity",
        "Mean Disconnected Time", "Max Disconnected Time",
    ]
    assert data["polarities"]["Percentage Connectivity"] == -1
    assert data["polarities"]["Mission Time"] == 1
    assert isinstance(data["scenarios"], list)
    assert len(data["scenarios"]) > 0


def test_tcd_moo_nsga2_scenario_row_schema(client):
    data = _get_grid(client, "TCD_MOO_NSGA2").json()
    for row in data["scenarios"]:
        assert "scenario" in row
        assert "number_of_drones" in row
        assert "comm_range" in row
        assert "comm_range_value" in row
        assert "n_visits" in row
        assert "n_solutions" in row
        assert "result_kind" in row
        assert "objective_stats" in row
        for obj in data["objectives"]:
            assert obj in row["objective_stats"]
            stat = row["objective_stats"][obj]
            assert "min" in stat
            assert "max" in stat
            assert "mean" in stat
            assert "best" in stat


def test_tcd_moo_nsga2_scenario_row_values(client):
    data = _get_grid(client, "TCD_MOO_NSGA2").json()
    for row in data["scenarios"]:
        assert row["number_of_drones"] in {4, 8, 12, 16}
        crv = row["comm_range_value"]
        assert crv is not None
        # comm_range_value should be one of the swept ranges: 2.0, sqrt(8) ≈ 2.828, or 4.0
        assert (
            abs(crv - 2.0) < 0.01
            or abs(crv - math.sqrt(8)) < 0.01
            or abs(crv - 4.0) < 0.01
        ), f"Unexpected comm_range_value {crv}"
        assert row["n_visits"] in {1, 2, 3}
        # objective stats for Mission Time must be numeric
        mt_stat = row["objective_stats"]["Mission Time"]
        assert isinstance(mt_stat["best"], float)
        assert mt_stat["best"] > 0
        # objective stats for Percentage Connectivity must be numeric
        conn_stat = row["objective_stats"]["Percentage Connectivity"]
        assert isinstance(conn_stat["best"], float)
        assert 0 < conn_stat["best"] <= 1.0


def test_tcd_moo_nsga2_polarity_aware_best(client):
    """
    For Mission Time (polarity +1 = minimize):  best == min.
    For Percentage Connectivity (polarity -1 = maximize): best == max.
    """
    data = _get_grid(client, "TCD_MOO_NSGA2").json()
    for row in data["scenarios"]:
        mt_stat = row["objective_stats"]["Mission Time"]
        assert mt_stat["best"] == mt_stat["min"], (
            f"Mission Time best should equal min for scenario {row['scenario']}"
        )
        conn_stat = row["objective_stats"]["Percentage Connectivity"]
        assert conn_stat["best"] == conn_stat["max"], (
            f"Connectivity best should equal max for scenario {row['scenario']}"
        )


def test_tcd_moo_nsga2_sorted(client):
    """Scenarios must be sorted by (number_of_drones, comm_range_value, n_visits)."""
    data = _get_grid(client, "TCD_MOO_NSGA2").json()
    scenarios = data["scenarios"]
    keys = [
        (
            row.get("number_of_drones") or 0,
            row.get("comm_range_value") or 0.0,
            row.get("n_visits") or 0,
        )
        for row in scenarios
    ]
    assert keys == sorted(keys), "Scenarios are not sorted by (drones, comm_range_value, n_visits)"


# ---------------------------------------------------------------------------
# Parameter-effect sanity: more drones ⟹ mission time non-increasing
# ---------------------------------------------------------------------------

def test_more_drones_better_or_equal_mission_time(client):
    """
    Holding comm_range_value ≈ 2.0 and n_visits == 2 fixed,
    the best Mission Time should be non-increasing as drones go 4→8→12→16.
    Combos that are absent (no seeded data) are skipped gracefully.
    """
    data = _get_grid(client, "TCD_MOO_NSGA2").json()

    # Collect best Mission Time per drone count for r≈2.0, n_visits=2
    drone_to_best: dict[int, float] = {}
    for row in data["scenarios"]:
        if row.get("n_visits") != 2:
            continue
        crv = row.get("comm_range_value")
        if crv is None or abs(crv - 2.0) > 0.01:
            continue
        n = row["number_of_drones"]
        best = row["objective_stats"]["Mission Time"]["best"]
        if best is not None:
            drone_to_best[n] = best

    # Need at least two data points to assert monotone trend
    assert len(drone_to_best) >= 2, (
        f"Expected at least 2 drone counts for r≈2.0, n_visits=2; got {drone_to_best}"
    )

    ordered = sorted(drone_to_best.items())  # [(n, best_mt), ...]
    for i in range(1, len(ordered)):
        prev_n, prev_mt = ordered[i - 1]
        curr_n, curr_mt = ordered[i]
        assert curr_mt <= prev_mt + 1e-6, (
            f"Mission Time should be non-increasing with more drones: "
            f"n={prev_n} → {prev_mt:.2f}, n={curr_n} → {curr_mt:.2f}"
        )


# ---------------------------------------------------------------------------
# Cheap endpoint: only Objectives pickles should be loaded (structural)
# ---------------------------------------------------------------------------

def test_grid_is_cheap(client):
    """
    The endpoint must NOT load heavy Solution pickles.
    We verify structurally: the route only calls model_grid(), which scans
    Objectives/ and checks Solutions/ existence but never reads Solutions.
    As a runtime proxy, the endpoint should complete in well under 5 seconds
    for TCD_MOO_NSGA2 (36 scenarios × small pkl).
    """
    start = time.monotonic()
    resp = _get_grid(client, "TCD_MOO_NSGA2")
    elapsed = time.monotonic() - start
    assert resp.status_code == 200
    assert elapsed < 5.0, f"Grid endpoint took {elapsed:.2f}s — suspiciously slow"
