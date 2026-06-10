"""Tests for the Optimizer endpoints (POST /api/optimize[/check], GET /api/optimize/{id})."""
import time

BASE_SCENARIO = {"number_of_drones": 4, "n_visits": 2}


def _cfg(**over):
    body = {
        "optimization_type": "MOO",
        "method": "NSGA2",
        "objectives": ["Mission Time", "Percentage Connectivity"],
        "pop_size": 12,
        "n_gen": 5,
        "scenario": BASE_SCENARIO,
    }
    body.update(over)
    return body


def _runnable(**over):
    """Disable the G-constraints (the speed constraint is always applied) so runs
    are cheaper. NOTE: even so, the path-adjacency speed constraint needs ~pop100
    to find any feasible solution — tiny runs legitimately return 0 solutions."""
    body = _cfg(pop_size=12, n_gen=5, max_mission_time=None, min_connectivity=None)
    body.update(over)
    return body


# ─── check / model synthesis ──────────────────────────────────────────────────

def test_check_preset_combo_resolves_and_exists(client):
    # MOO NSGA2 over {Mission Time, %Connectivity} == preset TC_MOO_NSGA2 (seeded).
    resp = client.post("/api/optimize/check", json=_cfg(scenario={"number_of_drones": 4, "n_visits": 1}))
    assert resp.status_code == 200
    d = resp.json()
    assert d["model_key"] == "TC_MOO_NSGA2"
    assert d["scenario_name"].startswith("MOO_NSGA2_TC_")
    assert d["exists"] is True


def test_check_novel_combo_does_not_exist(client):
    # SOO-GA over a single TBV objective is not a seeded preset.
    resp = client.post("/api/optimize/check", json=_cfg(
        optimization_type="SOO", method="GA", objectives=["Max Mean TBV"]))
    assert resp.status_code == 200
    assert resp.json()["exists"] is False


# ─── validators ───────────────────────────────────────────────────────────────

def test_soo_ga_requires_exactly_one_objective(client):
    resp = client.post("/api/optimize/check", json=_cfg(
        optimization_type="SOO", method="GA",
        objectives=["Mission Time", "Percentage Connectivity"]))
    assert resp.status_code == 422


def test_ws_weights_must_sum_to_one(client):
    resp = client.post("/api/optimize/check", json=_cfg(
        optimization_type="SOO", method="WS",
        objectives=["Mission Time", "Percentage Connectivity"],
        weights={"Mission Time": 0.5, "Percentage Connectivity": 0.4}))
    assert resp.status_code == 422


def test_ws_weights_sum_one_ok(client):
    resp = client.post("/api/optimize/check", json=_cfg(
        optimization_type="SOO", method="WS",
        objectives=["Mission Time", "Percentage Connectivity"],
        weights={"Mission Time": 0.6, "Percentage Connectivity": 0.4}))
    assert resp.status_code == 200


def test_ws_weights_rounding_tolerated(client):
    # Equal split over 3 objectives rounds to 0.3333 each (sum 0.9999). Four-decimal
    # rounding must be tolerated — it previously 422'd against a too-tight 1e-6
    # tolerance while the frontend's 1e-3 run-gate let it through.
    resp = client.post("/api/optimize/check", json=_cfg(
        optimization_type="SOO", method="WS",
        objectives=["Mission Time", "Percentage Connectivity", "Max Mean TBV"],
        weights={
            "Mission Time": 0.3333,
            "Percentage Connectivity": 0.3333,
            "Max Mean TBV": 0.3333,
        }))
    assert resp.status_code == 200


def test_moead_blocked(client):
    resp = client.post("/api/optimize/check", json=_cfg(method="MOEAD"))
    assert resp.status_code == 422


def test_moo_requires_two_objectives(client):
    resp = client.post("/api/optimize/check", json=_cfg(objectives=["Mission Time"]))
    assert resp.status_code == 422


def test_pop_size_and_gen_caps(client):
    # New caps: pop_size ≤ 500, n_gen ≤ 1000.
    assert client.post("/api/optimize/check", json=_cfg(pop_size=500, n_gen=1000)).status_code == 200
    assert client.post("/api/optimize/check", json=_cfg(pop_size=501)).status_code == 422
    assert client.post("/api/optimize/check", json=_cfg(n_gen=1001)).status_code == 422


def test_constraint_values_validated(client):
    assert client.post("/api/optimize/check", json=_cfg(max_mission_time=0)).status_code == 422
    assert client.post("/api/optimize/check", json=_cfg(min_connectivity=1.5)).status_code == 422
    # Disabling constraints (null) is allowed.
    assert client.post("/api/optimize/check", json=_cfg(max_mission_time=None, min_connectivity=None)).status_code == 200


def test_unknown_run_id_404(client):
    assert client.get("/api/optimize/nope").status_code == 404


def test_stop_unknown_run_404(client):
    assert client.post("/api/optimize/nope/stop").status_code == 404


def test_stop_run_returns_partial_front(client):
    # A run big enough that it's still going when we ask it to stop.
    cfg = _runnable(
        optimization_type="MOO", method="NSGA2",
        objectives=["Mission Time", "Percentage Connectivity"],
        pop_size=40, n_gen=80,
    )
    run_id = client.post("/api/optimize", json=cfg).json()["run_id"]

    # Let it run a few generations so the stop lands mid-run.
    for _ in range(60):
        s = client.get(f"/api/optimize/{run_id}").json()
        if s["state"] != "running" or (s.get("gen") or 0) >= 2:
            break
        time.sleep(0.2)

    stop = client.post(f"/api/optimize/{run_id}/stop")
    assert stop.status_code == 200

    front = None
    for _ in range(60):
        s = client.get(f"/api/optimize/{run_id}").json()
        if s["state"] == "done":
            front = s["front"]
            break
        if s["state"] == "failed":
            raise AssertionError(s.get("error"))
        time.sleep(0.5)
    assert front is not None, "run did not finish after stop"
    # If the stop landed while running, the front is the cancelled best-so-far,
    # halted well before the requested 80 generations.
    if stop.json()["stopping"]:
        assert front["cancelled"] is True
        assert front["stopped_at_gen"] < 80


# ─── a real tiny run end-to-end ───────────────────────────────────────────────

def test_run_to_completion_returns_front(client):
    start = client.post("/api/optimize", json=_runnable())
    assert start.status_code == 200
    run_id = start.json()["run_id"]

    front = None
    for _ in range(60):  # up to ~60s
        s = client.get(f"/api/optimize/{run_id}").json()
        if s["state"] == "done":
            front = s["front"]
            break
        if s["state"] == "failed":
            raise AssertionError(f"run failed: {s.get('error')}")
        time.sleep(1)
    # The run completes; with the always-on speed constraint a tiny run may find
    # 0 feasible solutions (a valid "no feasible solution found" outcome).
    assert front is not None, "run did not finish in time"
    assert front["objectives"] == ["Mission Time", "Percentage Connectivity"]
    assert front["n_solutions"] >= 0 and "solutions" in front


def test_save_run_makes_it_browsable(client):
    import os
    from app import settings

    # A custom (non-preset) combo so it doesn't collide with a seeded scenario.
    # pop100/gen100 reliably finds feasible solutions under the speed constraint.
    cfg = _runnable(
        optimization_type="MOO", method="NSGA2",
        objectives=["Mission Time", "Mean Disconnected Time"],
        pop_size=100, n_gen=100,
    )
    start = client.post("/api/optimize", json=cfg).json()
    run_id, scenario = start["run_id"], start["scenario_name"]

    for _ in range(60):
        s = client.get(f"/api/optimize/{run_id}").json()
        if s["state"] == "done":
            break
        if s["state"] == "failed":
            raise AssertionError(s.get("error"))
        time.sleep(1)

    obj_pkl = os.path.join(settings.RESULTS_ROOT, "Objectives", f"{scenario}-ObjectiveValues.pkl")
    sol_pkl = os.path.join(settings.RESULTS_ROOT, "Solutions", f"{scenario}-SolutionObjects.pkl")
    try:
        save = client.post(f"/api/optimize/{run_id}/save", json={"overwrite": True})
        assert save.status_code == 200
        assert save.json()["scenario_name"] == scenario
        # Now browsable via the existing fronts endpoint (custom model resolved).
        front = client.get(f"/api/fronts/{scenario}")
        assert front.status_code == 200
        assert "Mission Time" in front.json()["objectives"]
    finally:
        for p in (obj_pkl, sol_pkl):
            if os.path.isfile(p):
                os.remove(p)
