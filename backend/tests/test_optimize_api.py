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
    # MOO NSGA2 over TCD's objective set == preset TCD_MOO_NSGA2 (seeded on EC2).
    resp = client.post("/api/optimize/check", json=_cfg(
        objectives=["Mission Time", "Percentage Connectivity",
                    "Mean Disconnected Time", "Max Disconnected Time"],
        scenario={"number_of_drones": 4, "n_visits": 1}))
    assert resp.status_code == 200
    d = resp.json()
    assert d["model_key"] == "TCD_MOO_NSGA2"
    assert d["scenario_name"].startswith("MOO_NSGA2_TCD_")
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


def test_gen_strategy_max_is_accepted(client):
    assert client.post("/api/optimize/check", json=_cfg(gen_strategy="max")).status_code == 200
    # default is fixed
    assert client.post("/api/optimize/check", json=_cfg()).status_code == 200


def test_gen_strategy_invalid_is_422(client):
    assert client.post("/api/optimize/check", json=_cfg(gen_strategy="bogus")).status_code == 422


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


def test_max_mean_tbv_constraint_validated(client):
    assert client.post("/api/optimize/check", json=_cfg(max_mean_tbv=0)).status_code == 422
    assert client.post("/api/optimize/check", json=_cfg(max_mean_tbv=40)).status_code == 200
    # Omitted / null disables the constraint (the default).
    assert client.post("/api/optimize/check", json=_cfg(max_mean_tbv=None)).status_code == 200


def test_max_mean_tbv_added_to_model_constraints():
    """A TBV ceiling adds the 'Max Mean TBV Ceiling' inequality constraint to G,
    and the registered fn is feasible (<=0) iff Max Mean TBV <= the threshold."""
    from app.optimizer_service import resolve_model
    from PathFuncDict import model_metric_info

    _key, model = resolve_model(
        "MOO", "NSGA2", ["Mission Time", "Percentage Connectivity"],
        max_mission_time=None, min_connectivity=None, max_mean_tbv=40.0)
    assert "Max Mean TBV Ceiling" in model["G"]
    assert "Max Mean TBV Ceiling" in model_metric_info["Constraints"]

    fn = model_metric_info["Constraints"]["Max Mean TBV Ceiling"]

    class _Info:
        max_mean_tbv_constraint = 40.0

    sol = type("S", (), {})()
    sol.info = _Info()
    sol.max_mean_tbv = 30.0
    assert fn(sol) <= 0  # 30 <= 40 → feasible
    sol.max_mean_tbv = 55.0
    assert fn(sol) > 0   # 55 > 40 → infeasible


def test_max_mean_tbv_omitted_leaves_g_without_it():
    from app.optimizer_service import resolve_model
    _key, model = resolve_model(
        "MOO", "NSGA2", ["Mission Time", "Percentage Connectivity"],
        max_mission_time=None, min_connectivity=None, max_mean_tbv=None)
    assert "Max Mean TBV Ceiling" not in model["G"]


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


def test_save_empty_run_is_rejected(client, monkeypatch):
    """A run that found 0 feasible solutions must not be persisted — an empty
    front in the library breaks selection/replay endpoints with 500s."""
    import os
    import shutil
    from concurrent.futures import Future

    import pandas as pd
    from app import optimizer_service, settings

    run_id = "emptyrun_test"
    run_dir = os.path.join(settings.RESULTS_ROOT, ".runs", run_id)
    os.makedirs(run_dir, exist_ok=True)
    pd.to_pickle(
        pd.DataFrame(columns=["Mission Time", "Percentage Connectivity"]),
        os.path.join(run_dir, "Objectives.pkl"),
    )
    pd.to_pickle([], os.path.join(run_dir, "Solutions.pkl"))

    fut: Future = Future()
    fut.set_result({"n_solutions": 0, "solutions": []})
    scenario_name = "MOO_NSGA2_ZZQ_g_8_a_50_n_4_v_2.5_r_2_nvisits_2"
    optimizer_service._jobs[run_id] = {
        "future": fut,
        "run_dir": run_dir,
        "scenario_name": scenario_name,
        "model_key": "ZZQ_MOO_NSGA2",
        "model_dict": {
            "Type": "MOO", "Exp": "ZZQ", "Alg": "NSGA2",
            "F": ["Mission Time", "Percentage Connectivity"], "G": [], "H": [],
        },
    }
    obj = os.path.join(
        settings.RESULTS_ROOT, "Objectives", f"{scenario_name}-ObjectiveValues.pkl"
    )
    reg_path = os.path.join(settings.RESULTS_ROOT, "custom_models.json")
    reg_backup = open(reg_path, "rb").read() if os.path.isfile(reg_path) else None
    try:
        monkeypatch.setattr("app.settings.ALLOW_LIBRARY_SAVE", True)
        resp = client.post(f"/api/optimize/{run_id}/save", json={"overwrite": True})
        assert resp.status_code == 422, resp.text
        assert not os.path.isfile(obj), "empty run must not be written to the library"
    finally:
        optimizer_service._jobs.pop(run_id, None)
        shutil.rmtree(run_dir, ignore_errors=True)
        if os.path.isfile(obj):
            os.remove(obj)
        # Restore the registry in case the guard ever regresses and registers it.
        if reg_backup is not None:
            with open(reg_path, "wb") as fh:
                fh.write(reg_backup)
        elif os.path.isfile(reg_path):
            os.remove(reg_path)
        from app import models_registry
        models_registry._invalidate_cache()


def test_finished_run_recovers_from_disk_after_restart(client, monkeypatch):
    """A finished run whose in-memory _jobs entry was lost (backend restart) is
    still pollable and saveable from its on-disk run dir."""
    import json
    import os
    import shutil
    from concurrent.futures import Future  # noqa: F401  (kept for symmetry)

    import pandas as pd
    from app import optimizer_service, settings

    run_id = "abcdef012345"  # hex, like uuid4().hex[:12]
    run_dir = os.path.join(settings.RESULTS_ROOT, ".runs", run_id)
    os.makedirs(run_dir, exist_ok=True)
    scenario = "MOO_NSGA2_ZZD_g_8_a_50_n_4_v_2.5_r_9_nvisits_2"
    model_dict = {"Type": "MOO", "Exp": "ZZD", "Alg": "NSGA2",
                  "F": ["Mission Time", "Percentage Connectivity"], "G": [], "H": []}
    front = {
        "scenario": scenario, "model_key": "ZZD_MOO_NSGA2",
        "objectives": ["Mission Time", "Percentage Connectivity"],
        "polarities": {"Mission Time": 1, "Percentage Connectivity": -1},
        "result_kind": "front", "n_solutions": 2,
        "solutions": [{"index": 0, "objectives_signed": {}, "objectives_abs": {}}],
        "cancelled": False, "stopped_at_gen": 5,
    }
    with open(os.path.join(run_dir, "status.json"), "w") as fh:
        json.dump({"state": "done", "gen": 5, "n_gen": 5, "front": front}, fh)
    with open(os.path.join(run_dir, "meta.json"), "w") as fh:
        json.dump({"scenario_name": scenario, "model_key": "ZZD_MOO_NSGA2",
                   "model_dict": model_dict}, fh)
    pd.to_pickle(
        pd.DataFrame({"Mission Time": [1.0, 2.0], "Percentage Connectivity": [-0.5, -0.4]}),
        os.path.join(run_dir, "Objectives.pkl"),
    )
    pd.to_pickle(["A", "B"], os.path.join(run_dir, "Solutions.pkl"))
    optimizer_service._jobs.pop(run_id, None)  # simulate the wiped registry

    reg_path = os.path.join(settings.RESULTS_ROOT, "custom_models.json")
    reg_backup = open(reg_path, "rb").read() if os.path.isfile(reg_path) else None
    obj_lib = os.path.join(settings.RESULTS_ROOT, "Objectives", f"{scenario}-ObjectiveValues.pkl")
    sol_lib = os.path.join(settings.RESULTS_ROOT, "Solutions", f"{scenario}-SolutionObjects.pkl")
    try:
        s = client.get(f"/api/optimize/{run_id}")
        assert s.status_code == 200, s.text
        assert s.json()["state"] == "done"

        monkeypatch.setattr("app.settings.ALLOW_LIBRARY_SAVE", True)
        save = client.post(f"/api/optimize/{run_id}/save", json={"overwrite": True})
        assert save.status_code == 200, save.text
        assert os.path.isfile(obj_lib)
    finally:
        optimizer_service._jobs.pop(run_id, None)
        shutil.rmtree(run_dir, ignore_errors=True)
        for p in (obj_lib, sol_lib):
            if os.path.isfile(p):
                os.remove(p)
        if reg_backup is not None:
            with open(reg_path, "wb") as fh:
                fh.write(reg_backup)
        elif os.path.isfile(reg_path):
            os.remove(reg_path)
        from app import models_registry
        models_registry._invalidate_cache()


def test_save_overwrites_existing_scenario_and_writes_sidecar(client, monkeypatch):
    """A finished run whose scenario already exists overwrites it on overwrite=True
    (seeded missions included) and writes the RunConfig sidecar. A fabricated
    preset-key scenario (r_9 is never seeded) keeps the test off real seed data."""
    import json
    import os
    import shutil
    from concurrent.futures import Future

    import pandas as pd
    from app import optimizer_service, settings

    scenario = "MOO_NSGA2_TC_g_8_a_50_n_4_v_2.5_r_9_nvisits_2"
    obj_dir = os.path.join(settings.RESULTS_ROOT, "Objectives")
    sol_dir = os.path.join(settings.RESULTS_ROOT, "Solutions")
    meta_dir = os.path.join(settings.RESULTS_ROOT, "Metadata")
    for d in (obj_dir, sol_dir):
        os.makedirs(d, exist_ok=True)
    existing_obj = os.path.join(obj_dir, f"{scenario}-ObjectiveValues.pkl")
    existing_sol = os.path.join(sol_dir, f"{scenario}-SolutionObjects.pkl")
    meta_dst = os.path.join(meta_dir, f"{scenario}.json")
    pd.to_pickle(pd.DataFrame({"Mission Time": [1.0]}), existing_obj)
    pd.to_pickle(["OLD"], existing_sol)

    run_id = "overwrite_test1"
    run_dir = os.path.join(settings.RESULTS_ROOT, ".runs", run_id)
    os.makedirs(run_dir, exist_ok=True)
    pd.to_pickle(
        pd.DataFrame({"Mission Time": [9.0, 8.0], "Percentage Connectivity": [-0.1, -0.2]}),
        os.path.join(run_dir, "Objectives.pkl"),
    )
    pd.to_pickle(["NEW_A", "NEW_B"], os.path.join(run_dir, "Solutions.pkl"))
    with open(os.path.join(run_dir, "config.json"), "w") as fh:
        json.dump({"schema_version": 1, "source": "optimizer", "pop_size": 42}, fh)

    fut: Future = Future()
    fut.set_result({"n_solutions": 2, "solutions": []})
    optimizer_service._jobs[run_id] = {
        "future": fut, "run_dir": run_dir, "scenario_name": scenario,
        "model_key": "TC_MOO_NSGA2",
        "model_dict": {"Type": "MOO", "Exp": "TC", "Alg": "NSGA2",
                       "F": ["Mission Time", "Percentage Connectivity"], "G": [], "H": []},
    }
    reg_path = os.path.join(settings.RESULTS_ROOT, "custom_models.json")
    reg_backup = open(reg_path, "rb").read() if os.path.isfile(reg_path) else None
    try:
        monkeypatch.setattr("app.settings.ALLOW_LIBRARY_SAVE", True)
        # Without overwrite → 409 (scenario exists).
        no = client.post(f"/api/optimize/{run_id}/save", json={"overwrite": False})
        assert no.status_code == 409, no.text
        # With overwrite → 200, pkls replaced, sidecar written.
        ok = client.post(f"/api/optimize/{run_id}/save", json={"overwrite": True})
        assert ok.status_code == 200, ok.text
        assert pd.read_pickle(existing_sol) == ["NEW_A", "NEW_B"]
        assert os.path.isfile(meta_dst), "RunConfig sidecar must be written on save"
        with open(meta_dst) as fh:
            assert json.load(fh)["pop_size"] == 42
    finally:
        optimizer_service._jobs.pop(run_id, None)
        shutil.rmtree(run_dir, ignore_errors=True)
        for p in (existing_obj, existing_sol, meta_dst):
            if os.path.isfile(p):
                os.remove(p)
        if reg_backup is not None:
            with open(reg_path, "wb") as fh:
                fh.write(reg_backup)
        elif os.path.isfile(reg_path):
            os.remove(reg_path)
        from app import models_registry
        models_registry._invalidate_cache()


def test_serialize_finished_run_roundtrips(client):
    import time
    from app import optimizer_service
    from app.playground_schema import PlaygroundResult
    from app.playground_reconstruct import reconstruct_selector

    start = client.post("/api/optimize", json=_runnable(
        optimization_type="MOO", method="NSGA2",
        objectives=["Mission Time", "Mean Disconnected Time"],
        pop_size=100, n_gen=100))
    run_id = start.json()["run_id"]
    for _ in range(180):
        if client.get(f"/api/optimize/{run_id}").json()["state"] != "running":
            break
        time.sleep(1)
    s = client.get(f"/api/optimize/{run_id}").json()
    if s.get("front", {}).get("n_solutions", 0) == 0:
        import pytest
        pytest.skip("tiny run found no feasible solutions this seed")

    payload = optimizer_service.serialize_finished_run(run_id)
    result = PlaygroundResult.model_validate(payload)         # validates against schema
    assert result.model["model_key"].endswith("_MOO_NSGA2")   # full resolved key, not bare Exp
    sel = reconstruct_selector(result)                        # round-trips
    assert sel.n_solutions == len(result.solutions)


def test_save_run_makes_it_browsable(client, monkeypatch):
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

    state, front_meta = None, None
    for _ in range(180):  # generous budget — a loaded machine can exceed 60s
        s = client.get(f"/api/optimize/{run_id}").json()
        state = s["state"]
        if state == "done":
            front_meta = s["front"]
            break
        if state == "failed":
            raise AssertionError(s.get("error"))
        time.sleep(1)
    assert state == "done", f"run did not finish in time (last state: {state})"

    obj_pkl = os.path.join(settings.RESULTS_ROOT, "Objectives", f"{scenario}-ObjectiveValues.pkl")
    sol_pkl = os.path.join(settings.RESULTS_ROOT, "Solutions", f"{scenario}-SolutionObjects.pkl")
    try:
        monkeypatch.setattr("app.settings.ALLOW_LIBRARY_SAVE", True)
        save = client.post(f"/api/optimize/{run_id}/save", json={"overwrite": True})
        if front_meta["n_solutions"] == 0:
            # A run with no feasible solutions must be refused (empty library
            # scenarios break selection/replay). Browse-ability of a *non-empty*
            # custom run is covered by test_custom_model_resolution.
            assert save.status_code == 422
            assert not os.path.isfile(obj_pkl)
        else:
            assert save.status_code == 200, save.text
            assert save.json()["scenario_name"] == scenario
            # Now browsable via the existing fronts endpoint (custom model resolved).
            front = client.get(f"/api/fronts/{scenario}")
            assert front.status_code == 200
            assert "Mission Time" in front.json()["objectives"]
    finally:
        for p in (obj_pkl, sol_pkl):
            if os.path.isfile(p):
                os.remove(p)


def test_save_disabled_by_default_is_403(client):
    # No ALLOW_LIBRARY_SAVE → memoryless: saving to the library is refused.
    resp = client.post("/api/optimize/whatever/save", json={"overwrite": False})
    assert resp.status_code == 403


# ─── export endpoint ──────────────────────────────────────────────────────────

def test_export_unknown_run_404(client):
    assert client.get("/api/optimize/deadbeef/export").status_code == 404


def test_export_finished_run_is_downloadable(client):
    start = client.post("/api/optimize", json=_runnable(
        optimization_type="MOO", method="NSGA2",
        objectives=["Mission Time", "Mean Disconnected Time"],
        pop_size=100, n_gen=100))
    run_id = start.json()["run_id"]
    for _ in range(180):
        if client.get(f"/api/optimize/{run_id}").json()["state"] != "running":
            break
        time.sleep(1)
    resp = client.get(f"/api/optimize/{run_id}/export")
    if resp.status_code == 422:
        import pytest; pytest.skip("no feasible solutions this seed")
    assert resp.status_code == 200
    assert "attachment" in resp.headers.get("content-disposition", "")
    assert resp.json()["schema_version"] == 1
