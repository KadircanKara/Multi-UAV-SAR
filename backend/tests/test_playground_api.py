import copy

import pytest

from app.playground_export import serialize_run
from app.selector_service import get_selector

SEEDED = "MOO_NSGA2_TCD_g_8_a_50_n_4_v_2.5_r_2_nvisits_1"

# Every test builds its payload by exporting the seeded scenario above.
pytestmark = pytest.mark.needs_seed_data

def _payload():
    sel = get_selector(SEEDED)
    return serialize_run(sel.solutions, sel.F, sel.model, run_config={})

def test_playground_front_matches_seeded(client):
    seeded = client.get(f"/api/fronts/{SEEDED}").json()
    pg = client.post("/api/playground/front", json={"result": _payload()})
    assert pg.status_code == 200, pg.text
    body = pg.json()
    assert body["objectives"] == seeded["objectives"]
    assert body["n_solutions"] == seeded["n_solutions"]
    assert [s["objectives_signed"] for s in body["solutions"]] == \
           [s["objectives_signed"] for s in seeded["solutions"]]

def test_playground_replay_returns_metrics(client):
    body = {"result": _payload(), "index": 0,
            "config": {"merge_topology": "onboard", "time_model": "discrete",
                       "detection_prob": 0.8, "false_alarm_prob": 0.1,
                       "belief_threshold": 0.9, "target_locations": [12]}}
    r = client.post("/api/playground/replay", json=body)
    assert r.status_code == 200, r.text
    assert "cell_occupancy_probabilities" in r.json()

def test_playground_rejects_bad_schema_version(client):
    bad = _payload(); bad["schema_version"] = 999
    r = client.post("/api/playground/front", json={"result": bad})
    assert r.status_code == 422

def test_playground_comparison_reports_all_objectives(client):
    a = _payload()
    b_sel = get_selector("MOO_NSGA2_TCDT_g_8_a_50_n_4_v_2.5_r_2_nvisits_2")
    b = serialize_run(b_sel.solutions, b_sel.F, b_sel.model, run_config={})
    r = client.post("/api/playground/comparison", json={"results": [a, b]})
    assert r.status_code == 200, r.text
    data = r.json()
    assert len(data["scenarios"]) == 2
    for s in data["scenarios"]:
        assert set(s["objective_stats"].keys()) == {
            "Mission Time", "Percentage Connectivity",
            "Max Disconnected Time", "Mean Disconnected Time", "Max Mean TBV"}
        assert s["objective_stats"]["Mission Time"]["best"] is not None

def test_playground_malformed_model_missing_F_is_422(client):
    bad = _payload()
    del bad["model"]["F"]
    r = client.post("/api/playground/front", json={"result": bad})
    assert r.status_code == 422

def test_playground_frow_length_mismatch_is_422(client):
    bad = _payload()
    bad["solutions"][0]["f_row"].append(0.0)
    r = client.post("/api/playground/front", json={"result": bad})
    assert r.status_code == 422

def test_playground_oversized_body_is_413(client, monkeypatch):
    from app import settings
    monkeypatch.setattr(settings, "MAX_UPLOAD_BYTES", 10, raising=False)
    r = client.post("/api/playground/front", json={"result": _payload()})
    assert r.status_code == 413


def test_playground_total_path_cells_cap_is_422(client, monkeypatch):
    """The combined path-cell cap short-circuits an upload whose solutions sum to
    too many cells, BEFORE the per-cell validation loop. Monkeypatched tiny so the
    seeded payload (hundreds of solutions) trips it."""
    from app import playground_schema
    monkeypatch.setattr(playground_schema, "MAX_TOTAL_PATH_CELLS", 10, raising=False)
    r = client.post("/api/playground/front", json={"result": _payload()})
    assert r.status_code == 422


def test_playground_comparison_rejects_too_many_results(client):
    """_CompareObjReq.results is bounded so one request cannot fan out over an
    unbounded list of heavy uploads."""
    payloads = [_payload() for _ in range(9)]  # max_length is 8
    r = client.post("/api/playground/comparison", json={"results": payloads})
    assert r.status_code == 422

def test_playground_front_is_rate_limited(client, monkeypatch):
    from app import settings
    monkeypatch.setattr(settings, "OPTIMIZE_RATE_LIMIT", "1/minute", raising=False)
    client.app.state.limiter.reset()
    try:
        r1 = client.post("/api/playground/front", json={"result": _payload()})
        r2 = client.post("/api/playground/front", json={"result": _payload()})
        assert r1.status_code in (200, 429)
        assert r2.status_code == 429, r2.text
    finally:
        client.app.state.limiter.reset()


# ── Semantic guardrails: a schema-valid-but-broken upload must 422, never 500,
#    and never silently succeed on garbage. Every case reaches PathInfo /
#    PathSolution unless rejected first, so /front (light) suffices to trip them.

def _sol_patch(**patch):
    bad = _payload()
    bad["solutions"][0].update(patch)
    return bad

def _scen_patch(**patch):
    bad = _payload()
    bad["scenario"].update(patch)
    bad["scenario"]["target_positions"] = [0]   # stay in-range for any grid
    return bad

def test_playground_grid_size_over_cap_is_422(client):
    # grid_size drives a number_of_cells**2 == grid_size**4 distance matrix; the
    # playground path must honour the optimizer's MAX_GRID_SIZE ceiling (OOM DoS).
    from app import settings
    r = client.post("/api/playground/front",
                    json={"result": _scen_patch(grid_size=settings.MAX_GRID_SIZE + 1)})
    assert r.status_code == 422, r.text

def test_playground_drones_over_cap_is_422(client):
    from app import settings
    r = client.post("/api/playground/front",
                    json={"result": _scen_patch(number_of_drones=settings.MAX_DRONES + 1)})
    assert r.status_code == 422, r.text

def test_playground_empty_path_is_422(client):
    r = client.post("/api/playground/front", json={"result": _sol_patch(path=[])})
    assert r.status_code == 422, r.text

def test_playground_path_cell_out_of_grid_is_422(client):
    # grid 8 -> 64 cells; 9999 would silently wrap via path % number_of_cells.
    n = len(_payload()["solutions"][0]["path"])
    r = client.post("/api/playground/front", json={"result": _sol_patch(path=[9999] * n)})
    assert r.status_code == 422, r.text

def test_playground_path_negative_cell_is_422(client):
    n = len(_payload()["solutions"][0]["path"])
    r = client.post("/api/playground/front", json={"result": _sol_patch(path=[-5] * n)})
    assert r.status_code == 422, r.text

def test_playground_start_points_wrong_count_is_422(client):
    r = client.post("/api/playground/front", json={"result": _sol_patch(start_points=[0, 1])})
    assert r.status_code == 422, r.text

def test_playground_start_point_out_of_path_range_is_422(client):
    n = len(_payload()["solutions"][0]["path"])
    r = client.post("/api/playground/front",
                    json={"result": _sol_patch(start_points=[0, n, n + 1, n + 2])})
    assert r.status_code == 422, r.text

def test_playground_frow_non_finite_is_422_not_500(client):
    # NaN/Inf must reject cleanly. Regression guard: FastAPI's default 422 handler
    # echoes the input, and a non-finite float there used to make the error
    # payload itself unserializable -> 500. The custom handler sanitizes it.
    # Sent as raw content: httpx refuses to serialize NaN via json=, but stdlib
    # json.dumps emits the bare `NaN`/`Infinity` literals a real client can POST.
    import json as _json
    SENTINEL = "-999999999.5"   # distinctive; appears nowhere else in the payload
    for token in ("NaN", "Infinity", "-Infinity"):
        bad = _payload()
        bad["solutions"][0]["f_row"][0] = float(SENTINEL)
        raw = _json.dumps({"result": bad})
        assert raw.count(SENTINEL) == 1, "sentinel collided; pick another"
        raw = raw.replace(SENTINEL, token)
        r = client.post("/api/playground/front", content=raw,
                        headers={"content-type": "application/json"})
        assert r.status_code == 422, f"{token}: {r.status_code} {r.text[:200]}"
