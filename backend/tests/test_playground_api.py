from app.playground_export import serialize_run
from app.selector_service import get_selector

SEEDED = "MOO_NSGA2_TCD_g_8_a_50_n_4_v_2.5_r_2_nvisits_1"

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
