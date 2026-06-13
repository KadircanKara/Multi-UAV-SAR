"""API tests for the mission run-config read endpoint."""
import json
import os

from app import settings


def test_config_endpoint_returns_sidecar(client):
    scenario = "MOO_NSGA2_TC_g_8_a_50_n_4_v_2.5_r_7_nvisits_2"
    meta_dir = os.path.join(settings.RESULTS_ROOT, "Metadata")
    os.makedirs(meta_dir, exist_ok=True)
    path = os.path.join(meta_dir, f"{scenario}.json")
    payload = {"schema_version": 1, "source": "optimizer", "pop_size": 77, "seed": 3}
    with open(path, "w") as fh:
        json.dump(payload, fh)
    try:
        resp = client.get(f"/api/library/{scenario}/config")
        assert resp.status_code == 200, resp.text
        body = resp.json()
        assert body["pop_size"] == 77 and body["source"] == "optimizer"
    finally:
        os.remove(path)


def test_config_endpoint_unrecorded_returns_flag(client):
    resp = client.get("/api/library/MOO_NSGA2_NOPE_g_8_a_50_n_4_v_2.5_r_2_nvisits_2/config")
    assert resp.status_code == 200
    assert resp.json() == {"recorded": False}
