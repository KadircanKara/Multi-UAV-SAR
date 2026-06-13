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


def test_read_run_config_blocks_escape_outside_results_root():
    """A scenario name that traverses out of RESULTS_ROOT must NOT read the target
    file, even when it exists and is valid JSON (path-traversal defence)."""
    from app.optimizer_service import read_run_config

    root = os.path.abspath(settings.RESULTS_ROOT)
    parent = os.path.dirname(root)
    planted = os.path.join(parent, "rc_escape_probe.json")
    with open(planted, "w") as fh:
        json.dump({"escaped": True}, fh)
    try:
        name = "../../rc_escape_probe"  # {root}/Metadata/../../rc_escape_probe.json
        # Sanity: this name genuinely resolves to the planted file outside the root,
        # so a missing guard would leak {"escaped": True} instead of None.
        resolved = os.path.abspath(os.path.join(root, "Metadata", f"{name}.json"))
        assert resolved == planted
        assert read_run_config(name) is None
    finally:
        os.remove(planted)
