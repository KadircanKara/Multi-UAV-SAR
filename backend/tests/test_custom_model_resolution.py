"""
A saved optimizer run registers a *custom* (non-preset) model in
``Results/custom_models.json`` and becomes browsable. Every read path that
resolves a model key must consult ``models_registry`` (not just the preset
``AVAILABLE_MODELS``), or the saved run silently vanishes from comparison /
its model-grid page / scenario-validate / the models list.

These tests synthesize a custom scenario by cloning a seeded scenario's pickles
under a name that resolves to a freshly-registered custom key, then assert each
endpoint resolves it. The fixture restores ``custom_models.json`` and removes the
cloned pickles on teardown.
"""
import os
import shutil

import pytest

from app import models_registry, settings

# resolve_model_key("MOO_NSGA2_ZZ_g_...") -> "ZZ_MOO_NSGA2" (a non-preset key).
CUSTOM_KEY = "ZZ_MOO_NSGA2"
CUSTOM_SCENARIO = "MOO_NSGA2_ZZ_g_8_a_50_n_4_v_2.5_r_2_nvisits_2"
CUSTOM_MODEL = {
    "Type": "MOO",
    "Exp": "ZZ",
    "Alg": "NSGA2",
    "F": ["Mission Time", "Mean Disconnected Time"],
    "G": [],
    "H": ["Path Speed Violations as Constraint"],
}


def _seed_scenario(client):
    """A seeded MOO/NSGA2 scenario (n_visits=2) whose pickles we can clone."""
    grid = client.get("/api/models/TCD_MOO_NSGA2/grid").json()
    for row in grid.get("scenarios", []):
        if row.get("n_visits") == 2:
            return row["scenario"]
    return None


@pytest.fixture
def custom_scenario(client):
    src = _seed_scenario(client)
    if not src:
        pytest.skip("no seeded TCD_MOO_NSGA2 n_visits=2 scenario to clone")

    def _obj(s):
        return os.path.join(settings.RESULTS_ROOT, "Objectives", f"{s}-ObjectiveValues.pkl")

    def _sol(s):
        return os.path.join(settings.RESULTS_ROOT, "Solutions", f"{s}-SolutionObjects.pkl")

    reg_path = os.path.join(settings.RESULTS_ROOT, "custom_models.json")
    reg_backup = open(reg_path, "rb").read() if os.path.isfile(reg_path) else None

    shutil.copyfile(_obj(src), _obj(CUSTOM_SCENARIO))
    shutil.copyfile(_sol(src), _sol(CUSTOM_SCENARIO))
    models_registry.register(CUSTOM_KEY, CUSTOM_MODEL)

    from app.selector_service import _load_selector
    _load_selector.cache_clear()
    try:
        yield CUSTOM_SCENARIO
    finally:
        for p in (_obj(CUSTOM_SCENARIO), _sol(CUSTOM_SCENARIO)):
            if os.path.isfile(p):
                os.remove(p)
        if reg_backup is not None:
            with open(reg_path, "wb") as fh:
                fh.write(reg_backup)
        elif os.path.isfile(reg_path):
            os.remove(reg_path)
        models_registry._invalidate_cache()
        _load_selector.cache_clear()


def test_custom_model_compared_not_skipped(client, custom_scenario):
    data = client.post("/api/comparison", json={"scenarios": [custom_scenario]}).json()
    assert data["skipped"] == [], "custom model wrongly skipped by /api/comparison"
    assert [s["scenario"] for s in data["scenarios"]] == [custom_scenario]
    assert data["scenarios"][0]["type"] == "MOO"


def test_custom_model_time_metrics_has_type_and_alg(client, custom_scenario):
    resp = client.post(
        "/api/comparison/time",
        json={
            "scenarios": [custom_scenario],
            "config": {
                "merge_topology": "onboard",
                "time_model": "discrete",
                "detection_prob": 0.8,
                "false_alarm_prob": 0.1,
                "belief_threshold": 0.9,
                "target_locations": [12],
            },
            "strategy": "balanced",
        },
    )
    assert resp.status_code == 200
    s = resp.json()["scenarios"][0]
    assert s["type"] == "MOO" and s["algorithm"] == "NSGA2"


def test_custom_model_grid_endpoint_ok(client, custom_scenario):
    resp = client.get(f"/api/models/{CUSTOM_KEY}/grid")
    assert resp.status_code == 200, resp.text
    assert resp.json()["model_key"] == CUSTOM_KEY


def test_custom_model_validate_ok(client, custom_scenario):
    resp = client.post(
        "/api/scenarios/validate",
        json={"scenario": {"number_of_drones": 4, "n_visits": 2}, "model_key": CUSTOM_KEY},
    )
    assert resp.status_code == 200, resp.text
    assert resp.json()["scenario_str"]


def test_custom_model_in_models_list(client, custom_scenario):
    models = client.get("/api/models").json()
    names = {m["name"] for m in models}
    assert CUSTOM_KEY in names, "custom model missing from GET /api/models"
