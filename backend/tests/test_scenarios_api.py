"""Tests for GET /api/scenarios/default and POST /api/scenarios/validate."""
from app.schemas import ScenarioConfig


# ---------------------------------------------------------------------------
# GET /api/scenarios/default
# ---------------------------------------------------------------------------

def test_default_scenario_status(client):
    resp = client.get("/api/scenarios/default")
    assert resp.status_code == 200


def test_default_scenario_values(client):
    data = client.get("/api/scenarios/default").json()
    assert data["grid_size"] == 8
    assert data["cell_side_length"] == 50.0
    assert data["number_of_drones"] == 4
    assert data["max_drone_speed"] == 2.5
    assert data["comm_cell_range"] == 2.0
    assert data["n_visits"] == 2
    assert data["target_positions"] == [12]
    assert data["th"] == 0.9
    assert data["detection_probability"] == 0.7


# ---------------------------------------------------------------------------
# POST /api/scenarios/validate
# ---------------------------------------------------------------------------

_DEFAULT_PAYLOAD = {
    "scenario": {
        "grid_size": 8,
        "cell_side_length": 50,
        "number_of_drones": 4,
        "max_drone_speed": 2.5,
        "comm_cell_range": 2,
        "n_visits": 2,
        "target_positions": [12],
        "th": 0.9,
        "detection_probability": 0.7,
    }
}


def test_validate_default_with_model_key(client):
    payload = {**_DEFAULT_PAYLOAD, "model_key": "TC_MOO_NSGA2"}
    resp = client.post("/api/scenarios/validate", json=payload)
    assert resp.status_code == 200
    data = resp.json()
    assert data["valid"] is True
    assert data["derived"]["number_of_cells"] == 64
    assert data["derived"]["number_of_nodes"] == 5
    assert data["scenario_str"] is not None
    assert data["scenario_str"] == "MOO_NSGA2_TC_g_8_a_50_n_4_v_2.5_r_2_nvisits_2"


def test_validate_no_model_key(client):
    resp = client.post("/api/scenarios/validate", json=_DEFAULT_PAYLOAD)
    assert resp.status_code == 200
    data = resp.json()
    assert data["valid"] is True
    assert data["scenario_str"] is None
    # derived must still be present
    assert data["derived"]["number_of_cells"] == 64
    assert data["derived"]["number_of_nodes"] == 5


def test_validate_out_of_grid_target(client):
    payload = {
        "scenario": {**_DEFAULT_PAYLOAD["scenario"], "target_positions": [999]}
    }
    resp = client.post("/api/scenarios/validate", json=payload)
    assert resp.status_code == 422


def test_validate_bad_model_key(client):
    payload = {**_DEFAULT_PAYLOAD, "model_key": "NOPE"}
    resp = client.post("/api/scenarios/validate", json=payload)
    assert resp.status_code == 404


# ---------------------------------------------------------------------------
# GET /api/health (sanity check)
# ---------------------------------------------------------------------------

def test_health(client):
    resp = client.get("/api/health")
    assert resp.status_code == 200
    assert resp.json() == {"status": "ok"}


# ---------------------------------------------------------------------------
# Schema-level coercion: whole floats must be coerced to int
# ---------------------------------------------------------------------------

def test_scenario_config_coerces_whole_floats_to_int():
    """ScenarioConfig must coerce 50.0→50 and 2.0→2 so PathInfo filenames match disk."""
    from PathInfo import default_scenario

    cfg = ScenarioConfig(**default_scenario)
    d = cfg.to_scenario_dict()
    assert d["cell_side_length"] == 50
    assert type(d["cell_side_length"]) is int
    assert d["comm_cell_range"] == 2
    assert type(d["comm_cell_range"]) is int


def test_scenario_config_coerces_float_inputs_to_int():
    """Passing 50.0 and 2.0 as JSON floats must also yield ints after coercion."""
    from PathInfo import default_scenario

    overrides = {**default_scenario, "cell_side_length": 50.0, "comm_cell_range": 2.0}
    cfg = ScenarioConfig(**overrides)
    d = cfg.to_scenario_dict()
    assert d["cell_side_length"] == 50
    assert type(d["cell_side_length"]) is int
    assert d["comm_cell_range"] == 2
    assert type(d["comm_cell_range"]) is int


def test_scenario_config_preserves_non_whole_floats():
    """Non-whole floats (e.g. sqrt(8) ≈ 2.828) must NOT be coerced to int."""
    from math import sqrt
    from PathInfo import default_scenario

    sqrt8 = 2 * sqrt(2)
    overrides = {**default_scenario, "comm_cell_range": sqrt8}
    cfg = ScenarioConfig(**overrides)
    d = cfg.to_scenario_dict()
    assert d["comm_cell_range"] == sqrt8
    assert type(d["comm_cell_range"]) is float
