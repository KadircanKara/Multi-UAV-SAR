"""Tests for GET /api/models."""


def test_get_models_status(client):
    resp = client.get("/api/models")
    assert resp.status_code == 200


def test_get_models_count(client):
    from PathOptimizationModel import AVAILABLE_MODELS
    from app.model_aliases import to_display

    data = client.get("/api/models").json()
    names = {m["name"] for m in data}
    # All preset models are listed under their DISPLAY key (Max Mean TBV coded
    # as V: TCDT→TCDV) plus any saved custom models the registry holds.
    assert {to_display(k) for k in AVAILABLE_MODELS} <= names
    assert len(data) >= len(AVAILABLE_MODELS)


def test_get_models_known_names(client):
    names = {m["name"] for m in client.get("/api/models").json()}
    assert "TC_MOO_NSGA2" in names
    assert "MTSP" in names
    # TBV model is shown with the V code, never the legacy storage "TCDT".
    assert "TCDV_MOO_NSGA2" in names
    assert "TCDT_MOO_NSGA2" not in names


def test_get_models_schema(client):
    required_keys = {"name", "type", "algorithm", "objectives", "constraints"}
    for row in client.get("/api/models").json():
        assert required_keys <= row.keys(), f"Missing keys in row: {row}"


def test_tc_moo_nsga2_objectives(client):
    rows = client.get("/api/models").json()
    tc = next(r for r in rows if r["name"] == "TC_MOO_NSGA2")
    assert tc["objectives"] == ["Mission Time", "Percentage Connectivity"]
