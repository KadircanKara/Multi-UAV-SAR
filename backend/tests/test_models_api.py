"""Tests for GET /api/models."""


def test_get_models_status(client):
    resp = client.get("/api/models")
    assert resp.status_code == 200


def test_get_models_count(client):
    from PathOptimizationModel import AVAILABLE_MODELS

    data = client.get("/api/models").json()
    names = {m["name"] for m in data}
    # All preset models are listed (plus any saved custom models the registry holds).
    assert set(AVAILABLE_MODELS) <= names
    assert len(data) >= len(AVAILABLE_MODELS)


def test_get_models_known_names(client):
    names = {m["name"] for m in client.get("/api/models").json()}
    assert "TC_MOO_NSGA2" in names
    assert "MTSP" in names
    assert "TCDT_MOO_NSGA2" in names


def test_get_models_schema(client):
    required_keys = {"name", "type", "algorithm", "objectives", "constraints"}
    for row in client.get("/api/models").json():
        assert required_keys <= row.keys(), f"Missing keys in row: {row}"


def test_tc_moo_nsga2_objectives(client):
    rows = client.get("/api/models").json()
    tc = next(r for r in rows if r["name"] == "TC_MOO_NSGA2")
    assert tc["objectives"] == ["Mission Time", "Percentage Connectivity"]
