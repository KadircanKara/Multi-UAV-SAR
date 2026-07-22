"""Tests for the Max Mean TBV display alias (TCDT ↔ TCDV) and its API wiring."""
import pytest

from app.model_aliases import to_display, to_storage


# ── Pure helpers ────────────────────────────────────────────────────────────

@pytest.mark.parametrize("storage,display", [
    ("TCDT_MOO_NSGA2", "TCDV_MOO_NSGA2"),
    ("TCDT_MOO_NSGA3", "TCDV_MOO_NSGA3"),
    ("TCDT_WS", "TCDV_WS"),
    ("MOO_NSGA2_TCDT_g_8_a_50_n_4_v_2.5_r_2_nvisits_1",
     "MOO_NSGA2_TCDV_g_8_a_50_n_4_v_2.5_r_2_nvisits_1"),
    ("WS_GA_TCDT_g_8_a_50_n_8_v_2.5_r_2_nvisits_1",
     "WS_GA_TCDV_g_8_a_50_n_8_v_2.5_r_2_nvisits_1"),
])
def test_roundtrip(storage, display):
    assert to_display(storage) == display
    assert to_storage(display) == storage


@pytest.mark.parametrize("key", [
    # Non-TBV keys and near-misses must be untouched: TCD/TC must not be caught
    # by the TCDT rule, and TCT/TT are intentionally NOT aliased (they collide
    # with the synthesizer's V-codes and have no seeded data).
    "TCD_MOO_NSGA2", "TC_WS", "TCT_MOO_NSGA2", "TT_WS", "MTSP", "CONN",
    "MOO_NSGA2_TCD_g_8_a_50_n_4_v_2.5_r_2_nvisits_1",
    "", None,
])
def test_passthrough(key):
    assert to_display(key) == key
    assert to_storage(key) == key


def test_idempotent():
    assert to_display(to_display("TCDT_WS")) == "TCDV_WS"
    assert to_storage(to_storage("TCDV_WS")) == "TCDT_WS"
    # Applying the wrong direction is a no-op (already in target form).
    assert to_display("TCDV_WS") == "TCDV_WS"
    assert to_storage("TCDT_WS") == "TCDT_WS"


# ── API wiring ──────────────────────────────────────────────────────────────

def test_models_list_shows_display_key(client):
    names = {m["name"] for m in client.get("/api/models").json()}
    assert "TCDV_MOO_NSGA2" in names
    assert "TCDT_MOO_NSGA2" not in names


def test_grid_accepts_display_key_and_echoes_display(client):
    """The route param is the display key; the grid resolves the seeded TCDT
    data and echoes display identifiers back."""
    resp = client.get("/api/models/TCDV_MOO_NSGA2/grid")
    if resp.status_code == 404:
        pytest.skip("no seeded scenarios present")
    assert resp.status_code == 200
    grid = resp.json()
    if not grid["scenarios"]:
        pytest.skip("no seeded scenarios present")
    assert grid["model_key"] == "TCDV_MOO_NSGA2"
    for s in grid["scenarios"]:
        assert "TCDT" not in s["scenario"]
        assert "_TCDV_" in s["scenario"]


def test_grid_still_accepts_legacy_storage_key(client):
    """Old links using the storage key must keep working (normalized inbound)."""
    resp = client.get("/api/models/TCDT_MOO_NSGA2/grid")
    if resp.status_code == 404:
        pytest.skip("no seeded scenarios present")
    assert resp.status_code == 200
    assert resp.json()["model_key"] == "TCDV_MOO_NSGA2"


def test_front_roundtrips_display_scenario(client):
    """A display scenario from the grid loads its front and echoes display."""
    grid_resp = client.get("/api/models/TCDV_MOO_NSGA2/grid")
    if grid_resp.status_code != 200 or not grid_resp.json()["scenarios"]:
        pytest.skip("no seeded scenarios present")
    scenario = grid_resp.json()["scenarios"][0]["scenario"]  # display form (…TCDV…)
    resp = client.get(f"/api/fronts/{scenario}")
    assert resp.status_code == 200, resp.text
    front = resp.json()
    assert front["model_key"] == "TCDV_MOO_NSGA2"
    assert "TCDT" not in front["scenario"]
