"""
Tests for the precomputed-scenario library service and endpoints.

Layer 1 — pure unit tests of resolve_model_key and parse_scenario_params
          (no filesystem, always run).
Layer 1b — unit tests for _is_safe_scenario_name (security, no filesystem).
Layer 2 — integration tests via TestClient, guarded on seed presence.
"""
import pytest

from app.library_service import _is_safe_scenario_name, parse_scenario_params, resolve_model_key

# ---------------------------------------------------------------------------
# Layer 1 — unit tests (no filesystem, no pickles)
# ---------------------------------------------------------------------------

class TestResolveModelKey:
    def test_moo_nsga2_tc(self):
        assert resolve_model_key("MOO_NSGA2_TC_g_8_a_50_n_4_v_2.5_r_2_nvisits_2") == "TC_MOO_NSGA2"

    def test_moo_nsga2_tcdt(self):
        assert resolve_model_key("MOO_NSGA2_TCDT_g_8_a_50_n_4_v_2.5_r_4_nvisits_1") == "TCDT_MOO_NSGA2"

    def test_ws_ga_tc(self):
        assert resolve_model_key("WS_GA_TC_g_8_a_50_n_4_v_2.5_r_2_nvisits_3") == "TC_WS"

    def test_ws_ga_tcdt(self):
        assert resolve_model_key("WS_GA_TCDT_g_8_a_50_n_8_v_2.5_r_sqrt(8)_ntours_2") == "TCDT_WS"

    def test_soo_ga_mtsp(self):
        assert resolve_model_key("SOO_GA_MTSP_g_8_a_50_n_4_v_2.5_r_2_ntours_4") == "MTSP"

    def test_soo_nsga2_mtsp(self):
        assert resolve_model_key("SOO_NSGA2_MTSP_g_8_a_50_n_8_v_2.5_r_2_nvisits_3") == "MTSP"


class TestParseScenarioParams:
    def test_nvisits_example(self):
        params = parse_scenario_params(
            "MOO_NSGA2_TC_g_8_a_50_n_4_v_2.5_r_2_nvisits_2"
        )
        assert params["grid_size"] == 8
        assert params["number_of_drones"] == 4
        assert params["variant"] == "nvisits"
        assert params["variant_value"] == 2

    def test_ntours_sqrt_range(self):
        params = parse_scenario_params(
            "WS_GA_TCDT_g_8_a_50_n_8_v_2.5_r_sqrt(8)_ntours_2"
        )
        assert params["grid_size"] == 8
        assert params["number_of_drones"] == 8
        assert params["comm_range"] == "sqrt(8)"
        assert params["variant"] == "ntours"
        assert params["variant_value"] == 2

    def test_returns_dict_on_garbage_input(self):
        # Must never raise, just return an incomplete dict
        result = parse_scenario_params("total_garbage_string")
        assert isinstance(result, dict)

    def test_cell_side_length_int(self):
        params = parse_scenario_params(
            "MOO_NSGA2_TC_g_8_a_50_n_4_v_2.5_r_2_nvisits_2"
        )
        assert params["cell_side_length"] == 50
        assert isinstance(params["cell_side_length"], int)

    def test_max_drone_speed_float(self):
        params = parse_scenario_params(
            "MOO_NSGA2_TC_g_8_a_50_n_4_v_2.5_r_2_nvisits_2"
        )
        assert params["max_drone_speed"] == 2.5


# ---------------------------------------------------------------------------
# Layer 2 — integration tests (guarded on seed presence)
# ---------------------------------------------------------------------------

_WELL_KNOWN = "MOO_NSGA2_TC_g_8_a_50_n_4_v_2.5_r_2_nvisits_2"


def test_library_list_status(client):
    resp = client.get("/api/library")
    assert resp.status_code == 200


def test_library_list_is_array(client):
    data = client.get("/api/library").json()
    assert isinstance(data, list)


def test_library_all_model_keys_valid(client):
    from app import models_registry

    data = client.get("/api/library").json()
    if len(data) == 0:
        pytest.skip("no seeded scenarios present")
    for row in data:
        # A listed scenario resolves to a known model — a preset OR a saved
        # custom model (the library lists both since the registry migration).
        assert models_registry.known(row["model_key"]), (
            f"Unknown model_key: {row['model_key']}"
        )


def test_library_all_have_solutions(client):
    data = client.get("/api/library").json()
    if len(data) == 0:
        pytest.skip("no seeded scenarios present")
    for row in data:
        assert row["has_solutions"] is True


def test_library_well_known_scenario_fields(client):
    data = client.get("/api/library").json()
    names = {r["scenario"] for r in data}
    if _WELL_KNOWN not in names:
        pytest.skip(f"well-known seed {_WELL_KNOWN!r} not present")
    row = next(r for r in data if r["scenario"] == _WELL_KNOWN)
    assert row["model_key"] == "TC_MOO_NSGA2"
    assert row["type"] == "MOO"
    assert row["result_kind"] == "front"
    assert row["n_solutions"] > 1


def test_library_detail_well_known(client):
    data = client.get("/api/library").json()
    names = {r["scenario"] for r in data}
    if _WELL_KNOWN not in names:
        pytest.skip(f"well-known seed {_WELL_KNOWN!r} not present")
    resp = client.get(f"/api/library/{_WELL_KNOWN}")
    assert resp.status_code == 200
    detail = resp.json()
    assert detail["model"]["name"] == "TC_MOO_NSGA2"
    assert detail["n_solutions"] > 1
    assert detail["result_kind"] == "front"
    assert "grid_size" in detail["params"]


def test_library_detail_not_found(client):
    resp = client.get("/api/library/NOPE_does_not_exist")
    assert resp.status_code == 404


def test_library_list_nonempty_with_seeds(client):
    """Assert non-empty when seeds are present (fails loudly if seeds missing)."""
    data = client.get("/api/library").json()
    if len(data) == 0:
        pytest.skip("no seeded scenarios present")
    assert len(data) > 0


# ---------------------------------------------------------------------------
# Layer 1b — _is_safe_scenario_name unit tests (security, no filesystem)
# ---------------------------------------------------------------------------

class TestIsSafeScenarioName:
    # --- names that MUST be rejected ---

    def test_rejects_dotdot_slash(self):
        assert _is_safe_scenario_name("../x") is False

    def test_rejects_dotdot_alone(self):
        assert _is_safe_scenario_name("..") is False

    def test_rejects_dotdot_embedded(self):
        assert _is_safe_scenario_name("a/../b") is False

    def test_rejects_forward_slash(self):
        assert _is_safe_scenario_name("a/b") is False

    def test_rejects_backslash(self):
        assert _is_safe_scenario_name("a\\b") is False

    def test_rejects_empty_string(self):
        assert _is_safe_scenario_name("") is False

    def test_rejects_nul_byte(self):
        assert _is_safe_scenario_name("abc\x00def") is False

    def test_rejects_absolute_path(self):
        assert _is_safe_scenario_name("/etc/passwd") is False

    # --- names that MUST be accepted ---

    def test_accepts_real_seed_name(self):
        assert _is_safe_scenario_name(
            "MOO_NSGA2_TC_g_8_a_50_n_4_v_2.5_r_2_nvisits_2"
        ) is True

    def test_accepts_sqrt_range_name(self):
        assert _is_safe_scenario_name(
            "WS_GA_TCDT_g_8_a_50_n_8_v_2.5_r_sqrt(8)_ntours_2"
        ) is True


# ---------------------------------------------------------------------------
# Layer 2 — traversal endpoint tests (integration, always run)
# ---------------------------------------------------------------------------

class TestPathTraversal:
    def test_url_encoded_traversal_returns_404(self, client):
        """Percent-encoded '../../../etc/passwd' must yield 404, never 500."""
        resp = client.get("/api/library/..%2F..%2F..%2Fetc%2Fpasswd")
        assert resp.status_code == 404

    def test_literal_dotdot_returns_404(self, client):
        """A literal '..' segment must yield 404."""
        resp = client.get("/api/library/..")
        assert resp.status_code in (404, 422)  # FastAPI may 422 on routing edge cases

    def test_slash_in_name_returns_404(self, client):
        """A name with a forward slash must yield 404 (or 422 via router)."""
        resp = client.get("/api/library/a%2Fb")
        assert resp.status_code in (404, 422)

    def test_no_500_on_traversal_attempt(self, client):
        """Any traversal attempt must never produce a 500."""
        for bad in [
            "..%2F..%2Fetc%2Fpasswd",
            "..%5C..%5Cwindows%5Csystem32",
            "..",
            "a%2Fb",
        ]:
            resp = client.get(f"/api/library/{bad}")
            assert resp.status_code != 500, (
                f"Got 500 for traversal probe {bad!r}: {resp.text}"
            )


# ---------------------------------------------------------------------------
# Layer 2 — summary field tests
# ---------------------------------------------------------------------------

def test_library_summary_has_cell_side_length(client):
    data = client.get("/api/library").json()
    if len(data) == 0:
        pytest.skip("no seeded scenarios present")
    for row in data:
        assert "cell_side_length" in row, (
            f"cell_side_length missing from {row['scenario']}"
        )


def test_library_summary_has_max_drone_speed(client):
    data = client.get("/api/library").json()
    if len(data) == 0:
        pytest.skip("no seeded scenarios present")
    for row in data:
        assert "max_drone_speed" in row, (
            f"max_drone_speed missing from {row['scenario']}"
        )
