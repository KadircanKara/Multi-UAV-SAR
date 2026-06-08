"""
Tests for the Pareto-front / solution-selection endpoints.

Layer 1 — always run (no filesystem).
Layer 2 — integration tests via TestClient, guarded on seed presence.

Known seeds:
  TC_FRONT  = MOO_NSGA2_TC_g_8_a_50_n_4_v_2.5_r_2_nvisits_2
              (TC_MOO_NSGA2, front, n_solutions>1,
               objectives ["Mission Time","Percentage Connectivity"])
  TC_WS     = WS_GA_TC_g_8_a_50_n_4_v_2.5_r_2_nvisits_3
              (TC_WS, single, the_solution)
"""
import pytest

# ---------------------------------------------------------------------------
# Seed names
# ---------------------------------------------------------------------------

TC_FRONT = "MOO_NSGA2_TC_g_8_a_50_n_4_v_2.5_r_2_nvisits_2"
TC_WS = "WS_GA_TC_g_8_a_50_n_4_v_2.5_r_2_nvisits_3"


def _seeds_present(client) -> bool:
    """Return True if the TC front seed exists in the library."""
    resp = client.get("/api/library")
    if resp.status_code != 200:
        return False
    names = {r["scenario"] for r in resp.json()}
    return TC_FRONT in names


def _ws_present(client) -> bool:
    resp = client.get("/api/library")
    if resp.status_code != 200:
        return False
    names = {r["scenario"] for r in resp.json()}
    return TC_WS in names


# ---------------------------------------------------------------------------
# GET /api/fronts/{scenario}
# ---------------------------------------------------------------------------

class TestGetFront:
    def test_tc_front_200(self, client):
        if not _seeds_present(client):
            pytest.skip("TC front seed not present")
        resp = client.get(f"/api/fronts/{TC_FRONT}")
        assert resp.status_code == 200

    def test_tc_front_objectives(self, client):
        if not _seeds_present(client):
            pytest.skip("TC front seed not present")
        data = client.get(f"/api/fronts/{TC_FRONT}").json()
        assert data["objectives"] == ["Mission Time", "Percentage Connectivity"]

    def test_tc_front_polarities(self, client):
        if not _seeds_present(client):
            pytest.skip("TC front seed not present")
        data = client.get(f"/api/fronts/{TC_FRONT}").json()
        assert data["polarities"]["Percentage Connectivity"] == -1
        assert data["polarities"]["Mission Time"] == 1

    def test_tc_front_n_solutions(self, client):
        if not _seeds_present(client):
            pytest.skip("TC front seed not present")
        data = client.get(f"/api/fronts/{TC_FRONT}").json()
        assert data["n_solutions"] > 1
        assert len(data["solutions"]) == data["n_solutions"]

    def test_tc_front_objectives_abs_nonnegative(self, client):
        if not _seeds_present(client):
            pytest.skip("TC front seed not present")
        data = client.get(f"/api/fronts/{TC_FRONT}").json()
        for sol in data["solutions"]:
            for v in sol["objectives_abs"].values():
                assert v >= 0, f"Negative abs value: {v}"

    def test_tc_front_capabilities_balanced(self, client):
        if not _seeds_present(client):
            pytest.skip("TC front seed not present")
        data = client.get(f"/api/fronts/{TC_FRONT}").json()
        assert data["capabilities"]["balanced"] is True

    def test_tc_front_result_kind(self, client):
        if not _seeds_present(client):
            pytest.skip("TC front seed not present")
        data = client.get(f"/api/fronts/{TC_FRONT}").json()
        assert data["result_kind"] == "front"

    def test_ws_single_result_kind(self, client):
        if not _ws_present(client):
            pytest.skip("TC WS seed not present")
        data = client.get(f"/api/fronts/{TC_WS}").json()
        assert data["result_kind"] == "single"

    def test_ws_capabilities_the_solution(self, client):
        if not _ws_present(client):
            pytest.skip("TC WS seed not present")
        data = client.get(f"/api/fronts/{TC_WS}").json()
        assert data["capabilities"]["the_solution"] is True
        assert data["capabilities"]["best"] == []

    def test_missing_scenario_404(self, client):
        resp = client.get("/api/fronts/NOPE_missing")
        assert resp.status_code == 404

    def test_traversal_name_404(self, client):
        resp = client.get("/api/fronts/..%2F..%2Fetc%2Fpasswd")
        assert resp.status_code == 404

    def test_tc_front_scenario_and_model_key_fields(self, client):
        if not _seeds_present(client):
            pytest.skip("TC front seed not present")
        data = client.get(f"/api/fronts/{TC_FRONT}").json()
        assert data["scenario"] == TC_FRONT
        assert data["model_key"] == "TC_MOO_NSGA2"


# ---------------------------------------------------------------------------
# GET /api/fronts/{scenario}/capabilities
# ---------------------------------------------------------------------------

class TestGetCapabilities:
    def test_capabilities_matches_front(self, client):
        if not _seeds_present(client):
            pytest.skip("TC front seed not present")
        front = client.get(f"/api/fronts/{TC_FRONT}").json()
        caps = client.get(f"/api/fronts/{TC_FRONT}/capabilities").json()
        assert caps == front["capabilities"]

    def test_capabilities_missing_404(self, client):
        resp = client.get("/api/fronts/NOPE_missing/capabilities")
        assert resp.status_code == 404


# ---------------------------------------------------------------------------
# POST /api/fronts/{scenario}/select
# ---------------------------------------------------------------------------

class TestSelectSolution:
    def test_select_best_mission_time(self, client):
        if not _seeds_present(client):
            pytest.skip("TC front seed not present")
        resp = client.post(
            f"/api/fronts/{TC_FRONT}/select",
            json={"strategy": "best", "objective_name": "Mission Time"},
        )
        assert resp.status_code == 200
        data = resp.json()
        assert data["label"].startswith("Best")
        assert "index" in data
        assert "detail" in data

    def test_select_balanced(self, client):
        if not _seeds_present(client):
            pytest.skip("TC front seed not present")
        resp = client.post(
            f"/api/fronts/{TC_FRONT}/select",
            json={"strategy": "balanced"},
        )
        assert resp.status_code == 200
        data = resp.json()
        assert data["label"] == "Balanced"

    def test_select_by_index_zero(self, client):
        if not _seeds_present(client):
            pytest.skip("TC front seed not present")
        resp = client.post(
            f"/api/fronts/{TC_FRONT}/select",
            json={"strategy": "by_index", "index": 0},
        )
        assert resp.status_code == 200
        assert resp.json()["index"] == 0

    def test_select_the_solution_on_front_422(self, client):
        """the_solution is unavailable for a Pareto front — must return 422."""
        if not _seeds_present(client):
            pytest.skip("TC front seed not present")
        resp = client.post(
            f"/api/fronts/{TC_FRONT}/select",
            json={"strategy": "the_solution"},
        )
        assert resp.status_code == 422

    def test_select_best_no_objective_422(self, client):
        """strategy='best' without objective_name must return 422."""
        if not _seeds_present(client):
            pytest.skip("TC front seed not present")
        resp = client.post(
            f"/api/fronts/{TC_FRONT}/select",
            json={"strategy": "best"},
        )
        assert resp.status_code == 422

    def test_select_bogus_strategy_422(self, client):
        if not _seeds_present(client):
            pytest.skip("TC front seed not present")
        resp = client.post(
            f"/api/fronts/{TC_FRONT}/select",
            json={"strategy": "bogus"},
        )
        assert resp.status_code == 422

    def test_select_the_solution_on_ws(self, client):
        if not _ws_present(client):
            pytest.skip("TC WS seed not present")
        resp = client.post(
            f"/api/fronts/{TC_WS}/select",
            json={"strategy": "the_solution"},
        )
        assert resp.status_code == 200
        assert resp.json()["label"] == "The solution"

    def test_select_missing_scenario_404(self, client):
        resp = client.post(
            "/api/fronts/NOPE_missing/select",
            json={"strategy": "balanced"},
        )
        assert resp.status_code == 404

    def test_select_detail_objectives_abs_nonneg(self, client):
        if not _seeds_present(client):
            pytest.skip("TC front seed not present")
        resp = client.post(
            f"/api/fronts/{TC_FRONT}/select",
            json={"strategy": "balanced"},
        )
        data = resp.json()
        for v in data["detail"]["objectives_abs"].values():
            assert v >= 0
