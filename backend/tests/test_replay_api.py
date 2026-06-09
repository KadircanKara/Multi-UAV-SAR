"""
Tests for the sensing-replay and merging-compare endpoints.

Layer 1 — schema/validation (no filesystem required).
Layer 2 — integration tests via TestClient, guarded on seed presence.

Known seed:
  TC_MOO_NSGA2 = MOO_NSGA2_TC_g_8_a_50_n_4_v_2.5_r_2_nvisits_2
                 (TC_MOO_NSGA2, front, n_solutions>1, 8×8=64-cell grid)
"""
import math
import pytest

# ---------------------------------------------------------------------------
# Seed names and shared config
# ---------------------------------------------------------------------------

TC_FRONT = "MOO_NSGA2_TC_g_8_a_50_n_4_v_2.5_r_2_nvisits_2"

VALID_CFG = {
    "merge_topology": "onboard",
    "time_model": "discrete",
    "detection_prob": 0.7,
    "false_alarm_prob": 0.2,
    "belief_threshold": 0.9,
    "target_locations": [12],
}


def _seed_present(client) -> bool:
    resp = client.get("/api/library")
    if resp.status_code != 200:
        return False
    return TC_FRONT in {r["scenario"] for r in resp.json()}


# ---------------------------------------------------------------------------
# POST /api/replay/{scenario}
# ---------------------------------------------------------------------------

class TestReplay:
    def test_replay_200(self, client):
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        resp = client.post(
            f"/api/replay/{TC_FRONT}",
            json={"index": 0, "config": VALID_CFG},
        )
        assert resp.status_code == 200

    def test_replay_fields_present(self, client):
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        data = client.post(
            f"/api/replay/{TC_FRONT}",
            json={"index": 0, "config": VALID_CFG},
        ).json()
        # required metric
        assert "effective_mission_time" in data
        v = data["effective_mission_time"]
        assert v is None or isinstance(v, (int, float))
        # config scalars echoed back
        assert data["merge_topology"] == "onboard"
        assert data["time_model"] == "discrete"
        # per-cell belief series
        assert "cell_occupancy_probabilities" in data

    def test_replay_no_bulky_fields(self, client):
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        data = client.post(
            f"/api/replay/{TC_FRONT}",
            json={"index": 0, "config": VALID_CFG},
        ).json()
        for banned in ("solution", "search_map", "occupancy_status"):
            assert banned not in data, f"Bulky field {banned!r} leaked into response"

    def test_replay_no_nonfinite(self, client):
        """FastAPI must not see any inf/nan — to_dict must convert them to None."""
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        data = client.post(
            f"/api/replay/{TC_FRONT}",
            json={"index": 0, "config": VALID_CFG},
        ).json()
        # If a non-finite slipped through, FastAPI/orjson would have raised a 500
        # and the test above (status 200) would already have caught it.
        # Here we also walk the time metrics explicitly.
        for key in ("effective_mission_time", "detection_time",
                    "inform_time", "time_at_least_one_drone_knows_all"):
            v = data.get(key)
            if v is not None:
                assert math.isfinite(v), f"{key} is non-finite: {v}"

    def test_replay_p_leq_q_422(self, client):
        """detection_prob <= false_alarm_prob must return 422."""
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        bad_cfg = {**VALID_CFG, "detection_prob": 0.3, "false_alarm_prob": 0.5}
        resp = client.post(
            f"/api/replay/{TC_FRONT}",
            json={"index": 0, "config": bad_cfg},
        )
        assert resp.status_code == 422

    def test_replay_target_outside_grid_422(self, client):
        """target_locations with a cell id outside the grid must return 422."""
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        bad_cfg = {**VALID_CFG, "target_locations": [9999]}
        resp = client.post(
            f"/api/replay/{TC_FRONT}",
            json={"index": 0, "config": bad_cfg},
        )
        assert resp.status_code == 422

    def test_replay_index_out_of_range_422(self, client):
        """An index far outside the front must return 422."""
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        resp = client.post(
            f"/api/replay/{TC_FRONT}",
            json={"index": 1000000, "config": VALID_CFG},
        )
        assert resp.status_code == 422

    def test_replay_missing_scenario_404(self, client):
        resp = client.post(
            "/api/replay/NOPE_missing",
            json={"index": 0, "config": VALID_CFG},
        )
        assert resp.status_code == 404

    def test_replay_traversal_scenario_404(self, client):
        resp = client.post(
            "/api/replay/..%2F..%2Fetc%2Fpasswd",
            json={"index": 0, "config": VALID_CFG},
        )
        assert resp.status_code == 404


# ---------------------------------------------------------------------------
# POST /api/compare/{scenario}
# ---------------------------------------------------------------------------

class TestCompare:
    def _three_configs(self):
        return [
            {**VALID_CFG, "merge_topology": "none"},
            {**VALID_CFG, "merge_topology": "onboard"},
            {**VALID_CFG, "merge_topology": "gcs"},
        ]

    def test_compare_200(self, client):
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        resp = client.post(
            f"/api/compare/{TC_FRONT}",
            json={"index": 0, "configs": self._three_configs()},
        )
        assert resp.status_code == 200

    def test_compare_table_structure(self, client):
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        data = client.post(
            f"/api/compare/{TC_FRONT}",
            json={"index": 0, "configs": self._three_configs()},
        ).json()
        assert "table" in data
        assert "rows" in data
        assert len(data["table"]) == 3
        assert len(data["rows"]) == 3

    def test_compare_table_columns(self, client):
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        data = client.post(
            f"/api/compare/{TC_FRONT}",
            json={"index": 0, "configs": self._three_configs()},
        ).json()
        expected_cols = {
            "label",
            "Effective Mission Time",
            "Detection Time",
            "Inform Time",
            "Time At Least One Drone Knows All Targets",
        }
        for row in data["table"]:
            assert set(row.keys()) == expected_cols, (
                f"Table row keys mismatch: {set(row.keys())} != {expected_cols}"
            )

    def test_compare_labels(self, client):
        """When no explicit labels are given, labels come from merge_topology."""
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        data = client.post(
            f"/api/compare/{TC_FRONT}",
            json={"index": 0, "configs": self._three_configs()},
        ).json()
        labels = [row["label"] for row in data["table"]]
        assert labels == ["none", "onboard", "gcs"]

    def test_compare_rows_are_replay_dicts(self, client):
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        data = client.post(
            f"/api/compare/{TC_FRONT}",
            json={"index": 0, "configs": self._three_configs()},
        ).json()
        for rd in data["rows"]:
            assert "effective_mission_time" in rd
            assert "cell_occupancy_probabilities" in rd
            for banned in ("solution", "search_map", "occupancy_status"):
                assert banned not in rd

    def test_compare_table_json_safe(self, client):
        """No non-finite floats in the table (inf/nan must be None)."""
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        data = client.post(
            f"/api/compare/{TC_FRONT}",
            json={"index": 0, "configs": self._three_configs()},
        ).json()
        metric_keys = [
            "Effective Mission Time",
            "Detection Time",
            "Inform Time",
            "Time At Least One Drone Knows All Targets",
        ]
        for row in data["table"]:
            for key in metric_keys:
                v = row[key]
                if v is not None:
                    assert math.isfinite(v), f"{key} is non-finite: {v}"

    def test_compare_empty_configs_422(self, client):
        """An empty configs list must return 422."""
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        resp = client.post(
            f"/api/compare/{TC_FRONT}",
            json={"index": 0, "configs": []},
        )
        assert resp.status_code == 422
