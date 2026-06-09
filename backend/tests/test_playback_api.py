"""
Tests for the playback animation endpoint.

POST /api/playback/{scenario}

Layer 1 — schema/validation (no filesystem required).
Layer 2 — integration tests via TestClient, guarded on seed presence.

Known seed:
  TC_FRONT = MOO_NSGA2_TC_g_8_a_50_n_4_v_2.5_r_2_nvisits_2
             (TC_MOO_NSGA2, front, n_solutions>1, 8×8=64-cell grid, 4 drones + base = 5 nodes)
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
# POST /api/playback/{scenario} — 200 discrete
# ---------------------------------------------------------------------------

class TestPlaybackDiscrete:
    """Integration tests for the discrete time model."""

    def test_playback_200(self, client):
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        resp = client.post(
            f"/api/playback/{TC_FRONT}",
            json={"index": 0, "config": VALID_CFG},
        )
        assert resp.status_code == 200

    def test_playback_node_count(self, client):
        """number_of_nodes == 5 (4 drones + 1 base station)."""
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        data = client.post(
            f"/api/playback/{TC_FRONT}",
            json={"index": 0, "config": VALID_CFG},
        ).json()
        assert data["number_of_nodes"] == 5
        assert data["grid_size"] == 8

    def test_playback_trajectory_shape(self, client):
        """trajectories.x and .y have exactly number_of_nodes rows."""
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        data = client.post(
            f"/api/playback/{TC_FRONT}",
            json={"index": 0, "config": VALID_CFG},
        ).json()
        assert len(data["trajectories"]["x"]) == 5
        assert len(data["trajectories"]["y"]) == 5

    def test_playback_step_alignment(self, client):
        """
        Core alignment invariant:
        len(trajectories.x[0]) == steps == len(connectivity) == len(belief[0]) == len(targets_known)
        """
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        data = client.post(
            f"/api/playback/{TC_FRONT}",
            json={"index": 0, "config": VALID_CFG},
        ).json()
        steps = data["steps"]
        assert steps > 0, "steps must be positive"

        traj_len = len(data["trajectories"]["x"][0])
        conn_len = len(data["connectivity"])
        belief_len = len(data["belief"][0])
        tk_len = len(data["targets_known"])

        assert traj_len == steps, f"trajectories.x[0] length {traj_len} != steps {steps}"
        assert conn_len == steps, f"connectivity length {conn_len} != steps {steps}"
        assert belief_len == steps, f"belief[0] length {belief_len} != steps {steps}"
        assert tk_len == steps, f"targets_known length {tk_len} != steps {steps}"

        # All trajectory rows must have same length
        for i, row in enumerate(data["trajectories"]["x"]):
            assert len(row) == steps, f"trajectories.x[{i}] length {len(row)} != steps {steps}"
        for i, row in enumerate(data["trajectories"]["y"]):
            assert len(row) == steps, f"trajectories.y[{i}] length {len(row)} != steps {steps}"

        # All belief rows must have same length
        for i, row in enumerate(data["belief"]):
            assert len(row) == steps, f"belief[{i}] length {len(row)} != steps {steps}"

    def test_playback_belief_shape(self, client):
        """belief has exactly grid_size^2 = 64 rows."""
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        data = client.post(
            f"/api/playback/{TC_FRONT}",
            json={"index": 0, "config": VALID_CFG},
        ).json()
        assert len(data["belief"]) == 64

    def test_playback_connectivity_format(self, client):
        """Each connectivity entry is a list of [i, j] with i < j and 0 <= i, j < number_of_nodes."""
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        data = client.post(
            f"/api/playback/{TC_FRONT}",
            json={"index": 0, "config": VALID_CFG},
        ).json()
        n_nodes = data["number_of_nodes"]
        for step_idx, edges in enumerate(data["connectivity"]):
            assert isinstance(edges, list), f"Step {step_idx}: edges must be a list"
            for edge in edges:
                assert len(edge) == 2, f"Step {step_idx}: edge {edge} must have 2 elements"
                i, j = edge
                assert isinstance(i, int) and isinstance(j, int), (
                    f"Step {step_idx}: edge {edge} must be ints"
                )
                assert i < j, f"Step {step_idx}: edge {edge} must have i < j"
                assert 0 <= i < n_nodes, f"Step {step_idx}: i={i} out of range [0, {n_nodes})"
                assert 0 <= j < n_nodes, f"Step {step_idx}: j={j} out of range [0, {n_nodes})"

    def test_playback_json_safe(self, client):
        """No non-finite floats (inf/nan) should be in the payload; status 200 → serialised OK.
        Belief values may be None (from inf guard) but not bare inf/nan."""
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        data = client.post(
            f"/api/playback/{TC_FRONT}",
            json={"index": 0, "config": VALID_CFG},
        ).json()
        # Walk trajectory values
        for row in data["trajectories"]["x"]:
            for v in row:
                if v is not None:
                    assert math.isfinite(v), f"Non-finite trajectory x value: {v}"
        for row in data["trajectories"]["y"]:
            for v in row:
                if v is not None:
                    assert math.isfinite(v), f"Non-finite trajectory y value: {v}"
        # Walk belief values
        for cell_row in data["belief"]:
            for v in cell_row:
                if v is not None:
                    assert math.isfinite(v), f"Non-finite belief value: {v}"

    def test_playback_raw_lengths_present(self, client):
        """raw_lengths dict is present with the four expected keys."""
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        data = client.post(
            f"/api/playback/{TC_FRONT}",
            json={"index": 0, "config": VALID_CFG},
        ).json()
        rl = data["raw_lengths"]
        for key in ("trajectory", "connectivity", "belief", "targets_known"):
            assert key in rl, f"raw_lengths missing key {key!r}"
            assert rl[key] > 0, f"raw_lengths[{key!r}] must be positive"

    def test_discrete_granularity_invariant(self, client):
        """
        Regression: for discrete time_model, trajectory and connectivity
        raw_lengths must be at waypoint granularity (within 2 of belief length),
        NOT at interpolated sub-step granularity (~1225 vs ~78).

        This test FAILS with the old buggy code where get_real_paths() is used
        for discrete mode (trajectory=1225, belief=78, truncating to first 6%).
        """
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        data = client.post(
            f"/api/playback/{TC_FRONT}",
            json={"index": 0, "config": VALID_CFG},
        ).json()
        rl = data["raw_lengths"]
        belief_len = rl["belief"]
        traj_len = rl["trajectory"]
        conn_len = rl["connectivity"]

        # All four lengths must be close together at waypoint granularity.
        # With the bug: traj_len ~1225, belief_len ~78 — so traj_len >> belief_len + 2.
        assert traj_len <= belief_len + 2, (
            f"discrete trajectory raw_length {traj_len} is much larger than "
            f"belief raw_length {belief_len} (expected within 2) — "
            f"likely using interpolated paths instead of waypoint positions"
        )
        assert conn_len <= belief_len + 2, (
            f"discrete connectivity raw_length {conn_len} is much larger than "
            f"belief raw_length {belief_len} (expected within 2)"
        )

    def test_discrete_drones_traverse_full_mission(self, client):
        """
        Regression: drones must traverse the full mission in discrete mode,
        not just the first ~6% of an interpolated path.

        Checks that at least one drone visits more than 3 distinct (x, y)
        positions across all steps, or that the last position differs from
        the first position by more than one cell_side_length.

        With the bug (only first 78 of 1225 interpolated points kept), drones
        appear nearly stationary and this assertion fails.
        """
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        data = client.post(
            f"/api/playback/{TC_FRONT}",
            json={"index": 0, "config": VALID_CFG},
        ).json()
        cell_side = data["cell_side_length"]
        n_nodes = data["number_of_nodes"]
        xs = data["trajectories"]["x"]
        ys = data["trajectories"]["y"]

        # Check drone rows (skip node 0 = base station which stays fixed).
        traversal_ok = False
        for node in range(1, n_nodes):
            x_row = [v for v in xs[node] if v is not None]
            y_row = [v for v in ys[node] if v is not None]
            if len(x_row) < 2:
                continue
            # Distinct (x, y) positions across all steps.
            distinct = len(set(zip(x_row, y_row)))
            if distinct > 3:
                traversal_ok = True
                break
            # Or last position differs significantly from first.
            dx = x_row[-1] - x_row[0]
            dy = y_row[-1] - y_row[0]
            dist = math.sqrt(dx * dx + dy * dy)
            if dist > cell_side:
                traversal_ok = True
                break

        assert traversal_ok, (
            "No drone traverses the full mission in discrete mode — "
            "drones appear nearly stationary (bug: only first few interpolated steps kept)"
        )


# ---------------------------------------------------------------------------
# POST /api/playback/{scenario} — 200 realtime
# ---------------------------------------------------------------------------

class TestPlaybackRealtime:
    """Integration tests for the realtime time model — same alignment invariant."""

    REALTIME_CFG = {**VALID_CFG, "time_model": "realtime"}

    def test_playback_realtime_200(self, client):
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        resp = client.post(
            f"/api/playback/{TC_FRONT}",
            json={"index": 0, "config": self.REALTIME_CFG},
        )
        assert resp.status_code == 200

    def test_playback_realtime_step_alignment(self, client):
        """Same alignment invariant for realtime (steps may differ from discrete)."""
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        data = client.post(
            f"/api/playback/{TC_FRONT}",
            json={"index": 0, "config": self.REALTIME_CFG},
        ).json()
        steps = data["steps"]
        assert steps > 0

        traj_len = len(data["trajectories"]["x"][0])
        conn_len = len(data["connectivity"])
        belief_len = len(data["belief"][0])
        tk_len = len(data["targets_known"])

        assert traj_len == steps, f"traj {traj_len} != steps {steps}"
        assert conn_len == steps, f"conn {conn_len} != steps {steps}"
        assert belief_len == steps, f"belief {belief_len} != steps {steps}"
        assert tk_len == steps, f"targets_known {tk_len} != steps {steps}"

    def test_playback_realtime_time_model_echo(self, client):
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        data = client.post(
            f"/api/playback/{TC_FRONT}",
            json={"index": 0, "config": self.REALTIME_CFG},
        ).json()
        assert data["time_model"] == "realtime"


# ---------------------------------------------------------------------------
# POST /api/playback/{scenario} — stride downsampling
# ---------------------------------------------------------------------------

class TestPlaybackStride:
    """Stride downsamples the step axis uniformly; alignment still holds."""

    def test_playback_stride_2_200(self, client):
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        resp = client.post(
            f"/api/playback/{TC_FRONT}",
            json={"index": 0, "config": VALID_CFG, "stride": 2},
        )
        assert resp.status_code == 200

    def test_playback_stride_2_steps_about_half(self, client):
        """steps with stride=2 is about half of steps with stride=1."""
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        d1 = client.post(
            f"/api/playback/{TC_FRONT}",
            json={"index": 0, "config": VALID_CFG, "stride": 1},
        ).json()
        d2 = client.post(
            f"/api/playback/{TC_FRONT}",
            json={"index": 0, "config": VALID_CFG, "stride": 2},
        ).json()
        steps_1 = d1["steps"]
        steps_2 = d2["steps"]
        # numpy slicing: len(range(0, N, 2)) == ceil(N/2)
        expected = math.ceil(steps_1 / 2)
        assert steps_2 == expected, (
            f"stride=2 steps={steps_2}, expected ceil({steps_1}/2)={expected}"
        )

    def test_playback_stride_2_alignment(self, client):
        """Alignment invariant still holds after stride=2 downsampling."""
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        data = client.post(
            f"/api/playback/{TC_FRONT}",
            json={"index": 0, "config": VALID_CFG, "stride": 2},
        ).json()
        steps = data["steps"]
        assert steps > 0

        assert len(data["trajectories"]["x"][0]) == steps
        assert len(data["connectivity"]) == steps
        assert len(data["belief"][0]) == steps
        assert len(data["targets_known"]) == steps

    def test_playback_stride_echo(self, client):
        """stride is echoed in the payload."""
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        data = client.post(
            f"/api/playback/{TC_FRONT}",
            json={"index": 0, "config": VALID_CFG, "stride": 3},
        ).json()
        assert data["stride"] == 3


# ---------------------------------------------------------------------------
# Error cases → 422 / 404
# ---------------------------------------------------------------------------

class TestPlaybackErrors:
    def test_p_leq_q_422(self, client):
        """detection_prob <= false_alarm_prob must return 422."""
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        bad_cfg = {**VALID_CFG, "detection_prob": 0.3, "false_alarm_prob": 0.5}
        resp = client.post(
            f"/api/playback/{TC_FRONT}",
            json={"index": 0, "config": bad_cfg},
        )
        assert resp.status_code == 422

    def test_target_outside_grid_422(self, client):
        """target_locations with a cell id outside the grid must return 422."""
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        bad_cfg = {**VALID_CFG, "target_locations": [9999]}
        resp = client.post(
            f"/api/playback/{TC_FRONT}",
            json={"index": 0, "config": bad_cfg},
        )
        assert resp.status_code == 422

    def test_index_out_of_range_422(self, client):
        """An index far outside the front must return 422."""
        if not _seed_present(client):
            pytest.skip("TC front seed not present")
        resp = client.post(
            f"/api/playback/{TC_FRONT}",
            json={"index": 1_000_000, "config": VALID_CFG},
        )
        assert resp.status_code == 422

    def test_missing_scenario_404(self, client):
        resp = client.post(
            "/api/playback/NOPE_missing",
            json={"index": 0, "config": VALID_CFG},
        )
        assert resp.status_code == 404

    def test_traversal_scenario_404(self, client):
        resp = client.post(
            "/api/playback/..%2F..%2Fetc%2Fpasswd",
            json={"index": 0, "config": VALID_CFG},
        )
        assert resp.status_code == 404
