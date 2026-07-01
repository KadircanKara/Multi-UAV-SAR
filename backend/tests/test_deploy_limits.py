"""Deploy-safety guards on the optimizer: per-run parameter caps + rate limiting.

These bound the cost of a public deployment. Caps limit how big any single run
can be (the cost driver); the rate limit throttles how often runs can be started.
Both default to permissive values (the project's real maximums) and tighten via
environment variables for a public deploy — see settings.py.
"""
import time


def _cfg(**over):
    body = {
        "optimization_type": "MOO",
        "method": "NSGA2",
        "objectives": ["Mission Time", "Percentage Connectivity"],
        "pop_size": 12,
        "n_gen": 5,
        "scenario": {"number_of_drones": 4, "n_visits": 2},
    }
    body.update(over)
    return body


def _runnable(**over):
    body = _cfg(pop_size=12, n_gen=5, max_mission_time=None, min_connectivity=None)
    body.update(over)
    return body


# ─── per-run parameter caps ─────────────────────────────────────────────────

def test_drones_within_default_cap_ok(client):
    r = client.post("/api/optimize/check",
                    json=_cfg(scenario={"number_of_drones": 16, "n_visits": 2}))
    assert r.status_code == 200


def test_drones_over_default_cap_rejected(client):
    r = client.post("/api/optimize/check",
                    json=_cfg(scenario={"number_of_drones": 17, "n_visits": 2}))
    assert r.status_code == 422


def test_grid_size_over_default_cap_rejected(client):
    r = client.post("/api/optimize/check",
                    json=_cfg(scenario={"grid_size": 9, "number_of_drones": 4, "n_visits": 2}))
    assert r.status_code == 422


def test_drones_cap_is_env_configurable(client, monkeypatch):
    from app import settings
    monkeypatch.setattr(settings, "MAX_DRONES", 4, raising=False)
    assert client.post("/api/optimize/check",
                       json=_cfg(scenario={"number_of_drones": 4, "n_visits": 2})).status_code == 200
    assert client.post("/api/optimize/check",
                       json=_cfg(scenario={"number_of_drones": 5, "n_visits": 2})).status_code == 422


def test_n_gen_cap_is_env_configurable(client, monkeypatch):
    from app import settings
    monkeypatch.setattr(settings, "MAX_N_GEN", 50, raising=False)
    assert client.post("/api/optimize/check", json=_cfg(n_gen=50)).status_code == 200
    assert client.post("/api/optimize/check", json=_cfg(n_gen=51)).status_code == 422


# ─── rate limiting ──────────────────────────────────────────────────────────

def test_optimize_is_rate_limited(client, monkeypatch):
    """With the per-IP limit set to 1/minute, the second start within the window
    is refused with 429 (before the optimizer is even touched)."""
    from app import settings
    monkeypatch.setattr(settings, "OPTIMIZE_RATE_LIMIT", "1/minute", raising=False)
    client.app.state.limiter.reset()
    try:
        r1 = client.post("/api/optimize", json=_runnable())
        r2 = client.post("/api/optimize", json=_runnable())
        # First is allowed (200 start, or 409 if a prior run still occupies the
        # single worker); the second is blocked purely by the rate limiter.
        assert r1.status_code in (200, 409), r1.text
        assert r2.status_code == 429, r2.text
        # Drain the tiny run if one actually started, so it can't block later tests.
        if r1.status_code == 200:
            rid = r1.json()["run_id"]
            for _ in range(60):
                if client.get(f"/api/optimize/{rid}").json()["state"] != "running":
                    break
                time.sleep(0.5)
    finally:
        client.app.state.limiter.reset()
