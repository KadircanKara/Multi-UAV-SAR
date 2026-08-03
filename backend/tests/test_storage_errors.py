"""A run whose storage cannot be written must fail loudly and correctly.

start_run creates .runs/<id> and seeds status.json inside the lock with no error
handling, so a full / read-only / wrong-owner volume raised a bare OSError that
the router did not catch — an opaque 500 with no signal that STORAGE is the
cause. It should be a typed error surfaced as 503.
"""
import pytest

from app import optimizer_service, settings


def _boom(*args, **kwargs):
    raise OSError("No space left on device")


def test_start_run_raises_storage_error_when_run_dir_uncreatable(monkeypatch):
    from app.schemas import ScenarioConfig

    scenario = ScenarioConfig(number_of_drones=4, n_visits=2).to_scenario_dict()
    monkeypatch.setattr(optimizer_service.os, "makedirs", _boom)
    with pytest.raises(optimizer_service.StorageUnavailableError):
        optimizer_service.start_run(
            "MOO", "NSGA2",
            ["Mission Time", "Percentage Connectivity"], None,
            12, 5, 1, scenario,
        )


def test_optimize_endpoint_maps_storage_error_to_503(client, monkeypatch):
    from app.routers import optimize as optimize_router

    def raise_storage(*args, **kwargs):
        raise optimizer_service.StorageUnavailableError("storage unavailable")

    monkeypatch.setattr(optimize_router, "start_run", raise_storage)
    resp = client.post("/api/optimize", json={
        "optimization_type": "MOO",
        "method": "NSGA2",
        "objectives": ["Mission Time", "Percentage Connectivity"],
        "pop_size": 12,
        "n_gen": 5,
        "scenario": {"number_of_drones": 4, "n_visits": 2},
    })
    assert resp.status_code == 503
