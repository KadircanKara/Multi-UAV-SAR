"""The backend had no logging at all — a failed run or a 500 left no server-side
trace. At minimum, the paths that fail silently must emit a record so an operator
can tell a disk-full from a worker crash.
"""
import logging

import pytest

from app import optimizer_service
from app.schemas import ScenarioConfig


def _boom(*args, **kwargs):
    raise OSError("No space left on device")


def test_storage_failure_is_logged(monkeypatch, caplog):
    scenario = ScenarioConfig(number_of_drones=4, n_visits=2).to_scenario_dict()
    monkeypatch.setattr(optimizer_service.os, "makedirs", _boom)
    with caplog.at_level(logging.ERROR, logger="sar.optimizer"):
        with pytest.raises(optimizer_service.StorageUnavailableError):
            optimizer_service.start_run(
                "MOO", "NSGA2",
                ["Mission Time", "Percentage Connectivity"], None,
                12, 5, 1, scenario,
            )
    assert any(r.levelno >= logging.ERROR for r in caplog.records), \
        "an unwritable run volume must be logged, not just raised"
