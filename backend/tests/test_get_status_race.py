"""get_status must not crash on the brief dispatch window.

_dispatch_locked pops submit_args and only then, after executor.submit()
returns, sets job["future"]. In that window a job has neither submit_args nor a
future. A poll landing there used to fall through the _is_waiting() check and
evaluate job["future"].cancelled() on None -> AttributeError -> uncaught 500.
It should read as 'queued' instead.
"""
import json
import os

from app import optimizer_service, settings


def test_get_status_survives_the_dispatch_window(tmp_path, monkeypatch):
    monkeypatch.setattr(settings, "RESULTS_ROOT", str(tmp_path))
    run_id = "deadbeef0001"
    run_dir = os.path.join(tmp_path, ".runs", run_id)
    os.makedirs(run_dir)
    with open(os.path.join(run_dir, "status.json"), "w") as fh:
        json.dump({"state": "running", "gen": 0, "n_gen": 50}, fh)

    # Mid-dispatch: submit_args already popped, future not yet assigned. This is
    # exactly the state _is_waiting() returns False for.
    optimizer_service._jobs[run_id] = {
        "future": None,
        "run_dir": run_dir,
        "scenario_name": "S",
        "model_key": "M",
        "model_dict": {},
        "client_key": "-",
    }
    try:
        status = optimizer_service.get_status(run_id)
        assert status["state"] == "queued"
    finally:
        optimizer_service._jobs.pop(run_id, None)


def test_export_during_dispatch_window_reports_not_ready(tmp_path, monkeypatch):
    """serialize_finished_run must not read the (absent) result pickle during the
    same window — it should raise RunNotReadyError, not FileNotFoundError."""
    import pytest

    monkeypatch.setattr(settings, "RESULTS_ROOT", str(tmp_path))
    run_id = "cafe00000001"
    run_dir = os.path.join(tmp_path, ".runs", run_id)
    os.makedirs(run_dir)  # no Objectives.pkl yet — the worker hasn't written it

    optimizer_service._jobs[run_id] = {
        "future": None,
        "run_dir": run_dir,
        "scenario_name": "S",
        "model_key": "M",
        "model_dict": {},
        "client_key": "-",
    }
    try:
        with pytest.raises(optimizer_service.RunNotReadyError):
            optimizer_service.serialize_finished_run(run_id)
    finally:
        optimizer_service._jobs.pop(run_id, None)
