"""Tests for the optimizer run queue: admission control, queue position, cancel.

A run used to be rejected outright (409) whenever another was in flight. It is
now queued instead, so these tests cover who gets in, where they sit, and how
they leave.

The waiting line is the service's own, not the pool's. ProcessPoolExecutor
buffers more work items than it has workers and flips those futures to
``running()`` before any worker touches them, so its queue can neither be
counted nor positioned. The service therefore submits only as many runs as
there are workers and holds the rest itself.

The pool is stubbed with futures that stay PENDING until a test finishes them,
so the state machine is driven deterministically without real pymoo runs.
"""
import shutil
from concurrent.futures import Future

import pytest

from app import optimizer_service, settings


class _StubExecutor:
    """Stands in for the ProcessPoolExecutor. Every submission occupies a worker
    until the test finishes it; nothing actually executes.

    ``futures`` doubles as the record of what the service chose to dispatch, so
    tests assert on its length to check the service is holding runs back rather
    than handing everything to the pool.
    """

    def __init__(self):
        self.futures: list[Future] = []

    def submit(self, *args, **kwargs) -> Future:
        fut = Future()
        self.futures.append(fut)
        return fut

    def shutdown(self, *args, **kwargs) -> None:
        pass

    def finish(self, i: int, result: dict | None = None) -> None:
        """Run *i* completes, freeing its worker for whoever is next in line."""
        self.futures[i].set_result(result or {"state": "done", "front": None})


@pytest.fixture
def pool(monkeypatch):
    """Stub executor + a clean run registry + generous limits.

    Individual tests tighten whichever limit they are about.
    """
    stub = _StubExecutor()
    monkeypatch.setattr(optimizer_service, "_get_executor", lambda: stub)
    monkeypatch.setattr(settings, "OPTIMIZE_WORKERS", 1)
    monkeypatch.setattr(settings, "OPTIMIZE_QUEUE_MAX", 16)
    monkeypatch.setattr(settings, "OPTIMIZE_MAX_PER_CLIENT", 16)

    saved = dict(optimizer_service._jobs)
    optimizer_service._jobs.clear()
    yield stub
    for job in optimizer_service._jobs.values():
        shutil.rmtree(job["run_dir"], ignore_errors=True)
    optimizer_service._jobs.clear()
    optimizer_service._jobs.update(saved)


_SCENARIO = {"number_of_drones": 4, "n_visits": 2}


def _cfg(**over):
    body = {
        "optimization_type": "MOO",
        "method": "NSGA2",
        "objectives": ["Mission Time", "Percentage Connectivity"],
        "pop_size": 12,
        "n_gen": 5,
        "scenario": _SCENARIO,
    }
    body.update(over)
    return body


def _start(client, **over):
    resp = client.post("/api/optimize", json=_cfg(**over))
    assert resp.status_code == 200, resp.text
    return resp.json()


# ─── queueing instead of rejecting ────────────────────────────────────────────

def test_second_run_is_queued_instead_of_rejected(client, pool):
    _start(client, seed=1)

    second = client.post("/api/optimize", json=_cfg(seed=2))

    assert second.status_code == 200, second.text
    status = client.get(f"/api/optimize/{second.json()['run_id']}")
    assert status.json()["state"] == "queued"


def test_start_response_tells_caller_it_was_queued(client, pool):
    _start(client, seed=1)

    second = _start(client, seed=2)

    assert second["queued"] is True
    assert second["queue_position"] == 1


def test_first_run_is_not_reported_as_queued(client, pool):
    first = _start(client, seed=1)

    assert first["queued"] is False
    assert first["queue_position"] is None


# ─── dispatch: never hand the pool more than it has workers ───────────────────

def test_only_as_many_runs_as_workers_reach_the_pool(client, pool, monkeypatch):
    monkeypatch.setattr(settings, "OPTIMIZE_WORKERS", 2)

    for seed in (1, 2, 3, 4):
        _start(client, seed=seed)

    assert len(pool.futures) == 2


def test_runs_within_the_worker_count_all_start(client, pool, monkeypatch):
    monkeypatch.setattr(settings, "OPTIMIZE_WORKERS", 2)

    first = _start(client, seed=1)
    second = _start(client, seed=2)

    assert first["queued"] is False
    assert second["queued"] is False
    assert client.get(f"/api/optimize/{second['run_id']}").json()["state"] == "running"


def test_finishing_a_run_dispatches_the_next_in_line(client, pool, monkeypatch):
    monkeypatch.setattr(settings, "OPTIMIZE_WORKERS", 2)
    _start(client, seed=1)
    _start(client, seed=2)
    third = _start(client, seed=3)
    assert client.get(f"/api/optimize/{third['run_id']}").json()["state"] == "queued"

    pool.finish(0)

    assert len(pool.futures) == 3
    assert client.get(f"/api/optimize/{third['run_id']}").json()["state"] == "running"


def test_a_waiting_run_never_reports_generation_progress(client, pool, monkeypatch):
    """It has no worker, so it must not look like a run stuck at generation 0."""
    monkeypatch.setattr(settings, "OPTIMIZE_WORKERS", 1)
    _start(client, seed=1)
    second = _start(client, seed=2)

    body = client.get(f"/api/optimize/{second['run_id']}").json()

    assert body["state"] == "queued"
    assert body["gen"] is None


# ─── queue position ───────────────────────────────────────────────────────────

def test_queue_position_counts_runs_waiting_ahead(client, pool):
    _start(client, seed=1)
    second = _start(client, seed=2)
    third = _start(client, seed=3)

    assert client.get(f"/api/optimize/{second['run_id']}").json()["queue_position"] == 1
    assert client.get(f"/api/optimize/{third['run_id']}").json()["queue_position"] == 2


def test_queue_position_advances_when_the_run_ahead_starts(client, pool):
    _start(client, seed=1)
    second = _start(client, seed=2)
    third = _start(client, seed=3)
    assert client.get(f"/api/optimize/{third['run_id']}").json()["queue_position"] == 2

    pool.finish(0)  # the pool promotes the run that was waiting

    assert client.get(f"/api/optimize/{third['run_id']}").json()["queue_position"] == 1


def test_running_run_reports_running_with_no_queue_position(client, pool):
    first = _start(client, seed=1)

    body = client.get(f"/api/optimize/{first['run_id']}").json()

    assert body["state"] == "running"
    assert body["queue_position"] is None


# ─── admission control ────────────────────────────────────────────────────────

def test_full_queue_is_rejected_with_409(client, pool, monkeypatch):
    monkeypatch.setattr(settings, "OPTIMIZE_QUEUE_MAX", 2)
    _start(client, seed=1)
    _start(client, seed=2)

    third = client.post("/api/optimize", json=_cfg(seed=3))

    assert third.status_code == 409
    assert "queue" in third.json()["detail"].lower()


def test_client_holding_max_runs_is_rejected_with_409(client, pool, monkeypatch):
    monkeypatch.setattr(settings, "OPTIMIZE_MAX_PER_CLIENT", 1)
    _start(client, seed=1)

    second = client.post("/api/optimize", json=_cfg(seed=2))

    assert second.status_code == 409
    detail = second.json()["detail"].lower()
    assert "you" in detail or "your" in detail


def test_finished_run_frees_its_slot(client, pool, monkeypatch):
    monkeypatch.setattr(settings, "OPTIMIZE_QUEUE_MAX", 1)
    _start(client, seed=1)
    assert client.post("/api/optimize", json=_cfg(seed=2)).status_code == 409

    pool.finish(0)

    assert client.post("/api/optimize", json=_cfg(seed=3)).status_code == 200


# ─── leaving the queue ────────────────────────────────────────────────────────

def test_stopping_a_queued_run_keeps_it_from_ever_reaching_the_pool(client, pool):
    _start(client, seed=1)
    second = _start(client, seed=2)

    stop = client.post(f"/api/optimize/{second['run_id']}/stop", json={})

    assert stop.status_code == 200
    assert stop.json()["stopping"] is True
    assert len(pool.futures) == 1  # only the run that was already on a worker


def test_cancelled_run_reports_cancelled_not_failed(client, pool):
    _start(client, seed=1)
    second = _start(client, seed=2)
    client.post(f"/api/optimize/{second['run_id']}/stop", json={})

    body = client.get(f"/api/optimize/{second['run_id']}").json()

    assert body["state"] == "cancelled"


def test_cancelled_run_frees_its_slot(client, pool, monkeypatch):
    monkeypatch.setattr(settings, "OPTIMIZE_QUEUE_MAX", 2)
    _start(client, seed=1)
    second = _start(client, seed=2)
    assert client.post("/api/optimize", json=_cfg(seed=3)).status_code == 409

    client.post(f"/api/optimize/{second['run_id']}/stop", json={})

    assert client.post("/api/optimize", json=_cfg(seed=4)).status_code == 200


def test_stopping_a_running_run_still_uses_the_cancel_file(client, pool):
    """A started run can't be cancelled through the future — it stops
    cooperatively so it can return its best-so-far front."""
    import os

    first = _start(client, seed=1)

    stop = client.post(f"/api/optimize/{first['run_id']}/stop", json={})

    assert stop.json()["stopping"] is True
    assert not pool.futures[0].cancelled()
    run_dir = optimizer_service._jobs[first["run_id"]]["run_dir"]
    assert os.path.isfile(os.path.join(run_dir, "cancel"))
