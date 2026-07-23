"""Recovery from a dead worker pool.

A ProcessPoolExecutor breaks permanently when a worker dies (an OOM on a big
run, a native crash): every in-flight future resolves with BrokenProcessPool and
every subsequent submit raises it too. Before the fix the service built the pool
once and never rebuilt it, so one worker death wedged the optimizer until the
container restarted — queued runs never started, new /api/optimize calls 500'd.

The rebuild is deliberately confined to the submit path (_dispatch_locked): the
first submit onto a dead pool rebuilds exactly once, and every later submit in
the same lock-held sweep lands on the fresh pool. It must NOT happen eagerly in
_on_run_finished / get_status, because a worker death resolves every in-flight
future with BrokenProcessPool — so those callbacks fire once PER dead future, and
an unconditional rebuild there would tear down a pool a prior callback already
rebuilt and orphan the run dispatched onto it.

Two stub styles (mirroring test_optimize_queue.py):
  * ``flaky`` — a single stub whose submit raises a set number of times, for the
    submit-retry semantics;
  * ``pools`` — patches ProcessPoolExecutor with a counting factory so each
    _get_executor() build is a distinct "pool generation", which is what lets a
    test model a real worker death and count rebuilds.
"""
import shutil
from concurrent.futures import Future
from concurrent.futures.process import BrokenProcessPool

import pytest

from app import optimizer_service, settings


# ─── submit-retry semantics (single flaky stub) ───────────────────────────────

class _FlakyExecutor:
    """Stub pool whose ``submit`` raises BrokenProcessPool its first
    ``fail_times`` calls, then hands out normal PENDING futures. ``submit_calls``
    records every attempt so a test can prove a failed submit was retried."""

    def __init__(self, fail_times: int = 0):
        self.fail_times = fail_times
        self.submit_calls = 0
        self.futures: list[Future] = []

    def submit(self, *args, **kwargs) -> Future:
        self.submit_calls += 1
        if self.submit_calls <= self.fail_times:
            raise BrokenProcessPool("a worker process died")
        fut: Future = Future()
        self.futures.append(fut)
        return fut

    def shutdown(self, *args, **kwargs) -> None:
        pass


@pytest.fixture
def flaky(monkeypatch):
    """A single flaky executor returned by _get_executor + a clean registry."""
    stub = _FlakyExecutor()
    monkeypatch.setattr(optimizer_service, "_get_executor", lambda: stub)
    monkeypatch.setattr(settings, "OPTIMIZE_WORKERS", 1)
    monkeypatch.setattr(settings, "OPTIMIZE_QUEUE_MAX", 16)
    monkeypatch.setattr(settings, "OPTIMIZE_MAX_PER_CLIENT", 16)

    saved_jobs = dict(optimizer_service._jobs)
    saved_executor = optimizer_service._executor
    optimizer_service._jobs.clear()
    optimizer_service._executor = None
    yield stub
    for job in optimizer_service._jobs.values():
        shutil.rmtree(job["run_dir"], ignore_errors=True)
    optimizer_service._jobs.clear()
    optimizer_service._jobs.update(saved_jobs)
    optimizer_service._executor = saved_executor


# ─── worker-death recovery (counting pool factory) ────────────────────────────

class _Pool:
    """One "pool generation". A real worker death is modelled by marking the pool
    broken and resolving its in-flight futures with BrokenProcessPool; any further
    submit on THIS pool then raises (as a real broken pool does), so the service
    must build a fresh generation. ``shutdown_called`` records whether the pool was
    torn down — a live-run pool must never be."""

    def __init__(self, *args, **kwargs):
        self.futures: list[Future] = []
        self.broken = False
        self.shutdown_called = False

    def submit(self, *args, **kwargs) -> Future:
        if self.broken:
            raise BrokenProcessPool("the pool is broken")
        fut: Future = Future()
        self.futures.append(fut)
        return fut

    def shutdown(self, *args, **kwargs) -> None:
        self.shutdown_called = True


@pytest.fixture
def pools(monkeypatch):
    """Patch ProcessPoolExecutor with a factory that records every pool built, so
    a test can assert how many generations the service created (len == 1 + number
    of rebuilds). Clean registry + generous queue limits."""
    built: list[_Pool] = []

    def _factory(*args, **kwargs) -> _Pool:
        p = _Pool()
        built.append(p)
        return p

    monkeypatch.setattr(optimizer_service, "ProcessPoolExecutor", _factory)
    monkeypatch.setattr(settings, "OPTIMIZE_QUEUE_MAX", 16)
    monkeypatch.setattr(settings, "OPTIMIZE_MAX_PER_CLIENT", 16)

    saved_jobs = dict(optimizer_service._jobs)
    saved_executor = optimizer_service._executor
    optimizer_service._jobs.clear()
    optimizer_service._executor = None
    yield built
    for job in optimizer_service._jobs.values():
        shutil.rmtree(job["run_dir"], ignore_errors=True)
    optimizer_service._jobs.clear()
    optimizer_service._jobs.update(saved_jobs)
    optimizer_service._executor = saved_executor


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


_GENERIC_ERROR = "The optimization failed. Please try again."


# ─── submit path: rebuild + retry, and clean failure ──────────────────────────

def test_submit_hitting_broken_pool_once_recovers_on_retry(client, flaky):
    """A single BrokenProcessPool on submit must rebuild the pool and retry the
    same run onto it — the run starts rather than 500ing the request."""
    flaky.fail_times = 1

    resp = client.post("/api/optimize", json=_cfg(seed=1))

    assert resp.status_code == 200, resp.text
    run_id = resp.json()["run_id"]
    assert flaky.submit_calls == 2          # failed once, retried once
    assert len(flaky.futures) == 1          # retry landed a real future
    assert client.get(f"/api/optimize/{run_id}").json()["state"] == "running"


def test_persistently_broken_pool_fails_the_run_cleanly(client, flaky):
    """If the pool stays broken even after a rebuild, the run is reported failed
    with a generic message — never a 500, never a stuck spinner."""
    flaky.fail_times = 99

    resp = client.post("/api/optimize", json=_cfg(seed=1))

    assert resp.status_code == 200, resp.text   # handled, not a 500
    body = client.get(f"/api/optimize/{resp.json()['run_id']}").json()
    assert body["state"] == "failed"
    # Finding 5: internal error text must not leak in the 200 body.
    assert body["error"] == _GENERIC_ERROR
    assert "BrokenProcessPool" not in body["error"]


# ─── worker death: lazy, idempotent rebuild ───────────────────────────────────

def test_worker_death_lets_the_pool_linger_then_rebuilds_on_next_run(client, pools, monkeypatch):
    """With the queue empty, a worker death does not eagerly rebuild — the dead
    pool lingers (report the run failed, don't thrash) and the NEXT run rebuilds
    it on submit and runs."""
    monkeypatch.setattr(settings, "OPTIMIZE_WORKERS", 1)

    first = _start(client, seed=1)
    assert len(pools) == 1                          # p0 built on first submit
    p0 = pools[0]

    # The worker dies: pool breaks, its in-flight future resolves BrokenProcessPool.
    p0.broken = True
    p0.futures[0].set_exception(BrokenProcessPool("a worker process died"))

    # No queued run → nothing to dispatch → no eager rebuild; still one pool.
    assert len(pools) == 1
    dead = client.get(f"/api/optimize/{first['run_id']}").json()
    assert dead["state"] == "failed"
    assert dead["error"] == _GENERIC_ERROR

    # The next run rebuilds the pool lazily, on submit, and runs.
    second = _start(client, seed=2)
    assert len(pools) == 2                          # rebuilt exactly once
    assert not pools[1].shutdown_called
    assert client.get(f"/api/optimize/{second['run_id']}").json()["state"] == "running"


def test_queued_run_drains_onto_a_rebuilt_pool_after_a_death(client, pools, monkeypatch):
    """A run waiting in line when the worker dies must get dispatched onto a
    freshly rebuilt pool — not wait forever behind the dead one."""
    monkeypatch.setattr(settings, "OPTIMIZE_WORKERS", 1)

    first = _start(client, seed=1)                  # takes the single worker
    second = _start(client, seed=2)                 # waits in line
    assert client.get(f"/api/optimize/{second['run_id']}").json()["state"] == "queued"

    p0 = pools[0]
    p0.broken = True
    p0.futures[0].set_exception(BrokenProcessPool("a worker process died"))

    # The freed slot dispatches the queued run; its submit hits the dead pool and
    # rebuilds exactly once, landing the run on the fresh pool.
    assert len(pools) == 2
    assert client.get(f"/api/optimize/{first['run_id']}").json()["state"] == "failed"
    assert client.get(f"/api/optimize/{second['run_id']}").json()["state"] == "running"
    assert len(pools[1].futures) == 1               # dispatched exactly once
    assert not pools[1].shutdown_called


def test_simultaneous_worker_deaths_rebuild_the_pool_exactly_once(client, pools, monkeypatch):
    """The cascade regression: a worker death breaks the WHOLE pool, so every
    in-flight future resolves BrokenProcessPool and _on_run_finished fires once
    per dead future. The rebuild must happen exactly once (in the submit path),
    not once per dead future — otherwise a later callback tears down the pool an
    earlier callback just rebuilt and orphans the run dispatched onto it."""
    monkeypatch.setattr(settings, "OPTIMIZE_WORKERS", 2)

    rebuilds = {"n": 0}
    real_rebuild = optimizer_service._rebuild_executor_locked

    def _counting_rebuild():
        rebuilds["n"] += 1
        real_rebuild()

    monkeypatch.setattr(optimizer_service, "_rebuild_executor_locked", _counting_rebuild)

    a = _start(client, seed=1)
    b = _start(client, seed=2)
    rq = _start(client, seed=3)                     # waits behind the two workers
    assert client.get(f"/api/optimize/{rq['run_id']}").json()["state"] == "queued"
    assert len(pools) == 1
    p0 = pools[0]
    assert len(p0.futures) == 2                     # both workers busy

    # The pool breaks; BOTH in-flight futures resolve BrokenProcessPool, firing
    # _on_run_finished once each (serialized under _lock).
    p0.broken = True
    p0.futures[0].set_exception(BrokenProcessPool("a worker process died"))
    p0.futures[1].set_exception(BrokenProcessPool("a worker process died"))

    # Exactly one rebuild despite two dead futures.
    assert rebuilds["n"] == 1, f"expected exactly one rebuild, got {rebuilds['n']}"
    assert len(pools) == 2                          # p0 + a single replacement
    p1 = pools[1]
    assert not p1.shutdown_called                   # the fresh pool was NOT torn down

    # The queued run was dispatched exactly once onto the fresh pool and is
    # running — not orphaned or cancelled by a cascading second rebuild.
    assert len(p1.futures) == 1
    assert not p1.futures[0].done()
    assert client.get(f"/api/optimize/{rq['run_id']}").json()["state"] == "running"

    # Both dead runs report failed with the generic message.
    for r in (a, b):
        body = client.get(f"/api/optimize/{r['run_id']}").json()
        assert body["state"] == "failed"
        assert body["error"] == _GENERIC_ERROR
