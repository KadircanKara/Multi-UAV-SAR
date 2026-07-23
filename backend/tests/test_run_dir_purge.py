"""Temp run dirs under Results/.runs must be bounded, or the volume fills.

The first version purged only dirs older than RUN_TTL_HOURS, and only when a new
run started — so a burst that stopped, or a long-lived process, leaked disk
indefinitely. Purge now also caps the total count (keeping the newest) and never
touches a run that is still active.
"""
import os
import time

from app import optimizer_service, settings


def _mkrun(root, name, age_hours):
    d = os.path.join(root, ".runs", name)
    os.makedirs(d)
    # A couple of bytes so the dir is representative of a real run.
    with open(os.path.join(d, "status.json"), "w") as fh:
        fh.write("{}")
    stamp = time.time() - age_hours * 3600
    os.utime(d, (stamp, stamp))
    return d


def test_purge_removes_dirs_older_than_ttl(tmp_path, monkeypatch):
    monkeypatch.setattr(settings, "RESULTS_ROOT", str(tmp_path))
    monkeypatch.setattr(settings, "RUN_TTL_HOURS", 24)
    monkeypatch.setattr(settings, "OPTIMIZE_RUN_KEEP", 100)
    old = _mkrun(tmp_path, "old", age_hours=48)
    fresh = _mkrun(tmp_path, "fresh", age_hours=1)

    optimizer_service._purge_stale_runs()

    assert not os.path.exists(old)
    assert os.path.exists(fresh)


def test_purge_caps_count_keeping_newest(tmp_path, monkeypatch):
    monkeypatch.setattr(settings, "RESULTS_ROOT", str(tmp_path))
    monkeypatch.setattr(settings, "RUN_TTL_HOURS", 100000)  # age never triggers
    monkeypatch.setattr(settings, "OPTIMIZE_RUN_KEEP", 3)
    # r0 newest … r4 oldest.
    dirs = [_mkrun(tmp_path, f"r{i}", age_hours=i) for i in range(5)]

    optimizer_service._purge_stale_runs()

    assert all(os.path.exists(dirs[i]) for i in (0, 1, 2)), "newest KEEP must survive"
    assert not os.path.exists(dirs[3])
    assert not os.path.exists(dirs[4])


def test_purge_never_removes_a_protected_run(tmp_path, monkeypatch):
    monkeypatch.setattr(settings, "RESULTS_ROOT", str(tmp_path))
    monkeypatch.setattr(settings, "RUN_TTL_HOURS", 24)
    monkeypatch.setattr(settings, "OPTIMIZE_RUN_KEEP", 1)
    # Old (past TTL) AND over the count cap — would be purged twice over, but it
    # is an in-flight run's dir, so it must survive.
    active = _mkrun(tmp_path, "active", age_hours=48)
    _mkrun(tmp_path, "newer", age_hours=1)

    optimizer_service._purge_stale_runs(protected={"active"})

    assert os.path.exists(active)


def test_purge_is_best_effort_on_missing_root(tmp_path, monkeypatch):
    monkeypatch.setattr(settings, "RESULTS_ROOT", str(tmp_path / "does-not-exist"))
    # No .runs dir at all — must return quietly, not raise.
    assert optimizer_service._purge_stale_runs() == []


def test_janitor_disabled_when_interval_zero():
    optimizer_service.stop_janitor()
    optimizer_service.start_janitor(interval=0)
    assert optimizer_service._janitor_thread is None


def test_shutdown_does_a_final_sweep(tmp_path, monkeypatch):
    monkeypatch.setattr(settings, "RESULTS_ROOT", str(tmp_path))
    monkeypatch.setattr(settings, "RUN_TTL_HOURS", 24)
    monkeypatch.setattr(settings, "OPTIMIZE_RUN_KEEP", 100)
    old = _mkrun(tmp_path, "old", age_hours=48)

    optimizer_service.shutdown(wait=False)  # no active jobs → safe

    assert not os.path.exists(old), "a clean shutdown must sweep leftover run dirs"


def test_janitor_sweeps_on_its_interval(tmp_path, monkeypatch):
    monkeypatch.setattr(settings, "RESULTS_ROOT", str(tmp_path))
    monkeypatch.setattr(settings, "RUN_TTL_HOURS", 24)
    monkeypatch.setattr(settings, "OPTIMIZE_RUN_KEEP", 100)
    old = _mkrun(tmp_path, "old", age_hours=48)

    optimizer_service.stop_janitor()
    optimizer_service.start_janitor(interval=0.05)
    try:
        deadline = time.time() + 3.0
        while os.path.exists(old) and time.time() < deadline:
            time.sleep(0.05)
        assert not os.path.exists(old), "janitor should have swept the stale dir"
    finally:
        optimizer_service.stop_janitor()
