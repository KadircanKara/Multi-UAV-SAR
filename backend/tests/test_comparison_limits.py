"""The comparison endpoints fan one request out to up to 360 (objective) / 144
(time) heavy per-scenario operations. The per-IP rate limit meters *requests*,
not work, so one request can pin the box for minutes. These tests cover the two
guards that bound the cost without shrinking the (UI-matching) scenario caps:

  * a server-wide concurrency slot, so only N heavy comparisons run at once;
  * the endpoints returning 503 (not a silent pile-up) when the slots are full.
"""
import pytest

from app import concurrency, settings


def test_second_concurrent_slot_is_rejected(monkeypatch):
    monkeypatch.setattr(settings, "COMPARISON_CONCURRENCY", 1)
    with concurrency.comparison_slot():
        with pytest.raises(concurrency.BusyError):
            with concurrency.comparison_slot():
                pass


def test_slot_is_released_for_reuse(monkeypatch):
    monkeypatch.setattr(settings, "COMPARISON_CONCURRENCY", 1)
    with concurrency.comparison_slot():
        pass
    # Freed again — acquiring once more must succeed.
    with concurrency.comparison_slot():
        pass


def test_comparison_returns_503_when_busy(client, monkeypatch):
    """With every slot held, the endpoint refuses fast instead of queueing more
    synchronous work behind the ones already running."""
    monkeypatch.setattr(settings, "COMPARISON_CONCURRENCY", 1)
    sem = concurrency._get_sem()
    assert sem.acquire(blocking=False)
    try:
        resp = client.post("/api/comparison", json={"scenarios": ["anything"]})
        assert resp.status_code == 503
    finally:
        sem.release()


def test_comparison_time_returns_503_when_busy(client, monkeypatch):
    monkeypatch.setattr(settings, "COMPARISON_CONCURRENCY", 1)
    sem = concurrency._get_sem()
    assert sem.acquire(blocking=False)
    try:
        resp = client.post(
            "/api/comparison/time",
            json={
                "scenarios": ["anything"],
                "config": {
                    "detection_prob": 0.9,
                    "false_alarm_prob": 0.1,
                    "belief_threshold": 0.5,
                    "target_locations": [0],
                },
            },
        )
        assert resp.status_code == 503
    finally:
        sem.release()
