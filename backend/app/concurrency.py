"""Server-wide concurrency slots for the heavy comparison endpoints.

A comparison request fans out to up to hundreds of synchronous per-scenario
operations. The per-IP rate limit caps how *often* one can be started but not
how many run at once, so a burst of parallel comparisons can take every core and
drive the selector cache toward the memory limit. This bounds the number
running concurrently across all clients; the surplus is refused fast (503)
rather than queued behind CPU-bound work already in flight.
"""
from __future__ import annotations

import threading
from contextlib import contextmanager

from app import settings


class BusyError(Exception):
    """Raised when every comparison slot is taken."""


_lock = threading.Lock()
_sem: threading.BoundedSemaphore | None = None
_sem_size: int | None = None


def _get_sem() -> threading.BoundedSemaphore:
    """The shared slot semaphore, sized from settings.COMPARISON_CONCURRENCY.

    Rebuilt if the configured size changes (a live reconfigure, or a test
    monkeypatching the setting) — otherwise the same instance every call.
    """
    global _sem, _sem_size
    size = max(1, settings.COMPARISON_CONCURRENCY)
    with _lock:
        if _sem is None or _sem_size != size:
            _sem = threading.BoundedSemaphore(size)
            _sem_size = size
        return _sem


@contextmanager
def comparison_slot():
    """Hold one comparison slot for the duration of the block, or raise
    BusyError immediately if none are free (never blocks)."""
    sem = _get_sem()
    if not sem.acquire(blocking=False):
        raise BusyError(
            "The server is busy running other comparisons right now. "
            "Give it a moment and try again."
        )
    try:
        yield
    finally:
        sem.release()
