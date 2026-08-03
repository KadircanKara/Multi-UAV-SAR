"""Server-wide concurrency slots for the heavy read / replay endpoints.

Two families of endpoint do enough synchronous work per request that the per-IP
rate limit alone cannot bound their cost:

  * the comparison endpoints fan ONE request out to hundreds of per-scenario
    operations (selector unpickles / full sensing replays);
  * the single-mission reads (front / capabilities / select, one replay, one
    playback) each unpickle a ~160 MB selector.

The per-IP rate limit caps how *often* a client can start one but not how many
run at once — slowapi is a fixed-window counter, so a whole window's worth of
requests can arrive simultaneously, take every core, and drive the selector
cache toward the container memory limit. Each slot below bounds the number
running concurrently across all clients for its family; the surplus is refused
fast (503) rather than queued behind CPU-bound work already in flight.
"""
from __future__ import annotations

import threading
from contextlib import contextmanager

from app import settings


class BusyError(Exception):
    """Raised when every slot in a pool is taken."""


def _make_slot(size_getter, busy_message: str):
    """Build a (get_sem, slot) pair backed by a resizable bounded semaphore.

    The semaphore is sized from ``size_getter()`` and rebuilt whenever that size
    changes (a live reconfigure, or a test monkeypatching the setting) —
    otherwise the same instance is returned every call so acquisitions actually
    contend. ``slot()`` is a context manager that holds one permit for the block
    or raises BusyError immediately if none are free (it never blocks)."""
    lock = threading.Lock()
    state: dict = {"sem": None, "size": None}

    def get_sem() -> threading.BoundedSemaphore:
        size = max(1, size_getter())
        with lock:
            if state["sem"] is None or state["size"] != size:
                state["sem"] = threading.BoundedSemaphore(size)
                state["size"] = size
            return state["sem"]

    @contextmanager
    def slot():
        sem = get_sem()
        if not sem.acquire(blocking=False):
            raise BusyError(busy_message)
        try:
            yield
        finally:
            sem.release()

    return get_sem, slot


# Comparison fan-out (kept on its own pool so a burst of single-mission reads
# can never starve a comparison, or vice versa). Sized by COMPARISON_CONCURRENCY.
_get_sem, comparison_slot = _make_slot(
    lambda: settings.COMPARISON_CONCURRENCY,
    "The server is busy running other comparisons right now. "
    "Give it a moment and try again.",
)

# Single-mission heavy reads (front / capabilities / select, replay, playback).
# Sized by HEAVY_CONCURRENCY.
_get_heavy_sem, heavy_slot = _make_slot(
    lambda: settings.HEAVY_CONCURRENCY,
    "The server is busy loading other missions right now. "
    "Give it a moment and try again.",
)
