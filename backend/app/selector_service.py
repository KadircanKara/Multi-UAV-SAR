"""
Selector service: loads SolutionSelector instances with an LRU cache,
derives objective polarities from PathFuncDict, and exposes thin helpers
(build_front, get_capabilities, select) consumed by the /api/fronts router.

Import-safety: only SolutionSelection, PathOptimizationModel, PathFuncDict,
pandas, and numpy are used here.  PathAlgorithm / PathUnitTest / main are
NEVER imported.
"""
from __future__ import annotations

import functools
import threading
from typing import Optional

import os

import numpy as np
import pandas as pd

import app.rootpath  # side-effect: inserts repo root into sys.path
from app import settings, models_registry
from app.library_service import _is_safe_scenario_name, resolve_model_key
from app.model_aliases import to_display, to_storage

from PathOptimizationModel import AVAILABLE_MODELS
from PathFuncDict import model_metric_info
from SolutionSelection import SolutionSelector, StrategyUnavailableError


# ---------------------------------------------------------------------------
# Re-export StrategyUnavailableError so the router can catch it by name
# without importing SolutionSelection directly.
# ---------------------------------------------------------------------------
__all__ = [
    "StrategyUnavailableError",
    "get_selector",
    "get_polarities",
    "build_front",
    "get_capabilities",
    "select",
    "build_front_from_selector",
    "capabilities_from_selector",
    "select_from_selector",
]


# ---------------------------------------------------------------------------
# Polarity map — derived authoritatively from PathFuncDict
#   +1  → minimize (lower is better)
#   -1  → maximize (higher is better, stored negative in .F)
# ---------------------------------------------------------------------------

_POLARITY: dict[str, int] = {
    name: int(info[1])
    for name, info in model_metric_info["Objectives"].items()
}
# Fallback for objectives not explicitly listed in PathFuncDict (WS composite)
_DEFAULT_POLARITY = 1


def get_polarities(model: dict) -> dict[str, int]:
    """
    Return {objective_name: polarity} for all objectives in ``model["F"]``.

    +1 = minimize, -1 = maximize.  Derived from PathFuncDict; unknown
    objectives (e.g. WS composite names) default to +1.
    """
    return {name: _POLARITY.get(name, _DEFAULT_POLARITY) for name in model["F"]}


# ---------------------------------------------------------------------------
# Cached loader
# ---------------------------------------------------------------------------

class _SelectorNotFound(Exception):
    """Raised when scenario / model_key cannot be resolved (→ 404)."""


def _resolve_model_key(scenario: str, model_key: Optional[str]) -> str:
    """
    Resolve *model_key* (or auto-derive it) and validate it exists.

    Raises _SelectorNotFound for unknown keys. Returns the STORAGE key.
    """
    scenario = to_storage(scenario)
    model_key = to_storage(model_key)
    if model_key:
        if not models_registry.known(model_key):
            raise _SelectorNotFound(f"Unknown model_key {model_key!r}")
        return model_key
    try:
        resolved = resolve_model_key(scenario)
    except Exception as exc:
        raise _SelectorNotFound(f"Cannot derive model key for {scenario!r}: {exc}") from exc
    if not models_registry.known(resolved):
        raise _SelectorNotFound(f"Derived model key {resolved!r} is not a known model")
    return resolved


# Single-flight guard for _load_selector. Without it, N simultaneous cache
# MISSES for the same (scenario, resolved_model_key) each unpickle the same
# ~160 MB selector in parallel — a burst of identical requests multiplies the
# app's largest allocation by N and can OOM the container. A per-key lock
# serializes the misses: the first loads and warms the lru_cache, the rest wait
# and then hit the now-warm cache. Deadlock-free — only ever ONE lock is held at
# a time (per key), released in a finally by ``with``; the guard below is taken
# only briefly to fetch/create a key's lock and is never held across the load.
_key_locks: dict = {}
_key_locks_guard = threading.Lock()
# Hard cap on retained key locks. get_selector reaches here after only a charset
# check (_is_safe_scenario_name) and a NAME-derived model resolution — neither
# touches disk — so an attacker can mint unlimited DISTINCT valid-looking names
# (varying the numeric suffix: ..._nvisits_1, _nvisits_2, _g_9, ...) that never
# correspond to a real pickle. Retaining a lock per name forever would leak the
# same unbounded-growth class the run registry guards against. Bound it.
_KEY_LOCKS_MAX = 2048


def _get_key_lock(key: tuple) -> threading.Lock:
    """Return the process-wide lock for one cache key, creating it on first use.

    Bounded at _KEY_LOCKS_MAX entries: when the map is full, a new key drops it
    wholesale (``clear``) rather than tracking per-key liveness. Clearing while
    another thread still holds a just-handed-out lock is BENIGN — that thread
    keeps its own reference, so the Lock object stays alive and valid; only the
    dict's mapping to it is discarded. The one consequence is that a concurrent
    miss for a just-cleared key can create a second lock for it, permitting at
    most one non-deduped double-load of that scenario until the cache warms —
    rare and harmless, versus an unbounded leak."""
    with _key_locks_guard:
        lock = _key_locks.get(key)
        if lock is None:
            if len(_key_locks) >= _KEY_LOCKS_MAX:
                _key_locks.clear()
            lock = threading.Lock()
            _key_locks[key] = lock
        return lock


@functools.lru_cache(maxsize=settings.SELECTOR_CACHE_SIZE)
def _load_selector(scenario: str, resolved_model_key: str) -> SolutionSelector:
    """
    Cached loader — keyed on (scenario, resolved_model_key).

    Reads pickles via absolute paths derived from settings.RESULTS_ROOT so
    that CWD does not matter (unlike SolutionSelector.from_scenario which
    uses relative paths from FilePaths.py).

    The _is_safe_scenario_name check MUST run before this is called.
    Raises FileNotFoundError if pickles are missing (→ 404).
    """
    obj_path = os.path.join(
        settings.RESULTS_ROOT, "Objectives",
        f"{scenario}-ObjectiveValues.pkl",
    )
    sol_path = os.path.join(
        settings.RESULTS_ROOT, "Solutions",
        f"{scenario}-SolutionObjects.pkl",
    )
    missing = [p for p in (obj_path, sol_path) if not os.path.isfile(p)]
    if missing:
        raise FileNotFoundError(
            f"No saved results for scenario {scenario!r} "
            f"(missing: {', '.join(missing)})"
        )
    F: pd.DataFrame = pd.read_pickle(obj_path)
    raw_solutions = pd.read_pickle(sol_path)
    model = models_registry.get_model(resolved_model_key)
    # Normalise rows: SolutionObjects rows can be 1-element numpy arrays
    solutions = [
        s[0] if isinstance(s, np.ndarray) else s for s in list(raw_solutions)
    ]
    return SolutionSelector(F, solutions, model)


def get_selector(scenario: str, model_key: Optional[str] = None) -> SolutionSelector:
    """
    Validate safety, resolve model key, return a (cached) SolutionSelector.

    Raises:
        _SelectorNotFound — scenario name unsafe, unknown model, or missing pickles.
    """
    # Normalize display aliases (…TCDV…) to storage (…TCDT…) so file lookups and
    # the selector cache key both use the real on-disk names.
    scenario = to_storage(scenario)
    model_key = to_storage(model_key)
    if not _is_safe_scenario_name(scenario):
        raise _SelectorNotFound(f"Unsafe or invalid scenario name: {scenario!r}")
    resolved = _resolve_model_key(scenario, model_key)
    # Single-flight the (possibly cold) load: concurrent misses for the same key
    # queue on its lock instead of each unpickling ~160 MB in parallel. The lock
    # is keyed identically to the lru_cache, so the waiters hit the warm cache.
    try:
        with _get_key_lock((scenario, resolved)):
            return _load_selector(scenario, resolved)
    except FileNotFoundError as exc:
        raise _SelectorNotFound(str(exc)) from exc


# ---------------------------------------------------------------------------
# Public service helpers
# ---------------------------------------------------------------------------

def _float(v) -> float:
    """Cast numpy / pandas scalar to a JSON-safe Python float."""
    return float(v)


def _objectives_rows(selector: SolutionSelector) -> list[dict]:
    """
    Build the ``solutions`` list for ParetoFront:
      [{index, objectives_signed, objectives_abs}, ...]
    """
    rows = []
    for i, row in selector.F.iterrows():
        signed = {col: _float(row[col]) for col in selector.F.columns}
        abss = {col: _float(abs(row[col])) for col in selector.F.columns}
        rows.append({
            "index": int(i),
            "objectives_signed": signed,
            "objectives_abs": abss,
        })
    return rows


def build_front_from_selector(sel: SolutionSelector) -> dict:
    """
    Return full Pareto-front payload including capabilities for an
    already-constructed SolutionSelector (seeded or reconstructed).

    ``scenario`` and ``model_key`` are best-effort here (a bare selector has
    no scenario name and its model dict may not carry ``model_key``);
    scenario-based callers (``build_front``) overwrite both with the
    authoritative values after delegating here.
    """
    objectives = list(sel.F.columns)
    polarities = get_polarities(sel.model)
    return {
        "scenario": getattr(sel, "scenario", ""),
        "model_key": sel.model.get("model_key", ""),
        "objectives": objectives,
        "polarities": polarities,
        "result_kind": sel.result_kind,
        "n_solutions": int(len(sel.solutions)),
        "solutions": _objectives_rows(sel),
        "capabilities": capabilities_from_selector(sel),
    }


def capabilities_from_selector(sel: SolutionSelector) -> dict:
    return sel.capabilities()


def select_from_selector(
    sel: SolutionSelector,
    strategy: str,
    objective_name: Optional[str] = None,
    weights: Optional[dict[str, float]] = None,
    index: Optional[int] = None,
) -> dict:
    """
    Dispatch a selection strategy against an already-constructed
    SolutionSelector and return the result dict.

    Raises:
        StrategyUnavailableError — invalid strategy, wrong result_kind,
            missing required argument (→ 422).
    """
    if strategy == "best":
        if not objective_name:
            raise StrategyUnavailableError(
                "strategy='best' requires objective_name"
            )
        idx, _sol, label = sel.best(objective_name)
    elif strategy == "balanced":
        idx, _sol, label = sel.balanced()
    elif strategy == "knee":
        idx, _sol, label = sel.knee()
    elif strategy == "by_weights":
        if not weights:
            raise StrategyUnavailableError(
                "strategy='by_weights' requires weights dict"
            )
        idx, _sol, label = sel.by_weights(weights)
    elif strategy == "by_index":
        if index is None:
            raise StrategyUnavailableError(
                "strategy='by_index' requires index"
            )
        idx, _sol, label = sel.by_index(index)
    elif strategy == "the_solution":
        idx, _sol, label = sel.the_solution()
    else:
        raise StrategyUnavailableError(
            f"Unknown strategy {strategy!r}. Valid: best, balanced, knee, "
            f"by_weights, by_index, the_solution"
        )

    # Build absolute objectives for the selected row
    row = sel.F.iloc[idx]
    objectives_abs = {col: _float(abs(row[col])) for col in sel.F.columns}

    detail = {
        "index": int(idx),
        "label": str(label),
        "objectives_abs": objectives_abs,
    }
    return {
        "index": int(idx),
        "label": str(label),
        "detail": detail,
    }


def build_front(scenario: str, model_key: Optional[str] = None) -> dict:
    """
    Return full Pareto-front payload including capabilities.

    Raises _SelectorNotFound on bad scenario / missing pickles (→ 404).
    """
    selector = get_selector(scenario, model_key)
    resolved = _resolve_model_key(scenario, model_key)  # storage key
    out = build_front_from_selector(selector)
    # Emit the display form (…TCDV…) so the explorer badge / title show the
    # TBV "V" code. to_display yields the display form for storage OR display input.
    out["scenario"] = to_display(scenario)
    out["model_key"] = to_display(resolved)
    return out


def get_capabilities(scenario: str, model_key: Optional[str] = None) -> dict:
    """
    Return just the capabilities dict for a scenario.

    Raises _SelectorNotFound on bad scenario / missing pickles (→ 404).
    """
    return capabilities_from_selector(get_selector(scenario, model_key))


def select(
    scenario: str,
    strategy: str,
    model_key: Optional[str] = None,
    objective_name: Optional[str] = None,
    weights: Optional[dict[str, float]] = None,
    index: Optional[int] = None,
) -> dict:
    """
    Dispatch a selection strategy and return the result dict.

    Raises:
        _SelectorNotFound — bad scenario / pickles missing (→ 404).
        StrategyUnavailableError — invalid strategy, wrong result_kind,
            missing required argument (→ 422).
    """
    selector = get_selector(scenario, model_key)
    return select_from_selector(selector, strategy, objective_name, weights, index)
