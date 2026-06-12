"""
Models registry: resolves a model_key to its model dict, consulting the built-in
``AVAILABLE_MODELS`` first and then a persisted registry of CUSTOM models created
by the Optimizer's "Save to library" (so saved ad-hoc runs become browsable via
the existing fronts / replay / compare / library endpoints).

The custom registry lives at ``Results/custom_models.json`` ({model_key: model_dict}).
Preset model_keys are never written there (AVAILABLE_MODELS always wins).

Durability: the read cache is keyed on the file's mtime, so writes from another
process are picked up and a transient read fault never poisons a good cache. The
write path (``register``) always reads the current file *fresh* and refuses to
proceed if it is unreadable — it never rewrites the registry from an empty view,
which would silently erase every previously saved model.
"""
from __future__ import annotations

import json
import os
import threading
from typing import Optional

import app.rootpath  # noqa: F401
from app import settings
from PathOptimizationModel import AVAILABLE_MODELS

_lock = threading.Lock()
_cache: Optional[dict] = None
_cache_mtime: Optional[float] = None


def _path() -> str:
    return os.path.join(settings.RESULTS_ROOT, "custom_models.json")


def _invalidate_cache() -> None:
    """Drop the in-memory cache (used by tests / after a manual file change)."""
    global _cache, _cache_mtime
    _cache, _cache_mtime = None, None


def _read_disk() -> dict:
    """Read the registry file fresh. Returns {} when absent; raises (e.g.
    json.JSONDecodeError) when present but unreadable — callers on the WRITE path
    must let that propagate rather than clobber the file."""
    try:
        with open(_path()) as fh:
            return json.load(fh)
    except FileNotFoundError:
        return {}


def _load() -> dict:
    """Cached read keyed on the file's mtime. Degrades to an empty registry (or
    the last good cache) on a corrupt/unreadable file so read paths never crash —
    presets keep working — but a good cache is never overwritten by a bad read."""
    global _cache, _cache_mtime
    try:
        mtime = os.path.getmtime(_path())
    except OSError:
        # No file (or unstattable): no custom models.
        _cache, _cache_mtime = {}, None
        return _cache
    if _cache is None or mtime != _cache_mtime:
        try:
            with open(_path()) as fh:
                data = json.load(fh)
        except Exception:
            # Corrupt/partial read: don't poison the cache; serve last good (or {}).
            return _cache if _cache is not None else {}
        _cache, _cache_mtime = data, mtime
    return _cache


def custom_models() -> dict:
    """Return a shallow copy of the {model_key: model_dict} custom registry."""
    return dict(_load())


def get_model(key: str) -> Optional[dict]:
    """Return the model dict for *key* (preset first, then custom), or None."""
    if key in AVAILABLE_MODELS:
        return AVAILABLE_MODELS[key]
    return _load().get(key)


def known(key: str) -> bool:
    return key in AVAILABLE_MODELS or key in _load()


def register(key: str, model_dict: dict) -> None:
    """Persist a custom model dict (no-op for preset keys — presets always win).

    Reads the current file fresh and merges into it, so a stale/empty in-memory
    cache can never cause previously saved models to be dropped. If the existing
    file is unreadable, the underlying error propagates and the file is left
    untouched (we refuse to overwrite it from an empty view)."""
    if key in AVAILABLE_MODELS:
        return
    global _cache, _cache_mtime
    with _lock:
        reg = _read_disk()  # fresh; raises on a corrupt existing file
        reg[key] = model_dict
        os.makedirs(settings.RESULTS_ROOT, exist_ok=True)
        tmp = _path() + ".tmp"
        with open(tmp, "w") as fh:
            json.dump(reg, fh)
        os.replace(tmp, _path())
        # Refresh the cache to the just-written state.
        _cache = reg
        try:
            _cache_mtime = os.path.getmtime(_path())
        except OSError:
            _cache_mtime = None
