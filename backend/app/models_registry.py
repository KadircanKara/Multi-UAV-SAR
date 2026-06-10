"""
Models registry: resolves a model_key to its model dict, consulting the built-in
``AVAILABLE_MODELS`` first and then a persisted registry of CUSTOM models created
by the Optimizer's "Save to library" (so saved ad-hoc runs become browsable via
the existing fronts / replay / compare / library endpoints).

The custom registry lives at ``Results/custom_models.json`` ({model_key: model_dict}).
Preset model_keys are never written there (AVAILABLE_MODELS always wins).
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


def _path() -> str:
    return os.path.join(settings.RESULTS_ROOT, "custom_models.json")


def _load() -> dict:
    global _cache
    if _cache is None:
        try:
            with open(_path()) as fh:
                _cache = json.load(fh)
        except Exception:
            _cache = {}
    return _cache


def get_model(key: str) -> Optional[dict]:
    """Return the model dict for *key* (preset first, then custom), or None."""
    if key in AVAILABLE_MODELS:
        return AVAILABLE_MODELS[key]
    return _load().get(key)


def known(key: str) -> bool:
    return key in AVAILABLE_MODELS or key in _load()


def register(key: str, model_dict: dict) -> None:
    """Persist a custom model dict (no-op for preset keys — presets always win)."""
    if key in AVAILABLE_MODELS:
        return
    with _lock:
        reg = _load()
        reg[key] = model_dict
        os.makedirs(settings.RESULTS_ROOT, exist_ok=True)
        tmp = _path() + ".tmp"
        with open(tmp, "w") as fh:
            json.dump(reg, fh)
        os.replace(tmp, _path())
