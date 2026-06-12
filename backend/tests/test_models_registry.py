"""
Unit tests for the custom-model registry's durability.

The registry persists optimizer-saved models in ``custom_models.json``. A read
fault (corrupt/partial file) must never cause a subsequent ``register`` to
rewrite the file from an empty view and silently erase every previously saved
model.
"""
import json

import pytest

from app import models_registry, settings

_MODEL = {"Type": "MOO", "Alg": "NSGA2", "F": ["Mission Time"]}


@pytest.fixture
def isolated_registry(tmp_path, monkeypatch):
    monkeypatch.setattr(settings, "RESULTS_ROOT", str(tmp_path))
    models_registry._invalidate_cache()
    yield tmp_path / "custom_models.json"
    models_registry._invalidate_cache()


def test_register_does_not_erase_after_transient_read_error(isolated_registry):
    path = isolated_registry
    # 1) A corrupt file is read once (in the old code this poisons the cache with {}).
    path.write_text("{ not valid json")
    models_registry.known("probe")  # triggers a read
    # 2) The file is now valid and already holds a previously-saved model.
    path.write_text(json.dumps({"AA_MOO_NSGA2": _MODEL}))
    # 3) Saving a new model must keep the existing one.
    models_registry.register("BB_MOO_NSGA2", _MODEL)
    on_disk = json.loads(path.read_text())
    assert "AA_MOO_NSGA2" in on_disk and "BB_MOO_NSGA2" in on_disk


def test_register_refuses_to_clobber_corrupt_file(isolated_registry):
    path = isolated_registry
    path.write_text("CORRUPT NOT JSON")
    with pytest.raises(Exception):
        models_registry.register("BB_MOO_NSGA2", _MODEL)
    # The unreadable file is left intact, not replaced with a one-entry registry.
    assert path.read_text() == "CORRUPT NOT JSON"


def test_register_then_get_model_roundtrips(isolated_registry):
    models_registry.register("CC_MOO_NSGA2", _MODEL)
    assert models_registry.known("CC_MOO_NSGA2") is True
    assert models_registry.get_model("CC_MOO_NSGA2") == _MODEL


def test_missing_file_is_empty_not_error(isolated_registry):
    # No file on disk yet — read paths must degrade to "no custom models".
    assert models_registry.known("anything") is False
    assert models_registry.get_model("anything") is None
    assert models_registry.custom_models() == {}
