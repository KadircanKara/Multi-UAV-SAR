"""The selector cache is the app's largest memory consumer, so its size has to
be tunable per deployment rather than baked in.

One cached scenario can hold ~160 MB unpickled (measured on the largest seeded
solution set), so the default of 8 can pin over a gigabyte — enough to OOM a
small box when optimizer children are also running.
"""
from app import settings
from app.selector_service import _load_selector


def test_cache_size_is_configurable():
    assert hasattr(settings, "SELECTOR_CACHE_SIZE")


def test_cache_honours_the_configured_size():
    assert _load_selector.cache_info().maxsize == settings.SELECTOR_CACHE_SIZE


def test_default_cache_size_is_unchanged_from_before_it_was_tunable():
    """Making it tunable must not silently change existing behaviour."""
    assert settings.SELECTOR_CACHE_SIZE == 8


def test_cache_size_is_at_least_one():
    """maxsize=0 disables caching entirely and would reload a ~160 MB pickle on
    every request; None would make it unbounded. Neither is a safe deployment."""
    assert settings.SELECTOR_CACHE_SIZE >= 1
