"""
Test configuration for backend tests.

sys.path bootstrapping: pyproject.toml already sets pythonpath = ["..", "."],
so both the repo root (flat modules) and backend/ (app.*) are importable.
We still do an explicit insert here as a belt-and-suspenders guard for
editors / runners that ignore pyproject.toml's pythonpath setting.
"""
import os
import sys

# repo root = two levels up from this file (backend/tests/conftest.py)
_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_BACKEND_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

for _p in (_REPO_ROOT, _BACKEND_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import pytest
from fastapi.testclient import TestClient

from app import settings  # noqa: E402  (imports after sys.path setup intentional)
from app.main import app  # noqa: E402
from app.ratelimit import limiter  # noqa: E402

# Canonical seeded scenario most data-dependent tests are written against.
# Tests marked `needs_seed_data` are skipped when it is absent (e.g. in CI,
# where the 1.7G Results/ tree is not checked out).
_TCD_SEED = "MOO_NSGA2_TCD_g_8_a_50_n_4_v_2.5_r_2_nvisits_1"


def _seed_data_present() -> bool:
    return os.path.isfile(os.path.join(
        settings.RESULTS_ROOT, "Solutions", f"{_TCD_SEED}-SolutionObjects.pkl"))


def pytest_collection_modifyitems(config, items):
    if _seed_data_present():
        return
    skip = pytest.mark.skip(reason="seeded Results/ data not present")
    for item in items:
        if item.get_closest_marker("needs_seed_data"):
            item.add_marker(skip)


@pytest.fixture(scope="session")
def client() -> TestClient:
    """FastAPI TestClient for the whole test session."""
    with TestClient(app) as c:
        yield c


@pytest.fixture(autouse=True)
def _reset_rate_limiter():
    """Empty the shared per-IP buckets before each test. All tests hit the
    limiter from the same client address, so without this, suites that hammer
    a throttled endpoint (replay/playback/compare/comparison) would trip 429s
    across test boundaries."""
    limiter.reset()
    yield
