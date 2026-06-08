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

from app.main import app  # noqa: E402  (imports after sys.path setup intentional)


@pytest.fixture(scope="session")
def client() -> TestClient:
    """FastAPI TestClient for the whole test session."""
    with TestClient(app) as c:
        yield c
