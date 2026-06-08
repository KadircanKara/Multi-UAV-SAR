"""
Central settings/constants for the backend.
All paths are absolute so they never depend on CWD.
"""
import os

from app.rootpath import REPO_ROOT  # importing the module triggers its sys.path side-effect

RESULTS_ROOT: str = os.path.join(REPO_ROOT, "Results")

# CORS origins for the future Next.js dev server
CORS_ORIGINS: list[str] = [
    "http://localhost:3000",
    "http://127.0.0.1:3000",
]

# Caps for live optimiser runs (not used in Task 0.2, but defined here)
MAX_POP_SIZE: int = 60
MAX_N_GEN: int = 50
