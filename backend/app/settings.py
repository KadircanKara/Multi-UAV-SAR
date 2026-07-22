"""
Central settings/constants for the backend.
All paths are absolute so they never depend on CWD.
"""
import os

from app.rootpath import REPO_ROOT  # importing the module triggers its sys.path side-effect

RESULTS_ROOT: str = os.path.join(REPO_ROOT, "Results")


def _int_env(name: str, default: int) -> int:
    """Read an int from the environment, falling back to a default."""
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return default
    return int(raw)


def _csv_env(name: str, default: list[str]) -> list[str]:
    """Read a comma-separated list from the environment, falling back to a default."""
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return default
    return [item.strip() for item in raw.split(",") if item.strip()]


# ── CORS origins ────────────────────────────────────────────────────────────
# Defaults cover the local Next.js dev server; a deployed frontend origin must
# be supplied via env, e.g. SAR_CORS_ORIGINS=https://sar.example.com.
CORS_ORIGINS: list[str] = _csv_env(
    "SAR_CORS_ORIGINS",
    [
        "http://localhost:3000",
        "http://127.0.0.1:3000",
    ],
)


# ── Deploy-safety: per-run parameter caps ───────────────────────────────────
# Ceilings on a single optimizer run. The run cost scales with all four, so
# these bound how expensive any one request can be. Defaults are the project's
# real maximums (nothing that runs today is affected); tighten them via env for
# a public deploy, e.g. SAR_MAX_DRONES=8 SAR_MAX_N_GEN=200 SAR_MAX_POP_SIZE=100.
MAX_DRONES: int = _int_env("SAR_MAX_DRONES", 16)
MAX_GRID_SIZE: int = _int_env("SAR_MAX_GRID_SIZE", 8)
MAX_POP_SIZE: int = _int_env("SAR_MAX_POP_SIZE", 500)
MAX_N_GEN: int = _int_env("SAR_MAX_N_GEN", 1000)

# ── Deploy-safety: rate limit ───────────────────────────────────────────────
# Per-IP throttle on POST /api/optimize (the compute trigger). slowapi syntax,
# e.g. "10/minute", "100/hour". Override for a public deploy via
# SAR_OPTIMIZE_RATE_LIMIT.
OPTIMIZE_RATE_LIMIT: str = os.environ.get("SAR_OPTIMIZE_RATE_LIMIT", "30/minute")

# Per-IP throttle on the sensing-replay family (POST /api/replay, /api/compare,
# /api/playback, /api/comparison, /api/comparison/time). These run full sensing
# simulations synchronously in the request handler, so they need their own
# budget: lighter than an optimizer run (hence a higher default than
# OPTIMIZE_RATE_LIMIT) but far too heavy to leave unthrottled. Override via
# SAR_REPLAY_RATE_LIMIT.
REPLAY_RATE_LIMIT: str = os.environ.get("SAR_REPLAY_RATE_LIMIT", "60/minute")

# ── Deploy-safety: request body size cap ────────────────────────────────────
# Upper bound on request body size (bytes), enforced via the Content-Length
# header by a middleware in main.py. Guards against a schema-valid worst-case
# Playground upload (2000 solutions x 100k-int paths) ballooning into
# gigabytes of memory. Override via SAR_MAX_UPLOAD_BYTES.
MAX_UPLOAD_BYTES: int = _int_env("SAR_MAX_UPLOAD_BYTES", 25 * 1024 * 1024)

# ── Memoryless optimizer ────────────────────────────────────────────────────
# When False (default), the deployed optimizer never persists a run to the
# library — users download the run as JSON and re-upload to the Playground.
ALLOW_LIBRARY_SAVE: bool = _int_env("SAR_ALLOW_LIBRARY_SAVE", 0) == 1
# Temp per-run dirs under RESULTS_ROOT/.runs are swept once they exceed this age.
RUN_TTL_HOURS: int = _int_env("SAR_RUN_TTL_HOURS", 24)
