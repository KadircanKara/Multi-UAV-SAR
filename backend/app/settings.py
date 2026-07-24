"""
Central settings/constants for the backend.
All paths are absolute so they never depend on CWD.
"""
import os
import sys

from app.rootpath import REPO_ROOT  # importing the module triggers its sys.path side-effect

# Where the seeded mission data (Objectives/, Solutions/, Metadata/) lives.
# Defaults to the in-repo Results/ tree; point SAR_RESULTS_ROOT elsewhere when
# the data is provisioned outside the checkout (e.g. a Docker volume mount).
RESULTS_ROOT: str = os.path.abspath(
    os.environ.get("SAR_RESULTS_ROOT") or os.path.join(REPO_ROOT, "Results")
)

# Root log level (SAR_LOG_LEVEL). Names or numbers; INFO in production.
LOG_LEVEL: str = os.environ.get("SAR_LOG_LEVEL", "INFO").upper()


def _int_env(name: str, default: int) -> int:
    """Read an int from the environment, falling back to a default.

    A malformed value (e.g. ``SAR_OPTIMIZE_WORKERS=2x``) must NOT crash the
    process at import — a single typo would otherwise turn every container start
    into a restart loop. Fall back to the default and warn instead. Logging is
    not configured this early (main.py sets it up after settings import), so the
    warning goes to stderr."""
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return default
    try:
        return int(raw)
    except ValueError:
        print(
            f"WARNING: environment variable {name}={raw!r} is not a valid "
            f"integer; falling back to default {default}.",
            file=sys.stderr,
        )
        return default


def _float_env(name: str, default: float) -> float:
    """Read a float from the environment, falling back to a default.

    Same import-time robustness as ``_int_env``: a malformed value must not turn
    a container start into a restart loop, so fall back and warn to stderr."""
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return default
    try:
        return float(raw)
    except ValueError:
        print(
            f"WARNING: environment variable {name}={raw!r} is not a valid "
            f"number; falling back to default {default}.",
            file=sys.stderr,
        )
        return default


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
# pop_size x n_gen set how long one run holds a worker, and so how long everyone
# queued behind it waits — but they are sized for CONVERGENCE, not for latency.
# With the constraints active, a run much below these rarely converges at all
# (the path-adjacency speed constraint alone needs ~pop 100 to find any feasible
# solution), so cutting them to bound the worst case just yields fast garbage.
# Lower them only for a demo where an unconverged front is acceptable.
MAX_POP_SIZE: int = _int_env("SAR_MAX_POP_SIZE", 300)
MAX_N_GEN: int = _int_env("SAR_MAX_N_GEN", 1000)
# The largest multiplier a caller can reach: path length — and so the cost of
# every objective evaluation — scales with n_visits. The UI only ever offers
# 1-3; left uncapped, n_visits=100 made a single generation take 10 minutes and
# a full run over a week. Nothing else here bounds it.
MAX_N_VISITS: int = _int_env("SAR_MAX_N_VISITS", 3)

# cell_side_length and max_drone_speed together set the realtime sub-sample count
# (dt = ceil(leg_distance / speed), and leg_distance scales with cell_side_length):
# a huge cell or a near-zero speed inflates the interpolated trajectory toward an
# OOM. Time.get_real_paths' cumulative-columns bound is the real backstop, but
# these deploy caps let a public box reject the pathological combo fast at the API
# with a clear message instead of spending a worker on a run that will fail. The
# defaults equal the ScenarioConfig schema ceilings (le=1000 / ge=0.1) so nothing
# that runs today is affected; tighten them for a public deploy, e.g.
# SAR_MAX_CELL_SIDE_LENGTH=100 SAR_MIN_DRONE_SPEED=1.0 (real scenarios use 50 / 2.5).
MAX_CELL_SIDE_LENGTH: float = _float_env("SAR_MAX_CELL_SIDE_LENGTH", 1000)
MIN_DRONE_SPEED: float = _float_env("SAR_MIN_DRONE_SPEED", 0.1)

# ── Memory: cached scenarios ────────────────────────────────────────────────
# How many scenarios selector_service keeps unpickled in memory at once. This is
# the app's largest memory consumer by a wide margin: one large seeded scenario
# occupies ~160 MB once loaded (a 60 MB pickle expands ~2.7x), so the default 8
# can pin over a gigabyte. Combined with the optimizer pool, that is what sets
# the instance size — lower it on a small box, raise it if you have RAM spare
# and want fewer cold loads (a cache miss costs ~3-5 s).
SELECTOR_CACHE_SIZE: int = max(1, _int_env("SAR_SELECTOR_CACHE_SIZE", 8))


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

# Per-IP throttle on the two comparison endpoints (POST /api/comparison,
# /api/comparison/time). These fan ONE request out to up to 360 / 144 heavy
# per-scenario operations (selector unpickles / full sensing replays), so a
# request here is far more expensive than a single replay and gets its own,
# tighter budget. Override via SAR_COMPARISON_RATE_LIMIT.
COMPARISON_RATE_LIMIT: str = os.environ.get("SAR_COMPARISON_RATE_LIMIT", "10/minute")

# How many comparison requests may execute concurrently, server-wide (across all
# clients). The fan-out runs synchronously in the request threadpool; without a
# ceiling, a few parallel comparisons saturate every core and churn the selector
# cache toward the container memory limit. Excess requests get a fast 503 rather
# than piling more CPU-bound work behind the ones already running. Override via
# SAR_COMPARISON_CONCURRENCY (a 2-core box wants 1-2).
COMPARISON_CONCURRENCY: int = max(1, _int_env("SAR_COMPARISON_CONCURRENCY", 2))

# How many DISTINCT scenarios one comparison request may actually process. The
# schema caps (360 / 144) bound the request BODY, but the frontend can legitimately
# send every real scenario name (216 today), and each distinct one is a ~160 MB
# selector unpickle / full replay — so one request could force ~1.8 GB of serial
# work even while the concurrency slot bounds how many requests run at once. This
# bounds the per-request work: the comparison services process at most this many
# distinct scenarios and report the remainder in the response's ``skipped`` list
# (partial + honest, never silently truncated). Override via
# SAR_COMPARISON_MAX_SCENARIOS.
COMPARISON_MAX_SCENARIOS: int = max(1, _int_env("SAR_COMPARISON_MAX_SCENARIOS", 40))

# Per-request WALL-CLOCK budget (seconds) for the comparison fan-out. The count
# cap above bounds how MANY scenarios are attempted, but a warm selector cache
# and a cold one differ by ~50x per scenario, so a count that is quick when
# cached can still pin a core for minutes on a cold start. The services track
# elapsed time across the per-scenario loop and stop once this budget is spent,
# reporting the unprocessed remainder in ``skipped`` — so one request's CPU is
# bounded regardless of cache warmth. At least the first scenario always runs.
# Override via SAR_COMPARISON_TIME_BUDGET_SECONDS.
COMPARISON_TIME_BUDGET_SECONDS: int = max(1, _int_env("SAR_COMPARISON_TIME_BUDGET_SECONDS", 15))

# How many heavy single-mission reads may execute concurrently, server-wide
# (across all clients). Covers the endpoints that unpickle a selector to answer
# one request — the front / capabilities / select trio, plus a single sensing
# replay or playback. One large seeded scenario costs ~160 MB once loaded, so
# without a ceiling a burst of concurrent reads drives the selector cache past
# the container memory limit and OOMs it. The per-IP rate limit meters how OFTEN
# a client asks, not how many run at once (slowapi is a fixed-window counter, so
# a whole window's worth can arrive simultaneously). Excess requests get a fast
# 503 rather than all loading at once. Override via SAR_HEAVY_CONCURRENCY (a
# 2-core / small-RAM box wants 2-3).
HEAVY_CONCURRENCY: int = max(1, _int_env("SAR_HEAVY_CONCURRENCY", 3))

# ── Deploy-safety: request body size cap ────────────────────────────────────
# Upper bound on request body size (bytes), enforced on the received byte count
# by a middleware in main.py. Guards against a schema-valid worst-case Playground
# upload ballooning into memory, AND bounds how much JSON the event loop parses
# synchronously before the rate limiter / heavy_slot even run. A legitimate
# 2000-solution export is ~2 MB (measured: 16 drones, n_visits 3), so 10 MB is
# ~5x headroom while cutting the synchronous parse budget 2.5x from the old 25 MB.
# If you raise this, raise request_body max_size in the Caddyfile to match (Caddy
# rejects first otherwise). Override via SAR_MAX_UPLOAD_BYTES.
MAX_UPLOAD_BYTES: int = _int_env("SAR_MAX_UPLOAD_BYTES", 10 * 1024 * 1024)

# ── Optimizer queue ─────────────────────────────────────────────────────────
# Each in-flight run is one child process that saturates exactly ONE core, so
# OPTIMIZE_WORKERS is really "how many cores are you willing to give away".
# Keep it at (cores - 1) or below, leaving a core for the API itself — the
# replay/comparison endpoints run their sensing sims in the server process and
# go unresponsive if the pool takes every core. Default 2 suits a 2-4 core box.
OPTIMIZE_WORKERS: int = max(1, _int_env("SAR_OPTIMIZE_WORKERS", 2))

# Total runs the server will hold at once (running + waiting). Past this the
# API returns 409 rather than growing an unbounded backlog: a queue longer than
# people will actually wait through is worse than a clear refusal.
OPTIMIZE_QUEUE_MAX: int = max(1, _int_env("SAR_OPTIMIZE_QUEUE_MAX", 8))

# How many of those slots one client (IP) may hold. Without this a single
# caller fills the queue and everyone else sees a permanently full server.
OPTIMIZE_MAX_PER_CLIENT: int = max(1, _int_env("SAR_OPTIMIZE_MAX_PER_CLIENT", 2))


# ── Memoryless optimizer ────────────────────────────────────────────────────
# When False (default), the deployed optimizer never persists a run to the
# library — users download the run as JSON and re-upload to the Playground.
ALLOW_LIBRARY_SAVE: bool = _int_env("SAR_ALLOW_LIBRARY_SAVE", 0) == 1
# Temp per-run dirs under RESULTS_ROOT/.runs are swept once they exceed this age.
RUN_TTL_HOURS: int = _int_env("SAR_RUN_TTL_HOURS", 24)

# Ceiling on how many finished run dirs are kept, independent of age. Age alone
# does not bound disk: a burst of runs inside the TTL window, or a run that fails
# and is never followed by another (so the age sweep never fires), both leak.
# The newest OPTIMIZE_RUN_KEEP are retained (so /export still works for recent
# runs); older ones are removed even before they reach RUN_TTL_HOURS.
OPTIMIZE_RUN_KEEP: int = max(1, _int_env("SAR_OPTIMIZE_RUN_KEEP", 40))

# How often the background janitor sweeps .runs (seconds). Runs regardless of
# whether new optimizations are being started, so disk is bounded even when the
# app goes quiet. 0 disables the janitor (the start-of-run sweep still happens).
RUN_PURGE_INTERVAL_SECONDS: int = max(0, _int_env("SAR_RUN_PURGE_INTERVAL_SECONDS", 900))
