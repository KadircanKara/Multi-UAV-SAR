"""
FastAPI application entry-point.

Import rootpath FIRST so that root modules (PathInfo, PathOptimizationModel, …)
are importable everywhere before the routers are loaded.
"""
import app.rootpath  # side-effect: inserts repo root into sys.path

import logging
import math

from fastapi import FastAPI, Request
from fastapi.encoders import jsonable_encoder
from fastapi.exceptions import RequestValidationError
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, ORJSONResponse
from slowapi import _rate_limit_exceeded_handler
from slowapi.errors import RateLimitExceeded

from app import settings
from app.concurrency import BusyError
from app.limits import BodySizeLimitMiddleware
from app.ratelimit import limiter

# Configure logging once, at import. Without this the app ran silent: a failed
# run or an unhandled 500 left no server-side trace at all.
logging.basicConfig(
    level=settings.LOG_LEVEL,
    format="%(asctime)s %(levelname)s %(name)s %(message)s",
)
logger = logging.getLogger("sar")
from app.routers import (
    fronts, library, models, scenarios, replay, playback, comparison, optimize,
    playground,
)

app = FastAPI(
    title="Multi-UAV-SAR API",
    default_response_class=ORJSONResponse,
)
# Per-IP rate limiting. Routes opt in via @limiter.limit(...): the optimizer
# start (routers/optimize.py), the sensing-replay family (routers/replay.py,
# playback.py, comparison.py), and the playground (routers/playground.py).
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)


# Every heavy endpoint takes a concurrency slot (app/concurrency.py) that refuses
# rather than queues when full. Translating that to 503 here, once, means a route
# only has to acquire the slot — it cannot forget the `except BusyError` arm and
# turn a busy signal into an opaque 500. The messages are author-written and
# carry no internals, so they are safe to pass through verbatim.
@app.exception_handler(BusyError)
async def _busy_handler(request: Request, exc: BusyError):
    return JSONResponse(status_code=503, content={"detail": str(exc)})


def _json_safe(obj):
    """Replace non-finite floats (NaN/Inf) with their string form, recursively."""
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else repr(obj)
    if isinstance(obj, dict):
        return {k: _json_safe(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_json_safe(v) for v in obj]
    return obj


# Human labels for the request fields users can actually set, so a validation
# error reads "at most 8 configurations" rather than echoing the raw field path.
_FIELD_LABELS: dict[str, str] = {
    "configs": "configurations",
    "target_locations": "target cells",
    "target_positions": "target cells",
    "scenarios": "scenarios",
    "number_of_drones": "number of drones",
    "grid_size": "grid size",
    "n_visits": "number of visits",
    "pop_size": "population size",
    "n_gen": "number of generations",
    "detection_prob": "detection probability",
    "false_alarm_prob": "false-alarm probability",
    "belief_threshold": "belief threshold",
    "max_mission_time": "max mission time",
    "min_connectivity": "min connectivity",
    "max_mean_tbv": "max mean TBV",
    "index": "solution number",
    "stride": "step size",
}


def _field_label(loc: tuple) -> str:
    """Human label for the deepest named field in a Pydantic error location."""
    for part in reversed(loc):
        if isinstance(part, str) and part not in ("body", "query", "path"):
            return _FIELD_LABELS.get(part, part.replace("_", " "))
    return ""


def _friendly_validation_message(errors: list[dict]) -> str:
    """Turn Pydantic's structured errors into one plain-language sentence."""
    if not errors:
        return "Some of the values you entered aren't valid."
    err = errors[0]
    field = _field_label(tuple(err.get("loc", ())))
    etype = str(err.get("type", ""))
    ctx = err.get("ctx") or {}

    if etype == "too_long":
        return f"Too many {field or 'items'} — at most {ctx.get('max_length')} allowed."
    if etype in ("too_short", "missing"):
        return f"{(field or 'A required value').capitalize()} is required."
    if etype in ("less_than", "less_than_equal", "greater_than", "greater_than_equal"):
        op = {
            "less_than": "less than",
            "less_than_equal": "at most",
            "greater_than": "greater than",
            "greater_than_equal": "at least",
        }[etype]
        limit = next(
            (ctx[k] for k in ("le", "lt", "ge", "gt", "limit_value", "limit") if k in ctx),
            "",
        )
        return f"{(field or 'Value').capitalize()} must be {op} {limit}.".replace("  ", " ")
    if etype == "value_error":
        # Custom validator messages are already written for humans (see schemas).
        return err.get("msg", "").replace("Value error, ", "") or "That value isn't valid."
    # Anything else: label the field and pass the (lightly cleaned) message through.
    msg = err.get("msg", "That value isn't valid.")
    return f"{field.capitalize()}: {msg}" if field else msg


@app.exception_handler(RequestValidationError)
async def _validation_exception_handler(request: Request, exc: RequestValidationError):
    """Return a clean, human-readable 422.

    Two jobs: (1) sanitize non-finite floats — FastAPI's default handler echoes
    the offending input, and a NaN/Inf there makes jsonable_encoder raise "Out
    of range float values are not JSON compliant", turning a legitimate 422 into
    a 500; (2) collapse Pydantic's error array into a friendly ``detail`` string
    so the frontend can drop it straight into a toast. The raw structured errors
    stay available under ``errors`` for debugging.
    """
    errors = _json_safe(exc.errors())
    return JSONResponse(
        status_code=422,
        content={
            "detail": _friendly_validation_message(errors),
            "errors": jsonable_encoder(errors),
        },
    )

# Bound the request body regardless of framing. Unlike a Content-Length check,
# this counts the bytes actually received, so a chunked body cannot slip past.
# Added BEFORE CORS (Starlette applies middleware last-added-first, so CORS ends
# up OUTSIDE this) on purpose: a 413 emitted outside CORSMiddleware carries no
# Access-Control-Allow-Origin, and a cross-origin browser then reports an opaque
# network failure instead of "that request is too large". CORS never reads the
# body, so nothing is buffered ahead of this cap.
app.add_middleware(BodySizeLimitMiddleware)

# The API is anonymous (no cookies/auth), so credentialed CORS is disabled;
# never re-enable it together with a wildcard or reflected origin.
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.CORS_ORIGINS,
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)


# Log any exception a route lets escape, with method + path, before it becomes an
# opaque 500. Re-raised unchanged, so the response is exactly as before — this is
# purely so a server-side 500 leaves a trace to debug from.
@app.middleware("http")
async def _log_unhandled(request: Request, call_next):
    try:
        return await call_next(request)
    except Exception:
        logger.exception("unhandled error on %s %s", request.method, request.url.path)
        raise


app.include_router(models.router)
app.include_router(scenarios.router)
app.include_router(library.router)
app.include_router(fronts.router)
app.include_router(replay.router)
app.include_router(playback.router)
app.include_router(comparison.router)
app.include_router(optimize.router)
app.include_router(playground.router)


@app.on_event("startup")
def _start_run_janitor() -> None:
    """Start the background sweeper that bounds the temp .runs/ tree on a timer,
    so disk stays bounded even when no new optimizations are being started."""
    from app import optimizer_service
    optimizer_service.start_janitor()


@app.on_event("shutdown")
def _shutdown_optimizer() -> None:
    """Stop the janitor and release the optimizer worker pool so the process can
    exit cleanly (shutdown() also does a final sweep)."""
    from app import optimizer_service
    optimizer_service.shutdown(wait=False)


@app.get("/api/health")
def health() -> dict:
    return {"status": "ok"}
