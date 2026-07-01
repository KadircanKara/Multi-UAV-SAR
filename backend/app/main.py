"""
FastAPI application entry-point.

Import rootpath FIRST so that root modules (PathInfo, PathOptimizationModel, …)
are importable everywhere before the routers are loaded.
"""
import app.rootpath  # side-effect: inserts repo root into sys.path

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import ORJSONResponse
from slowapi import _rate_limit_exceeded_handler
from slowapi.errors import RateLimitExceeded

from app import settings
from app.ratelimit import limiter
from app.routers import (
    fronts, library, models, scenarios, replay, playback, comparison, optimize,
)

app = FastAPI(
    title="Multi-UAV-SAR API",
    default_response_class=ORJSONResponse,
)
# Per-IP rate limiting. Routes opt in via @limiter.limit(...); the expensive
# optimizer-start endpoint is throttled in routers/optimize.py.
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.CORS_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(models.router)
app.include_router(scenarios.router)
app.include_router(library.router)
app.include_router(fronts.router)
app.include_router(replay.router)
app.include_router(playback.router)
app.include_router(comparison.router)
app.include_router(optimize.router)


@app.on_event("shutdown")
def _shutdown_optimizer() -> None:
    """Release the optimizer worker pool so the process can exit cleanly."""
    from app import optimizer_service
    optimizer_service.shutdown(wait=False)


@app.get("/api/health")
def health() -> dict:
    return {"status": "ok"}
