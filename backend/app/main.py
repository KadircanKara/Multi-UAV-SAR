"""
FastAPI application entry-point.

Import rootpath FIRST so that root modules (PathInfo, PathOptimizationModel, …)
are importable everywhere before the routers are loaded.
"""
import app.rootpath  # side-effect: inserts repo root into sys.path

from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, ORJSONResponse
from slowapi import _rate_limit_exceeded_handler
from slowapi.errors import RateLimitExceeded

from app import settings
from app.ratelimit import limiter
from app.routers import (
    fronts, library, models, scenarios, replay, playback, comparison, optimize,
    playground,
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


@app.middleware("http")
async def limit_upload_size(request: Request, call_next):
    """Reject oversized request bodies before the route runs, based on the
    Content-Length header. Requests without a Content-Length header (e.g.
    chunked transfer-encoding) pass through unchecked here; body size for
    those is bounded only by whatever the route itself enforces.
    """
    content_length = request.headers.get("content-length")
    if content_length is not None:
        try:
            length = int(content_length)
        except ValueError:
            length = None
        if length is not None and length > settings.MAX_UPLOAD_BYTES:
            return JSONResponse(
                status_code=413, content={"detail": "request body too large"}
            )
    return await call_next(request)

app.include_router(models.router)
app.include_router(scenarios.router)
app.include_router(library.router)
app.include_router(fronts.router)
app.include_router(replay.router)
app.include_router(playback.router)
app.include_router(comparison.router)
app.include_router(optimize.router)
app.include_router(playground.router)


@app.on_event("shutdown")
def _shutdown_optimizer() -> None:
    """Release the optimizer worker pool so the process can exit cleanly."""
    from app import optimizer_service
    optimizer_service.shutdown(wait=False)


@app.get("/api/health")
def health() -> dict:
    return {"status": "ok"}
