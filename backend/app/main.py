"""
FastAPI application entry-point.

Import rootpath FIRST so that root modules (PathInfo, PathOptimizationModel, …)
are importable everywhere before the routers are loaded.
"""
import app.rootpath  # side-effect: inserts repo root into sys.path

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import ORJSONResponse

from app import settings
from app.routers import library, models, scenarios

app = FastAPI(
    title="Multi-UAV-SAR API",
    default_response_class=ORJSONResponse,
)

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


@app.get("/api/health")
def health() -> dict:
    return {"status": "ok"}
