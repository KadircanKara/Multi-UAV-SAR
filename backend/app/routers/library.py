"""
GET  /api/library          — list all precomputed scenarios in Results/.
GET  /api/library/{scenario} — detail for one precomputed scenario.
"""
import app.rootpath  # must come before any root-module import

from fastapi import APIRouter, HTTPException, Request

from app import settings
from app.library_service import get_scenario, list_scenarios
from app.model_aliases import to_display
from app.optimizer_service import read_run_config
from app.ratelimit import limiter
from app.schemas import ScenarioDetail, ScenarioSummary

router = APIRouter()


@router.get("/api/library", response_model=list[ScenarioSummary])
@limiter.limit(lambda: settings.REPLAY_RATE_LIMIT)
def get_library(request: Request) -> list[dict]:
    """Return all precomputed scenarios found in Results/."""
    return list_scenarios()


@router.get("/api/library/{scenario}", response_model=ScenarioDetail)
@limiter.limit(lambda: settings.REPLAY_RATE_LIMIT)
def get_library_scenario(request: Request, scenario: str) -> dict:
    """Return detail for a single precomputed scenario; 404 if not found."""
    detail = get_scenario(scenario)
    if detail is None:
        raise HTTPException(
            status_code=404,
            detail=f"Scenario {scenario!r} not found in Results/",
        )
    return detail


@router.get("/api/library/{scenario}/config")
@limiter.limit(lambda: settings.REPLAY_RATE_LIMIT)
def get_library_scenario_config(request: Request, scenario: str) -> dict:
    """Read-only optimizer run-config for a mission, or {recorded: false} if none.

    Returned verbatim (the sidecar's own shape) — the frontend types it as RunConfig;
    no strict response_model so the nested record isn't duplicated as a Pydantic tree."""
    cfg = read_run_config(scenario)
    if cfg is None:
        return {"recorded": False}
    # Sidecars store the storage identifiers (…TCDT…); present the display form.
    cfg = dict(cfg)
    for key in ("scenario_name", "model_key"):
        if isinstance(cfg.get(key), str):
            cfg[key] = to_display(cfg[key])
    return cfg
