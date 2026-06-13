"""
GET  /api/library          — list all precomputed scenarios in Results/.
GET  /api/library/{scenario} — detail for one precomputed scenario.
"""
import app.rootpath  # must come before any root-module import

from fastapi import APIRouter, HTTPException

from app.library_service import get_scenario, list_scenarios
from app.optimizer_service import read_run_config
from app.schemas import ScenarioDetail, ScenarioSummary

router = APIRouter()


@router.get("/api/library", response_model=list[ScenarioSummary])
def get_library() -> list[dict]:
    """Return all precomputed scenarios found in Results/."""
    return list_scenarios()


@router.get("/api/library/{scenario}", response_model=ScenarioDetail)
def get_library_scenario(scenario: str) -> dict:
    """Return detail for a single precomputed scenario; 404 if not found."""
    detail = get_scenario(scenario)
    if detail is None:
        raise HTTPException(
            status_code=404,
            detail=f"Scenario {scenario!r} not found in Results/",
        )
    return detail


@router.get("/api/library/{scenario}/config")
def get_library_scenario_config(scenario: str) -> dict:
    """Read-only optimizer run-config for a mission, or {recorded: false} if none.

    Returned verbatim (the sidecar's own shape) — the frontend types it as RunConfig;
    no strict response_model so the nested record isn't duplicated as a Pydantic tree."""
    cfg = read_run_config(scenario)
    return cfg if cfg is not None else {"recorded": False}
