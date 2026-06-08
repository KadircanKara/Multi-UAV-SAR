"""
GET  /api/scenarios/default   — returns default ScenarioConfig.
POST /api/scenarios/validate  — validates scenario + derives PathInfo quantities.
"""
import app.rootpath  # must come before any root-module import

from fastapi import APIRouter, HTTPException
from PathInfo import PathInfo, default_scenario
from PathOptimizationModel import AVAILABLE_MODELS

from app.schemas import (
    ScenarioConfig,
    ScenarioDerived,
    ScenarioValidateRequest,
    ScenarioValidateResponse,
)

router = APIRouter()


@router.get("/api/scenarios/default", response_model=ScenarioConfig)
def get_default_scenario() -> ScenarioConfig:
    """Return the default scenario configuration."""
    return ScenarioConfig(**default_scenario)


@router.post("/api/scenarios/validate", response_model=ScenarioValidateResponse)
def validate_scenario(req: ScenarioValidateRequest) -> ScenarioValidateResponse:
    """
    Validate a scenario config by constructing PathInfo and extracting
    derived quantities.  Optionally produces a scenario_str if model_key
    is supplied.
    """
    # Pydantic already validated ScenarioConfig; if PathInfo raises, surface it.
    try:
        info = PathInfo(req.scenario.to_scenario_dict())
    except Exception as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc

    derived = ScenarioDerived(
        number_of_cells=info.number_of_cells,
        number_of_nodes=info.number_of_nodes,
        comm_dist=info.comm_dist,
        miss_probability=info.miss_probability,
    )

    scenario_str: str | None = None
    if req.model_key is not None:
        if req.model_key not in AVAILABLE_MODELS:
            raise HTTPException(
                status_code=404,
                detail=f"model_key '{req.model_key}' not found in AVAILABLE_MODELS",
            )
        info.model = AVAILABLE_MODELS[req.model_key]
        scenario_str = str(info)

    return ScenarioValidateResponse(valid=True, derived=derived, scenario_str=scenario_str)
