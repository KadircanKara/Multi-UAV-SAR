"""
GET  /api/scenarios/default   — returns default ScenarioConfig.
POST /api/scenarios/validate  — validates scenario + derives PathInfo quantities.
"""
import app.rootpath  # must come before any root-module import

from fastapi import APIRouter, HTTPException, Request
from PathInfo import PathInfo, default_scenario

from app import models_registry, settings
from app.model_aliases import to_display
from app.ratelimit import limiter
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
@limiter.limit(lambda: settings.REPLAY_RATE_LIMIT)
def validate_scenario(request: Request, req: ScenarioValidateRequest) -> ScenarioValidateResponse:
    """
    Validate a scenario config by constructing PathInfo and extracting
    derived quantities.  Optionally produces a scenario_str if model_key
    is supplied.
    """
    # Deploy caps, mirroring OptimizeConfig._enforce_deploy_caps: PathInfo
    # allocates grid_size²-shaped structures, so validation of an uncapped
    # scenario is itself a memory/CPU amplifier.
    if req.scenario.grid_size > settings.MAX_GRID_SIZE:
        raise HTTPException(
            status_code=422,
            detail=(
                f"grid_size {req.scenario.grid_size} exceeds the cap of "
                f"{settings.MAX_GRID_SIZE}"
            ),
        )
    if req.scenario.number_of_drones > settings.MAX_DRONES:
        raise HTTPException(
            status_code=422,
            detail=(
                f"number_of_drones {req.scenario.number_of_drones} exceeds the "
                f"cap of {settings.MAX_DRONES}"
            ),
        )
    if req.scenario.n_visits > settings.MAX_N_VISITS:
        raise HTTPException(
            status_code=422,
            detail=(
                f"n_visits {req.scenario.n_visits} exceeds the cap of "
                f"{settings.MAX_N_VISITS}"
            ),
        )
    # cell_side_length x (1 / max_drone_speed) drives the realtime sub-sample
    # count; keep this validation gate in step with OptimizeConfig so the UI never
    # validates a scenario that /api/optimize would then reject.
    if req.scenario.cell_side_length > settings.MAX_CELL_SIDE_LENGTH:
        raise HTTPException(
            status_code=422,
            detail=(
                f"cell_side_length {req.scenario.cell_side_length} exceeds the "
                f"cap of {settings.MAX_CELL_SIDE_LENGTH}"
            ),
        )
    if req.scenario.max_drone_speed < settings.MIN_DRONE_SPEED:
        raise HTTPException(
            status_code=422,
            detail=(
                f"max_drone_speed {req.scenario.max_drone_speed} is below the "
                f"floor of {settings.MIN_DRONE_SPEED}"
            ),
        )

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
        model = models_registry.get_model(req.model_key)
        if model is None:
            raise HTTPException(
                status_code=404,
                detail=f"model_key '{req.model_key}' is not a known model",
            )
        info.model = model
        # str(info) embeds the storage Exp (TCDT); present the display form.
        scenario_str = to_display(str(info))

    return ScenarioValidateResponse(valid=True, derived=derived, scenario_str=scenario_str)
