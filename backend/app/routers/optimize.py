"""
Optimizer endpoints — configure + run an optimization (background + poll).

  POST /api/optimize/check  — config → {scenario_name, model_key, exists, seeded}
  POST /api/optimize        — start a run → {run_id, scenario_name, model_key, exists, seeded}
  GET  /api/optimize/{id}   — poll: running (gen X/Y) | done (front) | failed
"""
import app.rootpath  # must come before any root-module import

from fastapi import APIRouter, HTTPException

from app.schemas import (
    OptimizeConfig,
    OptimizeStartResponse,
    OptimizeCheckResponse,
    OptimizeStatusResponse,
    OptimizeStopResponse,
    OptimizeSaveRequest,
    OptimizeSaveResponse,
)
from app.optimizer_service import (
    check_config,
    start_run,
    get_status,
    request_stop,
    save_run,
    RunInProgressError,
    RunNotFoundError,
    RunNotReadyError,
    AlreadyExistsError,
    EmptyRunError,
)

router = APIRouter()


@router.post("/api/optimize/check", response_model=OptimizeCheckResponse)
def post_optimize_check(body: OptimizeConfig) -> dict:
    """Existence pre-check: does a run for this configuration already exist?"""
    return check_config(
        body.optimization_type, body.method, body.objectives,
        body.weights, body.scenario.to_scenario_dict(),
        body.max_mission_time, body.min_connectivity, body.max_mean_tbv,
    )


@router.post("/api/optimize", response_model=OptimizeStartResponse)
def post_optimize(body: OptimizeConfig) -> dict:
    """Start a run in the worker process. 409 if a run is already in flight."""
    try:
        return start_run(
            body.optimization_type, body.method, body.objectives, body.weights,
            body.pop_size, body.n_gen, body.seed, body.scenario.to_scenario_dict(),
            body.max_mission_time, body.min_connectivity, body.max_mean_tbv,
            body.gen_strategy, body.early_stop_patience, body.early_stop_threshold,
        )
    except RunInProgressError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@router.get("/api/optimize/{run_id}", response_model=OptimizeStatusResponse)
def get_optimize(run_id: str) -> dict:
    """Poll a run by id."""
    try:
        return get_status(run_id)
    except RunNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc


@router.post("/api/optimize/{run_id}/stop", response_model=OptimizeStopResponse)
def post_optimize_stop(run_id: str) -> dict:
    """Cooperatively stop a running optimization. The run finishes within a
    generation or two and returns its best-so-far front. 404 if unknown; a no-op
    (stopping=False) if the run had already finished."""
    try:
        return request_stop(run_id)
    except RunNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc


@router.post("/api/optimize/{run_id}/save", response_model=OptimizeSaveResponse)
def post_optimize_save(run_id: str, body: OptimizeSaveRequest) -> dict:
    """Persist a finished run into the library. 409 if the scenario already
    exists and overwrite was not requested (frontend then confirms overwrite)."""
    try:
        return save_run(run_id, body.overwrite)
    except RunNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    except RunNotReadyError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    except EmptyRunError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc
    except AlreadyExistsError as exc:
        raise HTTPException(
            status_code=409,
            detail=f"A run named {exc} already exists. Confirm overwrite to replace it.",
        ) from exc
