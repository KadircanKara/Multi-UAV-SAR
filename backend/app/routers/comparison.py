"""POST /api/comparison — compare scenarios across ALL objectives."""
import app.rootpath  # must come before any root-module import

from fastapi import APIRouter, HTTPException, Request

from app import settings
from app.ratelimit import limiter
from app.schemas import (
    ComparisonRequest,
    ComparisonResponse,
    TimeComparisonRequest,
    TimeComparisonResponse,
)
from app.comparison_service import compare_objectives, compare_time_metrics
from app.selector_service import StrategyUnavailableError

router = APIRouter()


@router.post("/api/comparison", response_model=ComparisonResponse)
@limiter.limit(lambda: settings.REPLAY_RATE_LIMIT)
def post_comparison(request: Request, body: ComparisonRequest) -> dict:
    """
    Compare the given scenarios across every objective — including objectives a
    model did not optimise (computed from the solution objects via
    ``objective_values``). Each scenario row carries per-objective min/max/mean/
    best aggregated over its Pareto front.

    Unknown / unloadable scenarios are skipped and listed in ``skipped``.
    404 only if NONE of the requested scenarios could be loaded.
    """
    result = compare_objectives(body.scenarios)
    if not result["scenarios"]:
        raise HTTPException(
            status_code=404,
            detail=(
                "None of the requested scenarios could be loaded: "
                f"{body.scenarios}"
            ),
        )
    return result


@router.post("/api/comparison/time", response_model=TimeComparisonResponse)
@limiter.limit(lambda: settings.REPLAY_RATE_LIMIT)
def post_comparison_time(request: Request, body: TimeComparisonRequest) -> dict:
    """
    Compare sensing-replay time metrics across scenarios under one shared
    sensing config. For each scenario a single solution is chosen via the given
    selection ``strategy`` (default Balanced), replayed, and its four time
    metrics recorded.

    Unknown / unloadable scenarios are skipped and listed in ``skipped``.
    422 — bad strategy (e.g. ``best`` without ``objective_name``) or bad
    sensing config (p<=q, grid bounds, etc.).
    404 only if NONE of the requested scenarios could be loaded.
    """
    try:
        result = compare_time_metrics(
            body.scenarios,
            body.config.to_cfg_dict(),
            strategy=body.strategy,
            objective_name=body.objective_name,
            weights=body.weights,
        )
    except StrategyUnavailableError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc

    if not result["scenarios"]:
        raise HTTPException(
            status_code=404,
            detail=(
                "None of the requested scenarios could be loaded: "
                f"{body.scenarios}"
            ),
        )
    return result
