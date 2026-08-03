"""
GET  /api/fronts/{scenario}              — Pareto front + capabilities in one shot.
GET  /api/fronts/{scenario}/capabilities — capabilities dict only.
POST /api/fronts/{scenario}/select       — select a solution by strategy.
"""
import app.rootpath  # must come before any root-module import

from fastapi import APIRouter, HTTPException, Query, Request
from typing import Optional

from app import settings
from app.concurrency import heavy_slot
from app.ratelimit import limiter
from app.schemas import ParetoFront, SelectRequest, SelectResponse
from app.selector_service import (
    StrategyUnavailableError,
    _SelectorNotFound,
    build_front,
    get_capabilities,
    select,
)

router = APIRouter()


def _not_found(scenario: str, detail: str = None) -> HTTPException:
    msg = detail or f"We couldn't find saved data for scenario {scenario}."
    return HTTPException(status_code=404, detail=msg)


@router.get("/api/fronts/{scenario}", response_model=ParetoFront)
@limiter.limit(lambda: settings.REPLAY_RATE_LIMIT)
def get_front(
    request: Request,
    scenario: str,
    model_key: Optional[str] = Query(default=None),
) -> dict:
    """
    Return the Pareto front for *scenario*, including capabilities.

    404 if scenario name is unsafe, model_key is unknown, or pickles are missing.
    503 when the server already has its share of heavy reads running.
    """
    try:
        with heavy_slot():
            return build_front(scenario, model_key or None)
    except _SelectorNotFound as exc:
        raise _not_found(scenario) from exc


@router.get("/api/fronts/{scenario}/capabilities")
@limiter.limit(lambda: settings.REPLAY_RATE_LIMIT)
def get_front_capabilities(
    request: Request,
    scenario: str,
    model_key: Optional[str] = Query(default=None),
) -> dict:
    """
    Return only the capabilities dict for *scenario*.

    404 if scenario not found / pickles missing.
    503 when the server already has its share of heavy reads running.
    """
    try:
        with heavy_slot():
            return get_capabilities(scenario, model_key or None)
    except _SelectorNotFound as exc:
        raise _not_found(scenario) from exc


@router.post("/api/fronts/{scenario}/select", response_model=SelectResponse)
@limiter.limit(lambda: settings.REPLAY_RATE_LIMIT)
def select_solution(
    request: Request,
    scenario: str,
    body: SelectRequest,
) -> dict:
    """
    Select a solution from *scenario*'s front by strategy.

    404 — scenario not found / pickles missing / unknown model_key.
    422 — strategy unavailable for this result_kind, or required argument missing.
    503 — the server already has its share of heavy reads running.
    """
    try:
        with heavy_slot():
            return select(
                scenario=scenario,
                strategy=body.strategy,
                model_key=body.model_key or None,
                objective_name=body.objective_name,
                weights=body.weights,
                index=body.index,
            )
    except _SelectorNotFound as exc:
        raise _not_found(scenario) from exc
    except StrategyUnavailableError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc
