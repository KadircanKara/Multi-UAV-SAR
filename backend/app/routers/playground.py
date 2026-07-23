"""Stateless Playground endpoints: reconstruct from an uploaded JSON result
(no disk, no pickle) and reuse the seeded analysis cores.
"""
import app.rootpath  # noqa: F401

from typing import Optional

from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel, Field

from app import settings
from app.ratelimit import limiter
from app.playground_schema import PlaygroundResult
from app.playground_reconstruct import (
    reconstruct_selector, reconstruct_info, reconstruct_solution,
)
from app.schemas import SensingConfigModel
from app import comparison_service, selector_service, replay_service, playback_service
from app.selector_service import StrategyUnavailableError

router = APIRouter()


def _unprocessable(detail: str) -> HTTPException:
    return HTTPException(status_code=422, detail=detail)


class _FrontReq(BaseModel):
    result: PlaygroundResult

class _SelectReq(BaseModel):
    result: PlaygroundResult
    strategy: str
    objective_name: Optional[str] = None
    weights: Optional[dict[str, float]] = None
    index: Optional[int] = None

class _ReplayReq(BaseModel):
    result: PlaygroundResult
    index: int
    config: SensingConfigModel
    label: Optional[str] = None

class _CompareReq(BaseModel):
    result: PlaygroundResult
    index: int
    # Same cost-multiplier cap as CompareRequest.configs (one replay per entry).
    configs: list[SensingConfigModel] = Field(..., min_length=1, max_length=8)
    labels: Optional[list[str]] = None

class _PlaybackReq(BaseModel):
    result: PlaygroundResult
    index: int
    config: SensingConfigModel
    stride: int = Field(default=1, ge=1, description="Step-axis downsampling factor (>=1)")

class _CompareObjReq(BaseModel):
    results: list[PlaygroundResult]


def _heavy_solution(result: PlaygroundResult, index: int):
    if index < 0 or index >= len(result.solutions):
        raise HTTPException(
            status_code=422,
            detail=(
                f"Solution #{index} doesn't exist — this result has "
                f"{len(result.solutions)} solution(s) (0 to {len(result.solutions) - 1})."
            ),
        )
    info = reconstruct_info(result.scenario.to_scenario_dict(), result.model)
    return reconstruct_solution(result.solutions[index], info, full=True)


@router.post("/api/playground/front")
@limiter.limit(lambda: settings.OPTIMIZE_RATE_LIMIT)
def playground_front(request: Request, body: _FrontReq) -> dict:
    """Rebuild a PlaygroundResult upload into a SolutionSelector and return
    the same Pareto-front payload shape as GET /api/fronts/{scenario}."""
    return selector_service.build_front_from_selector(reconstruct_selector(body.result))


@router.post("/api/playground/select")
@limiter.limit(lambda: settings.OPTIMIZE_RATE_LIMIT)
def playground_select(request: Request, body: _SelectReq) -> dict:
    """Dispatch a selection strategy (best/balanced/knee/by_weights/by_index/
    the_solution) against a reconstructed SolutionSelector.

    422 — unknown strategy, wrong result_kind, missing required argument,
    or index out of range (by_index).
    """
    sel = reconstruct_selector(body.result)
    try:
        return selector_service.select_from_selector(
            sel, body.strategy, body.objective_name, body.weights, body.index)
    except StrategyUnavailableError as exc:
        raise _unprocessable(str(exc)) from exc


@router.post("/api/playground/replay")
@limiter.limit(lambda: settings.OPTIMIZE_RATE_LIMIT)
def playground_replay(request: Request, body: _ReplayReq) -> dict:
    """Run a single sensing replay for one solution of an uploaded result.

    422 — index out of range, bad sensing config (p<=q, grid bounds, etc.).
    """
    sol = _heavy_solution(body.result, body.index)
    try:
        return replay_service.run_replay_for_solution(sol, body.config.to_cfg_dict(), body.label)
    except ValueError as exc:
        raise _unprocessable(str(exc)) from exc


@router.post("/api/playground/compare")
@limiter.limit(lambda: settings.OPTIMIZE_RATE_LIMIT)
def playground_compare(request: Request, body: _CompareReq) -> dict:
    """Run a replay per config for one solution and return a comparison table.

    422 — index out of range, empty configs, bad sensing config.
    """
    sol = _heavy_solution(body.result, body.index)
    try:
        return replay_service.run_compare_for_solution(
            sol, [c.to_cfg_dict() for c in body.configs], body.labels)
    except ValueError as exc:
        raise _unprocessable(str(exc)) from exc


@router.post("/api/playground/playback")
@limiter.limit(lambda: settings.OPTIMIZE_RATE_LIMIT)
def playground_playback(request: Request, body: _PlaybackReq) -> dict:
    """Build the animation playback payload for one solution of an uploaded
    result.

    Playground uploads have no canonical scenario name, so ``scenario`` and
    ``model_key``/``index`` (which build_playback_for_solution does not
    produce — see playback_service module docstring) are injected here to
    match the seeded /api/playback payload shape.

    422 — index out of range, bad sensing config, degenerate replay (zero-
    length step axis after alignment).
    """
    sol = _heavy_solution(body.result, body.index)
    try:
        out = playback_service.build_playback_for_solution(
            sol, body.config.to_cfg_dict(), body.stride)
    except ValueError as exc:
        raise _unprocessable(str(exc)) from exc
    out["scenario"] = ""
    out["model_key"] = body.result.model.get("model_key", "")
    out["index"] = body.index
    return out


@router.post("/api/playground/comparison")
@limiter.limit(lambda: settings.OPTIMIZE_RATE_LIMIT)
def playground_comparison(request: Request, body: _CompareObjReq) -> dict:
    """Cross-model objective comparison across multiple uploaded results.

    Same response shape as the seeded ``POST /api/comparison``, but reads
    each solution's STORED objectives straight from the upload rather than
    recomputing them (reconstructed playground solutions are "light" and have
    no cached objectives — recomputing would be slow and could diverge from
    the values the file was exported with).
    """
    return comparison_service.compare_objectives_from_results(body.results)
