"""
POST /api/replay/{scenario}   — single sensing replay.
POST /api/compare/{scenario}  — multi-config comparison table.
"""
import app.rootpath  # must come before any root-module import

from fastapi import APIRouter, HTTPException

from app.schemas import ReplayRequest, CompareRequest
from app.selector_service import _SelectorNotFound, StrategyUnavailableError
from app.replay_service import run_replay, run_compare

router = APIRouter()


def _not_found(scenario: str, detail: str = None) -> HTTPException:
    msg = detail or f"Scenario {scenario!r} not found or pickles missing"
    return HTTPException(status_code=404, detail=msg)


def _unprocessable(detail: str) -> HTTPException:
    return HTTPException(status_code=422, detail=detail)


@router.post("/api/replay/{scenario}")
def post_replay(scenario: str, body: ReplayRequest) -> dict:
    """
    Run a single sensing replay for one solution of *scenario*.

    404 — scenario not found / model unknown / pickles missing.
    422 — index out of range, bad sensing config (p<=q, grid bounds, etc.).
    """
    try:
        return run_replay(
            scenario=scenario,
            model_key=body.model_key or None,
            index=body.index,
            cfg_dict=body.config.to_cfg_dict(),
            label=body.label,
        )
    except _SelectorNotFound as exc:
        raise _not_found(scenario, str(exc)) from exc
    except StrategyUnavailableError as exc:
        raise _unprocessable(str(exc)) from exc
    except ValueError as exc:
        raise _unprocessable(str(exc)) from exc


@router.post("/api/compare/{scenario}")
def post_compare(scenario: str, body: CompareRequest) -> dict:
    """
    Run a sensing replay per config and return a comparison table.

    404 — scenario not found / model unknown / pickles missing.
    422 — empty configs, index out of range, bad sensing config.
    """
    try:
        return run_compare(
            scenario=scenario,
            model_key=body.model_key or None,
            index=body.index,
            cfg_dicts=[c.to_cfg_dict() for c in body.configs],
            labels=body.labels,
        )
    except _SelectorNotFound as exc:
        raise _not_found(scenario, str(exc)) from exc
    except StrategyUnavailableError as exc:
        raise _unprocessable(str(exc)) from exc
    except ValueError as exc:
        raise _unprocessable(str(exc)) from exc
