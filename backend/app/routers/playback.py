"""
POST /api/playback/{scenario} — animation playback payload.

Returns per-step trajectories, connectivity edges, belief heat, and
targets-known curve — all aligned to a single step axis.
"""
import app.rootpath  # must come before any root-module import

from fastapi import APIRouter, HTTPException, Request

from app import settings
from app.ratelimit import limiter
from app.schemas import PlaybackRequest
from app.selector_service import _SelectorNotFound, StrategyUnavailableError
from app.playback_service import build_playback

router = APIRouter()


def _not_found(scenario: str, detail: str = None) -> HTTPException:
    msg = detail or f"Scenario {scenario!r} not found or pickles missing"
    return HTTPException(status_code=404, detail=msg)


def _unprocessable(detail: str) -> HTTPException:
    return HTTPException(status_code=422, detail=detail)


@router.post("/api/playback/{scenario}")
@limiter.limit(lambda: settings.REPLAY_RATE_LIMIT)
def post_playback(request: Request, scenario: str, body: PlaybackRequest) -> dict:
    """
    Build the animation playback payload for one solution of *scenario*.

    Returns trajectories (meters), sparse connectivity edge lists, per-cell
    belief heat, and a targets-known curve — all truncated and optionally
    downsampled to a single aligned step axis.

    404 — scenario not found / model unknown / pickles missing.
    422 — index out of range, bad sensing config (p<=q, grid bounds, etc.),
          degenerate replay (zero-length step axis).
    """
    try:
        return build_playback(
            scenario=scenario,
            model_key=body.model_key or None,
            index=body.index,
            cfg_dict=body.config.to_cfg_dict(),
            stride=body.stride,
        )
    except _SelectorNotFound as exc:
        raise _not_found(scenario, str(exc)) from exc
    except StrategyUnavailableError as exc:
        raise _unprocessable(str(exc)) from exc
    except ValueError as exc:
        raise _unprocessable(str(exc)) from exc
