"""GET /api/models — returns the full list of ready-made optimisation models."""
import app.rootpath  # must come before any root-module import

from fastapi import APIRouter, HTTPException
from PathOptimizationModel import AVAILABLE_MODELS, list_models

from app.library_service import model_grid
from app.schemas import ModelGrid, ModelInfo

router = APIRouter()


@router.get("/api/models", response_model=list[ModelInfo])
def get_models() -> list[dict]:
    """Return all 20 ready-made optimisation models."""
    return list_models().to_dict("records")


@router.get("/api/models/{model_key}/grid", response_model=ModelGrid)
def get_model_grid(model_key: str) -> dict:
    """
    Return the parameter grid for a single model.

    Each row in ``scenarios`` covers one (drones, comm_range, n_visits)
    combination and includes per-objective summary statistics (min/max/mean/best)
    computed from the small Objectives pickles only — never loads Solution pickles.

    404 if the model_key is not in the registry or no seeded scenarios exist.
    """
    if model_key not in AVAILABLE_MODELS:
        raise HTTPException(status_code=404, detail=f"Unknown model: {model_key!r}")

    grid = model_grid(model_key)
    if grid is None:
        raise HTTPException(
            status_code=404,
            detail=f"No seeded scenarios found for model {model_key!r}",
        )
    return grid
