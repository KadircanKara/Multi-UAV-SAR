"""GET /api/models — returns the full list of optimisation models (preset + custom)."""
import app.rootpath  # must come before any root-module import

from fastapi import APIRouter, HTTPException
from PathOptimizationModel import (
    get_objectives_from_weighted_sum_model,
    list_models,
)

from app import models_registry
from app.library_service import model_grid
from app.model_aliases import to_display
from app.schemas import ModelGrid, ModelInfo

router = APIRouter()


def _custom_model_info(model_key: str, model: dict) -> dict:
    """Build a ModelInfo-shaped row for a saved custom model."""
    if model.get("Type") == "WS":
        try:
            objectives = list(get_objectives_from_weighted_sum_model(model))
        except Exception:
            objectives = list(model.get("F", []))
    else:
        objectives = list(model.get("F", []))
    return {
        "name": model_key,
        "type": model.get("Type", ""),
        "algorithm": model.get("Alg", ""),
        "objectives": objectives,
        "constraints": list(model.get("G", [])),
    }


@router.get("/api/models", response_model=list[ModelInfo])
def get_models() -> list[dict]:
    """Return the ready-made (preset) models plus any saved custom models."""
    rows = list_models().to_dict("records")
    rows.extend(
        _custom_model_info(key, model)
        for key, model in models_registry.custom_models().items()
    )
    # Show the TBV "V" display code (TCDT→TCDV) in the model name.
    for row in rows:
        row["name"] = to_display(row.get("name"))
    return rows


@router.get("/api/models/{model_key}/grid", response_model=ModelGrid)
def get_model_grid(model_key: str) -> dict:
    """
    Return the parameter grid for a single model.

    Each row in ``scenarios`` covers one (drones, comm_range, n_visits)
    combination and includes per-objective summary statistics (min/max/mean/best)
    computed from the small Objectives pickles only — never loads Solution pickles.

    404 if the model_key is not in the registry or no seeded scenarios exist.
    """
    if not models_registry.known(model_key):
        raise HTTPException(status_code=404, detail=f"Unknown model: {model_key!r}")

    grid = model_grid(model_key)
    if grid is None:
        raise HTTPException(
            status_code=404,
            detail=f"No seeded scenarios found for model {model_key!r}",
        )
    return grid
