"""GET /api/models — returns the full list of ready-made optimisation models."""
import app.rootpath  # must come before any root-module import

from fastapi import APIRouter
from PathOptimizationModel import list_models

from app.schemas import ModelInfo

router = APIRouter()


@router.get("/api/models", response_model=list[ModelInfo])
def get_models() -> list[dict]:
    """Return all 20 ready-made optimisation models."""
    return list_models().to_dict("records")
