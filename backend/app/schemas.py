"""Pydantic v2 schemas for the Multi-UAV-SAR API."""
from __future__ import annotations

from typing import Optional

from pydantic import BaseModel, Field, model_validator


class ScenarioConfig(BaseModel):
    """
    Mirrors the root ``default_scenario`` dict exactly.
    All keys are passed straight into ``PathInfo(scenario_dict)``.
    """

    grid_size: int = Field(default=8, ge=1)
    cell_side_length: float = Field(default=50.0, gt=0)
    number_of_drones: int = Field(default=4, ge=1)
    max_drone_speed: float = Field(default=2.5, gt=0)
    # comm_cell_range may be a non-integer (e.g. sqrt(8) ≈ 2.828…) in some scenarios
    comm_cell_range: float = Field(default=2.0, gt=0)
    n_visits: int = Field(default=2, ge=1)
    target_positions: list[int] = Field(default_factory=lambda: [12])
    th: float = Field(default=0.9, gt=0, lt=1)
    detection_probability: float = Field(default=0.7, gt=0, lt=1)

    @model_validator(mode="after")
    def validate_target_positions(self) -> "ScenarioConfig":
        if not self.target_positions:
            raise ValueError("target_positions must not be empty")
        max_cell = self.grid_size**2
        for t in self.target_positions:
            if not (0 <= t < max_cell):
                raise ValueError(
                    f"target position {t} is out of range [0, {max_cell})"
                )
        return self

    def to_scenario_dict(self) -> dict:
        """Return a plain dict with exactly the keys PathInfo expects."""
        return self.model_dump()


class ScenarioDerived(BaseModel):
    """Quantities computed by PathInfo from a ScenarioConfig."""

    number_of_cells: int
    number_of_nodes: int
    comm_dist: float
    miss_probability: float


class ScenarioValidateRequest(BaseModel):
    scenario: ScenarioConfig
    model_key: Optional[str] = None


class ScenarioValidateResponse(BaseModel):
    valid: bool
    derived: ScenarioDerived
    scenario_str: Optional[str] = None


class ModelInfo(BaseModel):
    name: str
    type: str
    algorithm: str
    objectives: list[str]
    constraints: list[str]
