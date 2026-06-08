"""Pydantic v2 schemas for the Multi-UAV-SAR API."""
from __future__ import annotations

from typing import Optional, Union

from pydantic import BaseModel, Field, field_validator, model_validator


class ScenarioConfig(BaseModel):
    """
    Mirrors the root ``default_scenario`` dict exactly.
    All keys are passed straight into ``PathInfo(scenario_dict)``.
    """

    grid_size: int = Field(default=8, ge=1)
    cell_side_length: Union[int, float] = Field(default=50, gt=0)
    number_of_drones: int = Field(default=4, ge=1)
    max_drone_speed: float = Field(default=2.5, gt=0)
    # comm_cell_range may be a non-integer (e.g. sqrt(8) ≈ 2.828…) in some scenarios
    comm_cell_range: Union[int, float] = Field(default=2, gt=0)
    n_visits: int = Field(default=2, ge=1)
    target_positions: list[int] = Field(default_factory=lambda: [12])
    th: float = Field(default=0.9, gt=0, lt=1)
    detection_probability: float = Field(default=0.7, gt=0, lt=1)

    @field_validator("cell_side_length", "comm_cell_range", mode="before")
    @classmethod
    def coerce_whole_float_to_int(cls, v: object) -> object:
        """Coerce whole-number floats to int so PathInfo filenames stay integer-formatted."""
        if isinstance(v, float) and v.is_integer():
            return int(v)
        return v

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


class ScenarioSummary(BaseModel):
    """One row returned by GET /api/library."""

    scenario: str
    model_key: str
    type: str
    algorithm: str
    objectives: list[str]
    n_solutions: int
    result_kind: str
    grid_size: Optional[int] = None
    number_of_drones: Optional[int] = None
    comm_range: Optional[str] = None
    variant: Optional[str] = None
    variant_value: Optional[int] = None
    has_solutions: bool


class ScenarioDetail(BaseModel):
    """Full detail returned by GET /api/library/{scenario}."""

    scenario: str
    model: ModelInfo
    n_solutions: int
    result_kind: str
    params: dict
