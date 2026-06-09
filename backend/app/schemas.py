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
    cell_side_length: Optional[Union[int, float]] = None
    max_drone_speed: Optional[float] = None
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


# ---------------------------------------------------------------------------
# Pareto-front / solution-selection schemas
# ---------------------------------------------------------------------------

class ParetoSolution(BaseModel):
    index: int
    objectives_signed: dict[str, float]
    objectives_abs: dict[str, float]


class ParetoFront(BaseModel):
    scenario: str
    model_key: str
    objectives: list[str]
    polarities: dict[str, int]
    result_kind: str
    n_solutions: int
    solutions: list[ParetoSolution]
    capabilities: dict


class SelectRequest(BaseModel):
    model_key: Optional[str] = None
    strategy: str
    objective_name: Optional[str] = None
    weights: Optional[dict[str, float]] = None
    index: Optional[int] = None


class SolutionDetail(BaseModel):
    index: int
    label: str
    objectives_abs: dict[str, float]


class SelectResponse(BaseModel):
    index: int
    label: str
    detail: SolutionDetail


# ---------------------------------------------------------------------------
# Sensing replay / compare schemas
# ---------------------------------------------------------------------------

class SensingConfigModel(BaseModel):
    """Per-field-validated sensing config. Cross-field checks (p>q, grid bounds)
    are delegated to SensingConfig.from_info() — no duplication here."""

    merge_topology: str = Field(
        default="onboard",
        description="Who shares beliefs: none, onboard, or gcs",
    )
    time_model: str = Field(
        default="discrete",
        description="Replay pipeline: discrete or realtime",
    )
    detection_prob: float = Field(
        default=0.7,
        gt=0.0,
        lt=1.0,
        description="Detection probability p ∈ (0, 1)",
    )
    false_alarm_prob: float = Field(
        default=0.2,
        gt=0.0,
        lt=1.0,
        description="False alarm probability q ∈ (0, 1)",
    )
    belief_threshold: float = Field(
        default=0.9,
        gt=0.0,
        lt=1.0,
        description="Belief threshold B ∈ (0, 1)",
    )
    target_locations: list[int] = Field(
        default_factory=lambda: [12],
        min_length=1,
        description="Non-empty list of 0-indexed grid cell ids",
    )

    @field_validator("merge_topology")
    @classmethod
    def validate_merge_topology(cls, v: str) -> str:
        allowed = {"none", "onboard", "gcs"}
        if v not in allowed:
            raise ValueError(f"merge_topology must be one of {sorted(allowed)}, got {v!r}")
        return v

    @field_validator("time_model")
    @classmethod
    def validate_time_model(cls, v: str) -> str:
        allowed = {"discrete", "realtime"}
        if v not in allowed:
            raise ValueError(f"time_model must be one of {sorted(allowed)}, got {v!r}")
        return v

    def to_cfg_dict(self) -> dict:
        return self.model_dump()


class ReplayRequest(BaseModel):
    model_key: Optional[str] = None
    index: int
    config: SensingConfigModel
    label: Optional[str] = None


class CompareRequest(BaseModel):
    model_key: Optional[str] = None
    index: int
    configs: list[SensingConfigModel]
    labels: Optional[list[str]] = None


class PlaybackRequest(BaseModel):
    model_key: Optional[str] = None
    index: int
    config: SensingConfigModel
    stride: int = Field(default=1, ge=1, description="Step-axis downsampling factor (≥1)")
