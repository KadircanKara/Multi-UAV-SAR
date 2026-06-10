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


# ---------------------------------------------------------------------------
# Model-grid schemas (parameter-effect analysis)
# ---------------------------------------------------------------------------

class ObjectiveStat(BaseModel):
    """Summary statistics for one objective across a scenario's solutions."""

    min: Optional[float] = None
    max: Optional[float] = None
    mean: Optional[float] = None
    best: Optional[float] = None


class ModelGridScenario(BaseModel):
    """One row in the model parameter grid."""

    scenario: str
    number_of_drones: Optional[int] = None
    comm_range: Optional[str] = None
    comm_range_value: Optional[float] = None
    n_visits: Optional[int] = None
    n_tours: Optional[int] = None
    n_solutions: int
    result_kind: str
    objective_stats: dict[str, ObjectiveStat]


class ModelGrid(BaseModel):
    """Full parameter grid for one optimisation model."""

    model_key: str
    type: str
    algorithm: str
    objectives: list[str]
    polarities: dict[str, int]
    scenarios: list[ModelGridScenario]


# ---------------------------------------------------------------------------
# Cross-model objective comparison schemas
# ---------------------------------------------------------------------------

class ComparisonRequest(BaseModel):
    """Body for POST /api/comparison — the scenarios to compare."""

    scenarios: list[str] = Field(..., min_length=1, max_length=24)


class ComparisonScenario(BaseModel):
    """All-objective summary for one scenario in a comparison."""

    scenario: str
    model_key: str
    type: str
    algorithm: str
    # The objective names this model actually optimised (WS expanded); the rest
    # are still reported, computed from the solution objects for comparability.
    optimized_objectives: list[str]
    number_of_drones: Optional[int] = None
    comm_range: Optional[str] = None
    comm_range_value: Optional[float] = None
    n_visits: Optional[int] = None
    n_solutions: int
    # None for an objective with no data (e.g. Max Mean TBV at n_visits == 1).
    objective_stats: dict[str, Optional[ObjectiveStat]]


class ComparisonResponse(BaseModel):
    objectives: list[str]
    polarities: dict[str, int]
    scenarios: list[ComparisonScenario]
    skipped: list[str]


# ---------------------------------------------------------------------------
# Cross-model time-metric comparison schemas
# ---------------------------------------------------------------------------

class TimeComparisonRequest(BaseModel):
    """Body for POST /api/comparison/time — scenarios + shared sensing config."""

    scenarios: list[str] = Field(..., min_length=1, max_length=24)
    config: SensingConfigModel
    strategy: str = "balanced"
    objective_name: Optional[str] = None
    weights: Optional[dict[str, float]] = None


class TimeComparisonScenario(BaseModel):
    """Time-metric summary for one scenario in a comparison."""

    scenario: str
    model_key: str
    type: str
    algorithm: str
    number_of_drones: Optional[int] = None
    comm_range: Optional[str] = None
    comm_range_value: Optional[float] = None
    n_visits: Optional[int] = None
    selected_index: int
    metric_values: dict[str, Optional[float]]


class TimeComparisonResponse(BaseModel):
    metrics: list[str]
    scenarios: list[TimeComparisonScenario]
    skipped: list[str]
    strategy: str


# ---------------------------------------------------------------------------
# Optimizer (Configure + Run) schemas
# ---------------------------------------------------------------------------

_VALID_OBJECTIVES = {
    "Mission Time",
    "Percentage Connectivity",
    "Max Disconnected Time",
    "Mean Disconnected Time",
    "Max Mean TBV",
}


class OptimizeConfig(BaseModel):
    """A user-configured optimization run."""

    optimization_type: str = Field(description="SOO | MOO")
    method: str = Field(description="SOO: GA | WS ; MOO: NSGA2 | NSGA3 (MOEAD blocked)")
    objectives: list[str] = Field(..., min_length=1)
    weights: Optional[dict[str, float]] = None
    pop_size: int = Field(default=100, ge=10, le=500)
    n_gen: int = Field(default=300, ge=5, le=1000)
    seed: int = Field(default=1, ge=0)
    # Configurable constraints (None = disabled). The speed-violation constraint
    # is ALWAYS applied by the optimizer (required for path interpolation) and is
    # not exposed here. mission_time is in seconds; connectivity is a fraction.
    max_mission_time: Optional[float] = Field(default=3600.0)
    min_connectivity: Optional[float] = Field(default=0.5)
    scenario: ScenarioConfig = Field(default_factory=ScenarioConfig)

    @model_validator(mode="after")
    def _validate(self) -> "OptimizeConfig":
        t, m = self.optimization_type, self.method
        if t not in ("SOO", "MOO"):
            raise ValueError("optimization_type must be 'SOO' or 'MOO'")
        if self.max_mission_time is not None and self.max_mission_time <= 0:
            raise ValueError("max_mission_time must be > 0")
        if self.min_connectivity is not None and not (0.0 <= self.min_connectivity <= 1.0):
            raise ValueError("min_connectivity must be between 0 and 1")
        if t == "SOO" and m not in ("GA", "WS"):
            raise ValueError("SOO method must be 'GA' or 'WS'")
        if t == "MOO":
            if m == "MOEAD":
                raise ValueError("MOEAD is not available yet")
            if m not in ("NSGA2", "NSGA3"):
                raise ValueError("MOO method must be 'NSGA2' or 'NSGA3'")
        bad = [o for o in self.objectives if o not in _VALID_OBJECTIVES]
        if bad:
            raise ValueError(f"unknown objectives: {bad}")
        if len(set(self.objectives)) != len(self.objectives):
            raise ValueError("duplicate objectives")
        if t == "SOO" and m == "GA" and len(self.objectives) != 1:
            raise ValueError("SOO-GA requires exactly one objective")
        if t == "SOO" and m == "WS":
            if len(self.objectives) < 2:
                raise ValueError("Weighted-sum requires at least two objectives")
            w = self.weights or {}
            if set(w.keys()) != set(self.objectives):
                raise ValueError("a weight must be provided for each objective")
            if any(v < 0 for v in w.values()):
                raise ValueError("weights must be non-negative")
            total = sum(w.values())
            # Tolerate 4-decimal rounding (e.g. an equal split of 3 objectives is
            # 0.3333×3 = 0.9999). Matches the frontend run-gate's 1e-3 tolerance so
            # any config it accepts the API accepts too; gross errors (sum 0.9 / 1.1)
            # are still well outside this band.
            if abs(total - 1.0) > 1e-3:
                raise ValueError(f"weights must sum to 1 (got {total:.4f})")
        if t == "MOO" and len(self.objectives) < 2:
            raise ValueError("MOO requires at least two objectives")
        return self


class OptimizeStartResponse(BaseModel):
    run_id: str
    scenario_name: str
    model_key: str
    exists: bool


class OptimizeCheckResponse(BaseModel):
    scenario_name: str
    model_key: str
    exists: bool


class OptimizeFrontSolution(BaseModel):
    index: int
    objectives_signed: dict[str, Optional[float]]
    objectives_abs: dict[str, Optional[float]]


class OptimizeFront(BaseModel):
    scenario: str
    model_key: str
    objectives: list[str]
    polarities: dict[str, int]
    result_kind: str
    n_solutions: int
    solutions: list[OptimizeFrontSolution]
    # True when the run was stopped early by the user; the front is the best-so-far.
    cancelled: bool = False
    stopped_at_gen: Optional[int] = None


class OptimizeStatusResponse(BaseModel):
    state: str  # running | done | failed
    gen: Optional[int] = None
    n_gen: Optional[int] = None
    front: Optional[OptimizeFront] = None
    error: Optional[str] = None
    exists_in_library: Optional[bool] = None
    # Live progress while running: per-objective best (absolute) + the current
    # non-dominated front as absolute objective points.
    best: Optional[dict[str, float]] = None
    live_front: Optional[list[dict[str, float]]] = None


class OptimizeStopResponse(BaseModel):
    run_id: str
    stopping: bool  # False if the run had already finished (nothing to stop)


class OptimizeSaveRequest(BaseModel):
    overwrite: bool = False


class OptimizeSaveResponse(BaseModel):
    scenario_name: str
    model_key: str


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
