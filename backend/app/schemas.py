"""Pydantic v2 schemas for the Multi-UAV-SAR API."""
from __future__ import annotations

import math
from typing import Optional, Union

from pydantic import (
    BaseModel,
    Field,
    ValidationInfo,
    field_validator,
    model_validator,
)


class ScenarioConfig(BaseModel):
    """
    Mirrors the root ``default_scenario`` dict exactly.
    All keys are passed straight into ``PathInfo(scenario_dict)``.
    """

    # le=64 is an absolute sanity ceiling, independent of SAR_MAX_GRID_SIZE (8):
    # a grid of a billion cells must never reach PathInfo however the deploy
    # cap is tuned. Cell count — and every per-cell scan — grows as grid_size².
    grid_size: int = Field(default=8, ge=1, le=64)
    # le=1000 is a safety ceiling: cell_side_length scales the per-leg distance,
    # which drives the realtime sub-sample count in Time.get_real_paths — a huge
    # value would explode that allocation (OOM). Real scenarios use 50.
    cell_side_length: Union[int, float] = Field(default=50, gt=0, le=1000)
    number_of_drones: int = Field(default=4, ge=1)
    # ge=0.1 is a safety floor: max_drone_speed divides the per-leg distance to
    # size the realtime sub-sample count in Time.get_real_paths — a near-zero
    # speed would explode that allocation (OOM). Real scenarios use 2.5.
    max_drone_speed: float = Field(default=2.5, ge=0.1)
    # comm_cell_range may be a non-integer (e.g. sqrt(8) ≈ 2.828…) in some scenarios
    comm_cell_range: Union[int, float] = Field(default=2, gt=0)
    # le=100 is a sanity/deploy bound: path length scales with n_visits, and the
    # project's real scenarios use single digits.
    n_visits: int = Field(default=2, ge=1, le=100)
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

    @field_validator("cell_side_length", "comm_cell_range", "max_drone_speed",
                     mode="after")
    @classmethod
    def reject_non_finite(cls, v: object, info: ValidationInfo) -> object:
        """``gt=0`` admits infinity (only NaN fails the comparison). An infinite
        distance or speed propagates into the objective computation as inf/NaN
        instead of failing here, so reject it at the edge."""
        if isinstance(v, float) and not math.isfinite(v):
            raise ValueError(f"{info.field_name} must be a finite number.")
        return v

    @model_validator(mode="after")
    def validate_target_positions(self) -> "ScenarioConfig":
        if not self.target_positions:
            raise ValueError("Enter at least one target cell.")
        max_cell = self.grid_size**2
        if len(self.target_positions) > max_cell:
            raise ValueError(
                f"You listed {len(self.target_positions)} target cells, but this "
                f"grid only has {max_cell}."
            )
        for t in self.target_positions:
            if not (0 <= t < max_cell):
                raise ValueError(
                    f"Cell {t} is out of range — valid cells are 0 to {max_cell - 1}."
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
    # max_length: the sensing loop does a per-cell, per-step membership scan
    # over this list, so its LENGTH (not just its values) is a cost multiplier.
    # 256 covers a full grid even at env-raised sizes (16×16); real scenarios
    # use a handful.
    target_locations: list[int] = Field(
        default_factory=lambda: [12],
        min_length=1,
        max_length=256,
        description="Non-empty list of 0-indexed grid cell ids",
    )

    @field_validator("merge_topology")
    @classmethod
    def validate_merge_topology(cls, v: str) -> str:
        allowed = {"none", "onboard", "gcs"}
        if v not in allowed:
            raise ValueError("Merge topology must be none, onboard, or gcs.")
        return v

    @field_validator("time_model")
    @classmethod
    def validate_time_model(cls, v: str) -> str:
        allowed = {"discrete", "realtime"}
        if v not in allowed:
            raise ValueError("Time model must be discrete or realtime.")
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
    """Body for POST /api/comparison — the scenarios to compare.

    Objective stats are read from each scenario's (cached) front, so this is much
    cheaper than the sensing-replay time comparison and allows a larger batch.
    Caps the chart at 36 bars × up to 10 stacked models (mirrors the web client)."""

    scenarios: list[str] = Field(..., min_length=1, max_length=360)


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
    """Body for POST /api/comparison/time — scenarios + shared sensing config.

    One sensing replay runs per scenario, so the ceiling is tighter than the
    objective comparison: 36 bars × up to 4 stacked models (mirrors the web client)."""

    scenarios: list[str] = Field(..., min_length=1, max_length=144)
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
    method: str = Field(description="SOO: GA | WS ; MOO: NSGA2 | NSGA3 | MOEAD")
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
    # Max Mean TBV ceiling (seconds); None = disabled. Only meaningful at
    # n_visits >= 2 (Max Mean TBV is 0 at n_visits == 1, so the ceiling no-ops).
    max_mean_tbv: Optional[float] = Field(default=None)
    # Generation strategy. "fixed" runs exactly n_gen generations; "max" treats
    # n_gen as a cap and stops early once the feasible objective optima converge
    # (no objective improves by >= early_stop_threshold for early_stop_patience
    # generations). Patience/threshold are predefined defaults.
    gen_strategy: str = Field(default="fixed")
    early_stop_patience: int = Field(default=10, ge=2, le=500)
    early_stop_threshold: float = Field(default=0.10, gt=0.0, le=1.0)
    scenario: ScenarioConfig = Field(default_factory=ScenarioConfig)

    @model_validator(mode="after")
    def _validate(self) -> "OptimizeConfig":
        t, m = self.optimization_type, self.method
        if t not in ("SOO", "MOO"):
            raise ValueError("optimization_type must be 'SOO' or 'MOO'")
        if self.gen_strategy not in ("fixed", "max"):
            raise ValueError("gen_strategy must be 'fixed' or 'max'")
        if self.max_mission_time is not None and self.max_mission_time <= 0:
            raise ValueError("max_mission_time must be > 0")
        if self.min_connectivity is not None and not (0.0 <= self.min_connectivity <= 1.0):
            raise ValueError("min_connectivity must be between 0 and 1")
        if self.max_mean_tbv is not None and self.max_mean_tbv <= 0:
            raise ValueError("max_mean_tbv must be > 0")
        if t == "SOO" and m not in ("GA", "WS"):
            raise ValueError("SOO method must be 'GA' or 'WS'")
        if t == "MOO" and m not in ("NSGA2", "NSGA3", "MOEAD"):
            raise ValueError("MOO method must be 'NSGA2', 'NSGA3' or 'MOEAD'")
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
        self._enforce_deploy_caps()
        return self

    def _enforce_deploy_caps(self) -> None:
        """Reject runs whose size exceeds the deploy caps (read at request time
        so environment overrides take effect). These bound the cost of any single
        run on a public deployment; see settings.py."""
        from app import settings

        if self.scenario.number_of_drones > settings.MAX_DRONES:
            raise ValueError(
                f"number_of_drones {self.scenario.number_of_drones} exceeds the "
                f"cap of {settings.MAX_DRONES}"
            )
        if self.scenario.grid_size > settings.MAX_GRID_SIZE:
            raise ValueError(
                f"grid_size {self.scenario.grid_size} exceeds the cap of "
                f"{settings.MAX_GRID_SIZE}"
            )
        if self.scenario.n_visits > settings.MAX_N_VISITS:
            raise ValueError(
                f"n_visits {self.scenario.n_visits} exceeds the cap of "
                f"{settings.MAX_N_VISITS}"
            )
        # cell_side_length x (1 / max_drone_speed) drives the realtime sub-sample
        # count; a huge cell or a near-zero speed can OOM a worker (see settings).
        if self.scenario.cell_side_length > settings.MAX_CELL_SIDE_LENGTH:
            raise ValueError(
                f"cell_side_length {self.scenario.cell_side_length} exceeds the "
                f"cap of {settings.MAX_CELL_SIDE_LENGTH}"
            )
        if self.scenario.max_drone_speed < settings.MIN_DRONE_SPEED:
            raise ValueError(
                f"max_drone_speed {self.scenario.max_drone_speed} is below the "
                f"floor of {settings.MIN_DRONE_SPEED}"
            )
        if self.pop_size > settings.MAX_POP_SIZE:
            raise ValueError(
                f"pop_size {self.pop_size} exceeds the cap of {settings.MAX_POP_SIZE}"
            )
        if self.n_gen > settings.MAX_N_GEN:
            raise ValueError(
                f"n_gen {self.n_gen} exceeds the cap of {settings.MAX_N_GEN}"
            )


class OptimizeStartResponse(BaseModel):
    run_id: str
    scenario_name: str
    model_key: str
    exists: bool
    seeded: bool = False
    # True when every pool worker was busy and the run is waiting its turn.
    queued: bool = False
    # 1-based place in the waiting line (1 = next to start); None if it started.
    queue_position: Optional[int] = None


class OptimizeCheckResponse(BaseModel):
    scenario_name: str
    model_key: str
    exists: bool
    seeded: bool = False


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
    # True when "Max Generations" converged and stopped before n_gen.
    early_stopped: bool = False
    stopped_at_gen: Optional[int] = None


class OptimizeStatusResponse(BaseModel):
    state: str  # queued | running | done | failed | cancelled
    # 1-based place in the waiting line while state == "queued".
    queue_position: Optional[int] = None
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
    # One full sensing replay runs per config, serially in the request handler,
    # so the list length is the request's cost multiplier. The web client sends
    # at most 3 (the merge topologies); 8 leaves headroom.
    configs: list[SensingConfigModel] = Field(..., min_length=1, max_length=8)
    labels: Optional[list[str]] = None


class PlaybackRequest(BaseModel):
    model_key: Optional[str] = None
    index: int
    config: SensingConfigModel
    stride: int = Field(default=1, ge=1, description="Step-axis downsampling factor (≥1)")
