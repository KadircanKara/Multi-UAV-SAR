"""Analysis-layer sensing/merging configuration and replay API (spec section 1-3).

Merging parameters live HERE, never in the scenario dict / PathInfo / filenames
(spec D1). This module is also the future web view-section's parameter schema.
"""
import numpy as np
from dataclasses import dataclass, field

# Sensing is imported at module level: grep confirms Sensing.py does NOT import
# SensingReplay, so there is no circular dependency.
from Sensing import sensing_and_discrete_info_sharing, sensing_and_realtime_info_sharing

VALID_MERGE_TOPOLOGIES = ("none", "onboard", "gcs")
VALID_TIME_MODELS = ("discrete", "realtime")


@dataclass
class SensingConfig:
    """Sensing & merging parameters for replay analysis (the future web UI's parameter schema). merge_topology: who shares beliefs; time_model: which replay pipeline; p/q/B control the Bayesian filter; target_locations are 0-indexed grid cell ids."""

    merge_topology: str = "onboard"
    time_model: str = "discrete"
    detection_prob: float = 0.7        # p
    false_alarm_prob: float = 0.2      # q
    belief_threshold: float = 0.9      # B
    target_locations: list[int] = field(default_factory=lambda: [12])

    def __post_init__(self):
        if self.merge_topology not in VALID_MERGE_TOPOLOGIES:
            raise ValueError(f"merge_topology {self.merge_topology!r} not in {VALID_MERGE_TOPOLOGIES}")
        if self.time_model not in VALID_TIME_MODELS:
            raise ValueError(f"time_model {self.time_model!r} not in {VALID_TIME_MODELS}")
        for name in ("detection_prob", "false_alarm_prob", "belief_threshold"):
            v = getattr(self, name)
            if not (0.0 < v < 1.0):
                raise ValueError(f"{name} must be in (0, 1), got {v}")
        if not self.target_locations:
            raise ValueError("target_locations must not be empty")
        if not all(isinstance(t, (int, np.integer)) for t in self.target_locations):
            raise ValueError(f"target_locations must be integer cell ids, got {self.target_locations!r}")

    @classmethod
    def from_info(cls, info, **overrides):
        """Build a config defaulting from PathInfo. The ONLY place old-pickle
        defaulting lives (spec: 'centralized, not scattered getattrs')."""
        defaults = dict(
            detection_prob=getattr(info, "detection_probability", 0.7),
            belief_threshold=getattr(info, "th", 0.9),
            target_locations=list(getattr(info, "target_locations", [12])),
        )
        defaults.update(overrides)
        cfg = cls(**defaults)
        n_cells = getattr(info, "number_of_cells", None)
        if n_cells is not None:
            bad = [t for t in cfg.target_locations if not (0 <= t < n_cells)]
            if bad:
                raise ValueError(f"target_locations {bad} outside grid (0..{n_cells - 1})")
        return cfg


@dataclass
class ReplayResult:
    """One sensing replay's outputs: spec-named metrics (Effective Mission Time etc.), per-step belief artifacts for plots/animations, and the truncated solution copy whose path matrices reflect any early return."""
    config: SensingConfig
    label: str
    effective_mission_time: float
    detection_time: float
    inform_time: float
    time_at_least_one_drone_knows_all: float
    cell_occupancy_probabilities: list
    occupancy_status: np.ndarray    # (nodes x cells) int flags
    search_map: np.ndarray          # (nodes x cells) object array of per-node observation lists
    solution: object                # truncated PathSolution copy (for animation)


def replay(solution, config, label=None):
    """Run one sensing replay. Dispatch is the ONLY thing that looks at
    time_model — the pipelines satisfy one return contract (spec AD1)."""
    if config.time_model == "discrete":
        metrics, x = sensing_and_discrete_info_sharing(solution, config)
    else:
        metrics, x = sensing_and_realtime_info_sharing(solution, config)
    return ReplayResult(
        config=config,
        label=label if label is not None else config.merge_topology,
        effective_mission_time=metrics["mission time"],
        detection_time=metrics["detection time"],
        inform_time=metrics["inform time"],
        time_at_least_one_drone_knows_all=metrics["time at least one drone knows all targets"],
        cell_occupancy_probabilities=metrics["cell occupancy probabilities"],
        occupancy_status=metrics["occupancy status"],
        search_map=metrics["search map"],
        solution=x,
    )
