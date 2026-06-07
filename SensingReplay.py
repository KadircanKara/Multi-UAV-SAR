"""Analysis-layer sensing/merging configuration and replay API (spec section 1-3).

Merging parameters live HERE, never in the scenario dict / PathInfo / filenames
(spec D1). This module is also the future web view-section's parameter schema.
"""
from dataclasses import dataclass, field

VALID_MERGE_TOPOLOGIES = ("none", "onboard", "gcs")
VALID_TIME_MODELS = ("discrete", "realtime")


@dataclass
class SensingConfig:
    merge_topology: str = "onboard"
    time_model: str = "discrete"
    detection_prob: float = 0.7        # p
    false_alarm_prob: float = 0.2      # q
    belief_threshold: float = 0.9      # B
    target_locations: list = field(default_factory=lambda: [12])

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
