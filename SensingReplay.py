"""Analysis-layer sensing/merging configuration and replay API (spec section 1-3).

Merging parameters live HERE, never in the scenario dict / PathInfo / filenames
(spec D1). This module is also the future web view-section's parameter schema.
"""
import os
from dataclasses import dataclass, field

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

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


METRIC_COLUMNS = {
    "Effective Mission Time": "effective_mission_time",
    "Detection Time": "detection_time",
    "Inform Time": "inform_time",
    "Time At Least One Drone Knows All Targets": "time_at_least_one_drone_knows_all",
}

DEFAULT_PLOT_DIR = "Figures/Sensing/Merging Comparisons"
DEFAULT_ANIM_DIR = "Results/Animations"


@dataclass
class ComparisonResult:
    table: pd.DataFrame
    csv_path: str
    plot_paths: list
    animation_paths: list
    replays: list


def _dedupe_labels(configs, labels):
    if labels is None:
        labels = [c.merge_topology for c in configs]
    counts = {}
    for lab in labels:
        counts[lab] = counts.get(lab, 0) + 1
    seen = {}
    out = []
    for lab in labels:
        if counts[lab] > 1:
            out.append(f"{lab}-{seen.get(lab, 0)}")
            seen[lab] = seen.get(lab, 0) + 1
        else:
            out.append(lab)
    return out


def _targets_known_curve(r):
    """Targets-known count per step: belief time series for each target cell,
    counted once its belief first exceeds the config's threshold."""
    cfg, probs = r.config, r.cell_occupancy_probabilities
    known_set = set()
    curve = []
    for step in range(len(probs[0])):
        for t in cfg.target_locations:
            if t not in known_set and probs[t][step] > cfg.belief_threshold:
                known_set.add(t)
        curve.append(len(known_set))
    return curve


def compare(solution, configs, labels=None, scenario_label="scenario",
            output_dir=None, plot_dir=None, anim_dir=None, animations=True):
    """Replay one solution under each config; emit table + plots (+ animations).

    output_dir, when given, overrides plot_dir/anim_dir/csv location (used by
    tests); otherwise artifacts land in the standard Figures/Results trees.
    Re-using the same scenario_label and directory overwrites prior artifacts.
    """
    labels = _dedupe_labels(configs, labels)
    replays = [replay(solution, c, label=l) for c, l in zip(configs, labels)]

    plot_dir = output_dir or plot_dir or DEFAULT_PLOT_DIR
    anim_dir = output_dir or anim_dir or DEFAULT_ANIM_DIR
    csv_dir = output_dir or plot_dir
    for d in (plot_dir, anim_dir, csv_dir):
        os.makedirs(d, exist_ok=True)

    # 1) metrics table -------------------------------------------------------
    table = pd.DataFrame(
        {col: [getattr(r, attr) for r in replays] for col, attr in METRIC_COLUMNS.items()},
        index=labels)
    csv_path = os.path.join(csv_dir, f"{scenario_label}-comparison.csv")
    table.to_csv(csv_path)

    # 2) time-series plots ---------------------------------------------------
    plot_paths = []
    fig, ax = plt.subplots(figsize=(8, 5))
    for r in replays:
        curve = _targets_known_curve(r)
        ax.plot(range(len(curve)), curve, label=r.label)
    ax.set_xlabel("step"); ax.set_ylabel("targets known")
    ax.set_title(f"Targets known over time — {scenario_label}")
    ax.legend()
    p1 = os.path.join(plot_dir, f"{scenario_label}-targets-over-time.png")
    fig.savefig(p1, dpi=150, bbox_inches="tight"); plt.close(fig)
    plot_paths.append(p1)

    fig, ax = plt.subplots(figsize=(8, 5))
    first_target = replays[0].config.target_locations[0]
    for r in replays:
        probs = r.cell_occupancy_probabilities[first_target]
        ax.plot(range(len(probs)), probs, label=r.label)
    for B in sorted({r.config.belief_threshold for r in replays}):
        ax.axhline(B, linestyle="--", color="grey", label=f"B = {B}")
    ax.set_xlabel("step"); ax.set_ylabel(f"max belief, cell {first_target}")
    ax.set_title(f"Belief evolution — {scenario_label}")
    ax.legend()
    p2 = os.path.join(plot_dir, f"{scenario_label}-belief-evolution.png")
    fig.savefig(p2, dpi=150, bbox_inches="tight"); plt.close(fig)
    plot_paths.append(p2)

    # 3) animations ----------------------------------------------------------
    animation_paths = []
    if animations:
        from matplotlib.animation import FuncAnimation, PillowWriter
        from PathAnimation import PathAnimation
        for r in replays:
            fig, ax = plt.subplots(figsize=(6, 6))
            anim_obj = PathAnimation(
                r.solution, fig, ax,
                target_locations=r.config.target_locations,
                cell_occupancy_probabilities=r.cell_occupancy_probabilities,
                B=r.config.belief_threshold)
            frames = anim_obj.paths[0].shape[1]
            # interval must be > 0: matplotlib's anim.save() computes a fallback
            # fps = 1000/interval when no fps arg is passed, and a pre-built
            # writer (PillowWriter) forbids passing fps to save(); interval=0
            # would ZeroDivisionError. 100 ms <=> 10 fps, matching the writer.
            anim = FuncAnimation(fig, anim_obj.update, frames=frames,
                                 init_func=anim_obj.initialize_figure,
                                 blit=False, interval=100)
            path = os.path.join(anim_dir, f"{scenario_label}-{r.label}-replay.gif")
            anim.save(path, writer=PillowWriter(fps=10))
            plt.close(fig)
            animation_paths.append(path)

    return ComparisonResult(table=table, csv_path=csv_path, plot_paths=plot_paths,
                            animation_paths=animation_paths, replays=replays)
