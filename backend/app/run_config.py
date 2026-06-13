"""Assemble the RunConfig metadata record persisted alongside a saved mission.

Pure functions only (no I/O): `build_run_config` shapes the record from primitive
run parameters; `seed_config_for` derives the record for a seeded mission from its
scenario name and the model's own G/H. Both are import-safe (no `main`/PathAlgorithm).
"""
from __future__ import annotations

import re
from typing import Optional

import app.rootpath  # noqa: F401  (repo root on sys.path for PathOptimizationModel)

SCHEMA_VERSION = 1

# Filename tail: ..._g_{grid}_a_{cell}_n_{drones}_v_{speed}_r_{comm}_nvisits_{nv}
_SEED_NAME_RE = re.compile(
    r"_g_(\d+)_a_([0-9.]+)_n_(\d+)_v_([0-9.]+)_r_(.+?)_nvisits_(\d+)$"
)


def _type_method(model_dict: dict) -> tuple[str, str]:
    """(model Type, Alg) → (optimization_type SOO|MOO, method GA|WS|NSGA2|NSGA3)."""
    t, alg = model_dict["Type"], model_dict["Alg"]
    if t == "WS":
        return "SOO", "WS"
    if t == "SOO":
        return "SOO", "GA"
    return "MOO", alg  # NSGA2 / NSGA3


def build_run_config(
    *,
    model_dict: dict,
    objectives: list,
    weights: Optional[dict],
    pop_size: int,
    n_gen: int,
    seed: int,
    gen_strategy: str,
    early_stop_patience: int,
    early_stop_threshold: float,
    max_mission_time: Optional[float],
    min_connectivity: Optional[float],
    max_mean_tbv: Optional[float],
    scenario: dict,
    n_solutions: int,
    cancelled: bool,
    early_stopped: bool,
    stopped_at_gen: Optional[int],
    source: str,
) -> dict:
    """Build the RunConfig dict. early-stop fields are recorded only for the
    'max' generation strategy; a constraint threshold of None means it was off."""
    optimization_type, method = _type_method(model_dict)
    is_max = gen_strategy == "max"
    return {
        "schema_version": SCHEMA_VERSION,
        "optimization_type": optimization_type,
        "method": method,
        "objectives": list(objectives),
        "weights": dict(weights) if weights else None,
        "pop_size": int(pop_size),
        "n_gen": int(n_gen),
        "seed": int(seed),
        "gen_strategy": gen_strategy,
        "early_stop_patience": int(early_stop_patience) if is_max else None,
        "early_stop_threshold": float(early_stop_threshold) if is_max else None,
        "constraints": {
            "speed_feasibility": True,
            "max_mission_time": float(max_mission_time) if max_mission_time is not None else None,
            "min_connectivity": float(min_connectivity) if min_connectivity is not None else None,
            "max_mean_tbv": float(max_mean_tbv) if max_mean_tbv is not None else None,
        },
        "scenario": {
            "number_of_drones": scenario.get("number_of_drones"),
            "comm_range": scenario.get("comm_cell_range", scenario.get("comm_range")),
            "n_visits": scenario.get("n_visits"),
            "max_drone_speed": scenario.get("max_drone_speed"),
            "grid_size": scenario.get("grid_size"),
            "cell_side_length": scenario.get("cell_side_length"),
        },
        "outcome": {
            "n_solutions": int(n_solutions),
            "cancelled": bool(cancelled),
            "early_stopped": bool(early_stopped),
            "stopped_at_gen": int(stopped_at_gen) if stopped_at_gen is not None else None,
        },
        "source": source,
    }


def seed_config_for(scenario_name: str, n_solutions: int = 0) -> Optional[dict]:
    """RunConfig for a seeded mission, or None if the name matches no preset model.

    Matches the preset whose '{Type}_{Alg}_{Exp}_' prefixes the scenario name, reads
    the constraints the run used straight off that model's G/H (at the engine-default
    thresholds), and records the known seeded run-config (pop 300, n_gen 800, seed 1,
    fixed). Seeded WS models carry no Weights key → equal weighting (1/n)."""
    from PathOptimizationModel import (
        AVAILABLE_MODELS,
        get_objectives_from_weighted_sum_model,
    )

    matched = None
    for _key, m in AVAILABLE_MODELS.items():
        if scenario_name.startswith(f"{m['Type']}_{m['Alg']}_{m['Exp']}_"):
            matched = m
            break
    if matched is None:
        return None

    mt = _SEED_NAME_RE.search(scenario_name)
    if mt is None:
        return None
    grid = int(mt.group(1))
    cell = float(mt.group(2))
    drones = int(mt.group(3))
    speed = float(mt.group(4))
    comm = mt.group(5)
    n_visits = int(mt.group(6))

    G = matched.get("G", [])
    max_mission_time = (
        (3600.0 if n_visits < 4 else float(n_visits * 600))
        if "Max Mission Time" in G else None
    )
    min_connectivity = (
        (0.5 if drones > 2 else 0.0)
        if "Min Percentage Connectivity" in G else None
    )

    if matched["Type"] == "WS":
        objectives = list(get_objectives_from_weighted_sum_model(matched))
        weights = {o: 1.0 / len(objectives) for o in objectives}
    else:
        objectives = list(matched["F"])
        weights = None

    scenario = {
        "number_of_drones": drones,
        "comm_cell_range": comm,
        "n_visits": n_visits,
        "max_drone_speed": speed,
        "grid_size": grid,
        "cell_side_length": cell,
    }
    return build_run_config(
        model_dict=matched, objectives=objectives, weights=weights,
        pop_size=300, n_gen=800, seed=1, gen_strategy="fixed",
        early_stop_patience=10, early_stop_threshold=0.10,
        max_mission_time=max_mission_time, min_connectivity=min_connectivity,
        max_mean_tbv=None, scenario=scenario, n_solutions=n_solutions,
        cancelled=False, early_stopped=False, stopped_at_gen=800, source="seed",
    )
