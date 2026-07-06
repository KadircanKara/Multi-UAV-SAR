"""Serialize finished PathSolution objects to the Playground JSON schema.

Reads CACHED objective scalars (never recompute) so the exported values match
exactly what the optimizer produced. Used by the web optimizer export and the
local CLI export.
"""
from __future__ import annotations

import app.rootpath  # noqa: F401  (repo root on sys.path)

import pandas as pd

from PathFuncDict import objective_values


def _scenario_dict(info) -> dict:
    return {
        "grid_size": info.grid_size,
        "cell_side_length": info.cell_side_length,
        "number_of_drones": info.number_of_drones,
        "max_drone_speed": info.max_drone_speed,
        "comm_cell_range": info.comm_cell_range,
        "n_visits": info.n_visits,
        "target_positions": list(info.target_locations),
        "th": info.th,
        "detection_probability": info.detection_probability,
    }


def serialize_run(solutions, F_df: pd.DataFrame, model: dict, run_config: dict,
                  model_key: str | None = None) -> dict:
    info = solutions[0].info
    from app.selector_service import get_polarities

    sols_json = []
    for i, sol in enumerate(solutions):
        sols_json.append({
            "index": i,
            "objectives": objective_values(sol),  # 5 cached scalars = raw objective_values output (unsigned; see schema note)
            "f_row": [float(v) for v in F_df.iloc[i].tolist()],
            "path": [int(c) for c in sol.path],
            "start_points": [int(c) for c in sol.start_points],
        })
    return {
        "schema_version": 1,
        "scenario": _scenario_dict(info),
        "model": {k: model[k] for k in ("Type", "Alg", "Exp", "F", "G", "H") if k in model}
        | {"model_key": model_key or model.get("model_key") or model.get("Exp", "")},
        "polarities": get_polarities(model),
        "run_config": run_config,
        "solutions": sols_json,
    }
