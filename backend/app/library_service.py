"""
Library service: scans Results/ for precomputed scenarios and exposes
structured metadata without triggering any heavy algorithm imports.

Import-safety: only PathInfo, PathOptimizationModel (for AVAILABLE_MODELS),
and pandas are used here.  PathAlgorithm / PathUnitTest / main are never
imported.
"""
import os
import re
from typing import Optional

import pandas as pd

import app.rootpath  # side-effect: inserts repo root into sys.path
from app import settings
from PathOptimizationModel import AVAILABLE_MODELS


# ---------------------------------------------------------------------------
# Model-key resolution
# ---------------------------------------------------------------------------

def resolve_model_key(scenario: str) -> str:
    """
    Map a scenario name to a key in AVAILABLE_MODELS.

    Examples
    --------
    MOO_NSGA2_TC_g_8_...   → TC_MOO_NSGA2
    WS_GA_TCDT_g_8_...     → TCDT_WS
    SOO_GA_MTSP_g_8_...    → MTSP
    SOO_NSGA2_MTSP_g_8_... → MTSP
    """
    prefix = scenario.split("_g_")[0]          # e.g. "MOO_NSGA2_TC", "WS_GA_TCDT"
    parts = prefix.split("_")
    typ, alg, exp = parts[0], parts[1], "_".join(parts[2:])

    if exp in ("MTSP", "CONN"):
        return exp                              # single registry entry; algorithm ignored

    if typ == "WS":
        return f"{exp}_WS"

    if typ == "MOO":
        return f"{exp}_MOO_{alg}"

    return exp


# ---------------------------------------------------------------------------
# Parameter parsing
# ---------------------------------------------------------------------------

_TAIL_RE = re.compile(
    r"_g_(?P<grid>\d+)"
    r"_a_(?P<cell>[^_]+)"
    r"_n_(?P<ndrones>\d+)"
    r"_v_(?P<speed>[^_]+)"
    r"_r_(?P<range>[^_]+)"
    r"_(?P<variant>nvisits|ntours)_(?P<vval>\d+)"
)


def parse_scenario_params(scenario: str) -> dict:
    """
    Best-effort parse of the ``_g_..._a_..._n_..._v_..._r_..._(nvisits|ntours)_...``
    tail into a structured dict.

    Never raises: missing fields are simply omitted from the returned dict.
    comm_range is kept as a raw string (may be "sqrt(8)").
    """
    params: dict = {}
    try:
        m = _TAIL_RE.search(scenario)
        if m is None:
            return params
        g = m.groupdict()

        try:
            params["grid_size"] = int(g["grid"])
        except (ValueError, KeyError):
            pass

        try:
            cell = g["cell"]
            params["cell_side_length"] = int(cell) if "." not in cell else float(cell)
        except (ValueError, KeyError):
            pass

        try:
            params["number_of_drones"] = int(g["ndrones"])
        except (ValueError, KeyError):
            pass

        try:
            params["max_drone_speed"] = float(g["speed"])
        except (ValueError, KeyError):
            pass

        try:
            params["comm_range"] = g["range"]
        except KeyError:
            pass

        try:
            params["variant"] = g["variant"]
            params["variant_value"] = int(g["vval"])
        except (ValueError, KeyError):
            pass

    except Exception:
        # Absolute fallback — never propagate parsing errors
        pass

    return params


# ---------------------------------------------------------------------------
# Filesystem scan helpers
# ---------------------------------------------------------------------------

def _objectives_dir() -> str:
    return os.path.join(settings.RESULTS_ROOT, "Objectives")


def _solutions_dir() -> str:
    return os.path.join(settings.RESULTS_ROOT, "Solutions")


def _obj_path(scenario: str) -> str:
    return os.path.join(_objectives_dir(), f"{scenario}-ObjectiveValues.pkl")


def _sol_path(scenario: str) -> str:
    return os.path.join(_solutions_dir(), f"{scenario}-SolutionObjects.pkl")


# ---------------------------------------------------------------------------
# Public service functions
# ---------------------------------------------------------------------------

def list_scenarios() -> list[dict]:
    """
    Scan Results/Objectives/*-ObjectiveValues.pkl (excluding *Abs* files).

    For each file:
      - derive the scenario name
      - require a matching Solutions pickle (else skip)
      - resolve model_key → skip if not in AVAILABLE_MODELS
      - read the objectives DataFrame to get n_solutions and column names
      - determine result_kind ("front" or "single")
      - merge parsed params

    Returns a list of dicts matching ScenarioSummary.
    """
    obj_dir = _objectives_dir()
    if not os.path.isdir(obj_dir):
        return []

    rows: list[dict] = []

    for fname in sorted(os.listdir(obj_dir)):
        if not fname.endswith("-ObjectiveValues.pkl"):
            continue
        if "ObjectiveValuesAbs" in fname:
            continue

        scenario = fname[: -len("-ObjectiveValues.pkl")]

        # Require matching solutions pickle
        if not os.path.isfile(_sol_path(scenario)):
            continue

        # Resolve model key
        try:
            model_key = resolve_model_key(scenario)
        except Exception:
            continue
        if model_key not in AVAILABLE_MODELS:
            continue

        model = AVAILABLE_MODELS[model_key]

        # Read objectives DataFrame (small — just need shape + columns)
        try:
            df: pd.DataFrame = pd.read_pickle(_obj_path(scenario))
            n_solutions: int = int(df.shape[0])
            objectives: list[str] = list(df.columns)
        except Exception:
            continue

        result_kind = (
            "front"
            if model["Type"] == "MOO" and n_solutions > 1
            else "single"
        )

        params = parse_scenario_params(scenario)

        # Build row — flat fields for ScenarioSummary
        row: dict = {
            "scenario": scenario,
            "model_key": model_key,
            "type": model["Type"],
            "algorithm": model["Alg"],
            "objectives": objectives,
            "n_solutions": n_solutions,
            "result_kind": result_kind,
            "grid_size": params.get("grid_size"),
            "number_of_drones": params.get("number_of_drones"),
            "comm_range": params.get("comm_range"),
            "variant": params.get("variant"),
            "variant_value": params.get("variant_value"),
            "has_solutions": True,
        }
        rows.append(row)

    # Sort by (type, number of objectives, number_of_drones)
    rows.sort(key=lambda r: (
        r["type"] or "",
        len(r["objectives"]),
        r["number_of_drones"] or 0,
    ))

    return rows


def get_scenario(scenario: str) -> Optional[dict]:
    """
    Return a detailed dict for a single scenario, or None if pickles are missing
    or the scenario cannot be resolved.
    """
    if not os.path.isfile(_sol_path(scenario)):
        return None

    try:
        model_key = resolve_model_key(scenario)
    except Exception:
        return None
    if model_key not in AVAILABLE_MODELS:
        return None

    model_dict = AVAILABLE_MODELS[model_key]

    try:
        df: pd.DataFrame = pd.read_pickle(_obj_path(scenario))
        n_solutions: int = int(df.shape[0])
        objectives: list[str] = list(df.columns)
    except Exception:
        return None

    result_kind = (
        "front"
        if model_dict["Type"] == "MOO" and n_solutions > 1
        else "single"
    )

    params = parse_scenario_params(scenario)

    # Build ModelInfo-compatible sub-dict
    model_info = {
        "name": model_key,
        "type": model_dict["Type"],
        "algorithm": model_dict["Alg"],
        "objectives": objectives,
        "constraints": model_dict.get("G", []),
    }

    return {
        "scenario": scenario,
        "model": model_info,
        "n_solutions": n_solutions,
        "result_kind": result_kind,
        "params": params,
    }
