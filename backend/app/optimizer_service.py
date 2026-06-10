"""
Optimizer service: synthesizes a model dict from a user's config, derives the
scenario name, runs the optimization in a single-worker ProcessPoolExecutor
(background + poll), and (later) saves a finished run to the library.

Import-safety: imports PathInfo / PathOptimizationModel / PathFuncDict (all
import-safe — no `main` / `PathAlgorithm`). The heavy operators are imported
only inside the worker child process (`optimizer_worker`).
"""
from __future__ import annotations

import json
import os
import threading
import uuid
from concurrent.futures import ProcessPoolExecutor
from typing import Optional

import app.rootpath  # noqa: F401  (repo root on sys.path)
from app import settings

from PathOptimizationModel import (
    AVAILABLE_MODELS,
    get_objectives_from_weighted_sum_model,
    get_weighted_sum_objective_name_from_objectives,
)
from PathFuncDict import model_metric_info


# Canonical objective order + polarity (authoritative from PathFuncDict).
OBJECTIVES: list[str] = list(model_metric_info["Objectives"].keys())
_POLARITY: dict[str, int] = {
    name: int(info[1]) for name, info in model_metric_info["Objectives"].items()
}

# Short code per objective for synthesizing a custom Exp (only used for combos
# that don't match a preset — preset combos reuse the preset's Exp).
_OBJ_CODE = {
    "Mission Time": "T",
    "Percentage Connectivity": "C",
    "Max Disconnected Time": "Dx",
    "Mean Disconnected Time": "Dn",
    "Max Mean TBV": "V",
}


# ─── Errors (mapped to HTTP by the router) ────────────────────────────────────

class RunInProgressError(Exception):
    """A run is already in flight (single worker)."""


class RunNotFoundError(Exception):
    pass


class RunNotReadyError(Exception):
    pass


class AlreadyExistsError(Exception):
    """Scenario already in the library and overwrite was not requested."""


# ─── Model synthesis ──────────────────────────────────────────────────────────

def _derive_type_alg(optimization_type: str, method: str) -> tuple[str, str]:
    """(optimization_type SOO|MOO, method) → (model Type, Alg)."""
    if optimization_type == "SOO":
        return ("WS", "GA") if method == "WS" else ("SOO", "GA")
    return ("MOO", method)  # NSGA2 / NSGA3


def _find_preset(model_type: str, alg: str, objectives: list[str]):
    """Return (model_key, model_dict) of a preset whose (Type, Alg, objective-set)
    matches, else (None, None)."""
    target = set(objectives)
    for key, m in AVAILABLE_MODELS.items():
        if m["Type"] != model_type or m["Alg"] != alg:
            continue
        mobjs = (
            set(get_objectives_from_weighted_sum_model(m))
            if model_type == "WS"
            else set(m["F"])
        )
        if mobjs == target:
            return key, m
    return None, None


def _synth_exp(objectives: list[str]) -> str:
    return "".join(_OBJ_CODE[o] for o in OBJECTIVES if o in objectives) or "X"


def _constraints(max_mission_time, min_connectivity) -> tuple[list, list]:
    """Build (G, H) from the user's constraint config. The speed-violation
    constraint is ALWAYS in H (required for drone-path interpolation)."""
    G: list = []
    if max_mission_time is not None:
        G.append("Max Mission Time")
    if min_connectivity is not None:
        G.append("Min Percentage Connectivity")
    H = ["Path Speed Violations as Constraint"]
    return G, H


def resolve_model(
    optimization_type: str,
    method: str,
    objectives: list[str],
    weights: Optional[dict] = None,
    max_mission_time: Optional[float] = None,
    min_connectivity: Optional[float] = None,
) -> tuple[str, dict]:
    """Synthesize (model_key, model_dict) from the user's config. Matches an
    existing preset's Type/Alg/Exp/F when the (type, method, objective-set) lines
    up (so the existence check finds seeded runs); the constraints are taken from
    the user's config (speed-violation always applied)."""
    model_type, alg = _derive_type_alg(optimization_type, method)
    G, H = _constraints(max_mission_time, min_connectivity)

    key, preset = _find_preset(model_type, alg, objectives)
    if preset is not None:
        model = dict(preset)
        model["G"] = G
        model["H"] = H
        if model_type == "WS" and weights:
            model["Weights"] = dict(weights)
        return key, model

    exp = _synth_exp(objectives)
    if model_type == "WS":
        F = [get_weighted_sum_objective_name_from_objectives(objectives)]
    else:
        F = list(objectives)
    model = {"Type": model_type, "Exp": exp, "Alg": alg, "F": F, "G": G, "H": H}
    if model_type == "WS" and weights:
        model["Weights"] = dict(weights)

    if model_type == "MOO":
        model_key = f"{exp}_MOO_{alg}"
    elif model_type == "WS":
        model_key = f"{exp}_WS"
    else:
        model_key = exp
    return model_key, model


def scenario_name_for(model_dict: dict, scenario_dict: dict) -> str:
    """Build the canonical scenario name (str(PathInfo) with the patched model)."""
    from PathInfo import PathInfo

    info = PathInfo(scenario_dict)
    info.model = model_dict
    return str(info)


def _exists(scenario_name: str) -> bool:
    obj = os.path.join(settings.RESULTS_ROOT, "Objectives", f"{scenario_name}-ObjectiveValues.pkl")
    sol = os.path.join(settings.RESULTS_ROOT, "Solutions", f"{scenario_name}-SolutionObjects.pkl")
    return os.path.isfile(obj) and os.path.isfile(sol)


def check_config(
    optimization_type: str, method: str, objectives: list[str],
    weights: Optional[dict], scenario_dict: dict,
    max_mission_time: Optional[float] = None, min_connectivity: Optional[float] = None,
) -> dict:
    """Existence pre-check: derive the scenario name + model key and report
    whether a matching run already exists in the library."""
    model_key, model_dict = resolve_model(
        optimization_type, method, objectives, weights, max_mission_time, min_connectivity)
    scenario_name = scenario_name_for(model_dict, scenario_dict)
    return {
        "scenario_name": scenario_name,
        "model_key": model_key,
        "exists": _exists(scenario_name),
    }


# ─── Run registry + executor ──────────────────────────────────────────────────

_executor: Optional[ProcessPoolExecutor] = None
_jobs: dict[str, dict] = {}
_lock = threading.Lock()


def _get_executor() -> ProcessPoolExecutor:
    global _executor
    if _executor is None:
        _executor = ProcessPoolExecutor(max_workers=1)
    return _executor


def start_run(
    optimization_type: str, method: str, objectives: list[str],
    weights: Optional[dict], pop_size: int, n_gen: int, seed: int,
    scenario_dict: dict,
    max_mission_time: Optional[float] = None, min_connectivity: Optional[float] = None,
) -> dict:
    """Submit a run to the worker process; returns {run_id, scenario_name,
    model_key, exists}. Raises RunInProgressError if one is already running."""
    from app.optimizer_worker import run_optimization

    model_key, model_dict = resolve_model(
        optimization_type, method, objectives, weights, max_mission_time, min_connectivity)
    scenario_name = scenario_name_for(model_dict, scenario_dict)
    polarities = {o: _POLARITY[o] for o in objectives}
    alg = model_dict["Alg"]

    with _lock:
        if any(not j["future"].done() for j in _jobs.values()):
            raise RunInProgressError("A run is already in progress.")
        run_id = uuid.uuid4().hex[:12]
        run_dir = os.path.join(settings.RESULTS_ROOT, ".runs", run_id)
        os.makedirs(run_dir, exist_ok=True)
        # Seed the status file so the very first poll already shows n_gen
        # (the worker child takes a moment to spawn and write its own).
        with open(os.path.join(run_dir, "status.json"), "w") as fh:
            json.dump({"state": "running", "gen": 0, "n_gen": int(n_gen)}, fh)
        future = _get_executor().submit(
            run_optimization, run_id, model_dict, scenario_dict, alg,
            int(pop_size), int(n_gen), int(seed), run_dir,
            scenario_name, model_key, list(objectives), polarities,
            max_mission_time, min_connectivity,
        )
        _jobs[run_id] = {
            "future": future, "run_dir": run_dir,
            "scenario_name": scenario_name, "model_key": model_key,
            "model_dict": model_dict,
        }
    return {
        "run_id": run_id, "scenario_name": scenario_name,
        "model_key": model_key, "exists": _exists(scenario_name),
    }


def get_status(run_id: str) -> dict:
    """Poll a run: running (with gen X/Y), done (with the front), or failed."""
    job = _jobs.get(run_id)
    if job is None:
        raise RunNotFoundError(f"Unknown run_id {run_id!r}")
    fut = job["future"]
    if fut.done():
        exc = fut.exception()
        if exc is not None:
            return {"state": "failed", "error": str(exc)}
        result = dict(fut.result())
        result["exists_in_library"] = _exists(job["scenario_name"])
        return result
    # still running — read the worker's status file for gen progress + live front
    try:
        with open(os.path.join(job["run_dir"], "status.json")) as fh:
            s = json.load(fh)
        return {
            "state": "running",
            "gen": int(s.get("gen", 0)),
            "n_gen": s.get("n_gen"),
            "best": s.get("best"),
            "live_front": s.get("live_front"),
        }
    except Exception:
        return {"state": "running", "gen": 0, "n_gen": None}


def request_stop(run_id: str) -> dict:
    """Cooperatively cancel a running optimization by dropping a ``cancel`` flag
    file the worker checks each generation. The run stops within a generation or
    two and returns its best-so-far front (a partial result), exactly like a
    completed run. Idempotent: a no-op (``stopping: False``) if already finished."""
    job = _jobs.get(run_id)
    if job is None:
        raise RunNotFoundError(f"Unknown run_id {run_id!r}")
    if job["future"].done():
        return {"run_id": run_id, "stopping": False}
    try:
        open(os.path.join(job["run_dir"], "cancel"), "w").close()
    except OSError:
        pass
    return {"run_id": run_id, "stopping": True}


def save_run(run_id: str, overwrite: bool) -> dict:
    """Persist a finished run into the library (Objectives + Solutions pkls) and
    register its (possibly custom) model so it's browsable. Raises
    AlreadyExistsError if the scenario exists and overwrite is False."""
    import shutil

    job = _jobs.get(run_id)
    if job is None:
        raise RunNotFoundError(f"Unknown run_id {run_id!r}")
    fut = job["future"]
    if not fut.done() or fut.exception() is not None:
        raise RunNotReadyError("Run has not finished successfully.")

    scenario_name = job["scenario_name"]
    if _exists(scenario_name) and not overwrite:
        raise AlreadyExistsError(scenario_name)

    obj_dst = os.path.join(settings.RESULTS_ROOT, "Objectives", f"{scenario_name}-ObjectiveValues.pkl")
    sol_dst = os.path.join(settings.RESULTS_ROOT, "Solutions", f"{scenario_name}-SolutionObjects.pkl")
    os.makedirs(os.path.dirname(obj_dst), exist_ok=True)
    os.makedirs(os.path.dirname(sol_dst), exist_ok=True)
    shutil.copyfile(os.path.join(job["run_dir"], "Objectives.pkl"), obj_dst)
    shutil.copyfile(os.path.join(job["run_dir"], "Solutions.pkl"), sol_dst)

    # Register custom (non-preset) models so the library/selector can resolve them.
    from app import models_registry
    models_registry.register(job["model_key"], job["model_dict"])

    # Bust the selector LRU cache so an overwrite serves the new front.
    try:
        from app.selector_service import _load_selector
        _load_selector.cache_clear()
    except Exception:
        pass

    return {"scenario_name": scenario_name, "model_key": job["model_key"]}
