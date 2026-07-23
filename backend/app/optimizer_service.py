"""
Optimizer service: synthesizes a model dict from a user's config, derives the
scenario name, runs the optimization on a ProcessPoolExecutor (background +
poll), and (later) saves a finished run to the library.

Runs queue rather than being refused: the pool holds settings.OPTIMIZE_WORKERS
child processes, and anything submitted while they are all busy waits its turn
and reports its place in line. Two ceilings keep that line honest —
OPTIMIZE_QUEUE_MAX bounds it overall, OPTIMIZE_MAX_PER_CLIENT bounds any one
caller's share — and both surface as 409 rather than an unbounded backlog.

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
from app.model_aliases import to_display, to_storage

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

class QueueFullError(Exception):
    """No free slot: running + waiting runs already fill settings.OPTIMIZE_QUEUE_MAX."""


class ClientLimitError(Exception):
    """This client already holds settings.OPTIMIZE_MAX_PER_CLIENT runs."""


class RunNotFoundError(Exception):
    pass


class RunNotReadyError(Exception):
    pass


class AlreadyExistsError(Exception):
    """Scenario already in the library and overwrite was not requested."""


class EmptyRunError(Exception):
    """The run found no feasible solutions, so there is nothing to save."""


# ─── Model synthesis ──────────────────────────────────────────────────────────

def _derive_type_alg(optimization_type: str, method: str) -> tuple[str, str]:
    """(optimization_type SOO|MOO, method) → (model Type, Alg)."""
    if optimization_type == "SOO":
        return ("WS", "GA") if method == "WS" else ("SOO", "GA")
    return ("MOO", method)  # NSGA2 / NSGA3 / MOEAD


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


def _constraints(max_mission_time, min_connectivity, max_mean_tbv=None) -> tuple[list, list]:
    """Build (G, H) from the user's constraint config. The speed-violation
    constraint is ALWAYS in H (required for drone-path interpolation)."""
    G: list = []
    if max_mission_time is not None:
        G.append("Max Mission Time")
    if min_connectivity is not None:
        G.append("Min Percentage Connectivity")
    if max_mean_tbv is not None:
        G.append("Max Mean TBV Ceiling")
    H = ["Path Speed Violations as Constraint"]
    return G, H


def resolve_model(
    optimization_type: str,
    method: str,
    objectives: list[str],
    weights: Optional[dict] = None,
    max_mission_time: Optional[float] = None,
    min_connectivity: Optional[float] = None,
    max_mean_tbv: Optional[float] = None,
) -> tuple[str, dict]:
    """Synthesize (model_key, model_dict) from the user's config. Matches an
    existing preset's Type/Alg/Exp/F when the (type, method, objective-set) lines
    up (so the existence check finds seeded runs); the constraints are taken from
    the user's config (speed-violation always applied)."""
    model_type, alg = _derive_type_alg(optimization_type, method)
    G, H = _constraints(max_mission_time, min_connectivity, max_mean_tbv)

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


def read_run_config(scenario_name: str) -> Optional[dict]:
    """Return the persisted RunConfig sidecar for a mission, or None if absent."""
    # Display (…TCDV…) names from the API must hit the real …TCDT… sidecars.
    scenario_name = to_storage(scenario_name)
    path = os.path.join(settings.RESULTS_ROOT, "Metadata", f"{scenario_name}.json")
    # Defence-in-depth: never read outside RESULTS_ROOT (mirrors library_service).
    root = os.path.abspath(settings.RESULTS_ROOT) + os.sep
    if not os.path.abspath(path).startswith(root) or not os.path.isfile(path):
        return None
    try:
        with open(path) as fh:
            return json.load(fh)
    except Exception:
        return None


def check_config(
    optimization_type: str, method: str, objectives: list[str],
    weights: Optional[dict], scenario_dict: dict,
    max_mission_time: Optional[float] = None, min_connectivity: Optional[float] = None,
    max_mean_tbv: Optional[float] = None,
) -> dict:
    """Existence pre-check: derive the scenario name + model key and report
    whether a matching run already exists in the library."""
    model_key, model_dict = resolve_model(
        optimization_type, method, objectives, weights, max_mission_time, min_connectivity,
        max_mean_tbv)
    scenario_name = scenario_name_for(model_dict, scenario_dict)
    return {
        "scenario_name": to_display(scenario_name),
        "model_key": to_display(model_key),
        "exists": _exists(scenario_name),
        "seeded": model_key in AVAILABLE_MODELS and _exists(scenario_name),
    }


# ─── Run registry + executor ──────────────────────────────────────────────────
# This registry is PER-PROCESS state: the API must be served by exactly ONE
# server worker (uvicorn --workers 1, the default). With N workers, a run
# started in one worker 404s when polled/stopped from another (disk recovery in
# _disk_status only covers finished runs), and the single-run guard and rate
# limiter fragment into N independent copies. See README "Production notes".

_executor: Optional[ProcessPoolExecutor] = None
_jobs: dict[str, dict] = {}
# Re-entrant: a future that is already resolved runs its done-callback in the
# calling thread, so _dispatch_locked can be re-entered while it holds the lock.
_lock = threading.RLock()


def _get_executor() -> ProcessPoolExecutor:
    global _executor
    if _executor is None:
        _executor = ProcessPoolExecutor(max_workers=settings.OPTIMIZE_WORKERS)
    return _executor


def _is_waiting(job: dict) -> bool:
    """True while a run is in the line, not yet handed to a pool worker."""
    return job.get("future") is None and not job.get("cancelled") and "submit_args" in job


def _holds_worker(job: dict) -> bool:
    """True while a run occupies one of the pool's workers."""
    fut = job.get("future")
    return fut is not None and not fut.done()


def _is_pending(job: dict) -> bool:
    """True while a run still occupies a queue slot — waiting or running."""
    return _is_waiting(job) or _holds_worker(job)


def _awaiting_result(job: dict) -> bool:
    """True when a job has no successful result to read yet — still waiting,
    still running, cancelled, or failed. A job rebuilt from disk by _disk_job
    carries no submit_args and is finished by construction."""
    if job.get("cancelled"):
        return True
    fut = job.get("future")
    if fut is None:
        return "submit_args" in job
    return not fut.done() or fut.cancelled() or fut.exception() is not None


def _queue_position(run_id: str) -> Optional[int]:
    """1-based place in the waiting line, or None if the run is not waiting.

    Position 1 means "next to start". Runs already on a worker are not in the
    line, so they are skipped rather than counted ahead.
    """
    job = _jobs.get(run_id)
    if job is None or not _is_waiting(job):
        return None
    ahead = 0
    # _jobs preserves submission order, which is the order runs are dispatched.
    for rid, other in _jobs.items():
        if rid == run_id:
            break
        if _is_waiting(other):
            ahead += 1
    return ahead + 1


def _dispatch_locked() -> None:
    """Hand waiting runs to the pool while it has free workers. Caller holds _lock.

    The pool's own queue is deliberately left empty: ProcessPoolExecutor buffers
    extra work items and marks their futures ``running()`` before any worker
    picks them up, which would make both the wait count and every position
    wrong. Submitting only as many runs as there are workers keeps the
    "running" state honest.
    """
    busy = sum(1 for j in _jobs.values() if _holds_worker(j))
    for job in _jobs.values():
        if busy >= settings.OPTIMIZE_WORKERS:
            return
        if not _is_waiting(job):
            continue
        future = _get_executor().submit(*job.pop("submit_args"))
        job["future"] = future
        busy += 1
        # Set last: the callback re-enters this function, and it must see the
        # job as dispatched so it isn't submitted twice.
        future.add_done_callback(_on_run_finished)


def _on_run_finished(_future) -> None:
    """A worker freed up — give it to whoever is next. Runs in the pool's thread."""
    with _lock:
        _dispatch_locked()


def _seeded_n_gen(job: dict) -> Optional[int]:
    """The run's target generation count, read from the status file seeded at
    submit time — lets a queued run show "0 / N" before its worker starts."""
    try:
        with open(os.path.join(job["run_dir"], "status.json")) as fh:
            return json.load(fh).get("n_gen")
    except Exception:
        return None


def shutdown(wait: bool = False) -> None:
    """Tear down the worker executor (on app shutdown / test teardown) so the
    interpreter doesn't block at exit joining a stale pool."""
    global _executor
    if _executor is not None:
        try:
            _executor.shutdown(wait=wait, cancel_futures=True)
        except TypeError:  # cancel_futures added in 3.9
            _executor.shutdown(wait=wait)
        _executor = None


# ─── On-disk recovery (survives a backend restart that wiped _jobs) ───────────

import re as _re

_RUN_ID_RE = _re.compile(r"^[0-9a-fA-F]{6,32}$")


def _run_dir(run_id: str) -> Optional[str]:
    """Validated path to a run dir, or None if the id is unsafe/absent."""
    if not _RUN_ID_RE.match(run_id):
        return None
    d = os.path.join(settings.RESULTS_ROOT, ".runs", run_id)
    return d if os.path.isdir(d) else None


def _disk_status(run_id: str) -> Optional[dict]:
    """Reconstruct a *finished* run's status from its on-disk run dir, or None.

    Only DONE runs are recoverable — a run that was mid-flight when the process
    died left a stale "running" status with no live worker, so it is treated as
    gone (404) rather than a stuck spinner."""
    d = _run_dir(run_id)
    if d is None:
        return None
    try:
        with open(os.path.join(d, "status.json")) as fh:
            s = json.load(fh)
    except Exception:
        return None
    if s.get("state") != "done":
        return None
    return s


def _disk_job(run_id: str) -> Optional[dict]:
    """Reconstruct a job dict (for save_run) from a finished run's meta.json."""
    d = _run_dir(run_id)
    if d is None:
        return None
    try:
        with open(os.path.join(d, "meta.json")) as fh:
            meta = json.load(fh)
    except Exception:
        return None
    return {
        "future": None,
        "run_dir": d,
        "scenario_name": meta.get("scenario_name"),
        "model_key": meta.get("model_key"),
        "model_dict": meta.get("model_dict"),
    }


def _purge_stale_runs(now: Optional[float] = None) -> list[str]:
    """Remove temp .runs/<id> dirs older than settings.RUN_TTL_HOURS. Best-effort:
    never raises (a sweep failure must not block a new run)."""
    import shutil
    import time as _time
    now = now if now is not None else _time.time()
    root = os.path.join(settings.RESULTS_ROOT, ".runs")
    cutoff = now - settings.RUN_TTL_HOURS * 3600
    purged: list[str] = []
    try:
        entries = os.listdir(root)
    except OSError:
        return purged
    for name in entries:
        d = os.path.join(root, name)
        try:
            if os.path.isdir(d) and os.path.getmtime(d) < cutoff:
                shutil.rmtree(d, ignore_errors=True)
                purged.append(name)
        except OSError:
            continue
    return purged


def serialize_finished_run(run_id: str) -> dict:
    """Serialize a finished run's on-disk artifacts to the Playground JSON schema.
    Reuses playground_export.serialize_run; embeds the full resolved model_key."""
    import numpy as np
    import pandas as pd
    from app.playground_export import serialize_run

    job = _jobs.get(run_id) or _disk_job(run_id)
    if job is None:
        raise RunNotFoundError(f"Unknown run_id {run_id!r}")
    if _awaiting_result(job):
        raise RunNotReadyError("Run has not finished successfully.")

    run_dir = job["run_dir"]
    F_df = pd.read_pickle(os.path.join(run_dir, "Objectives.pkl"))
    if int(F_df.shape[0]) < 1:
        raise EmptyRunError("Run found no feasible solutions; nothing to export.")
    raw_solutions = pd.read_pickle(os.path.join(run_dir, "Solutions.pkl"))
    # Normalise rows: SolutionObjects rows can be 1-element numpy arrays
    # (mirrors selector_service._load_selector).
    solutions = [s[0] if isinstance(s, np.ndarray) else s for s in list(raw_solutions)]

    run_config = {}
    cfg_path = os.path.join(run_dir, "config.json")
    if os.path.isfile(cfg_path):
        with open(cfg_path) as fh:
            run_config = json.load(fh)

    return serialize_run(solutions, F_df, job["model_dict"], run_config,
                          model_key=to_display(job["model_key"]))


def start_run(
    optimization_type: str, method: str, objectives: list[str],
    weights: Optional[dict], pop_size: int, n_gen: int, seed: int,
    scenario_dict: dict,
    max_mission_time: Optional[float] = None, min_connectivity: Optional[float] = None,
    max_mean_tbv: Optional[float] = None,
    gen_strategy: str = "fixed",
    early_stop_patience: int = 10, early_stop_threshold: float = 0.10,
    client_key: str = "-",
) -> dict:
    """Queue a run on the worker pool; returns {run_id, scenario_name, model_key,
    exists, seeded, queued, queue_position}.

    The run starts immediately if a pool worker is free and waits its turn
    otherwise. Raises QueueFullError when every slot is taken, or
    ClientLimitError when *client_key* already holds its share of them."""
    from app.optimizer_worker import run_optimization

    model_key, model_dict = resolve_model(
        optimization_type, method, objectives, weights, max_mission_time, min_connectivity,
        max_mean_tbv)
    scenario_name = scenario_name_for(model_dict, scenario_dict)
    polarities = {o: _POLARITY[o] for o in objectives}
    alg = model_dict["Alg"]

    with _lock:
        _purge_stale_runs()
        pending = [j for j in _jobs.values() if _is_pending(j)]
        if len(pending) >= settings.OPTIMIZE_QUEUE_MAX:
            raise QueueFullError(
                "The optimization queue is full. Wait for a run to finish and try again."
            )
        mine = sum(1 for j in pending if j.get("client_key") == client_key)
        if mine >= settings.OPTIMIZE_MAX_PER_CLIENT:
            raise ClientLimitError(
                f"You already have {mine} optimization"
                f"{'s' if mine != 1 else ''} queued or running. "
                "Wait for one to finish before starting another."
            )
        run_id = uuid.uuid4().hex[:12]
        run_dir = os.path.join(settings.RESULTS_ROOT, ".runs", run_id)
        os.makedirs(run_dir, exist_ok=True)
        # Seed the status file so the very first poll already shows n_gen
        # (the worker child takes a moment to spawn and write its own).
        with open(os.path.join(run_dir, "status.json"), "w") as fh:
            json.dump({"state": "running", "gen": 0, "n_gen": int(n_gen)}, fh)
        _jobs[run_id] = {
            "future": None, "run_dir": run_dir,
            "scenario_name": scenario_name, "model_key": model_key,
            "model_dict": model_dict, "client_key": client_key,
            "submit_args": (
                run_optimization, run_id, model_dict, scenario_dict, alg,
                int(pop_size), int(n_gen), int(seed), run_dir,
                scenario_name, model_key, list(objectives), polarities,
                max_mission_time, min_connectivity, max_mean_tbv,
                gen_strategy, int(early_stop_patience), float(early_stop_threshold),
            ),
        }
        _dispatch_locked()
        position = _queue_position(run_id)
    return {
        "run_id": run_id, "scenario_name": to_display(scenario_name),
        "model_key": to_display(model_key), "exists": _exists(scenario_name),
        "seeded": model_key in AVAILABLE_MODELS and _exists(scenario_name),
        "queued": position is not None, "queue_position": position,
    }


def _displayify_done(result: dict) -> dict:
    """Rewrite a done payload's front identifiers to the display form (TCDT→TCDV).
    The worker/job state keeps storage names; only the API response is aliased."""
    front = result.get("front")
    if isinstance(front, dict):
        front = dict(front)
        front["scenario"] = to_display(front.get("scenario"))
        front["model_key"] = to_display(front.get("model_key"))
        result["front"] = front
    return result


def get_status(run_id: str) -> dict:
    """Poll a run: queued (with its place in line), running (with gen X/Y),
    done (with the front), cancelled, or failed."""
    job = _jobs.get(run_id)
    if job is None:
        # Recover a finished run from disk after a restart that wiped _jobs.
        disk = _disk_status(run_id)
        if disk is not None:
            result = dict(disk)
            dj = _disk_job(run_id)
            scen = dj["scenario_name"] if dj else None
            result["exists_in_library"] = _exists(scen) if scen else False
            return _displayify_done(result)
        raise RunNotFoundError(f"Unknown run_id {run_id!r}")
    if job.get("cancelled"):
        # Left the line before a worker picked it up, so there is no partial
        # front to hand back — distinct from a run that failed.
        return {"state": "cancelled"}
    if _is_waiting(job):
        return {"state": "queued", "queue_position": _queue_position(run_id),
                "n_gen": _seeded_n_gen(job)}
    fut = job["future"]
    if fut.cancelled():
        return {"state": "cancelled"}
    if fut.done():
        exc = fut.exception()
        if exc is not None:
            return {"state": "failed", "error": str(exc)}
        result = dict(fut.result())
        result["exists_in_library"] = _exists(job["scenario_name"])
        return _displayify_done(result)
    # on a worker — read its status file for gen progress + live front
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
    """Cancel a run, by whichever route its state allows.

    A run that is still waiting is dropped from the pool outright and ends up
    ``cancelled`` — no worker ever touched it, so there is nothing to salvage.
    A run already executing is cancelled cooperatively via a ``cancel`` flag
    file the worker checks each generation: it stops within a generation or two
    and returns its best-so-far front, exactly like a completed run.

    Idempotent: a no-op (``stopping: False``) if already finished."""
    job = _jobs.get(run_id)
    if job is None:
        # A finished run recovered from disk is already done — nothing to stop.
        if _disk_status(run_id) is not None:
            return {"run_id": run_id, "stopping": False}
        raise RunNotFoundError(f"Unknown run_id {run_id!r}")
    with _lock:
        if job.get("cancelled"):
            return {"run_id": run_id, "stopping": False}
        # Still in line: drop it before it ever reaches a worker. A cancel file
        # would never be read, since no worker opens the run dir.
        if _is_waiting(job):
            job["cancelled"] = True
            job.pop("submit_args", None)
            _dispatch_locked()  # its slot just freed up
            return {"run_id": run_id, "stopping": True}
    fut = job["future"]
    if fut.done():
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
        # Recover a finished run from its on-disk meta after a restart.
        job = _disk_job(run_id)
        if job is None:
            raise RunNotFoundError(f"Unknown run_id {run_id!r}")
    if _awaiting_result(job):
        raise RunNotReadyError("Run has not finished successfully.")

    scenario_name = job["scenario_name"]

    # Refuse to persist a run that found no feasible solutions — an empty front
    # in the library breaks selection/replay endpoints with 500s.
    src_obj = os.path.join(job["run_dir"], "Objectives.pkl")
    try:
        import pandas as pd
        n_solutions = int(pd.read_pickle(src_obj).shape[0])
    except Exception:
        n_solutions = 0
    if n_solutions < 1:
        raise EmptyRunError("Run found no feasible solutions; nothing to save.")

    if _exists(scenario_name) and not overwrite:
        raise AlreadyExistsError(scenario_name)

    obj_dst = os.path.join(settings.RESULTS_ROOT, "Objectives", f"{scenario_name}-ObjectiveValues.pkl")
    sol_dst = os.path.join(settings.RESULTS_ROOT, "Solutions", f"{scenario_name}-SolutionObjects.pkl")
    os.makedirs(os.path.dirname(obj_dst), exist_ok=True)
    os.makedirs(os.path.dirname(sol_dst), exist_ok=True)
    shutil.copyfile(os.path.join(job["run_dir"], "Objectives.pkl"), obj_dst)
    shutil.copyfile(os.path.join(job["run_dir"], "Solutions.pkl"), sol_dst)

    # Copy the RunConfig sidecar (present for worker-produced runs) into the library.
    cfg_src = os.path.join(job["run_dir"], "config.json")
    if os.path.isfile(cfg_src):
        meta_dst = os.path.join(settings.RESULTS_ROOT, "Metadata", f"{scenario_name}.json")
        os.makedirs(os.path.dirname(meta_dst), exist_ok=True)
        shutil.copyfile(cfg_src, meta_dst)

    # Register custom (non-preset) models so the library/selector can resolve them.
    from app import models_registry
    models_registry.register(job["model_key"], job["model_dict"])

    # Bust the selector LRU cache so an overwrite serves the new front.
    try:
        from app.selector_service import _load_selector
        _load_selector.cache_clear()
    except Exception:
        pass

    return {"scenario_name": to_display(scenario_name),
            "model_key": to_display(job["model_key"])}
