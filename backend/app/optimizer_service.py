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
import logging
import os
import threading
import time
import uuid
from concurrent.futures import Future, ProcessPoolExecutor
from concurrent.futures.process import BrokenProcessPool
from typing import Optional

logger = logging.getLogger("sar.optimizer")

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


class StorageUnavailableError(Exception):
    """The run's storage could not be written — a full, read-only, or
    wrong-owner Results volume. Surfaced as 503, not an opaque 500."""


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
    # Parity with the other scenario paths: reject traversal / unsafe names before
    # building a path. The abspath containment check below already contains this,
    # so it is defence-in-depth (mirrors library_service._is_safe_scenario_name).
    from app.library_service import _is_safe_scenario_name
    if not _is_safe_scenario_name(scenario_name):
        return None
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


def _rebuild_executor_locked() -> None:
    """Discard a dead pool so the next _get_executor() builds a fresh one.

    A worker death (OOM on a big run, a native crash) breaks the whole pool:
    every in-flight future resolves with BrokenProcessPool and every subsequent
    submit raises it too. Left alone the optimizer is wedged until the container
    restarts — queued runs never start and new /api/optimize calls 500. Setting
    _executor to None makes _get_executor() spawn a replacement on the next
    submit. Caller holds _lock; shutting the dead pool down is best-effort."""
    global _executor
    dead = _executor
    _executor = None
    if dead is not None:
        try:
            dead.shutdown(wait=False)
        except Exception:
            pass


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
    # Snapshot: add_done_callback below runs the callback INLINE when the future
    # is already resolved (a pool that dies between submit() and the callback
    # registration resolves it from the executor's management thread). _lock is
    # an RLock, so that inline _on_run_finished re-enters and its
    # _evict_finished_locked() pops from _jobs — which would raise
    # "dictionary changed size during iteration" on a live view.
    for job in list(_jobs.values()):
        if busy >= settings.OPTIMIZE_WORKERS:
            return
        if not _is_waiting(job):
            continue
        # Peek submit_args (don't pop yet): a broken pool makes submit raise, and
        # the args are needed to retry onto the rebuilt pool.
        try:
            future = _get_executor().submit(*job["submit_args"])
        except BrokenProcessPool:
            # A worker died and poisoned the pool. Rebuild it and retry this
            # submit once onto the fresh pool.
            _rebuild_executor_locked()
            try:
                future = _get_executor().submit(*job["submit_args"])
            except BrokenProcessPool as exc:
                # Still unrecoverable — fail this run cleanly instead of wedging
                # the queue or letting it escape start_run as a 500. A resolved
                # failed future reports "failed" through the normal status path
                # (fut.done() + exception). No done-callback is attached: the
                # future is already resolved, so the callback would recurse into
                # this same function for no gain — the loop below already drives
                # the remaining waiting runs, and start_run bounds _jobs.
                logger.error("run submit failed (pool unrecoverable): %s", exc)
                failed: Future = Future()
                failed.set_exception(exc)
                job.pop("submit_args", None)
                job["future"] = failed
                continue
        job.pop("submit_args", None)
        job["future"] = future
        job["started_at"] = time.time()
        busy += 1
        # Set last: the callback re-enters this function, and it must see the
        # job as dispatched so it isn't submitted twice.
        future.add_done_callback(_on_run_finished)


def _on_run_finished(_future) -> None:
    """A worker freed up — log the outcome and give the slot to whoever is next.
    Runs in the pool's callback thread."""
    with _lock:
        run_id = next(
            (rid for rid, j in _jobs.items() if j.get("future") is _future), None)
        if run_id is not None:
            job = _jobs[run_id]
            dur = time.time() - job.get("started_at", time.time())
            if _future.cancelled():
                logger.info("run %s cancelled after %.1fs", run_id, dur)
            elif _future.exception() is not None:
                logger.error("run %s failed after %.1fs: %s",
                             run_id, dur, _future.exception())
            else:
                logger.info("run %s finished after %.1fs", run_id, dur)
        # Give the freed slot to the next run. Do NOT rebuild the pool here even
        # when this future carries a BrokenProcessPool: a worker death resolves
        # EVERY in-flight future that way, so this callback fires once per dead
        # future, and an unconditional rebuild would tear down a pool an earlier
        # callback already rebuilt — orphaning the run just dispatched onto it.
        # Rebuilding is the submit path's job precisely because it is idempotent:
        # the first submit onto the dead pool (inside _dispatch_locked) rebuilds
        # exactly once, and every later submit in the same lock-held sweep lands
        # on the fresh pool. An empty queue simply lets the dead pool linger until
        # the next start_run's submit rebuilds it.
        _dispatch_locked()
        _evict_finished_locked()


def _evict_finished_locked() -> None:
    """Bound the in-memory _jobs registry. Entries are added in start_run and
    otherwise never removed, so a long-lived server leaks one dict per run
    forever. Keep the newest settings.OPTIMIZE_RUN_KEEP FINISHED (non-pending)
    entries and drop the older ones; pending (waiting/running) jobs are always
    kept. Safe because get_status falls back to _disk_status and export/save fall
    back to _disk_job for any run no longer in _jobs. Caller holds _lock; _jobs
    preserves insertion order, so the slice keeps the most recent."""
    finished = [rid for rid, j in _jobs.items() if not _is_pending(j)]
    for rid in finished[:-settings.OPTIMIZE_RUN_KEEP]:
        _jobs.pop(rid, None)


def _n_gen_from_dir(run_dir: str) -> Optional[int]:
    """Target generation count from a run dir's seeded status file — lets a
    queued run show "0 / N" before its worker starts. run_dir is immutable for
    the life of a job, so this is safe to read without the lock."""
    try:
        with open(os.path.join(run_dir, "status.json")) as fh:
            return json.load(fh).get("n_gen")
    except Exception:
        return None


def _protected_run_ids() -> set:
    """Run ids of every in-flight (waiting or running) job — never purge these."""
    with _lock:
        return {rid for rid, j in _jobs.items() if _is_pending(j)}


_janitor_stop = threading.Event()
_janitor_thread: Optional[threading.Thread] = None


def start_janitor(interval: Optional[int] = None) -> None:
    """Start the background sweeper that bounds .runs on a timer, independent of
    whether new runs are being started. Idempotent; a no-op if the interval is 0."""
    global _janitor_thread
    interval = settings.RUN_PURGE_INTERVAL_SECONDS if interval is None else interval
    if interval <= 0 or _janitor_thread is not None:
        return
    _janitor_stop.clear()

    def _loop() -> None:
        # wait() returns True only when stop is set, so this exits promptly on
        # shutdown instead of sleeping out the interval.
        while not _janitor_stop.wait(interval):
            try:
                _purge_stale_runs(protected=_protected_run_ids())
            except Exception:
                pass  # a sweep must never take the janitor thread down

    _janitor_thread = threading.Thread(target=_loop, name="run-janitor", daemon=True)
    _janitor_thread.start()


def stop_janitor() -> None:
    """Signal the janitor to exit and wait for it (on app shutdown / teardown).

    The join matters: without it a stop/start cycle (an in-process restart, a
    test teardown followed by a fresh TestClient) would have start_janitor's
    ``_janitor_stop.clear()`` un-set the flag while the old thread is still
    parked in ``wait(interval)`` — it would never exit, and every cycle would
    leave another thread sweeping the same tree."""
    global _janitor_thread
    thread = _janitor_thread
    _janitor_stop.set()
    _janitor_thread = None
    if thread is not None and thread.is_alive():
        thread.join(timeout=5)


def shutdown(wait: bool = False) -> None:
    """Tear down the worker executor (on app shutdown / test teardown) so the
    interpreter doesn't block at exit joining a stale pool. Also sweeps .runs one
    last time so a clean stop does not leave the last batch of dirs behind."""
    global _executor
    stop_janitor()
    try:
        _purge_stale_runs(protected=_protected_run_ids())
    except Exception:
        pass
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


def _purge_stale_runs(
    now: Optional[float] = None, protected: Optional[set] = None
) -> list[str]:
    """Bound the temp .runs/ tree. Removes a dir when it is older than
    settings.RUN_TTL_HOURS OR beyond the newest settings.OPTIMIZE_RUN_KEEP — the
    count cap is what actually bounds disk, since age alone leaks under a burst
    or a process that goes quiet. Never touches a run whose id is in *protected*
    (the in-flight runs), and never raises: a sweep failure must not block a run.
    """
    import shutil
    import time as _time
    now = now if now is not None else _time.time()
    # Default to the LIVE in-flight set, not the empty set: the destructive
    # reading must not be the one a caller gets by forgetting an argument.
    protected = _protected_run_ids() if protected is None else protected
    root = os.path.join(settings.RESULTS_ROOT, ".runs")
    cutoff = now - settings.RUN_TTL_HOURS * 3600
    purged: list[str] = []

    # Collect (name, path, mtime) for every candidate dir, newest first.
    candidates: list[tuple[str, str, float]] = []
    try:
        entries = os.listdir(root)
    except OSError:
        return purged
    for name in entries:
        if name in protected:
            continue
        d = os.path.join(root, name)
        try:
            if os.path.isdir(d):
                candidates.append((name, d, os.path.getmtime(d)))
        except OSError:
            continue
    candidates.sort(key=lambda c: c[2], reverse=True)

    keep = settings.OPTIMIZE_RUN_KEEP
    for rank, (name, d, mtime) in enumerate(candidates):
        # Delete if past the age cutoff, or ranked beyond the newest `keep`.
        if mtime < cutoff or rank >= keep:
            shutil.rmtree(d, ignore_errors=True)
            purged.append(name)
    return purged


def _ready_job(run_id: str) -> dict:
    """Return the job for a run that has FINISHED SUCCESSFULLY, or raise.

    The in-memory lookup runs under _lock so the dispatch window (future popped
    to None before the real future is set) cannot be mistaken for 'finished' —
    which would send a reader at a result pickle the worker has not written yet.
    Falls back to an on-disk done run (finished by construction). Raises
    RunNotFoundError if unknown, RunNotReadyError if not finished-ok."""
    with _lock:
        job = _jobs.get(run_id)
        if job is not None:
            fut = job.get("future")
            if (fut is None or not fut.done()
                    or fut.cancelled() or fut.exception() is not None):
                raise RunNotReadyError("Run has not finished successfully.")
            return dict(job)
    disk = _disk_job(run_id)
    if disk is None:
        raise RunNotFoundError(f"Unknown run_id {run_id!r}")
    if _awaiting_result(disk):
        raise RunNotReadyError("Run has not finished successfully.")
    return disk


def serialize_finished_run(run_id: str) -> dict:
    """Serialize a finished run's on-disk artifacts to the Playground JSON schema.
    Reuses playground_export.serialize_run; embeds the full resolved model_key."""
    import numpy as np
    import pandas as pd
    from app.playground_export import serialize_run

    job = _ready_job(run_id)

    run_dir = job["run_dir"]
    # The janitor sweeps .runs on a timer, so a run's dir can vanish between the
    # _ready_job lookup and these reads (or the _jobs entry can outlive the dir).
    # Turn the resulting FileNotFoundError/OSError into a clean 404 instead of
    # letting it escape as an uncaught 500.
    try:
        F_df = pd.read_pickle(os.path.join(run_dir, "Objectives.pkl"))
        raw_solutions = pd.read_pickle(os.path.join(run_dir, "Solutions.pkl"))
    except (FileNotFoundError, OSError) as exc:
        raise RunNotFoundError(
            f"Run {run_id!r} is no longer available (its data was cleaned up)."
        ) from exc
    if int(F_df.shape[0]) < 1:
        raise EmptyRunError("Run found no feasible solutions; nothing to export.")
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

    # Sweep BEFORE taking _lock, in the janitor's shape: the sweep is a listdir
    # plus an rmtree per stale dir, and _lock is what the pool's callback thread
    # and every status poll contend on — holding it across that I/O freezes the
    # whole optimizer subsystem for the duration.
    _purge_stale_runs(protected=_protected_run_ids())

    with _lock:
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
        # A full / read-only / wrong-owner Results volume fails here. Turn the
        # raw OSError into a typed 503 so the operator sees storage as the cause
        # instead of an opaque 500 on every optimize attempt.
        try:
            os.makedirs(run_dir, exist_ok=True)
            # Seed the status file so the very first poll already shows n_gen
            # (the worker child takes a moment to spawn and write its own).
            with open(os.path.join(run_dir, "status.json"), "w") as fh:
                json.dump({"state": "running", "gen": 0, "n_gen": int(n_gen)}, fh)
        except OSError as exc:
            logger.error("run storage unwritable under %s: %s", run_dir, exc)
            raise StorageUnavailableError(
                "The server could not write run storage. Try again shortly."
            ) from exc
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
        # Bound the registry on the path where it GROWS. _on_run_finished also
        # evicts, but that callback is deliberately not attached on the
        # unrecoverable-pool path — so a persistently broken pool would otherwise
        # add one entry per request and never drop one.
        _evict_finished_locked()
        _dispatch_locked()
        position = _queue_position(run_id)
    logger.info(
        "run %s queued (model=%s scenario=%s, position=%s)",
        run_id, model_key, scenario_name, position if position is not None else "running",
    )
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
    done (with the front), cancelled, or failed.

    The job-state classification runs under _lock so it cannot observe a job
    mid-dispatch (submit_args popped, future not yet set) — that window would
    otherwise crash on ``None.cancelled()``. File reads for the running/queued
    progress happen after the lock is released, so a slow disk never stalls
    dispatch."""
    waiting = False
    queue_position = None
    run_dir = None
    with _lock:
        job = _jobs.get(run_id)
        if job is not None:
            if job.get("cancelled"):
                # Left the line before a worker picked it up, so there is no
                # partial front to hand back — distinct from a run that failed.
                return {"state": "cancelled"}
            fut = job.get("future")
            # future is None both while the run waits in line AND during the
            # brief dispatch window; either way it has not started, so report it
            # as queued rather than touching a None future.
            if _is_waiting(job) or fut is None:
                waiting = True
                queue_position = _queue_position(run_id)
                run_dir = job["run_dir"]
            elif fut.cancelled():
                return {"state": "cancelled"}
            elif fut.done():
                exc = fut.exception()
                if exc is not None:
                    # Never surface internal error text: this 200 body bypasses
                    # the frontend's 5xx suppression, so log the real cause
                    # server-side and hand the client a generic message. Reporting
                    # a failure is a read — it must NOT rebuild the pool. A worker
                    # death resolves many futures at once, so many concurrent polls
                    # would race to rebuild and thrash the fresh pool; rebuilding is
                    # the submit path's idempotent job.
                    logger.error("run %s failed: %s", run_id, exc)
                    return {"state": "failed",
                            "error": "The optimization failed. Please try again."}
                result = dict(fut.result())
                result["exists_in_library"] = _exists(job["scenario_name"])
                return _displayify_done(result)
            else:
                run_dir = job["run_dir"]

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

    if waiting:
        return {"state": "queued", "queue_position": queue_position,
                "n_gen": _n_gen_from_dir(run_dir)}

    # on a worker — read its status file for gen progress + live front
    try:
        with open(os.path.join(run_dir, "status.json")) as fh:
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


def _write_run_sibling(scenario_name: str, run_dir: str, sol_dst: str) -> None:
    """Write the run's -AllObjectives.pkl next to its copied pickles.

    Without this a saved run is invisible to the Compare page: the objectives
    endpoint reads ONLY the sibling, precisely so it can never fall back to a
    ~160 MB solution load. Best-effort — a failure here must not fail an
    otherwise-successful save; the backfill script can repair it.

    *sol_dst* is the library's -SolutionObjects.pkl (already written by the
    caller before this runs) — the stamp is taken from THAT file, not the run
    dir's copy, because that is the path the comparison consumer stats against.
    """
    import numpy as np  # noqa: PLC0415
    import pandas as pd  # noqa: PLC0415

    from app.all_objectives import all_objectives_path, write_all_objectives

    sol_src = os.path.join(run_dir, "Solutions.pkl")
    try:
        raw = list(pd.read_pickle(sol_src))
        # SolutionObjects rows can be 1-element numpy arrays (PathUnitTest.py).
        solutions = [s[0] if isinstance(s, np.ndarray) else s for s in raw]
        write_all_objectives(scenario_name, solutions, source_path=sol_dst)
    except Exception:
        # A MISSING sibling is honest — the scenario is skipped by /compare and
        # backfill_all_objectives.py repairs it. A STALE one is not: on an
        # overwrite the pickles have already been replaced, so a leftover
        # sibling from the previous save can still match the new row count and
        # would serve the old run's numbers forever. Remove it.
        try:
            p = all_objectives_path(scenario_name)
            if os.path.isfile(p):
                os.unlink(p)
        except OSError:
            pass
        logger.exception(
            "failed to write -AllObjectives.pkl for %s; the scenario will not "
            "appear in comparisons until backfill_all_objectives.py is run",
            scenario_name)


def save_run(run_id: str, overwrite: bool) -> dict:
    """Persist a finished run into the library (Objectives + Solutions pkls) and
    register its (possibly custom) model so it's browsable. Raises
    AlreadyExistsError if the scenario exists and overwrite is False."""
    import shutil

    job = _ready_job(run_id)

    scenario_name = job["scenario_name"]

    # Refuse to persist a run that found no feasible solutions — an empty front
    # in the library breaks selection/replay endpoints with 500s.
    src_obj = os.path.join(job["run_dir"], "Objectives.pkl")
    # The janitor sweeps .runs on a timer and protects only IN-FLIGHT runs, so a
    # finished run's dir can vanish between _ready_job and here. That must read
    # as "gone", not as "found nothing" — the old bare `except Exception: 0` told
    # users their optimization was infeasible when its data had been cleaned up.
    # Mirrors serialize_finished_run's translation of the same window.
    try:
        import pandas as pd
        n_solutions = int(pd.read_pickle(src_obj).shape[0])
    except FileNotFoundError as exc:
        raise RunNotFoundError(
            f"Run {run_id!r} is no longer available (its data was cleaned up)."
        ) from exc
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
    # Same sweep window as above — a dir removed between the read and the copies
    # must be a clean 404, not an uncaught 500 out of shutil.
    try:
        shutil.copyfile(os.path.join(job["run_dir"], "Objectives.pkl"), obj_dst)
        shutil.copyfile(os.path.join(job["run_dir"], "Solutions.pkl"), sol_dst)
    except FileNotFoundError as exc:
        raise RunNotFoundError(
            f"Run {run_id!r} is no longer available (its data was cleaned up)."
        ) from exc

    _write_run_sibling(scenario_name, job["run_dir"], sol_dst)

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

    # Also bust the library list/grid memos: they key on the Objectives/ dir
    # mtime, which an in-place OVERWRITE of an existing pickle may not move, so
    # without this an overwrite would keep serving stale list/grid stats.
    try:
        from app.library_service import _bust_scenario_memos
        _bust_scenario_memos()
    except Exception:
        pass

    return {"scenario_name": to_display(scenario_name),
            "model_key": to_display(job["model_key"])}
