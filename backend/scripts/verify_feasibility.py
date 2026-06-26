"""Local FEASIBILITY probe for the missing optimizer scenarios.

Sibling to `verify_convergence.py`, but answers a different, cheaper question:
*under the current GA parameters, can each missing scenario reach feasibility at
all?* — the gate before paying for the full-length runs on EC2.

It runs each cell with the REAL worker build (same operators, POP=300, seed=1,
seeded constraint thresholds) and BREAKS OUT EARLY the moment a positive signal
appears, per run type:

  SOO / WS (GA)   stop at the first generation with >=1 FEASIBLE solution
                  (constraint violation CV <= 0).  -> FEASIBLE
  MOO (NSGA2/3)   stop at the first generation whose FEASIBLE non-dominated
                  front (algorithm.opt, filtered CV <= 0) has >= K points
                  ("a Pareto front forming").       -> FEASIBLE

A generation CAP bounds every run; reaching it is the negative signal:

  FEASIBLE     the break-out criterion was met before the cap.
  INFEASIBLE   the cap was reached and NO feasible solution ever appeared.
  THIN         (MOO only) feasible solutions appeared but the front never grew
               to K within the cap — feasible, but the front is still immature.
  CRASH        the run raised.

Feasibility for these constrained models onsets LATE (empirically ~gen 60-100;
the first generations are entirely infeasible), which is why the default cap is
500 rather than the convergence probe's tens-of-gens.

Faithful to the seeded runs: model dicts come from AVAILABLE_MODELS (their G/H
encode which constraints apply), pop_size 300, seed 1, max_mission_time 3600,
min_connectivity 0.5. No artifacts are written (feasibility only); results stream
to backend/scripts/verify_results/feasibility.json (resumable).

Usage (from repo root):
    .venv/bin/python backend/scripts/verify_feasibility.py list
    .venv/bin/python backend/scripts/verify_feasibility.py run
    .venv/bin/python backend/scripts/verify_feasibility.py run --cap 500 --k 3
"""
from __future__ import annotations

import os

# Pin BLAS/OpenMP to one thread per worker BEFORE numpy loads, so N parallel
# optimizer processes don't oversubscribe cores with nested thread pools.
for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
           "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(_v, "1")

import argparse
import itertools
import json
import math
import multiprocessing
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed

# Path bootstrap: runnable as `python backend/scripts/verify_feasibility.py`.
_BACKEND = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _BACKEND not in sys.path:
    sys.path.insert(0, _BACKEND)
import app.rootpath  # noqa: F401,E402  (side-effect: repo root on sys.path)


# ─── Scope ─────────────────────────────────────────────────────────────────────
# GA / single-objective models first (cheap first-feasible probe), MOO last. The
# task queue is also stable-sorted GA-before-MOO in main(), so the SOO/WS cells
# (incl. CONN) clear before any slow NSGA2 cell starts consuming a worker.
MODELS = ["MTSP", "CONN", "TCD_WS", "TCDT_WS", "TCD_MOO_NSGA2", "TCDT_MOO_NSGA2"]
DRONES = [4, 8, 12, 16]
COMM = ["2", "sqrt(8)", "4"]
NV = [1, 2, 3]
COMM_VAL = {"2": 2.0, "sqrt(8)": 2.0 * math.sqrt(2), "4": 4.0}

# Seeded recipe (from Results/Metadata — uniform across the library).
POP = 300
SEED = 1
MAX_MISSION_TIME = 3600.0
MIN_CONNECTIVITY = 0.5

# Probe knobs (overridable on the CLI).
DEFAULT_CAP = 500          # generation ceiling; reaching it => INFEASIBLE / THIN
DEFAULT_FRONT_K = 3        # MOO: feasible non-dominated points = "front forming"
FEAS_EPS = 1e-9            # CV <= FEAS_EPS counts as feasible


def cell_key(m, d, c, nv):
    return f"feas|{m}|d{d}|r{c}|nv{nv}"


def _scenario_dict(drones, comm_label, nv):
    """The seeded scenario template (matches PathInfo defaults exactly)."""
    return {
        "grid_size": 8, "cell_side_length": 50,
        "number_of_drones": drones, "max_drone_speed": 2.5,
        "comm_cell_range": COMM_VAL[comm_label], "n_visits": nv,
        "target_positions": [12], "th": 0.9, "detection_probability": 0.7,
    }


# ─── Enumerate the missing cells (same source the API/matrix uses) ─────────────

def _comm_label(row):
    lbl = row.get("comm_range")
    if lbl in COMM:
        return lbl
    v = row.get("comm_range_value")
    if v is not None:
        for cand in COMM:
            if abs(v - COMM_VAL[cand]) < 1e-6:
                return cand
    return lbl


def enumerate_cells():
    """Return the list of missing (model, drones, comm, n_visits) cells in scope."""
    from app.library_service import model_grid
    full = list(itertools.product(DRONES, COMM, NV))
    missing = []
    for m in MODELS:
        grid = model_grid(m) or {"scenarios": []}
        present = {(r.get("number_of_drones"), _comm_label(r), r.get("n_visits"))
                   for r in grid["scenarios"]}
        missing += [(m, d, c, nv) for (d, c, nv) in full if (d, c, nv) not in present]
    return missing


# ─── Run one cell (picklable; executes in a worker process) ────────────────────

def run_cell(task):
    """Run one optimization, breaking out at first feasibility / front-forming.

    Returns a verdict dict. Builds the algorithm DIRECTLY from the worker's
    shared operators/helpers (never run_optimization — its early-stop is a
    convergence plateau, not the first-feasibility signal we want here)."""
    import numpy as np
    from pymoo.optimize import minimize
    from pymoo.core.callback import Callback
    from pymoo.core.duplicate import NoDuplicateElimination

    from PathSampling import PathSampling
    from PathMutation import PathMutation
    from PathCrossover import PathCrossover
    from PathRepair import PathRepair
    from PathInfo import PathInfo
    from PathProblem import PathProblem
    from PathOptimizationModel import AVAILABLE_MODELS
    from app.optimizer_worker import _MUTATION_CONFIG, _build_algorithm

    mk = task["model"]
    d, c, nv = task["drones"], task["comm"], task["nv"]
    cap, k = task["cap"], task["k"]
    seed = int(task.get("seed", SEED))
    key = cell_key(mk, d, c, nv)

    model = dict(AVAILABLE_MODELS[mk])
    is_moo = model["Type"] == "MOO"
    base = {"key": key, "model": mk, "drones": d, "comm": c, "nv": nv,
            "type": model["Type"], "alg": model["Alg"], "is_moo": is_moo,
            "cap": cap, "k": k, "seed": seed}

    # Mutable state shared with the callback (callback lives in this process only).
    st = {"first": None, "stop": None, "front": 0, "maxfront": 0}

    def _feasible_count(cv, n):
        """How many of n individuals satisfy all constraints (CV <= eps)."""
        if cv is None:
            return n
        cv = np.asarray(cv, dtype=float)
        mask = (cv <= FEAS_EPS).all(axis=1) if cv.ndim > 1 else (cv <= FEAS_EPS)
        return int(mask.sum())

    class _FeasStop(Callback):
        def notify(self, algorithm):
            gen = int(algorithm.n_gen or 0)
            pop = algorithm.pop
            n_feas_pop = _feasible_count(pop.get("CV"), len(pop))
            if st["first"] is None and n_feas_pop >= 1:
                st["first"] = gen
            if is_moo:
                opt = algorithm.opt
                front = _feasible_count(opt.get("CV"), len(opt)) if (opt is not None and len(opt)) else 0
                st["front"] = front
                st["maxfront"] = max(st["maxfront"], front)
                if front >= k and st["stop"] is None:  # record the FIRST trigger gen
                    st["stop"] = gen
                    algorithm.termination.terminate()
            else:
                st["front"] = n_feas_pop
                st["maxfront"] = max(st["maxfront"], n_feas_pop)
                if n_feas_pop >= 1 and st["stop"] is None:
                    st["stop"] = gen
                    algorithm.termination.terminate()

    operators = dict(
        sampling=PathSampling(),
        mutation=PathMutation(_MUTATION_CONFIG),
        crossover=PathCrossover(prob=0.9, ox_prob=1.0, n_offsprings=2),
        repair=PathRepair(),
        eliminate_duplicates=NoDuplicateElimination(),
    )
    info = PathInfo(_scenario_dict(d, c, nv))
    info.model = model
    info.max_mission_time_constraint = MAX_MISSION_TIME
    info.min_connectivity_constraint = MIN_CONNECTIVITY

    t0 = time.perf_counter()
    try:
        algorithm = _build_algorithm(model["Alg"], POP, len(model["F"]), seed, operators)
        res = minimize(problem=PathProblem(info), algorithm=algorithm,
                       termination=("n_gen", cap), seed=seed,
                       save_history=False, verbose=False, callback=_FeasStop())
    except Exception as e:
        import traceback
        return {**base, "status": "CRASH", "error": repr(e),
                "trace": traceback.format_exc()[-1500:],
                "secs": round(time.perf_counter() - t0, 1)}

    secs = round(time.perf_counter() - t0, 1)
    stop_gen = st["stop"] if st["stop"] is not None else \
        min(cap, int(getattr(getattr(res, "algorithm", None), "n_gen", cap) or cap))

    if st["stop"] is not None:
        status = "FEASIBLE"
    elif is_moo and st["first"] is not None:
        status = "THIN"            # feasible appeared, front never reached k by cap
    else:
        status = "INFEASIBLE"      # cap reached, never feasible

    return {**base, "status": status, "first_feasible_gen": st["first"],
            "stop_gen": stop_gen, "front_size": st["maxfront"], "secs": secs}


# ─── Pool driver (incremental + resumable) ─────────────────────────────────────

def _load_done(path):
    if os.path.isfile(path):
        try:
            return {r["key"]: r for r in json.load(open(path))}
        except Exception:
            return {}
    return {}


def _save(path, results):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(list(results.values()), fh, indent=2)
    os.replace(tmp, path)


def _run_pool(tasks, out_path, workers):
    results = _load_done(out_path)
    todo = [t for t in tasks
            if cell_key(t["model"], t["drones"], t["comm"], t["nv"]) not in results]
    print(f"  {len(results)} cached, {len(todo)} to run, {workers} workers", flush=True)
    if not todo:
        return results
    with ProcessPoolExecutor(max_workers=workers) as ex:
        futs = [ex.submit(run_cell, t) for t in todo]
        done = 0
        for fut in as_completed(futs):
            r = fut.result()
            results[r["key"]] = r
            _save(out_path, results)
            done += 1
            print(f"  [{done}/{len(todo)}] {r['status']:10s} {r['key']:34s} "
                  f"first={r.get('first_feasible_gen')} stop@{r.get('stop_gen')} "
                  f"front={r.get('front_size')} {r.get('secs')}s", flush=True)
            if r["status"] == "CRASH":
                print(f"        {r.get('error', '')[:200]}", flush=True)
    return results


def _workers(cap=None):
    w = max(1, multiprocessing.cpu_count() - 2)
    return min(w, cap) if cap else w


# ─── Main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["list", "run"])
    ap.add_argument("--out", default=os.path.join(_BACKEND, "scripts", "verify_results"))
    ap.add_argument("--cap", type=int, default=DEFAULT_CAP, help="generation ceiling")
    ap.add_argument("--k", type=int, default=DEFAULT_FRONT_K, help="MOO front-forming size")
    ap.add_argument("--workers", type=int, default=0, help="0 = auto (cpu-2)")
    args = ap.parse_args()

    missing = enumerate_cells()
    print(f"Scope: {MODELS}")
    print(f"Missing cells: {len(missing)}   cap={args.cap}  MOO front K={args.k}")
    per = {m: sum(1 for x in missing if x[0] == m) for m in MODELS}
    for m in MODELS:
        print(f"   {m:16s} {per[m]:2d} cells")

    if args.mode == "list":
        for (m, d, c, nv) in missing:
            print(f"   {m:16s} d={d:2d} r={c:7s} nv={nv}")
        return

    tasks = [dict(model=m, drones=d, comm=c, nv=nv, cap=args.cap, k=args.k)
             for (m, d, c, nv) in missing]
    tasks.sort(key=lambda t: "_MOO_" in t["model"])  # GA/SOO/WS first, MOO (NSGA2/3) last
    out = os.path.join(args.out, "feasibility.json")
    print("\n=== Feasibility probe (break at first feasible / front forming) ===", flush=True)
    res = _run_pool(tasks, out, args.workers or _workers())

    # ── Summary ────────────────────────────────────────────────────────────────
    order = ["FEASIBLE", "THIN", "INFEASIBLE", "CRASH"]
    print("\n── Summary by model ──")
    for m in MODELS:
        rows = [r for r in res.values() if r["model"] == m]
        counts = {s: sum(1 for r in rows if r["status"] == s) for s in order}
        tail = "  ".join(f"{s}={counts[s]}" for s in order if counts[s])
        print(f"   {m:16s} {len(rows):2d} cells | {tail}")
    total = {s: sum(1 for r in res.values() if r["status"] == s) for s in order}
    print("\n── Totals ──  " + "  ".join(f"{s}={total[s]}" for s in order))
    flagged = [r for r in res.values() if r["status"] in ("INFEASIBLE", "CRASH")]
    if flagged:
        print(f"\n{len(flagged)} cell(s) need attention before the EC2 batch:")
        for r in sorted(flagged, key=lambda x: x["key"]):
            print(f"   {r['status']:10s} {r['key']}  "
                  f"first={r.get('first_feasible_gen')} stop@{r.get('stop_gen')}")
    else:
        print("\nAll scope cells reached feasibility within the cap. Safe to run the batch.")


if __name__ == "__main__":
    main()
