"""
EC2 batch runner — computes all missing cells (6 models × 36 grid) with K seeds,
merges results (merged non-dom front for MOO, best-of-K for SOO/WS), saves
library-compatible pkl artifacts, syncs to S3 after each cell, then terminates.

Usage (from repo root):
    .venv/bin/python backend/scripts/ec2_batch_run.py \\
        --s3-bucket my-sar-results \\
        --s3-region us-east-1 \\
        [--seeds 5] [--ngen 1000] [--workers 14] [--dry-run] [--no-terminate]
"""
from __future__ import annotations

import os
for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
           "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(_v, "1")

import argparse, json, math, multiprocessing, subprocess, sys, time
from concurrent.futures import ProcessPoolExecutor, as_completed

_BACKEND = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _BACKEND not in sys.path:
    sys.path.insert(0, _BACKEND)
import app.rootpath  # noqa: F401,E402  — adds repo root to sys.path

POP = 300
MODELS = ["MTSP", "CONN", "TCD_WS", "TCDT_WS", "TCD_MOO_NSGA2", "TCDT_MOO_NSGA2"]
DRONES = [4, 8, 12, 16]
COMM_KEYS = ["2", "sqrt(8)", "4"]
NVISITS = [1, 2, 3]

# Int values for 2 and 4 so PathInfo.__str__ omits the ".0" suffix
COMM_VAL = {"2": 2, "sqrt(8)": 2 * math.sqrt(2), "4": 4}

MAX_MISSION_TIME = 3600.0
MIN_CONNECTIVITY = 0.5


# ── per-seed worker (runs in subprocess) ──────────────────────────────────────

def run_seed(task: dict) -> dict:
    """
    Run one (model, drones, comm, nvisits, seed) cell to n_gen generations.
    Returns a dict with keys: status, n_feasible, F (list-of-lists, signed), sols, secs.
    Solution objects (sols) are pickle-serialisable and are returned in-band via
    ProcessPoolExecutor's natural pickle transport — no extra round-trip needed.
    """
    import numpy as np
    from pymoo.optimize import minimize
    from pymoo.core.duplicate import NoDuplicateElimination
    from PathSampling import PathSampling
    from PathMutation import PathMutation
    from PathCrossover import PathCrossover
    from PathRepair import PathRepair
    from PathInfo import PathInfo
    from PathProblem import PathProblem
    from PathFuncDict import compute_all_objectives
    from PathOptimizationModel import AVAILABLE_MODELS
    from app.optimizer_worker import _MUTATION_CONFIG, _build_algorithm

    mk = task["model"]; d = task["drones"]; c = task["comm"]
    nv = task["nv"]; seed = task["seed"]; ngen = task["ngen"]

    model = dict(AVAILABLE_MODELS[mk])
    scen = {
        "grid_size": 8, "cell_side_length": 50, "number_of_drones": d,
        "max_drone_speed": 2.5, "comm_cell_range": COMM_VAL[c],
        "n_visits": nv, "target_positions": [12], "th": 0.9,
        "detection_probability": 0.7,
    }
    info = PathInfo(scen)
    info.model = model
    info.max_mission_time_constraint = MAX_MISSION_TIME
    info.min_connectivity_constraint = MIN_CONNECTIVITY

    operators = dict(
        sampling=PathSampling(), mutation=PathMutation(_MUTATION_CONFIG),
        crossover=PathCrossover(prob=0.9, ox_prob=1.0, n_offsprings=2),
        repair=PathRepair(), eliminate_duplicates=NoDuplicateElimination(),
    )

    t0 = time.perf_counter()
    try:
        alg = _build_algorithm(model["Alg"], POP, len(model["F"]), seed, operators)
        res = minimize(PathProblem(info), alg, ("n_gen", ngen), seed=seed,
                       save_history=False, verbose=False)
    except Exception as exc:
        import traceback
        return {"status": "CRASH", "error": repr(exc),
                "trace": traceback.format_exc()[-800:], "secs": round(time.perf_counter()-t0, 1)}

    raw = np.atleast_1d(res.X).flatten() if res.X is not None else np.array([])
    sols = [x[0] if isinstance(x, np.ndarray) else x for x in raw]
    secs = round(time.perf_counter() - t0, 1)

    if not sols:
        return {"status": "INFEASIBLE", "n_feasible": 0, "secs": secs, "sols": []}

    for sol in sols:
        compute_all_objectives(sol)

    F = np.atleast_2d(res.F).reshape(len(sols), len(model["F"])) if res.F is not None \
        else np.zeros((len(sols), len(model["F"])))

    return {
        "status": "OK", "n_feasible": len(sols),
        "F": F.tolist(),   # signed (pymoo minimization space)
        "sols": sols,      # PathSolution objects — pickled by ProcessPoolExecutor
        "secs": secs,
    }


# ── merge helpers ─────────────────────────────────────────────────────────────

def _merge_moo(seed_results: list) -> tuple:
    """Concatenate all feasible fronts; return non-dominated (F_np_signed, sols)."""
    import numpy as np
    from pymoo.util.nds.non_dominated_sorting import NonDominatedSorting

    F_parts, sol_parts = [], []
    for r in seed_results:
        if r.get("status") == "OK" and r.get("n_feasible", 0) > 0:
            F_parts.append(np.array(r["F"], dtype=float))
            sol_parts.extend(r["sols"])

    if not F_parts:
        return None, None

    F_all = np.vstack(F_parts)
    idx = NonDominatedSorting().do(F_all, only_non_dominated_front=True)
    return F_all[idx], [sol_parts[i] for i in idx]


def _merge_soo_ws(seed_results: list) -> tuple:
    """Return (F_1row_signed_np, [sol]) for the seed with minimum F[:, 0]."""
    import numpy as np

    best_val, best_F, best_sol = None, None, None
    for r in seed_results:
        if r.get("status") != "OK" or not r.get("sols"):
            continue
        F = np.array(r["F"], dtype=float)
        bi = int(np.argmin(F[:, 0]))
        val = float(F[bi, 0])
        if best_val is None or val < best_val:
            best_val = val
            best_F = F[bi:bi+1]
            best_sol = r["sols"][bi]

    if best_sol is None:
        return None, None
    return best_F, [best_sol]


# ── persistence ───────────────────────────────────────────────────────────────

def _results_root() -> str:
    from app.config import get_settings
    return get_settings().RESULTS_ROOT


def _exists(scenario_name: str, results_root: str) -> bool:
    obj = os.path.join(results_root, "Objectives", f"{scenario_name}-ObjectiveValues.pkl")
    sol = os.path.join(results_root, "Solutions", f"{scenario_name}-SolutionObjects.pkl")
    return os.path.isfile(obj) and os.path.isfile(sol)


def _save_cell(scenario_name: str, model_key: str, model_dict: dict,
               F_np, sols: list, results_root: str, n_seeds: int, ngen: int) -> None:
    import numpy as np
    import pandas as pd

    obj_dir = os.path.join(results_root, "Objectives")
    sol_dir = os.path.join(results_root, "Solutions")
    meta_dir = os.path.join(results_root, "Metadata")
    for d in (obj_dir, sol_dir, meta_dir):
        os.makedirs(d, exist_ok=True)

    pd.DataFrame(np.array(F_np), columns=model_dict["F"]).to_pickle(
        os.path.join(obj_dir, f"{scenario_name}-ObjectiveValues.pkl"))
    pd.to_pickle(sols, os.path.join(sol_dir, f"{scenario_name}-SolutionObjects.pkl"))
    with open(os.path.join(meta_dir, f"{scenario_name}.json"), "w") as fh:
        json.dump({
            "scenario_name": scenario_name, "model_key": model_key,
            "model_dict": model_dict, "objectives": model_dict["F"],
            "n_solutions": len(sols), "n_seeds": n_seeds, "n_gen": ngen,
            "source": "ec2_batch_run",
        }, fh, indent=2)


# ── S3 / EC2 helpers ─────────────────────────────────────────────────────────

def _s3_sync(results_root: str, bucket: str, region: str) -> None:
    subprocess.run(
        ["aws", "s3", "sync", results_root, f"s3://{bucket}/Results/",
         "--region", region, "--quiet"],
        check=True,
    )


def _ec2_terminate() -> None:
    try:
        meta = "http://169.254.169.254/latest/meta-data/"
        iid = subprocess.check_output(
            ["curl", "-s", "-m", "5", meta + "instance-id"], text=True).strip()
        region = subprocess.check_output(
            ["curl", "-s", "-m", "5", meta + "placement/region"], text=True).strip()
        subprocess.run(
            ["aws", "ec2", "terminate-instances",
             "--instance-ids", iid, "--region", region], check=True)
    except Exception as exc:
        print(f"[terminate] failed: {exc}. Shut down manually.", flush=True)


# ── main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--s3-bucket", default="")
    ap.add_argument("--s3-region", default="us-east-1")
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--ngen", type=int, default=1000)
    ap.add_argument("--workers", type=int, default=0,
                    help="Parallel worker processes (default: cpu-2, capped at seeds×4)")
    ap.add_argument("--dry-run", action="store_true",
                    help="List missing cells and exit without running")
    ap.add_argument("--no-terminate", action="store_true",
                    help="Skip EC2 self-terminate (for local testing)")
    args = ap.parse_args()

    from PathOptimizationModel import AVAILABLE_MODELS
    from PathInfo import PathInfo

    results_root = _results_root()
    seeds = list(range(1, args.seeds + 1))
    workers = args.workers or min(max(1, multiprocessing.cpu_count() - 2), len(seeds) * 4)
    model_cache = {mk: dict(AVAILABLE_MODELS[mk]) for mk in MODELS}

    def _scenario_name(mk: str, d: int, c: str, nv: int) -> str:
        scen = {"grid_size": 8, "cell_side_length": 50, "number_of_drones": d,
                "max_drone_speed": 2.5, "comm_cell_range": COMM_VAL[c],
                "n_visits": nv, "target_positions": [12], "th": 0.9,
                "detection_probability": 0.7}
        info = PathInfo(scen); info.model = model_cache[mk]
        return str(info)

    all_cells = [(mk, d, c, nv) for mk in MODELS for d in DRONES
                 for c in COMM_KEYS for nv in NVISITS]
    missing = [(mk, d, c, nv) for mk, d, c, nv in all_cells
               if not _exists(_scenario_name(mk, d, c, nv), results_root)]

    print(f"Cells: {len(all_cells)} total, {len(all_cells)-len(missing)} done, "
          f"{len(missing)} missing × {args.seeds} seeds = {len(missing)*args.seeds} runs "
          f"@ n_gen={args.ngen}  workers={workers}")
    print(f"Results root: {results_root}")

    if args.dry_run:
        for mk, d, c, nv in missing:
            print(f"  MISSING: {_scenario_name(mk,d,c,nv)}")
        return

    if not missing:
        print("Nothing to do.")
        if args.s3_bucket:
            _s3_sync(results_root, args.s3_bucket, args.s3_region)
        if not args.no_terminate:
            _ec2_terminate()
        return

    import numpy as np
    completed = 0
    failed = []

    for ci, (mk, d, c, nv) in enumerate(missing, 1):
        model = model_cache[mk]
        scenario = _scenario_name(mk, d, c, nv)
        print(f"\n[{ci}/{len(missing)}] {scenario}  [{model['Type']}/{model['Alg']}]",
              flush=True)

        tasks = [dict(model=mk, drones=d, comm=c, nv=nv, seed=s, ngen=args.ngen)
                 for s in seeds]

        t0 = time.perf_counter()
        seed_results = []
        with ProcessPoolExecutor(max_workers=min(workers, len(tasks))) as ex:
            futs = {ex.submit(run_seed, t): t["seed"] for t in tasks}
            for f in as_completed(futs):
                r = f.result()
                seed_results.append(r)
                st = r.get("status"); n = r.get("n_feasible", 0)
                print(f"  seed {futs[f]}: {st} n={n}  {r.get('secs','?')}s", flush=True)

        # Reorder seed_results to match original seed order (as_completed is unordered)
        seed_results.sort(key=lambda r: r.get("secs", 0))  # stable enough for merge

        if model["Type"] == "MOO":
            F_merged, sols_merged = _merge_moo(seed_results)
        else:
            F_merged, sols_merged = _merge_soo_ws(seed_results)

        cell_secs = round(time.perf_counter() - t0, 1)

        if F_merged is None:
            print(f"  !! ALL SEEDS INFEASIBLE/CRASHED — skipping", flush=True)
            failed.append(scenario)
            continue

        print(f"  merged: {len(sols_merged)} solutions  wall={cell_secs}s", flush=True)
        _save_cell(scenario, mk, model, F_merged, sols_merged,
                   results_root, n_seeds=args.seeds, ngen=args.ngen)
        completed += 1
        print(f"  saved.", flush=True)

        if args.s3_bucket:
            try:
                _s3_sync(results_root, args.s3_bucket, args.s3_region)
                print(f"  s3://{args.s3_bucket}/Results/ synced", flush=True)
            except Exception as e:
                print(f"  S3 sync failed: {e} — continuing", flush=True)

    print(f"\n{'='*60}")
    print(f"Done. {completed}/{len(missing)} cells computed.  "
          f"{len(failed)} failed: {failed if failed else 'none'}")

    if args.s3_bucket:
        print("Final S3 sync …")
        _s3_sync(results_root, args.s3_bucket, args.s3_region)

    if not args.no_terminate:
        print("Terminating EC2 instance …")
        _ec2_terminate()


if __name__ == "__main__":
    main()
