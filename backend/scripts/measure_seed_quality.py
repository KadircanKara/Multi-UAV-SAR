"""Measure how much solution QUALITY improves across seeds, to pick K for EC2.

The feasibility probe answered "can it reach feasibility"; this answers "how much
better is best-of-K seeds than a single seed", which sizes the seed count K.

Runs representative cells to FULL length (no early stop), retaining solutions, and
scores quality per seed:
  SOO  (MTSP, CONN)  -> the single objective value (Mission Time min / %Conn max)
  WS   (TCD_WS, ...) -> the weighted-sum score (lower better)
  MOO  (NSGA2)       -> hypervolume of the feasible non-dominated front
Then reports, per cell: per-seed spread, best-of-K (merged front for MOO), the
%gain of best-of-K vs the median single seed, and the cumulative best-of-j curve
(j=1..K) so diminishing returns are visible.

No artifacts are written (in-memory only). Results -> verify_results/seed_quality.json.

Usage (from repo root):
    .venv/bin/python backend/scripts/measure_seed_quality.py --smoke   # quick validation
    .venv/bin/python backend/scripts/measure_seed_quality.py --seeds 1,2,3,4,5 --ngen 1000
"""
from __future__ import annotations

import os
for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
           "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(_v, "1")

import argparse, json, math, multiprocessing, statistics as st, sys, time
from concurrent.futures import ProcessPoolExecutor, as_completed

_BACKEND = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _BACKEND not in sys.path:
    sys.path.insert(0, _BACKEND)
import app.rootpath  # noqa: F401,E402

POP = 300
COMM_VAL = {"2": 2.0, "sqrt(8)": 2.0 * math.sqrt(2), "4": 4.0}

# Representative cells: type (SOO/WS/MOO) x difficulty (easy/mid/hard).
CELLS = [
    ("TCD_WS",         4, "4", 1),   # WS  easy (low-end bound)
    ("CONN",           8, "2", 2),   # SOO connectivity, mid, tight comm
    ("MTSP",          16, "4", 3),   # SOO mission-time, hard
    ("TCD_MOO_NSGA2",  8, "4", 2),   # MOO mid
    ("TCDT_WS",       12, "4", 2),   # WS  mid
    ("TCDT_MOO_NSGA2",16, "2", 3),   # MOO hard corner (max variance expected)
]
SMOKE_CELLS = [("TCD_WS", 4, "4", 1), ("TCD_MOO_NSGA2", 4, "4", 1)]


def ckey(m, d, c, nv, seed):
    return f"{m}|d{d}|r{c}|nv{nv}|s{seed}"


def run_one(task):
    """Full-length run; return absolute objective data (no metric yet). Picklable."""
    import numpy as np
    from pymoo.optimize import minimize
    from pymoo.core.duplicate import NoDuplicateElimination
    from PathSampling import PathSampling
    from PathMutation import PathMutation
    from PathCrossover import PathCrossover
    from PathRepair import PathRepair
    from PathInfo import PathInfo
    from PathProblem import PathProblem
    from PathFuncDict import compute_all_objectives, objective_values
    from PathOptimizationModel import (AVAILABLE_MODELS,
                                       get_objectives_from_weighted_sum_model,
                                       calculate_ws_score_from_ws_objective)
    from app.optimizer_worker import _MUTATION_CONFIG, _build_algorithm

    mk, d, c, nv, seed, ngen = (task["model"], task["drones"], task["comm"],
                                task["nv"], task["seed"], task["ngen"])
    model = dict(AVAILABLE_MODELS[mk]); typ = model["Type"]
    base = {"key": ckey(mk, d, c, nv, seed), "model": mk, "drones": d, "comm": c,
            "nv": nv, "seed": seed, "type": typ, "alg": model["Alg"]}

    scen = {"grid_size": 8, "cell_side_length": 50, "number_of_drones": d,
            "max_drone_speed": 2.5, "comm_cell_range": COMM_VAL[c], "n_visits": nv,
            "target_positions": [12], "th": 0.9, "detection_probability": 0.7}
    info = PathInfo(scen); info.model = model
    info.max_mission_time_constraint = 3600.0
    info.min_connectivity_constraint = 0.5
    operators = dict(sampling=PathSampling(), mutation=PathMutation(_MUTATION_CONFIG),
                     crossover=PathCrossover(prob=0.9, ox_prob=1.0, n_offsprings=2),
                     repair=PathRepair(), eliminate_duplicates=NoDuplicateElimination())

    t0 = time.perf_counter()
    try:
        alg = _build_algorithm(model["Alg"], POP, len(model["F"]), seed, operators)
        res = minimize(PathProblem(info), alg, ("n_gen", ngen), seed=seed,
                       save_history=False, verbose=False)
    except Exception as e:
        import traceback
        return {**base, "status": "CRASH", "error": repr(e),
                "trace": traceback.format_exc()[-1200:], "secs": round(time.perf_counter()-t0, 1)}

    raw = np.atleast_1d(res.X).flatten() if res.X is not None else np.array([])
    sols = [x[0] if isinstance(x, np.ndarray) else x for x in raw]
    secs = round(time.perf_counter() - t0, 1)
    if not sols:
        return {**base, "status": "INFEASIBLE", "n_feasible": 0, "secs": secs}

    if typ == "MOO":
        front = []
        for sol in sols:
            compute_all_objectives(sol); vals = objective_values(sol)
            front.append({o: float(vals[o]) for o in model["F"]})
        return {**base, "status": "OK", "n_feasible": len(sols),
                "objectives": list(model["F"]), "front": front, "secs": secs}

    # SOO / WS -> single best solution
    sol = sols[0]
    compute_all_objectives(sol); vals = objective_values(sol)
    if typ == "WS":
        score = float(calculate_ws_score_from_ws_objective(sol))
        underlying = get_objectives_from_weighted_sum_model(model)
        return {**base, "status": "OK", "n_feasible": len(sols), "scalar": score,
                "scalar_kind": "min", "metric": "WS score",
                "obj_vals": {o: float(vals[o]) for o in underlying}, "secs": secs}
    # SOO
    from app.optimizer_service import _POLARITY
    obj = model["F"][0]
    kind = "min" if _POLARITY[obj] > 0 else "max"
    return {**base, "status": "OK", "n_feasible": len(sols), "scalar": float(vals[obj]),
            "scalar_kind": kind, "metric": obj, "secs": secs}


# ── HV helpers (main process) ──────────────────────────────────────────────────

def _moo_quality(seed_results, objectives):
    """Per-seed HV + cumulative merged-front HV, normalized in shared min-space."""
    import numpy as np
    from pymoo.indicators.hv import HV
    from pymoo.util.nds.non_dominated_sorting import NonDominatedSorting
    from app.optimizer_service import _POLARITY
    pol = np.array([_POLARITY[o] for o in objectives], float)

    mats = []  # one (n_sol x n_obj) min-space matrix per seed (seed order)
    for r in seed_results:
        if r.get("front"):
            mats.append(np.array([[row[o] for o in objectives] for row in r["front"]], float) * pol)
    if not mats:
        return None
    pool = np.vstack(mats)
    ideal = pool.min(axis=0); nadir = pool.max(axis=0)
    rng = np.where(nadir - ideal > 1e-12, nadir - ideal, 1.0)
    ref = np.array([1.1] * len(objectives))
    hv = HV(ref_point=ref)
    nrm = lambda M: (M - ideal) / rng
    per_seed = [float(hv(nrm(M))) for M in mats]

    nd = NonDominatedSorting()
    curve, merged = [], None
    for M in mats:
        merged = M if merged is None else np.vstack([merged, M])
        idx = nd.do(merged, only_non_dominated_front=True)
        curve.append(float(hv(nrm(merged[idx]))))
    return {"per_seed": per_seed, "merged_curve": curve}


def _pct(a, b):
    return None if (b is None or b == 0) else round((a - b) / abs(b) * 100, 1)


# ── pool plumbing ──────────────────────────────────────────────────────────────

def _load(path):
    if os.path.isfile(path):
        try:
            return {r["key"]: r for r in json.load(open(path))}
        except Exception:
            return {}
    return {}


def _save(path, results):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    json.dump(list(results.values()), open(tmp, "w"), indent=2)
    os.replace(tmp, path)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=os.path.join(_BACKEND, "scripts", "verify_results"))
    ap.add_argument("--seeds", default="1,2,3,4,5")
    ap.add_argument("--ngen", type=int, default=1000)
    ap.add_argument("--workers", type=int, default=0)
    ap.add_argument("--smoke", action="store_true", help="2 cells x 2 seeds x 150 gens")
    args = ap.parse_args()

    cells = SMOKE_CELLS if args.smoke else CELLS
    seeds = [1, 2] if args.smoke else [int(s) for s in args.seeds.split(",") if s.strip()]
    ngen = 150 if args.smoke else args.ngen
    out_path = os.path.join(args.out, "seed_quality_smoke.json" if args.smoke else "seed_quality.json")

    tasks = [dict(model=m, drones=d, comm=c, nv=nv, seed=s, ngen=ngen)
             for (m, d, c, nv) in cells for s in seeds]
    results = _load(out_path)
    todo = [t for t in tasks if ckey(t["model"], t["drones"], t["comm"], t["nv"], t["seed"]) not in results]
    workers = args.workers or min(max(1, multiprocessing.cpu_count() - 2), len(todo) or 1)
    print(f"{len(cells)} cells x {len(seeds)} seeds = {len(tasks)} runs @ n_gen={ngen}")
    print(f"  {len(results)} cached, {len(todo)} to run, {workers} workers", flush=True)

    if todo:
        with ProcessPoolExecutor(max_workers=workers) as ex:
            futs = [ex.submit(run_one, t) for t in todo]
            done = 0
            for f in as_completed(futs):
                r = f.result(); results[r["key"]] = r; _save(out_path, results); done += 1
                q = (r.get("scalar") if r.get("scalar") is not None
                     else (f"front={r.get('n_feasible')}" if r.get("status") == "OK" else r.get("status")))
                print(f"  [{done}/{len(todo)}] {r['key']:34s} {str(q):>10}  {r.get('secs','?')}s", flush=True)

    # ── aggregate per cell ───────────────────────────────────────────────────────
    print("\n" + "=" * 78)
    print("QUALITY vs SEEDS  (best-of-K = best scalar / merged front; gain vs median single seed)")
    print("=" * 78)
    for (m, d, c, nv) in cells:
        rs = [results.get(ckey(m, d, c, nv, s)) for s in seeds]
        rs = [r for r in rs if r and r.get("status") == "OK"]
        if not rs:
            print(f"\n{m} d={d} r={c} nv={nv}: no feasible result"); continue
        typ = rs[0]["type"]
        print(f"\n{m}  d={d} r={c} nv={nv}  [{typ}/{rs[0]['alg']}]  ({len(rs)}/{len(seeds)} seeds feasible)")
        if typ == "MOO":
            q = _moo_quality(rs, rs[0]["objectives"])
            if not q:
                print("  (no front)"); continue
            ps = q["per_seed"]; mc = q["merged_curve"]
            med = st.median(ps)
            print(f"  metric: hypervolume (normalized, higher=better)")
            print(f"  per-seed HV : {[round(x,3) for x in ps]}  (min {min(ps):.3f} / median {med:.3f} / max {max(ps):.3f})")
            print(f"  merged 1..K : {[round(x,3) for x in mc]}")
            print(f"  best-of-{len(ps)} (merged) vs median single: +{_pct(mc[-1], med)}%   spread across seeds: {_pct(max(ps),min(ps))}%")
        else:
            kind = rs[0]["scalar_kind"]; metric = rs[0]["metric"]
            vals = [r["scalar"] for r in rs]
            best = min(vals) if kind == "min" else max(vals)
            med = st.median(vals)
            # cumulative best-of-j in seed order
            cum, curve = None, []
            for v in vals:
                cum = v if cum is None else (min(cum, v) if kind == "min" else max(cum, v))
                curve.append(round(cum, 2))
            gain = _pct(med, best) if kind == "min" else _pct(best, med)
            spread = _pct(max(vals), min(vals))
            print(f"  metric: {metric} ({'lower' if kind=='min' else 'higher'} = better)")
            print(f"  per-seed    : {[round(v,2) for v in vals]}  (best {best:.2f} / median {med:.2f})")
            print(f"  best-of-j   : {curve}")
            print(f"  best-of-{len(vals)} vs median single: {gain}% better   spread across seeds: {abs(spread or 0)}%")

    crashes = [r for r in results.values() if r.get("status") == "CRASH"]
    if crashes:
        print(f"\n!! {len(crashes)} crashes:")
        for r in crashes:
            print(f"   {r['key']}: {r.get('error','')[:160]}")


if __name__ == "__main__":
    main()
