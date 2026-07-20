"""MOEAD-vs-NSGA2 validation sweep (manual research gate, K seeds).

Runs both engines on the same scenario/model across seeds and reports
feasible-solution counts, distinct-solution counts (de-duplicated by objective
vector -- MOEA/D's neighbourhood replacement can alias one Individual into
several population slots, inflating the raw feasible count), hypervolume
(signed objective space, union-normalized), and runtime. Usage (from backend/):

    ../.venv/bin/python scripts/validate_moead.py            # 5 seeds, pop 52, 100 gen (~long)
    ../.venv/bin/python scripts/validate_moead.py --seeds 2 --pop 20 --gen 20   # quick sanity
"""
import argparse
import os
import sys
import time

# Path bootstrap: `python scripts/validate_moead.py` from backend/ puts only
# `scripts/` on sys.path, not `backend/` itself — add it before `import app.*`.
_BACKEND = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _BACKEND not in sys.path:
    sys.path.insert(0, _BACKEND)
import app.rootpath  # noqa: F401,E402  (side-effect: repo root on sys.path)
import numpy as np


def run_one(alg_name: str, seed: int, pop_size: int, n_gen: int):
    from pymoo.optimize import minimize
    from pymoo.core.duplicate import NoDuplicateElimination

    from app.optimizer_worker import _MUTATION_CONFIG, _build_algorithm
    from PathSampling import PathSampling
    from PathMutation import PathMutation
    from PathCrossover import PathCrossover
    from PathRepair import PathRepair
    from PathInfo import PathInfo, default_scenario
    from PathProblem import PathProblem

    model = {
        "Type": "MOO", "Exp": "TC", "Alg": alg_name,
        "F": ["Mission Time", "Percentage Connectivity"],
        "G": ["Max Mission Time", "Min Percentage Connectivity"],
        "H": ["Path Speed Violations as Constraint"],
    }
    # PathInfo has no per-key fallback — a partial dict raises KeyError on the
    # first field it doesn't carry — so start from its own defaults and
    # override just what this scenario needs (4 drones, single-visit).
    info = PathInfo(dict(default_scenario, number_of_drones=4, n_visits=1))
    info.model = model
    info.max_mission_time_constraint = 3600.0
    info.min_connectivity_constraint = 0.5

    operators = dict(
        sampling=PathSampling(),
        mutation=PathMutation(_MUTATION_CONFIG),
        crossover=PathCrossover(prob=0.9, ox_prob=1.0, n_offsprings=2),
        repair=PathRepair(),
        eliminate_duplicates=NoDuplicateElimination(),
    )
    algorithm = _build_algorithm(alg_name, pop_size, len(model["F"]), seed, operators)

    t0 = time.time()
    res = minimize(PathProblem(info), algorithm, ("n_gen", n_gen),
                   seed=seed, save_history=False, verbose=False)
    dt = time.time() - t0

    F = np.atleast_2d(res.F) if res.F is not None else np.empty((0, len(model["F"])))
    n_uniq = len(np.unique(F, axis=0)) if len(F) else 0
    return {"alg": alg_name, "seed": seed, "n_feasible": len(F), "n_distinct": n_uniq,
            "F": F, "sec": dt}


def hypervolume(F, ideal, nadir):
    """HV of the signed front, union-normalized to [0,1], ref point (1.05, ...)."""
    if len(F) == 0:
        return 0.0
    from pymoo.indicators.hv import HV
    span = np.maximum(nadir - ideal, 1e-12)
    return float(HV(ref_point=np.full(F.shape[1], 1.05))((F - ideal) / span))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--pop", type=int, default=52)
    ap.add_argument("--gen", type=int, default=100)
    args = ap.parse_args()

    runs = [run_one(alg, seed, args.pop, args.gen)
            for alg in ("NSGA2", "MOEAD")
            for seed in range(1, args.seeds + 1)]

    fronts = [r["F"] for r in runs if len(r["F"])]
    if fronts:
        allF = np.vstack(fronts)
        ideal, nadir = allF.min(axis=0), allF.max(axis=0)
        for r in runs:
            r["hv"] = hypervolume(r["F"], ideal, nadir)
    else:
        for r in runs:
            r["hv"] = 0.0

    print(f"\n{'alg':7} {'seed':4} {'feas':>5} {'uniq':>5} {'HV':>8} {'sec':>7}")
    for r in runs:
        print(f"{r['alg']:7} {r['seed']:4d} {r['n_feasible']:5d} {r['n_distinct']:5d} "
              f"{r['hv']:8.4f} {r['sec']:7.1f}")
    for alg in ("NSGA2", "MOEAD"):
        rs = [r for r in runs if r["alg"] == alg]
        print(f"{alg}: mean feas {np.mean([r['n_feasible'] for r in rs]):.1f}, "
              f"mean distinct {np.mean([r['n_distinct'] for r in rs]):.1f}, "
              f"mean HV {np.mean([r['hv'] for r in rs]):.4f}, "
              f"mean sec {np.mean([r['sec'] for r in rs]):.0f}")


if __name__ == "__main__":
    main()
