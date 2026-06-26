"""Confirm the cells the feasibility probe flagged INFEASIBLE (cap 500, seed 1).

Re-runs ONLY those cells across several seeds at a higher cap, to tell apart:

  RECOVERED    feasible under at least one seed within the cap  -> safe for EC2
               (the gen-500/seed-1 miss was search luck, not infeasibility)
  CONFIRMED    no seed reaches feasibility even at the higher cap -> structural;
               diagnose the binding constraint before spending EC2 time.

Reuses verify_feasibility.run_cell verbatim (same build, operators, constraints),
only varying seed + cap. Results -> backend/scripts/verify_results/confirm.json.

Usage (from repo root):
    .venv/bin/python backend/scripts/confirm_flagged.py
    .venv/bin/python backend/scripts/confirm_flagged.py --cap 1000 --seeds 1,2,3
"""
from __future__ import annotations

import os

for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
           "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(_v, "1")

import argparse
import json
import multiprocessing
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
from verify_feasibility import run_cell, cell_key  # noqa: E402  (shares build/constants)

# The cells the cap-500/seed-1 probe flagged INFEASIBLE.
FLAGGED = [
    ("CONN", 12, "2", 1),
    ("CONN", 12, "sqrt(8)", 1),
    ("TCD_WS", 12, "4", 1),
    ("TCDT_WS", 12, "4", 1),
    ("TCDT_MOO_NSGA2", 16, "2", 3),
]
DEFAULT_SEEDS = [1, 2, 3]
DEFAULT_CAP = 1000
FRONT_K = 3


def skey(m, d, c, nv, seed):
    return f"{cell_key(m, d, c, nv)}|s{seed}"


def _load(path):
    if os.path.isfile(path):
        try:
            return {r["skey"]: r for r in json.load(open(path))}
        except Exception:
            return {}
    return {}


def _save(path, results):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(list(results.values()), fh, indent=2)
    os.replace(tmp, path)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=os.path.join(_HERE, "verify_results"))
    ap.add_argument("--cap", type=int, default=DEFAULT_CAP)
    ap.add_argument("--seeds", default=",".join(map(str, DEFAULT_SEEDS)))
    ap.add_argument("--workers", type=int, default=0)
    args = ap.parse_args()
    seeds = [int(s) for s in args.seeds.split(",") if s.strip()]

    tasks = []
    for (m, d, c, nv) in FLAGGED:
        for seed in seeds:
            tasks.append(dict(model=m, drones=d, comm=c, nv=nv,
                              cap=args.cap, k=FRONT_K, seed=seed))
    out_path = os.path.join(args.out, "confirm.json")
    results = _load(out_path)
    todo = [t for t in tasks
            if skey(t["model"], t["drones"], t["comm"], t["nv"], t["seed"]) not in results]
    workers = args.workers or min(max(1, multiprocessing.cpu_count() - 2), len(todo) or 1)

    print(f"Confirming {len(FLAGGED)} flagged cells x {len(seeds)} seeds "
          f"= {len(tasks)} runs (cap {args.cap}, seeds {seeds})")
    print(f"  {len(results)} cached, {len(todo)} to run, {workers} workers", flush=True)

    if todo:
        with ProcessPoolExecutor(max_workers=workers) as ex:
            fut = {ex.submit(run_cell, t):
                   skey(t["model"], t["drones"], t["comm"], t["nv"], t["seed"]) for t in todo}
            done = 0
            for f in as_completed(fut):
                r = f.result()
                r["skey"] = fut[f]
                results[r["skey"]] = r
                _save(out_path, results)
                done += 1
                print(f"  [{done}/{len(todo)}] {r['status']:10s} {r['skey']:42s} "
                      f"first={r.get('first_feasible_gen')} stop@{r.get('stop_gen')} "
                      f"front={r.get('front_size')} {r.get('secs')}s", flush=True)

    # ── Per-cell verdict across seeds ───────────────────────────────────────────
    print("\n── Confirmation verdicts ──")
    n_recovered = 0
    for (m, d, c, nv) in FLAGGED:
        rs = [results.get(skey(m, d, c, nv, s)) for s in seeds]
        rs = [r for r in rs if r]
        feasible_seeds = [r["seed"] for r in rs if r["status"] == "FEASIBLE"]
        cell = f"{m} d={d} r={c} nv={nv}"
        if feasible_seeds:
            n_recovered += 1
            gens = {r["seed"]: r.get("first_feasible_gen") for r in rs if r["status"] == "FEASIBLE"}
            print(f"  RECOVERED   {cell:34s} feasible under seeds {feasible_seeds} "
                  f"(onset {gens})")
        else:
            print(f"  CONFIRMED   {cell:34s} 0/{len(rs)} seeds feasible at cap {args.cap} "
                  f"-> structural")
    print(f"\n{n_recovered}/{len(FLAGGED)} flagged cells recovered with a different seed; "
          f"{len(FLAGGED) - n_recovered} confirmed structural.")


if __name__ == "__main__":
    main()
