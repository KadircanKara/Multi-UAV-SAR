"""Local convergence-verification gate for the 106 missing optimizer scenarios.

Run this BEFORE spending anything on the AWS/EC2 batch (see project memory
`ec2-missing-runs-plan`). It answers one question cheaply and locally: *will the
missing runs converge, so we don't pay to discover failures on AWS?*

Three tiers (modes):

  enumerate  Tier A — list the 106 missing (model, drones, comm, n_visits) cells
             and the 8 undominated "hard" cells (the (16 drones, n_visits 3)
             corner + a few siblings) the static domination audit can't vouch
             for. Pure bookkeeping; reproduces the audit's 106/8 split and
             self-checks against the known 8-cell list.

  smoke      Tier B — crash-check ALL 106 missing cells with the REAL worker
             build at a low generation count. Asserts ONLY "executes without
             exception, objective values finite". Deliberately NOT a feasibility
             check: constrained models spend the first generations entirely
             infeasible, so a gen-N feasibility assertion would throw false
             failures. ~1 core-hour total; minutes wall-clock parallelized.

  probe      Tier C — convergence-depth probe on the 8 hard cells only. Runs the
             "Max Generations" early-stop (the production `_EarlyStop`) with the
             interactive defaults INVERTED for strict verification:
             threshold 0.005 (0.5%), patience 30 — faithful to pymoo's own
             DefaultMultiObjectiveTermination (ftol 0.005 / period 50) — and the
             cap raised to 1200 (not 800) so a run that plateaus late tells us
             *by how much* 800 would fall short. PASS = early-stops at <= 800.

  all        smoke then probe.

Faithful to the seeded runs: model dicts come straight from AVAILABLE_MODELS
(their G/H already encode which constraints apply — MTSP carries no Max Mission
Time constraint, the rest do), pop_size 300, seed 1, and the seeded constraint
thresholds (max_mission_time 3600, min_connectivity 0.5). The worker applies a
threshold only where the model's G lists that constraint, so passing both
universally reproduces every model's seeded constraint set.

Usage (from repo root):
    .venv/bin/python backend/scripts/verify_convergence.py enumerate
    .venv/bin/python backend/scripts/verify_convergence.py all
Results stream to backend/scripts/verify_results/{smoke,probe}.json (resumable).
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

# Path bootstrap: runnable as `python backend/scripts/verify_convergence.py`.
_BACKEND = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _BACKEND not in sys.path:
    sys.path.insert(0, _BACKEND)
import app.rootpath  # noqa: F401,E402  (side-effect: repo root on sys.path)


# ─── The grid (identical to the static domination audit) ──────────────────────
MODELS = ["TC_MOO_NSGA2", "TT_MOO_NSGA2", "TCT_MOO_NSGA2", "TCD_MOO_NSGA2",
          "TCDT_MOO_NSGA2", "MTSP", "TCDT_WS", "TCT_WS", "TC_WS"]
DRONES = [4, 8, 12, 16]
COMM = ["2", "sqrt(8)", "4"]
NV = [1, 2, 3]
COMM_VAL = {"2": 2.0, "sqrt(8)": 2.0 * math.sqrt(2), "4": 4.0}

# Seeded recipe (from Results/Metadata — uniform across the library).
POP = 300
SEED = 1
MAX_MISSION_TIME = 3600.0
MIN_CONNECTIVITY = 0.5
MAX_MEAN_TBV = None

# Tier knobs.
SMOKE_GENS = 25            # crash-check generation count
PROBE_CAP = 1200           # Max-Generations cap for the convergence probe
PROBE_PATIENCE = 30
PROBE_THRESHOLD = 0.005

# Known undominated set (from the static audit) — self-check for `enumerate`.
EXPECTED_HARD = {
    ("TC_MOO_NSGA2", 16, "2", 3), ("TT_MOO_NSGA2", 12, "2", 1),
    ("TCT_MOO_NSGA2", 8, "2", 3), ("TCT_MOO_NSGA2", 16, "2", 3),
    ("TCD_MOO_NSGA2", 16, "2", 3), ("TCDT_MOO_NSGA2", 16, "2", 3),
    ("TCDT_MOO_NSGA2", 16, "sqrt(8)", 3), ("TCDT_MOO_NSGA2", 16, "4", 3),
}


def cell_key(tier, m, d, c, nv):
    return f"{tier}|{m}|d{d}|r{c}|nv{nv}"


def _scenario_dict(drones, comm_label, nv):
    """The seeded scenario template (matches PathInfo defaults exactly)."""
    return {
        "grid_size": 8, "cell_side_length": 50,
        "number_of_drones": drones, "max_drone_speed": 2.5,
        "comm_cell_range": COMM_VAL[comm_label], "n_visits": nv,
        "target_positions": [12], "th": 0.9, "detection_probability": 0.7,
    }


# ─── Tier A: enumerate ────────────────────────────────────────────────────────

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


def _present_per_model():
    """Cells already in the library, per model — via the same service the API uses."""
    from app.library_service import model_grid
    out = {}
    for m in MODELS:
        grid = model_grid(m) or {"scenarios": []}
        out[m] = {(r.get("number_of_drones"), _comm_label(r), r.get("n_visits"))
                  for r in grid["scenarios"]}
    return out


def enumerate_cells():
    """Return (missing, hard): missing = all 106 cells, hard = 8 undominated."""
    present = _present_per_model()
    full = list(itertools.product(DRONES, COMM, NV))
    missing = [(m, d, c, nv) for m in MODELS for (d, c, nv) in full
               if (d, c, nv) not in present[m]]
    hard = []
    for (m, d, c, nv) in missing:
        cv = COMM_VAL[c]
        # A smaller-comm existing sibling = a HARDER-connectivity case that already
        # converged → this (easier) cell is vouched for. No such sibling = hard.
        dominated = any(COMM_VAL[cc] < cv and (d, cc, nv) in present[m] for cc in COMM)
        if not dominated:
            hard.append((m, d, c, nv))
    return missing, hard


# ─── Tiers B/C: run one cell (picklable, runs in a worker process) ────────────

def _objectives_for(model_dict):
    from PathOptimizationModel import get_objectives_from_weighted_sum_model
    if model_dict.get("Type") == "WS":
        return list(get_objectives_from_weighted_sum_model(model_dict))
    return list(model_dict["F"])


def run_cell(task):
    """Run one optimization with the real worker build; return a result dict."""
    import shutil
    from PathOptimizationModel import AVAILABLE_MODELS
    from app.optimizer_worker import run_optimization
    from app.optimizer_service import _POLARITY

    tier, mk = task["tier"], task["model"]
    d, c, nv = task["drones"], task["comm"], task["nv"]
    key = cell_key(tier, mk, d, c, nv)
    safe = key.replace("|", "_").replace("(", "").replace(")", "")
    run_dir = os.path.join(task["out_root"], "runs", safe)

    model_dict = dict(AVAILABLE_MODELS[mk])
    objectives = _objectives_for(model_dict)
    polarities = {o: _POLARITY[o] for o in objectives}
    scen = _scenario_dict(d, c, nv)

    base = {"key": key, "tier": tier, "model": mk, "drones": d, "comm": c, "nv": nv}
    t0 = time.perf_counter()
    try:
        payload = run_optimization(
            safe, model_dict, scen, model_dict["Alg"], POP, task["n_gen"], SEED,
            run_dir, key, mk, objectives, polarities,
            MAX_MISSION_TIME, MIN_CONNECTIVITY, MAX_MEAN_TBV,
            task["gen_strategy"], task["patience"], task["threshold"],
        )
    except Exception as e:
        import traceback
        return {**base, "status": "CRASH", "error": repr(e),
                "trace": traceback.format_exc()[-1500:],
                "secs": round(time.perf_counter() - t0, 1)}

    front = payload.get("front", {})
    n_sol = int(front.get("n_solutions", 0))
    stopped = int(front.get("stopped_at_gen", task["n_gen"]))
    early = bool(front.get("early_stopped", False))

    finite = True
    for row in front.get("solutions", []):
        for v in (row.get("objectives_abs") or {}).values():
            if v is not None and not math.isfinite(float(v)):
                finite = False
                break
        if not finite:
            break

    res = {**base, "n_solutions": n_sol, "stopped_at_gen": stopped,
           "early_stopped": early, "finite": finite,
           "secs": round(time.perf_counter() - t0, 1)}

    if tier == "smoke":
        res["status"] = "OK" if finite else "NONFINITE"   # crash check only
    else:  # probe
        if not finite:
            res["status"] = "NONFINITE"
        elif n_sol < 1:
            res["status"] = "INFEASIBLE"        # no feasible front at the cap
        elif early and stopped <= 800:
            res["status"] = "CONVERGED"         # plateaued within the seeded budget
        elif early and stopped < PROBE_CAP:
            res["status"] = "LATE"              # needed >800 to plateau → bump n_gen
        else:
            res["status"] = "NOPLATEAU"         # ran to cap, still improving

    if not task["keep_artifacts"]:
        shutil.rmtree(run_dir, ignore_errors=True)
    return res


# ─── Pool driver (incremental + resumable) ────────────────────────────────────

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
            if cell_key(t["tier"], t["model"], t["drones"], t["comm"], t["nv"]) not in results]
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
            print(f"  [{done}/{len(todo)}] {r['status']:10s} {r['key']:42s} "
                  f"nsol={r.get('n_solutions', '-')} gen={r.get('stopped_at_gen', '-')} "
                  f"{r.get('secs', '?')}s", flush=True)
            if r["status"] == "CRASH":
                print(f"        {r.get('error', '')[:200]}", flush=True)
    return results


def _workers(cap=None):
    w = max(1, multiprocessing.cpu_count() - 2)
    return min(w, cap) if cap else w


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("mode", choices=["enumerate", "smoke", "probe", "all"])
    ap.add_argument("--out", default=os.path.join(_BACKEND, "scripts", "verify_results"))
    ap.add_argument("--workers", type=int, default=0, help="0 = auto (cpu-2)")
    args = ap.parse_args()

    missing, hard = enumerate_cells()
    print(f"Full grid: {len(MODELS)}×{len(DRONES)}×{len(COMM)}×{len(NV)} = {len(MODELS) * 36}")
    print(f"Missing cells: {len(missing)}")
    print(f"Undominated (hard) cells: {len(hard)}")
    for h in hard:
        print("   ", h)
    match = set(hard) == EXPECTED_HARD
    print(f"Self-check vs audit's known 8: {'MATCH' if match else 'MISMATCH!'}")
    sane = (len(missing) == 106 and match)
    if not sane:
        print("WARNING: enumeration drifted from the audit (expected 106 missing / "
              "known 8 hard). Investigate before trusting Tiers B/C.")

    if args.mode == "enumerate":
        sys.exit(0 if sane else 1)

    os.makedirs(args.out, exist_ok=True)

    if args.mode in ("smoke", "all"):
        print("\n=== Tier B: crash-check all missing cells (25 gens) ===", flush=True)
        tasks = [dict(tier="smoke", model=m, drones=d, comm=c, nv=nv, out_root=args.out,
                      n_gen=SMOKE_GENS, gen_strategy="fixed", patience=10, threshold=0.10,
                      keep_artifacts=False) for (m, d, c, nv) in missing]
        res = _run_pool(tasks, os.path.join(args.out, "smoke.json"),
                        args.workers or _workers())
        ok = sum(1 for r in res.values() if r["status"] == "OK")
        bad = [r for r in res.values() if r["status"] != "OK"]
        print(f"\nTier B: {ok}/{len(res)} OK; {len(bad)} need attention")
        for r in bad:
            print(f"  !! {r['status']:10s} {r['key']}  {r.get('error', '')[:160]}")

    if args.mode in ("probe", "all"):
        print("\n=== Tier C: convergence probe on the 8 hard cells "
              f"(max-gen, thr={PROBE_THRESHOLD}, patience={PROBE_PATIENCE}, cap={PROBE_CAP}) ===",
              flush=True)
        tasks = [dict(tier="probe", model=m, drones=d, comm=c, nv=nv, out_root=args.out,
                      n_gen=PROBE_CAP, gen_strategy="max", patience=PROBE_PATIENCE,
                      threshold=PROBE_THRESHOLD, keep_artifacts=True) for (m, d, c, nv) in hard]
        res = _run_pool(tasks, os.path.join(args.out, "probe.json"),
                        args.workers or _workers(cap=len(hard)))
        print("\nTier C summary:")
        for r in sorted(res.values(), key=lambda x: x["key"]):
            print(f"  {r['status']:10s} {r['key']:42s} nsol={r.get('n_solutions')} "
                  f"stopped@{r.get('stopped_at_gen')} early={r.get('early_stopped')} "
                  f"{r.get('secs')}s")
        good = sum(1 for r in res.values() if r["status"] == "CONVERGED")
        print(f"\n{good}/{len(res)} hard cells CONVERGED within 800 gens "
              "(PASS = safe to run the whole batch at n_gen=800).")


if __name__ == "__main__":
    main()
