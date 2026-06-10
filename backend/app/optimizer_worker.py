"""
Optimizer worker — runs ONE user-configured pymoo optimization, executed in a
child process (ProcessPoolExecutor) so the FastAPI event loop stays free to
serve status polls.

Import-safety: builds the pymoo algorithm DIRECTLY from fresh operators — it
NEVER imports `PathAlgorithm` or `main` (which bake globals at import time).
The per-run model is patched onto the PathInfo instance (`info.model = …`), so
no shared module globals are mutated.
"""
from __future__ import annotations

import json
import os

import app.rootpath  # noqa: F401  (side-effect: repo root on sys.path)


# Mutation config — copied verbatim from main.py (the tuned interactive config).
_MUTATION_CONFIG = {
    "swap_last_point": (0, 1),
    "swap": (0.3, 1),
    "inversion": (0.4, 1),
    "scramble": (0.3, 1),
    "insertion": (0, 1),
    "displacement": (0, 1),
    "block inversion": (0, 1),
    "random_one_sp_mutation": (0.4, 1),
    "random_n_sp_mutation": (0.0, 1),
    "all_sp_mutation": (0.0, 1),
    "longest_path_sp_mutation": (0.0, 1),
    "randomly_selected_sp_mutation": (0.0, 1),
}


def _write_status(run_dir: str, payload: dict) -> None:
    """Atomically write the per-run status file (read by the poll endpoint)."""
    path = os.path.join(run_dir, "status.json")
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(payload, fh)
    os.replace(tmp, path)


def _build_algorithm(alg: str, pop_size: int, n_obj: int, seed: int, operators: dict):
    from pymoo.algorithms.moo.nsga2 import NSGA2
    from pymoo.algorithms.moo.nsga3 import NSGA3
    from pymoo.algorithms.soo.nonconvex.ga import GA
    from pymoo.util.ref_dirs import get_reference_directions

    if alg == "NSGA2":
        return NSGA2(pop_size=pop_size, **operators)
    if alg == "NSGA3":
        # `energy` gives exactly pop_size well-spread reference directions for any
        # objective count (avoids the "pop_size < ref_dirs" warning of das-dennis).
        ref_dirs = get_reference_directions("energy", n_obj, n_points=pop_size, seed=seed)
        return NSGA3(ref_dirs=ref_dirs, pop_size=pop_size, **operators)
    # SOO-GA and WS both run a single-objective GA
    return GA(pop_size=pop_size, **operators)


def run_optimization(
    run_id: str,
    model_dict: dict,
    scenario_dict: dict,
    alg: str,
    pop_size: int,
    n_gen: int,
    seed: int,
    run_dir: str,
    scenario_name: str,
    model_key: str,
    objectives: list,
    polarities: dict,
    max_mission_time=None,
    min_connectivity=None,
) -> dict:
    """Run the optimization, persist artifacts into ``run_dir``, return the
    inline front payload. Executed in a child process; args must be picklable."""
    import numpy as np
    import pandas as pd

    from pymoo.optimize import minimize
    from pymoo.core.callback import Callback
    from pymoo.core.duplicate import NoDuplicateElimination

    from PathSampling import PathSampling
    from PathMutation import PathMutation
    from PathCrossover import PathCrossover
    from PathRepair import PathRepair
    from PathInfo import PathInfo
    from PathProblem import PathProblem
    from PathFuncDict import compute_all_objectives, objective_values

    os.makedirs(run_dir, exist_ok=True)
    cancel_path = os.path.join(run_dir, "cancel")
    _write_status(run_dir, {"state": "running", "gen": 0, "n_gen": n_gen})

    # Objective columns + polarity (PathProblem stores actual*polarity in F, so the
    # absolute/display value is F_signed * polarity — cheap to invert, no recompute).
    _cols = list(model_dict["F"])
    _pol = np.array([polarities.get(c, 1) for c in _cols], dtype=float)

    class _Progress(Callback):
        def notify(self, algorithm):
            try:
                # Cooperative cancel: if the service dropped a `cancel` flag, force
                # pymoo to terminate after this generation (best-so-far is kept).
                if os.path.exists(cancel_path):
                    algorithm.termination.terminate()
                snap = {
                    "state": "running",
                    "gen": int(algorithm.n_gen or 0),
                    "n_gen": int(n_gen),
                }
                # Live optimum: the current non-dominated front (MOO) / best (SOO).
                opt = getattr(algorithm, "opt", None)
                Fsigned = opt.get("F") if opt is not None else None
                if Fsigned is not None and len(Fsigned):
                    Fsigned = np.atleast_2d(np.asarray(Fsigned, dtype=float))
                    Fabs = Fsigned * _pol  # back to actual objective values
                    snap["live_front"] = [
                        {_cols[c]: float(Fabs[r, c]) for c in range(len(_cols))}
                        for r in range(Fabs.shape[0])
                    ]
                    # Best (optimal) absolute value per objective = abs at argmin(signed).
                    snap["best"] = {
                        _cols[c]: float(Fabs[int(np.argmin(Fsigned[:, c])), c])
                        for c in range(len(_cols))
                    }
                _write_status(run_dir, snap)
            except Exception:
                pass

    operators = dict(
        sampling=PathSampling(),
        mutation=PathMutation(_MUTATION_CONFIG),
        crossover=PathCrossover(prob=0.9, ox_prob=1.0, n_offsprings=2),
        repair=PathRepair(),
        eliminate_duplicates=NoDuplicateElimination(),
    )

    info = PathInfo(scenario_dict)
    info.model = model_dict  # patch the per-run model (PathProblem reads info.model)
    # User-set constraint thresholds (read by max_mission_time / min_perc_conn_constraint).
    if max_mission_time is not None:
        info.max_mission_time_constraint = float(max_mission_time)
    if min_connectivity is not None:
        info.min_connectivity_constraint = float(min_connectivity)

    algorithm = _build_algorithm(alg, pop_size, len(model_dict["F"]), seed, operators)

    res = minimize(
        problem=PathProblem(info),
        algorithm=algorithm,
        termination=("n_gen", n_gen),
        seed=seed,
        save_history=False,
        verbose=False,
        callback=_Progress(),
    )

    # Was the run stopped early (cancel flag present)? The front below is then the
    # best-so-far population at the generation it reached.
    cancelled = os.path.exists(cancel_path)
    stopped_at_gen = int(getattr(getattr(res, "algorithm", None), "n_gen", n_gen) or n_gen)

    # Unwrap solution objects (mirrors selector_service / PathUnitTest).
    raw = np.atleast_1d(res.X).flatten() if res.X is not None else np.array([])
    sols = [x[0] if isinstance(x, np.ndarray) else x for x in raw]
    F = np.atleast_2d(res.F) if res.F is not None else np.empty((0, len(model_dict["F"])))

    # Back-fill ALL objectives on each solution so the saved run behaves like a
    # seeded one (Explore / Compare / Animation), and read individual values.
    rows = []
    for i, sol in enumerate(sols):
        compute_all_objectives(sol)
        vals = objective_values(sol)
        signed = {o: (vals.get(o) if vals.get(o) is None else vals[o] * polarities.get(o, 1)) for o in objectives}
        abss = {o: vals.get(o) for o in objectives}
        rows.append({"index": i, "objectives_signed": signed, "objectives_abs": abss})

    result_kind = "front" if (model_dict["Type"] == "MOO" and len(sols) > 1) else "single"

    # Persist artifacts for an optional later "Save to library" (fast file copy).
    pd.DataFrame(F, columns=model_dict["F"]).to_pickle(os.path.join(run_dir, "Objectives.pkl"))
    pd.to_pickle(sols, os.path.join(run_dir, "Solutions.pkl"))
    with open(os.path.join(run_dir, "meta.json"), "w") as fh:
        json.dump({
            "scenario_name": scenario_name,
            "model_key": model_key,
            "model_dict": model_dict,
            "objectives": objectives,
            "polarities": polarities,
            "result_kind": result_kind,
            "cancelled": cancelled,
            "stopped_at_gen": stopped_at_gen,
        }, fh)

    front = {
        "scenario": scenario_name,
        "model_key": model_key,
        "objectives": objectives,
        "polarities": polarities,
        "result_kind": result_kind,
        "n_solutions": len(sols),
        "solutions": rows,
        "cancelled": cancelled,
        "stopped_at_gen": stopped_at_gen,
    }
    payload = {
        "state": "done",
        "gen": stopped_at_gen if cancelled else n_gen,
        "n_gen": n_gen,
        "front": front,
    }
    _write_status(run_dir, payload)
    return payload
