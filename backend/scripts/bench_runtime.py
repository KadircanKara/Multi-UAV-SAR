"""Throwaway benchmark: measure per-generation wall-time for representative
scenarios so we can project the cost of the 800-generation seeded runs.

Mirrors optimizer_worker.run_optimization's build, but runs only a handful of
generations and times each one via a callback. Per-gen time is ~constant for a
fixed scenario (evaluation cost dominates), so total ≈ per_gen_median * 800.
"""
import app.rootpath  # noqa: F401  repo root on sys.path
import time
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

POP = 300
BENCH_GENS = 6          # time this many generations, take median delta
TARGET_GENS = 800       # the real seeded run length


class _Timer(Callback):
    def __init__(self):
        super().__init__()
        self.stamps = []

    def notify(self, algorithm):
        self.stamps.append(time.perf_counter())


def bench(model_key, drones, comm, nv):
    model = AVAILABLE_MODELS[model_key]
    scenario = {
        'grid_size': 8, 'cell_side_length': 50,
        'number_of_drones': drones, 'max_drone_speed': 2.5,
        'comm_cell_range': comm, 'n_visits': nv,
        'target_positions': [12], 'th': 0.9, 'detection_probability': 0.7,
    }
    info = PathInfo(scenario)
    info.model = model
    operators = dict(
        sampling=PathSampling(),
        mutation=PathMutation(_MUTATION_CONFIG),
        crossover=PathCrossover(prob=0.9, ox_prob=1.0, n_offsprings=2),
        repair=PathRepair(),
        eliminate_duplicates=NoDuplicateElimination(),
    )
    alg = _build_algorithm(model['Alg'], POP, len(model['F']), 1, operators)
    timer = _Timer()
    t0 = time.perf_counter()
    minimize(problem=PathProblem(info), algorithm=alg,
             termination=("n_gen", BENCH_GENS), seed=1,
             save_history=False, verbose=False, callback=timer)
    total = time.perf_counter() - t0
    # deltas between generation callbacks; drop gen-1 (includes sampling/init)
    deltas = np.diff(timer.stamps)
    per_gen = float(np.median(deltas)) if len(deltas) else total / BENCH_GENS
    proj = per_gen * TARGET_GENS
    print(f"{model_key:16s} d={drones:2d} r={str(comm):7s} nv={nv}  "
          f"per_gen={per_gen:6.2f}s  proj_800={proj/60:6.1f}min  ({len(model['F'])} obj)",
          flush=True)
    return per_gen, proj


if __name__ == "__main__":
    print(f"POP={POP}  BENCH_GENS={BENCH_GENS}  -> projecting to {TARGET_GENS} gens\n", flush=True)
    # drones x n_visits scaling on a light 2-obj MOO model (comm=4 = the missing column)
    for d, nv in [(4, 1), (4, 3), (8, 2), (12, 3), (16, 1), (16, 3)]:
        bench("TC_MOO_NSGA2", d, 4, nv)
    print(flush=True)
    # model-complexity factor at a fixed mid scenario (8 drones, nv2, comm=4)
    for mk in ["MTSP", "TC_MOO_NSGA2", "TCT_MOO_NSGA2", "TCDT_MOO_NSGA2"]:
        bench(mk, 8, 4, 2)
    print(flush=True)
    # heaviest realistic point: TCDT 5-obj at 16 drones nv3
    bench("TCDT_MOO_NSGA2", 16, 4, 3)
