"""_build_algorithm: engine factory (no optimization is run here)."""


def _operators():
    # Mirror run_optimization's operator dict exactly (all constructible
    # without a scenario — the worker builds them before PathInfo too).
    from pymoo.core.duplicate import NoDuplicateElimination
    from PathSampling import PathSampling
    from PathMutation import PathMutation
    from PathCrossover import PathCrossover
    from PathRepair import PathRepair
    from app.optimizer_worker import _MUTATION_CONFIG

    return dict(
        sampling=PathSampling(),
        mutation=PathMutation(_MUTATION_CONFIG),
        crossover=PathCrossover(prob=0.9, ox_prob=1.0, n_offsprings=2),
        repair=PathRepair(),
        eliminate_duplicates=NoDuplicateElimination(),
    )


def test_build_algorithm_moead():
    from app.optimizer_worker import _build_algorithm
    from PathMOEAD import ConstrainedMOEAD

    alg = _build_algorithm("MOEAD", pop_size=12, n_obj=2, seed=1,
                           operators=_operators())
    assert isinstance(alg, ConstrainedMOEAD)
    assert alg.ref_dirs.shape == (12, 2)  # energy ref dirs: exactly pop_size


def test_build_algorithm_nsga_unaffected():
    from pymoo.algorithms.moo.nsga2 import NSGA2
    from pymoo.algorithms.moo.nsga3 import NSGA3
    from app.optimizer_worker import _build_algorithm

    assert isinstance(_build_algorithm("NSGA2", 12, 2, 1, _operators()), NSGA2)
    assert isinstance(_build_algorithm("NSGA3", 12, 2, 1, _operators()), NSGA3)
