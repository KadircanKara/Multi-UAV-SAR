"""ConstrainedMOEAD: MOEA/D with feasibility-first replacement.

Tests use pymoo's BNH problem (2 objectives, 2 inequality constraints) — no
Path* machinery, so they are fast and deterministic-by-seed."""
import numpy as np
import pytest
from pymoo.core.evaluator import Evaluator
from pymoo.core.population import Population
from pymoo.optimize import minimize
from pymoo.problems import get_problem
from pymoo.util.ref_dirs import get_reference_directions

from PathMOEAD import ConstrainedMOEAD


def test_vanilla_moead_rejects_constraints():
    # Documents WHY the subclass exists; also alarms if a pymoo upgrade changes this.
    from pymoo.algorithms.moo.moead import MOEAD
    problem = get_problem("bnh")
    ref_dirs = get_reference_directions("energy", 2, n_points=10, seed=1)
    with pytest.raises(AssertionError, match="does not support any constraints"):
        minimize(problem, MOEAD(ref_dirs=ref_dirs), ("n_gen", 2), seed=1, verbose=False)


def test_constrained_moead_solves_bnh_feasibly():
    problem = get_problem("bnh")
    ref_dirs = get_reference_directions("energy", 2, n_points=20, seed=1)
    res = minimize(problem, ConstrainedMOEAD(ref_dirs=ref_dirs), ("n_gen", 20),
                   seed=1, verbose=False)
    assert res.X is not None and len(res.F) > 0
    assert np.max(res.CV) <= 1e-9  # every returned solution is feasible
    assert res.F.shape[1] == 2


def _evaluated(problem, X):
    pop = Population.new(X=np.atleast_2d(np.asarray(X, dtype=float)))
    Evaluator().eval(problem, pop)
    return pop


def test_replace_is_feasibility_first():
    """Direct unit test of the replacement rule.

    BNH points: (0, 0) is feasible (g1 = 0, g2 << 0); (0, 3) violates g1
    ((25 + 9 - 25)/25 = 0.36 > 0), so CV > 0."""
    problem = get_problem("bnh")
    ref_dirs = get_reference_directions("energy", 2, n_points=5, seed=1)
    alg = ConstrainedMOEAD(ref_dirs=ref_dirs, n_neighbors=3)
    alg.setup(problem, termination=("n_gen", 1), seed=1)
    alg.ideal = np.zeros(2)

    feasible_X, infeasible_X = [0.0, 0.0], [0.0, 3.0]

    # An infeasible offspring must not displace feasible incumbents…
    alg.pop = _evaluated(problem, [feasible_X] * 5)
    off = _evaluated(problem, infeasible_X)[0]
    before = alg.pop.get("X").copy()
    alg._replace(0, off)
    assert np.array_equal(alg.pop.get("X"), before)

    # …and a feasible offspring must displace infeasible incumbents.
    alg.pop = _evaluated(problem, [infeasible_X] * 5)
    off = _evaluated(problem, feasible_X)[0]
    alg._replace(0, off)
    assert np.all(alg.pop.get("CV")[alg.neighbors[0], 0] <= 1e-9)
