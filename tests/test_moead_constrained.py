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

from PathMOEAD import ConstrainedMOEAD, MAX_REPLACEMENTS


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

    # …and a feasible offspring displaces infeasible incumbents, up to the
    # MAX_REPLACEMENTS cap: n_neighbors=3 here exceeds MAX_REPLACEMENTS=2, so
    # only the closest 2 of the 3 neighbours flip feasible; the 3rd is left
    # infeasible (CV=0.36) rather than the whole neighbourhood being claimed.
    alg.pop = _evaluated(problem, [infeasible_X] * 5)
    off = _evaluated(problem, feasible_X)[0]
    alg._replace(0, off)
    cv = alg.pop.get("CV")[alg.neighbors[0], 0]
    # Guard the constant itself: the assertion below is relative to
    # MAX_REPLACEMENTS, so without this a cap of 0 (replacement disabled -- MOEA/D
    # never evolves) would leave this test green. 3 == n_neighbors in this fixture.
    assert 0 < MAX_REPLACEMENTS < 3
    assert np.sum(cv <= 1e-9) == MAX_REPLACEMENTS


def test_setup_override_matches_pymoo_minus_the_assert():
    """Drift guard: our _setup is a copy of pymoo's minus its constraint assert.

    requirements.txt allows any pymoo 0.6.x, so an in-range upgrade could change
    MOEAD._setup and leave our override silently stale. Comparing AST-normalized
    statements makes this immune to reformatting/comments while still catching a
    real logic change. On failure: re-diff the two methods and update our copy.
    """
    import ast
    import inspect
    import textwrap
    from pymoo.algorithms.moo.moead import MOEAD

    def statements(fn):
        tree = ast.parse(textwrap.dedent(inspect.getsource(fn)))
        body = tree.body[0].body
        first = body[0]
        if (isinstance(first, ast.Expr) and isinstance(first.value, ast.Constant)
                and isinstance(first.value.value, str)):
            body = body[1:]  # drop the docstring
        return [ast.unparse(node) for node in body]

    theirs = statements(MOEAD._setup)
    assert any(s.startswith("assert not problem.has_constraints") for s in theirs), \
        "pymoo no longer asserts against constraints — ConstrainedMOEAD may be obsolete"
    assert statements(ConstrainedMOEAD._setup) == [
        s for s in theirs if not s.startswith("assert not problem.has_constraints")
    ]


def _replacement_winners(scale):
    """Which neighbourhood slots an offspring claims, with objective 0 x scale.

    Uses an UNCONSTRAINED problem so the feasibility-first branch is bypassed and
    only the decomposition's scale behaviour is under test.
    """
    from pymoo.core.population import Population

    problem = get_problem("zdt1")  # 2 objectives, no constraints
    ref_dirs = get_reference_directions("energy", 2, n_points=8, seed=1)
    alg = ConstrainedMOEAD(ref_dirs=ref_dirs, n_neighbors=8)
    alg.setup(problem, termination=("n_gen", 1), seed=1)

    base = np.array([[0.1, 0.9], [0.2, 0.8], [0.3, 0.7], [0.4, 0.6],
                     [0.5, 0.5], [0.6, 0.4], [0.7, 0.3], [0.8, 0.2]])
    F = base * np.array([scale, 1.0])
    alg.pop = Population.new(F=F)
    alg.ideal = F.min(axis=0)

    off = Population.new(F=np.array([[0.55 * scale, 0.25]]))[0]

    before = alg.pop.get("F").copy()
    alg._replace(0, off)
    after = alg.pop.get("F")
    return tuple(np.where(~np.all(before == after, axis=1))[0])


def test_replacement_is_scale_invariant():
    """The same trade-off must win the same subproblems at any objective scale.

    PathProblem reports Mission Time in seconds (~1e3) beside Percentage
    Connectivity as a fraction (~1). If the decomposition sees raw magnitudes the
    large objective dominates every subproblem and the weight vectors stop
    discriminating -- scaling one objective changes which neighbours an offspring
    claims, which is exactly the bug this asserts against.
    """
    assert _replacement_winners(1.0) == _replacement_winners(1000.0)
