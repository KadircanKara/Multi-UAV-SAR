"""_unwrap_front: drop MOEA/D's aliased duplicate solutions from a pymoo result."""
import numpy as np

from app.optimizer_worker import _unwrap_front


class _Res:
    def __init__(self, X, F):
        self.X, self.F = X, F


class _Sol:
    """Stand-in for PathSolution — _unwrap_front only looks at object identity."""


def test_aliased_duplicates_are_dropped_and_F_stays_row_aligned():
    a, b, c = _Sol(), _Sol(), _Sol()
    X = np.array([[a], [b], [a], [c], [a]], dtype=object)
    F = np.array([[1.0, 1.0], [2.0, 2.0], [1.0, 1.0], [3.0, 3.0], [1.0, 1.0]])
    sols, out = _unwrap_front(_Res(X, F), 2)
    assert sols == [a, b, c]
    assert np.array_equal(out, np.array([[1.0, 1.0], [2.0, 2.0], [3.0, 3.0]]))


def test_distinct_solutions_sharing_objective_values_are_kept():
    # NSGA2 legitimately returns distinct solutions with equal objectives.
    a, b, c = _Sol(), _Sol(), _Sol()
    X = np.array([[a], [b], [c]], dtype=object)
    F = np.array([[1.0, 1.0]] * 3)
    sols, out = _unwrap_front(_Res(X, F), 2)
    assert sols == [a, b, c]
    assert out.shape == (3, 2)


def test_empty_results_give_an_empty_front_of_the_right_width():
    for res in (_Res(None, None),
                _Res(np.array([], dtype=object), None),
                _Res(None, np.array([[1.0, 2.0]]))):
        sols, out = _unwrap_front(res, 2)
        assert sols == []
        assert out.shape == (0, 2)
