"""Model-aware Pareto-front navigation (spec section 4).

The selector is self-describing via capabilities(); UIs must consult it instead
of hardcoding which strategies exist for which model type.
"""
import warnings

import numpy as np
import pandas as pd

from FilePaths import objective_values_filepath, solutions_filepath
from PathFileManagement import load_pickle


class StrategyUnavailableError(Exception):
    """Raised when a selection strategy does not exist for this result shape."""


class SolutionSelector:

    def __init__(self, F: pd.DataFrame, solutions, model: dict):
        self.F = F
        # SolutionObjects.pkl rows can be 1-element arrays (PathUnitTest.py:104-110)
        self.solutions = [s[0] if isinstance(s, np.ndarray) else s for s in solutions]
        self.model = model
        if model["Type"] == "MOO" and len(self.solutions) > 1:
            self.result_kind = "front"
        else:
            self.result_kind = "single"
            if model["Type"] == "MOO":
                warnings.warn(
                    "MOO front collapsed to a single non-dominated solution — "
                    "treating as 'single'. This is a convergence signal worth checking.")

    @classmethod
    def from_scenario(cls, scenario: str, model: dict):
        F = pd.read_pickle(f"{objective_values_filepath}{scenario}-ObjectiveValues.pkl")
        solutions = load_pickle(f"{solutions_filepath}{scenario}-SolutionObjects.pkl")
        return cls(F, list(solutions), model)

    # --- capability discovery ------------------------------------------------
    def capabilities(self):
        if self.result_kind == "single":
            return {"best": [], "balanced": False, "knee": False, "by_weights": False,
                    "by_index": len(self.solutions) - 1, "the_solution": True}
        return {"best": list(self.model["F"]),
                "balanced": True,
                "knee": self.F.shape[1] >= 2 and len(self.solutions) >= 3,
                "by_weights": True,
                "by_index": len(self.solutions) - 1,
                "the_solution": False}

    def _require_front(self, name):
        if self.result_kind == "front":
            return
        if self.model["Type"] == "WS":
            raise StrategyUnavailableError(
                f"'{name}' unavailable: WS models bake objective weights in before the "
                f"run, producing a single solution. Re-run with different WS weights, or "
                f"use the MOO variant to explore trade-offs interactively. "
                f"Use the_solution() for this result.")
        raise StrategyUnavailableError(
            f"'{name}' unavailable for single-solution results; use the_solution().")

    # --- strategies -----------------------------------------------------------
    def the_solution(self):
        if self.result_kind != "single":
            raise StrategyUnavailableError(
                "the_solution() is for single-solution results; this is a Pareto front — "
                "use best()/balanced()/knee()/by_weights()/by_index().")
        return 0, self.solutions[0], "The solution"

    def by_index(self, i):
        if not (0 <= i < len(self.solutions)):
            raise StrategyUnavailableError(
                f"index {i} out of range (0..{len(self.solutions) - 1})")
        return i, self.solutions[i], f"Solution #{i}"

    def _normalized_F(self):
        F_norm = (self.F - self.F.min(axis=0)) / (self.F.max(axis=0) - self.F.min(axis=0))
        return F_norm.fillna(0.5)    # zero-range column -> all solutions equal

    def best(self, objective_name):
        self._require_front("best")
        if objective_name not in self.model["F"]:
            raise StrategyUnavailableError(
                f"{objective_name!r} was not optimized by this model "
                f"(valid: {list(self.model['F'])}). Best-of-a-non-optimized metric "
                f"would be a sampling accident, not an answer.")
        # ObjectiveValues stores SIGNED values (polarity already applied by
        # PathProblem), so min is best for every objective.
        idx = int(self.F[objective_name].idxmin())
        return idx, self.solutions[idx], f"Best {objective_name}"

    # Reimplements get_median_index_of_scenario's normalize-centroid-argmin formula
    # (PathOptimizationModel.py:43-56) on in-memory F instead of re-reading pickles;
    # fillna additionally guards zero-range columns.
    def balanced(self):
        self._require_front("balanced")
        F_norm = self._normalized_F()
        centroid = F_norm.mean(axis=0)
        dists = np.linalg.norm(F_norm.values - centroid.values, axis=1)
        idx = int(np.argmin(dists))
        return idx, self.solutions[idx], "Balanced"

    def by_weights(self, weights: dict):
        self._require_front("by_weights")
        from pymoo.mcdm.pseudo_weights import PseudoWeights
        unknown = set(weights) - set(self.model["F"])
        if unknown:
            raise StrategyUnavailableError(
                f"Unknown objective(s) {sorted(unknown)}; valid: {list(self.model['F'])}")
        w = np.array([float(weights.get(name, 0.0)) for name in self.model["F"]])
        if w.sum() <= 0:
            raise StrategyUnavailableError("at least one weight must be positive")
        w = w / w.sum()
        idx = int(PseudoWeights(w).do(self.F.values))
        pretty = {name: round(float(wi), 3) for name, wi in zip(self.model["F"], w)}
        return idx, self.solutions[idx], f"Weights {pretty}"

    def knee(self):
        self._require_front("knee")
        if self.F.shape[1] < 2 or len(self.solutions) < 3:
            raise StrategyUnavailableError(
                "knee() needs >= 2 objectives and >= 3 solutions on the front")
        from pymoo.mcdm.high_tradeoff import HighTradeoffPoints
        try:
            # HighTradeoffPoints uses raw values for neighbor-finding (no built-in
            # normalization), so normalize to [0,1] per objective first.
            # Wrap in catch_warnings: pymoo._do calls warnings.filterwarnings('ignore')
            # globally, which would swallow our UserWarning if emitted afterwards.
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                idxs = HighTradeoffPoints().do(self._normalized_F().values)
        except Exception:
            idxs = None    # numerically degenerate fronts: treat as no knee
        if idxs is None or len(np.atleast_1d(idxs)) == 0:
            warnings.warn("No high-tradeoff point found; falling back to balanced().",
                          UserWarning, stacklevel=2)
            idx, sol, _ = self.balanced()
            return idx, sol, "Balanced (knee fallback)"
        idxs = np.atleast_1d(idxs)
        if len(idxs) > 1:    # spec: nearest-to-centroid among knee candidates
            F_norm = self._normalized_F()
            centroid = F_norm.mean(axis=0)
            d = np.linalg.norm(F_norm.values[idxs] - centroid.values, axis=1)
            idx = int(idxs[int(np.argmin(d))])
        else:
            idx = int(idxs[0])
        return idx, self.solutions[idx], "Knee"
