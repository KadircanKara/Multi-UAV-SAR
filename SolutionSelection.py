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

    def by_weights(self, weights):
        self._require_front("by_weights")
        raise NotImplementedError  # completed in Task 17
