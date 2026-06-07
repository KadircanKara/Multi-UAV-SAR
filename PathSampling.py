from pymoo.core.sampling import Sampling
import numpy as np
import random
from PathSolution import *

class AdaptiveRelaySampling(Sampling):

    def __init__(self) -> None:
        super().__init__()

    def _do(self, problem, n_samples, **kwargs):

        X = np.full((n_samples, problem.n_var), None, dtype=PathSolution)

        cells = np.arange(problem.info.number_of_cells)

        for i in range(n_samples):
            X[i] = np.random.choice(cells, size=problem.info.number_of_drones-1, replace=False)

        return X


class PathSampling(Sampling):

    def __init__(self) -> None:
        super().__init__()

    def _do(self, problem, n_samples, **kwargs):

        X = np.full((n_samples, 1), None, dtype=PathSolution)
        for i in range(n_samples):
            # Random path
            path = np.random.permutation(problem.info.n_visits * problem.info.number_of_cells)%problem.info.number_of_cells
            # Random start points
            start_points = sorted(random.sample([i for i in range(1, len(path))], problem.info.number_of_drones - 1))
            start_points.insert(0, 0)
            X[i, :] = PathSolution(path, start_points, problem.info, calculate_pathplan=False, calculate_tbv=False, calculate_connectivity=False, calculate_disconnectivity=False)

        return X