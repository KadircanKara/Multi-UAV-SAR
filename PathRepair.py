from collections import defaultdict

from pymoo.core.repair import Repair
import numpy as np
from PathSolution import *
from PathInfo import *
import numpy as np
from pymoo.core.repair import Repair
from PathSolution import *
from PathProblem import *
from Distance import *
from PathAnimation import *


def sign(x):
    return 1 if x > 0 else -1 if x < 0 else 0  # Returns 0 if x is exactly 0


class PathRepair(Repair):

    def _do(self, problem, X, **kwargs):

        calculate_connectivity = True
        calculate_disconnectivity = False
        calculate_tbv = True

        for obj in problem.model["F"]:
            if "Disconnected Time" in obj:
                calculate_disconnectivity = True
            if "TBV" in obj:
                calculate_tbv = True

        for k in range(len(X)):
            sol : PathSolution = X[k, 0]

            new_path = self.interpolate_path(sol)

            X[k, 0] = PathSolution(new_path, np.copy(sol.start_points), sol.info, calculate_pathplan=True, calculate_tbv=calculate_tbv, calculate_connectivity=calculate_connectivity, calculate_disconnectivity=calculate_disconnectivity)

        return X

    def interpolate_path(self, sol:PathSolution):

        path = list(sol.path)
        n_visits = sol.info.n_visits
        target_length = sol.info.number_of_cells * n_visits

        new_path = []
        # Kept equal to new_path.count(c) for every c. The visit ceiling used to
        # be checked by rescanning new_path per candidate cell, which made the
        # loop quadratic in path length — and it dominated the whole optimizer.
        visits = defaultdict(int)

        city_prev = path[0]
        index = 1  # walk an index; popping the front of a list is itself O(n)

        while len(new_path) < target_length:

            city = path[index]
            index += 1

            if visits[city] < n_visits:
                # Interpolate cities
                mid_cities = self.interpolate_between_cities(sol, city_prev, city)
                for city_mid in mid_cities:
                    # NOTE: a mid-cell already at its ceiling is skipped, so the
                    # emitted path can jump. That is intentional — the speed
                    # violation is carried as a constraint, not repaired here.
                    if visits[city_mid] < n_visits:
                        new_path.append(city_mid)
                        visits[city_mid] += 1

            city_prev = city

        return new_path



    def interpolate_between_cities(self, sol:PathSolution, city_prev, city):

        interpolated_path = [city_prev]

        info = sol.info
        coords_prev = sol.get_coords(city_prev)
        coords = sol.get_coords(city)
        coords_delta = coords - coords_prev
        axis_inc = np.array([sign(coords_delta[0]), sign(coords_delta[1])])

        num_mid_cities = int(max(abs(coords_delta))/info.cell_side_length)

        coords_temp = coords_prev.copy()

        for _ in range(num_mid_cities):
            if coords_temp[0] != coords[0]:
                coords_temp[0] += info.cell_side_length * axis_inc[0]
            if coords_temp[1] != coords[1]:
                coords_temp[1] += info.cell_side_length * axis_inc[1]
            mid_city = sol.get_city(coords_temp)
            interpolated_path.append(mid_city)
        
        return interpolated_path