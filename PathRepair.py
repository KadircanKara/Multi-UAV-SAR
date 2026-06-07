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

        copy_path = list(sol.path).copy()

        new_path = []

        city_prev = copy_path[0]

        copy_path.pop(0)

        while(len(new_path) < sol.info.number_of_cells * sol.info.n_visits):

            city = copy_path[0]

            copy_path.pop(0)

            if new_path.count(city) < sol.info.n_visits:
                # Interpolate cities
                mid_cities = self.interpolate_between_cities(sol, city_prev, city)
                for city_mid in mid_cities:
                    if new_path.count(city_mid) < sol.info.n_visits:
                        new_path.append(city_mid)

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