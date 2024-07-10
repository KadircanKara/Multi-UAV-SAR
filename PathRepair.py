from pymoo.core.repair import Repair
import numpy as np
from copy import deepcopy

from PathSolution import *
from PathInfo import *

from Time import get_real_paths
from Distance import get_city

class PathRepair(Repair):

    def _do(self, problem, pop, **kwargs):

        X = deepcopy(pop)

        info:PathInfo = X[0][0].info

        for ch in X:
            sol:PathSolution = ch[0]
            # Repair sol
            discrete_real_time_path_matrix = sol.real_time_path_matrix
            xs, ys = get_real_paths(sol)
            real_time_coords = np.empty(xs.shape, dtype=tuple)
            real_time_cells = np.empty(xs.shape, dtype=int)
            realtime_real_time_path_matrix = deepcopy(real_time_cells)
            for node in range(info.number_of_nodes):
                real_time_coords[node] = list(zip(xs[node],ys[node]))
                real_time_cells[node] = list(map(lambda x: get_city(x, info.grid_size, info.cell_side_length), real_time_coords[node]))
                for i in range(len(real_time_cells[node])-1):
                    current_cell, next_cell = real_time_cells[node][i], real_time_cells[node][i+1]
                    if current_cell != next_cell:
                        realtime_real_time_path_matrix[node][i] = current_cell
                if real_time_cells[node][-1] != real_time_cells[node][-2]:
                    realtime_real_time_path_matrix[node][-1] = real_time_cells[node][-1]
            
                print(f"realtime_real_time_path_matrix:{realtime_real_time_path_matrix}")


        pop.set("X",X)
        return pop