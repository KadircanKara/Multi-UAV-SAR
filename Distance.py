import numpy as np
from math import floor, sqrt
import copy

# from Time import calculate_visit_times, get_real_paths

from PathSolution import *

def get_path_matrix(sol:PathSolution):
    info = sol.info
    return np.where(sol.real_time_path_matrix != -1, sol.real_time_path_matrix % info.number_of_cells, sol.real_time_path_matrix)


def get_total_distance(sol:PathSolution):
    if not sol.total_distance:
        if not sol.subtour_lengths:
            calculate_subtour_lengths(sol)
        sol.total_distance = sum(sol.subtour_lengths.values())
    return sol.total_distance


# def get_total_distance_with_revisit_penalty(sol:PathSolution, penalty_cofactor=100):
#     num_revisits = 0
#     # Get cell visits
#     real_x, real_y = get_real_paths(sol)
#     number_of_nodes, time_slots = real_x.shape
#     cell_visits = np.zeros(sol.info.number_of_cells, dtype=int)
#     for step in range(time_slots):
#         x_at_step, y_at_step = real_x[:,step].reshape((number_of_nodes,1)), real_y[:,step].reshape((number_of_nodes,1))
#         # print(f"x_at_step: {x_at_step}, y_at_step: {y_at_step}")
#         xy_at_step = np.hstack((x_at_step, y_at_step))
#         for xy in xy_at_step:
#             # print("xy:",xy)
#             cell = sol.get_city(xy)
#             cell_visits[cell] += 1
#     # Cell visits variable: cell_visits
#     for cell in range(sol.info.number_of_cells):
#         # print("-->",cell_visits[cell])
#         if cell_visits[cell] > sol.info.max_visits:
#             num_revisits += (cell_visits[cell] - sol.info.max_visits)
#     return sol.total_distance + penalty_cofactor * num_revisits


def get_subtour_range(sol:PathSolution):
    if not sol.longest_subtour:
        get_longest_subtour(sol)
    if not sol.shortest_subtour:
        get_shortest_subtour(sol)
    sol.subtour_range = sol.longest_subtour - sol.shortest_subtour
    return sol.subtour_range


def get_longest_subtour(sol:PathSolution):
    if not sol.longest_subtour:
        if not sol.subtour_lengths:
            calculate_subtour_lengths(sol)
        sol.longest_subtour = max(sol.subtour_lengths)
    return sol.longest_subtour


def get_shortest_subtour(sol:PathSolution):
    if not sol.shortest_subtour:
        if not sol.subtour_lengths:
            calculate_subtour_lengths(sol)
        sol.shortest_subtour = min(sol.subtour_lengths)
        # print(f"shortest subtour {sol.shortest_subtour}")
    return sol.shortest_subtour


def calculate_subtour_lengths(sol:PathSolution):

    if not sol.subtour_lengths:

        info = sol.info

        path_matrix = sol.real_time_path_matrix

        Nd, time_steps = path_matrix.shape
        Nd -= 1 # Remove base station

        subtour_lengths = dict()

        for i in range(info.number_of_drones):
            drone_path = path_matrix[i+1]
            drone_dist = 0
            for j in range(time_steps-1):
                drone_dist += info.D[drone_path[j],drone_path[j+1]]
            subtour_lengths[i] = drone_dist

        sol.subtour_lengths = subtour_lengths

    return sol.subtour_lengths


def calculate_number_of_long_jumps(sol:PathSolution):

    if not sol.long_jump_violations :
        path = np.array(sol.path)
        info = sol.info
        path = np.where(path != -1, path % info.number_of_cells, path)
        long_jump_violations = 0
        for i in range(len(path)-1):
            if info.D[path[i], path[i+1]] > info.cell_side_length * sqrt(2):
                long_jump_violations += 1
        sol.long_jump_violations = long_jump_violations
    return sol.long_jump_violations

        # path_matrix = sol.real_time_path_matrix
        # long_jump_violations = 0
        # for i in range(info.number_of_drones):
        #     for j in range(path_matrix.shape[1] - 1):
        #         if info.D[path_matrix[i+1,j],path_matrix[i+1,j+1]] > info.cell_side_length * sqrt(2):
        #             long_jump_violations += 1
    #     sol.long_jump_violations = long_jump_violations
    # return sol.long_jump_violations


def min_cells_per_drone_constr(sol:PathSolution):

    info = sol.info

    # if "Percentage Connectivity" not in info.model["F"]: # More like mtsp, so the drones' flight times may be closer to each other
    #     start_points = sol.start_points
    #     last_start_point_subtractor = info.min_visits * info.number_of_cells
    # else:
    #     start_points = sol.start_points[::] # 1 drone will fly significantly more and others will end the tour early to contribute to percentage connectivity
    #     last_start_point_subtractor = sol.start_points[-1]

    start_points = sol.start_points
    last_start_point_subtractor = info.min_visits * info.number_of_cells

    cells_per_drone = []

    for i in range(len(start_points)-1):
        num_cells = start_points[i+1] - start_points[i]
        cells_per_drone.append(num_cells)

    cells_per_drone.append(last_start_point_subtractor - start_points[-1])

    constr = info.number_of_cells * info.min_visits // info.number_of_drones - 1

    # print("start_points:", start_points)
    # print(f"constr: {constr}  max cells per drone: {max(cells_per_drone)}")

    return -min(cells_per_drone) + constr




    # if "Percentage Connectivity" not in info.model["F"]: # More like mtsp, so the drones' flight times may be closer to each other

    #     cells_per_drone = []

    #     for i in range(info.number_of_drones-1):
    #         num_cells = sol.start_points[i+1] - sol.start_points[i]
    #         cells_per_drone.append(num_cells)
    #     cells_per_drone.append(info.min_visits*info.number_of_cells-sol.start_points[-1])

    #     constr = info.number_of_cells * info.min_visits // info.number_of_drones

    #     # return max(cells_per_drone) - min(cells_per_drone) - constr
    #     return max(cells_per_drone) - constr

    # else: # 1 drone will fly significantly more and others will end the tour early to contribute to percentage connectivity

    #     search_node_start_points = sol.start_points[:-1]

    #     cells_per_search_drone = []

    #     for i in range(len(search_node_start_points)-1):
    #         num_cells =

    # info = sol.info

    # cells_per_drone = []

    # constr = info.Nc * info.min_visits // info.Nd

    # print(f"drone dict keys: {drone_dict.keys()}")

    # for i in range(info.Nd):
    #     drone_path = drone_dict[i][2:-2] # To eliminate -1 and 0 at the start and the end
    #     cells_per_drone.append(len(drone_path))

    # sol.cells_per_drone_constr = max(cells_per_drone) - min(cells_per_drone) - constr

    # # print("cell per drone cv:", max(cells_per_drone) - min(cells_per_drone) - constr)

    # return sol.cells_per_drone_constr


def long_jumps_eq_constr(sol:PathSolution):

    if not sol.long_jump_violations :
        calculate_number_of_long_jumps(sol)

    # return sol.long_jump_violations / sol.info.number_of_drones * sol.info.min_visits
    return sol.long_jump_violations

def long_jumps_ieq_constr(sol:PathSolution, constr=28):

    constr = 28*sol.info.min_visits

    info = sol.info

    # constr = (7*info.min_visits) * info.number_of_drones

    # constr = 5 * info.number_of_drones * info.min_visits

    if not sol.long_jump_violations :
        calculate_number_of_long_jumps(sol)

    # constr = 2*sol.info.Nd

    # return sol.long_jump_violations - sol.info.Nd * sol.info.min_visits * 2
    #

    # print("# Long Jumps:", sol.long_jump_violations)

    return sol.long_jump_violations - constr



    # print("long jump cv:", long_jump_violations)

    # sol.long_jump_violations = long_jump_violations - constr

    # cofactor = 2
    # # bias = 5
    # constr = info.Nd * info.min_visits * cofactor      # 33 for Nd=8 min_visits=2 (cofactor=2.0625)
    # #                                                      37.5 for N=8 min_visits=3 (cofactor=1.5625)
    # #                                                      107 for Nd=16 min_visits=3 (cofactor=4.45)
    #
    # sol.long_jump_violations_constr = long_jump_violations - constr
    #
    # return sol.long_jump_violations_constr


def max_subtour_range_constr(sol:PathSolution):
    if not sol.subtour_range:
        get_subtour_range(sol)
    return sol.subtour_range - sol.info.grid_size*(sol.info.cell_side_length*sqrt(2))


def max_longest_subtour_constr(sol:PathSolution):
    if not sol.longest_subtour :
        get_longest_subtour(sol)
    return sol.longest_subtour - sol.info.max_subtour_length_threshold


def min_longest_subtour_constr(sol:PathSolution):
    if not sol.shortest_subtour :
        get_shortest_subtour(sol)
    return - sol.shortest_subtour + sol.info.min_subtour_length_threshold


def get_coords(sol:PathSolution, cell):

    grid_size = sol.info.grid_size
    A = sol.info.cell_side_length

    if cell == -1:
        x = -A / 2
        y = -A / 2
    else:
        # x = ((cell % n) % self.info.grid_size + 0.5) * self.info.cell_len
        x = (cell % grid_size + 0.5) * A
        # y = ((cell % n) // self.info.grid_size + 0.5) * self.info.cell_len
        y = (cell // grid_size + 0.5) * A
    return np.array([x, y])


def get_city(coords, grid_size, A):

    if coords[0] < 0 and coords[1] < 0:
        return -1
    else:
        x, y = coords
        return floor(y / A) * grid_size + floor(x / A)


def get_x_coords(cell, grid_size, A):

    if cell == -1:
        x = -A / 2
    else:
        # x = ((cell % n) % self.info.grid_size + 0.5) * self.info.cell_len
        x = (cell %grid_size + 0.5) * A
    return x


def get_y_coords(self, cell, grid_size, A):

    if cell == -1:
        y = -A / 2
    else:
        # y = ((cell % n) // grid_size + 0.5) * A
        y = (cell // grid_size + 0.5) * self.info.cell_side_length
    return y
