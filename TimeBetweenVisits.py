from PathSolution import *
from Time import *
from PathFileManagement import load_pickle

from math import sqrt
import pandas as pd

def get_visit_times(sol:PathSolution):
    info = sol.info
    drone_path_matrix = sol.real_time_path_matrix[1:,:]
    visit_times = [[] for _ in range(info.number_of_cells)]
    # print(f"Path Matrix:\n{drone_path_matrix}")
    for cell in range(info.number_of_cells):
        # print(f"cell {cell} visit steps: {np.where(sol.real_time_path_matrix==cell)[1].tolist()}")
        visit_times[cell] = np.sort(np.where(drone_path_matrix==cell)[1])[:info.n_visits] # Last bit is to exclude hovering steps

    # print("visit times:", visit_times)

    sol.visit_times = visit_times

    return visit_times

def calculate_tbv(sol:PathSolution):

    debug_mode = True

    if sol.info.n_visits > 1:
        # Calculate REAL-TIME TBV instead of timestep TBV
        real_time_tbv = []
        for cell_visit_steps in sol.visit_times:
            print("cell visit steps:", cell_visit_steps) if debug_mode else None
            real_time_tbv.append([]) # Initialize the list for the cell
            for step in range(len(cell_visit_steps)-1):
                time_elapsed_between_steps = 0
                current_step = cell_visit_steps[step]
                next_step = cell_visit_steps[step+1]
                # print("step:", current_step, "next step:", next_step) if debug_mode else None
                # Check if at least one drone goes diagonally, if so, add cell_length*sqrt(2)/max_drone_speed for that step, else, ad cell_side_length/max_drone_speed
                mid_path_matrix = sol.real_time_path_matrix[1:,current_step:next_step+1].T #
                # print("mid path matrix:", mid_path_matrix) if debug_mode else None
                for i in range(len(mid_path_matrix)-1):
                    diagonal_path_exists = False
                    drone_positions = mid_path_matrix[i]
                    next_drone_positions = mid_path_matrix[i+1]
                    # print("drone positions:", drone_positions, "next drone positions:", next_drone_positions) if debug_mode else None
                    for drone_no in range(len(drone_positions)):
                        if sol.info.D[drone_positions[drone_no], next_drone_positions[drone_no]] > sol.info.cell_side_length:
                            diagonal_path_exists = True
                            break
                    # print("diagonal path exists:", diagonal_path_exists) if debug_mode else None
                    if diagonal_path_exists:
                        time_elapsed_between_steps += sol.info.cell_side_length*sqrt(2)/sol.info.max_drone_speed
                    else:
                        time_elapsed_between_steps += sol.info.cell_side_length/sol.info.max_drone_speed
                    # print("updated time elapsed between steps:", time_elapsed_between_steps) if debug_mode else None
                real_time_tbv[-1].append(time_elapsed_between_steps)
            print("real time tbv:", real_time_tbv[-1]) if debug_mode else None


        tbv = [np.diff(x) for x in sol.visit_times]
    else:
        tbv = [[0] for _ in range(sol.info.number_of_cells)]
        real_time_tbv = tbv.copy()

    sol.tbv = tbv
    sol.real_time_tbv = real_time_tbv

    # print("tbv:", tbv)

    return tbv

def calculate_mean_tbv(sol:PathSolution):
    # mean_tbv = list(map(lambda x: np.mean(x), sol.tbv))
    mean_tbv = list(map(lambda x: np.mean(x), sol.real_time_tbv))
    sol.mean_tbv = mean_tbv
    sol.max_mean_tbv = max(sol.mean_tbv)

    # print("mean tbv:", mean_tbv, "max mean tbv:", max(mean_tbv))
    return sol.mean_tbv
