from math import atan, atan2
import numpy as np
import pandas as pd
from Results import save_best_solutions
# from PathInfo import *
# from FilePaths import *
from PathFileManagement import load_pickle
from Connectivity import get_connected_node_ids, connected_components, PathSolution, connected_nodes_at_step # get_connected_nodes
# from PathOptimizationModel import *
# from Distance import interpolate_between_cities
# from array_operations import create_array_of_lists
import itertools
from copy import copy, deepcopy
import math
from math import inf, ceil, log10
from scipy.optimize import linear_sum_assignment

from PathAnimation import *
from Time import get_real_connectivity_matrix, get_real_paths, isCoordinateDiscrete, intp_between_coords

# from matplotlib import pyplot as plt
# import seaborn as sns

from PathSolution import *

import sys


def sign(x):
    return 1 if x > 0 else -1 if x < 0 else 0  # Returns 0 if x is exactly 0


def interpolate_between_cities(sol:PathSolution, city_prev, city):

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


    pass


VALID_MERGE_TOPOLOGIES = ("none", "onboard", "gcs")

def merge_maps(conn_comp, search_map, merge_topology="onboard"):
    if merge_topology not in VALID_MERGE_TOPOLOGIES:
        raise ValueError(
            f"Unknown merge_topology {merge_topology!r}; valid: {VALID_MERGE_TOPOLOGIES}")
    if merge_topology == "none":
        return search_map
    number_of_nodes, number_of_cells = search_map.shape
    # number_of_nodes -= 1  # Exclude base station (node 0)

    for clique in conn_comp:
        if merge_topology == "gcs" and 0 not in clique:
            continue

        for cell in range(number_of_cells):
            # Collect all observations in this cell across nodes in the clique
            clique_observations = list(itertools.chain.from_iterable(search_map[node, cell] for node in clique))
            seen = {}
            for d in clique_observations:
                # Convert the dictionary to a frozenset of its items (hashable)
                key = frozenset(d.items())
                if key not in seen:
                    seen[key] = d  # Store the original dictionary
            unique_clique_observations = list(seen.values())

            # if not clique_observations:
            #     continue

            # Find the most recent timestep
            clique_recent_timestep = max(obs["timestep"] for obs in unique_clique_observations)

            # Filter observations to keep only those from the most recent timestep
            clique_recent_observations = [obs for obs in unique_clique_observations if obs["timestep"] == clique_recent_timestep]

            # # Merge info: max n_obs, mean prob
            # merged_obs = {
            #     "timestep": recent_timestep,
            #     "n_obs": max(obs["n_obs"] for obs in recent_obs),
            #     "prob": np.mean([obs["prob"] for obs in recent_obs])
            # }

            # Update each node's observation list in the clique for this cell
            for node in clique:
                node_obs = search_map[node, cell]
                node_recent_timestep = node_obs[-1]["timestep"]
                # node_recent_observations = [obs for obs in node_obs if obs["timestep"] == node_recent_timestep]
                node_recent_observation = node_obs[-1]
                for obs in clique_recent_observations:
                    # print(obs)
                    if obs["timestep"] > node_recent_timestep:
                        node_obs.append(obs)
                    elif obs["timestep"] == node_recent_timestep:
                        # Compare n_obs and prob
                        if obs["n_obs"] > node_recent_observation["n_obs"] or obs["prob"] != node_recent_observation["prob"]:
                            node_obs.append(obs)
                    else:
                        continue

    return search_map


def sensing_and_realtime_info_sharing(sol: PathSolution, merging_strategy="onboard", target_locations=[12], B=0.9, p=0.9, q=0.2):

    # for cell in range(-1, sol.info.number_of_cells):
    #     cell_xy = sol.get_coords(cell)
    #     print(f"Cell {cell} Coordinates: {cell_xy} IsDiscrete: {isCoordinateDiscrete(cell_xy[0], cell_xy[1], sol)}")


    x = deepcopy(sol)
    info = x.info
    path_matrix = x.real_time_path_matrix
    drone_path_matrix = path_matrix[1:, :]
    realtime_x, realtime_y = get_real_paths(x)

    # final_search_steps = [len(q) - 2 for q in list(x.drone_dict.values())]
    # drone_path_matrix = x.real_time_path_matrix[1:, :]
    number_of_nodes, timesteps = realtime_x.shape # drone_path_matrix.shape
    number_of_drones = number_of_nodes - 1  # Exclude base station (node 0)

    # print("->", number_of_drones)
    connectivity_matrix = get_real_connectivity_matrix(realtime_x, realtime_y, sol)

    default_obs = [{"n_obs": 0, "timestep": -1, "prob": 0.5}]
    search_map = np.full(
        (x.info.number_of_nodes, x.info.number_of_cells),
        fill_value=None,
        dtype=object
    )
    for i in range(x.info.number_of_nodes):
        for j in range(x.info.number_of_cells):
            search_map[i, j] = default_obs.copy()  # ensure each cell has its own list
    # search_map = np.array([[ [{"n_obs": 0, "timestep": -1, "prob": 0.5}]
    #                          for _ in range(info.number_of_cells)]
    #                          for _ in range(info.number_of_nodes)], dtype=object)
    

    drone_search_status = [True for _ in range(number_of_drones)]
    timestep_bs_knows_all_targets = np.inf
    timestep_at_least_one_drone_knows_all_targets = np.inf
    timestep_all_targets_are_known = np.inf
    timestep_drones_are_back_at_bs = np.inf

    x.mission_time = 0
    x.time_elapsed_at_steps = []

    x.target_detection_times = {target:None for target in target_locations}

    drones_at_center = {drone:False for drone in range(sol.info.number_of_drones)}
    drone_positions = {drone: -1 for drone in range(number_of_drones)}
    discrete_step = -1

    cell_0_coords = sol.get_coords(0)
    cell_0_x, cell_0_y = cell_0_coords

    cell_bs_coords = sol.get_coords(-1)
    cell_bs_x, cell_bs_y = cell_bs_coords

    for step in range(timesteps):

        # Increment discrete step if all drones are at discrete cell locations
        for drone in range(number_of_drones):
            pos_x = realtime_x[drone+1, step]
            pos_y = realtime_y[drone+1, step]
            # print(f"Coords: ({pos_x, pos_y}), Discrete: {isCoordinateDiscrete(pos_x, pos_y, sol)}")
            drone_positions[drone] = sol.get_city((pos_x, pos_y))
            if isCoordinateDiscrete(pos_x, pos_y, sol):
                drones_at_center[drone] = True
            else:
                drones_at_center[drone] = False
        if all(list(drones_at_center.values())):
            # print(list(zip(realtime_x[:,step].ravel(), realtime_y[:,step].ravel())))
            is_step_discrete = True
            discrete_step += 1
        else:
            is_step_discrete = False

        adj_mat = connectivity_matrix[step]
        conn_comp = connected_components(adj_mat)
        # search_map = merge_maps(conn_comp, search_map, merging_strategy)

        if is_step_discrete:
            for drone in range(number_of_drones):
                # print(f"Position: {drone_positions[drone]}", end=" ")
                if drone_positions[drone] == -1:
                    continue
                # if drone_positions[drone] in target_locations:
                #     print(f"Drone {drone+1} Sensing Target {drone_positions[drone]} Cell ...", end=" ")
                # else:
                #     print(f"Drone {drone+1} Sensing Empty Cell ...", end=" ")
                pos = drone_positions[drone]
                prior = search_map[drone + 1, pos][-1]["prob"]
                if pos in target_locations:
                    new_prob = p * prior / (p * prior + q * (1 - prior))
                else:
                    new_prob = (1 - p) * prior / ((1 - p) * prior + (1 - q) * (1 - prior))
                n_obs = len(np.where(drone_path_matrix[drone, :discrete_step + 1] == pos)[0])
                prev_obs = search_map[drone + 1, pos][-1]["n_obs"]
                # search_map[drone + 1, pos].append({"n_obs": prev_obs+1, "timestep": discrete_step, "prob": new_prob})
                search_map[drone + 1, pos].append({"n_obs": n_obs, "timestep": discrete_step, "prob": new_prob})

                # print(f"Prior prob: {prior}, New prob: {new_prob} New obs: {search_map[drone + 1, pos][-1]}")
            

        # Drones update probabilities if they are at discrete cell locations
        # for drone in range(number_of_drones):
        #     # if step > final_search_steps[drone]:
        #     #     continue
        #     # pos = drone_path_matrix[drone, step]
        #     pos_x = realtime_x[drone+1, step]
        #     pos_y = realtime_y[drone+1, step]
        #     if isCoordinateDiscrete(pos_x, pos_y, sol):
        #         discrete_

        #     pos = sol.get_city((pos_x, pos_y))
        #     if pos == -1:
        #         continue
        #     if isCoordinateDiscrete(pos_x, pos_y, sol):
        #         prior = search_map[drone + 1, pos][-1]["prob"]
        #         if pos in target_locations:
        #             new_prob = p * prior / (p * prior + q * (1 - prior))
        #         else:
        #             new_prob = (1 - p) * prior / ((1 - p) * prior + (1 - q) * (1 - prior))
        #         print(f"Sensing in process at {(pos_x, pos_y)}. Prior prob: {prior}, New prob: {new_prob} ")
        #         # n_obs = np.count_nonzero(drone_path_matrix[drone, :step + 1] == pos)
        #         # n_obs = len(np.where(drone_path_matrix[drone, :step + 1] == pos)[0])
        #         n_obs = len(np.where(drone_path_matrix[drone, :step + 1] == pos)[0])
        #         search_map[drone + 1, pos].append({"n_obs": n_obs, "timestep": step, "prob": new_prob})

        # merging_strategy here is a merge topology ("none"/"onboard"/"gcs"); Task 11 renames it via SensingConfig
        search_map = merge_maps(conn_comp, search_map, merging_strategy)

        # Occupancy Status Check
        occupancy_status = np.zeros((info.number_of_nodes, info.number_of_cells), dtype=int)
        for row in range(info.number_of_nodes):
            for col in range(info.number_of_cells):
                probs = [entry["prob"] for entry in search_map[row, col]]
                if any(prob > B for prob in probs):
                    occupancy_status[row, col] = 1

        # Track detection/inform time steps
        if timestep_all_targets_are_known == np.inf:
            if len(np.unique(np.where(occupancy_status == 1)[1])) >= len(target_locations):
                timestep_all_targets_are_known = step

        if timestep_bs_knows_all_targets == np.inf:
            if np.sum(occupancy_status[0]) >= len(target_locations):
                timestep_bs_knows_all_targets = step

        if timestep_at_least_one_drone_knows_all_targets == np.inf:
            if np.any(np.sum(occupancy_status[1:], axis=1) >= len(target_locations)):
                timestep_at_least_one_drone_knows_all_targets = step

        # Compute time elapsed at step
        # Since this is realtime coordinates, time elapsed will always be 1 second
        x.time_elapsed_at_steps.append(1)
        x.mission_time += 1
        # positions_now = x.real_time_path_matrix[1:, step]
        # if step < timesteps - 1:
        #     positions_next = x.real_time_path_matrix[1:, step + 1]
        #     dists = [info.D[pos_now, pos_next] for pos_now, pos_next in zip(positions_now, positions_next)]
        #     max_dist = max(dists)
        #     time_elapsed = max_dist / info.max_drone_speed
        #     x.time_elapsed_at_steps.append(time_elapsed)
        #     x.mission_time += time_elapsed

        # Check if targets are detected and update target detection timesteps if detected
        missing_targets = [target for target in list(x.target_detection_times.keys()) if x.target_detection_times[target] is None]
        if len(missing_targets) != 0:
            if np.sum(occupancy_status[:,missing_targets]) != 0:
                for target in missing_targets:
                    if occupancy_status[:, target].any():
                        x.target_detection_times[target] = sum(x.time_elapsed_at_steps[:step])




        # Drones that know enough info return to base
        for m in range(number_of_drones):
            drone_cell_pos = drone_positions[m]
            drone_xy_pos = (realtime_x[m+1, step], realtime_y[m+1, step])
            # if step > 0 and sol.get_city((realtime_x[m+1, step], realtime_y[m+1, step]))==-1: # x.real_time_path_matrix[m + 1, step] == -1:
            if step > 0 and drone_cell_pos==-1: # x.real_time_path_matrix[m + 1, step] == -1:
                continue
            if drone_search_status[m]:
                knows_all = np.sum(occupancy_status[m + 1]) >= len(target_locations)
                is_connected_to_bs = m + 1 in get_connected_node_ids(adj_mat, 0)
                if timestep_bs_knows_all_targets != np.inf and is_connected_to_bs or knows_all:
                    drone_search_status[m] = False
                    # Make drone m return to cell 0 and then to Base Station (to avoid going out of the map)
                    drone_x_pos, drone_y_pos = drone_xy_pos
                    x_mid_1, y_mid_1 = intp_between_coords(drone_x_pos, drone_y_pos, cell_0_x, cell_0_y, info.max_drone_speed)
                    x_mid_2, y_mid_2 = intp_between_coords(cell_0_x, cell_0_y, cell_bs_x, cell_bs_y, info.max_drone_speed)
                    x_mid, y_mid = np.hstack((x_mid_1, x_mid_2)), np.hstack((y_mid_1, y_mid_2))
                    x_mid, y_mid = np.hstack((x_mid, np.full(timesteps-(step+len(x_mid)), fill_value=cell_bs_x))), np.hstack((y_mid, np.full(timesteps-(step+len(y_mid)), fill_value=cell_bs_y)))

                    # dist_to_0 = info.D[drone_cell_pos, 0]
                    # dt = ceil(dist_to_0 / info.max_drone_speed)
                    # drone_x_path_to_0, drone_y_path_to_0 = np.hstack((realtime_x[m+1, :step], np.linspace(drone_x_pos, cell_0_x, dt, endpoint=False))), np.hstack((realtime_y[m+1, :step], np.linspace(drone_y_pos, cell_0_y, dt, endpoint=False)))
                    # dist_from_cell_0_to_bs = info.D[0, -1]
                    # dt_cell_0_to_bs = ceil(dist_from_cell_0_to_bs / info.max_drone_speed)
                    # drone_x_path_to_bs, drone_y_path_to_bs = np.hstack((drone_x_path_to_0, np.linspace(cell_0_x, cell_bs_x, dt_cell_0_to_bs, endpoint=False))), np.hstack((drone_y_path_to_0, np.linspace(cell_0_y, cell_bs_y, dt_cell_0_to_bs, endpoint=False)))
                    # drone_x_path_to_bs = np.hstack((np.linspace(drone_x_pos, cell_0_x, dt), np.linspace(cell_0_x, cell_bs_x, ceil(info.D[0, -1]))[1:]))
                    # drone_y_path_to_bs = np.hstack((np.linspace(drone_y_pos, cell_0_y, dt), np.linspace(cell_0_y, cell_bs_y, ceil(info.D[0, -1]))[1:]))
                    try:
                        realtime_x[m+1][step:], realtime_y[m+1][step:]  = x_mid, y_mid
                        # realtime_x[m+1] = np.hstack((drone_x_path_to_bs, np.full(shape=timesteps - len(drone_x_path_to_bs), fill_value=cell_bs_x)))
                        # realtime_y[m+1] = np.hstack((drone_y_path_to_bs, np.full(shape=timesteps - len(drone_y_path_to_bs), fill_value=cell_bs_y)))
                    except:
                        print("ERROR: Could not update drone path matrix.")
                        # with np.printoptions(threshold=np.inf):
                        #     if drone_path_matrix[m, discrete_step+1]==0:
                        #         print("OG:")
                        #         print(list(zip(realtime_x[m+1, step:].ravel(), realtime_y[m+1, step:].ravel())))
                        #         print("PATH TO BS")
                        #         print(list(zip(drone_x_path_to_bs[step:].ravel(), drone_y_path_to_bs[step:].ravel())))
                        # print("LENGTHS:", timesteps, len(drone_x_path_to_bs), len(drone_y_path_to_bs))


                    # # current_coords = np.array([realtime_x[m+1, step], realtime_y[m, step]])
                    # cell_0_coords = sol.get_coords(0)
                    # coords_diff = cell_0_coords - drone_xy_pos
                    # # current_pos = sol.get_city(current_coords)  # x.real_time_path_matrix[m + 1, step]
                    # path_to_0 = interpolate_between_cities(x, drone_cell_pos, 0)
                    # theta = atan2(coords_diff[1], coords_diff[0])
                    # coords_to_0 = [sol.get_coords(city) for city in path_to_0]
                    # # discrete_x_coords_to_0 = [coords[0] for coords in coords_to_0]
                    # realtime_x_coords_to_0 = [np.arange(current_coords[0], cell_0_coords[0], info.max_drone_speed*np.cos(theta))]
                    # realtime_y_coords_to_0 = [np.arange(current_coords[1], cell_0_coords[1], info.max_drone_speed*np.sin(theta))]
                    # # discrete_y_coords_to_0 = [coords[1] for coords in coords_to_0]
                    # # padded_path = path_to_0 + [-1] * (timesteps - len(path_to_0))
                    # padded_x_coords = realtime_x_coords_to_0 + [sol.get_coords(-1)[0]] * (timesteps - len(realtime_x_coords_to_0))
                    # padded_y_coords = realtime_y_coords_to_0 + [sol.get_coords(-1)[1]] * (timesteps - len(realtime_y_coords_to_0))
                    # # print(padded_path)
                    # # x.real_time_path_matrix[m + 1, step:] = padded_path
                    # print(realtime_x[m+1, step:])
                    # print(padded_x_coords[:timesteps - step])
                    # realtime_x[m+1, step:] = padded_x_coords[:timesteps - step]
                    # realtime_y[m+1, step:] = padded_y_coords[:timesteps - step]
                    # # x.real_time_path_matrix[m + 1, step:] = padded_path[:timesteps - step]
                    # # print(f"Shortened Drone Path: {x.real_time_path_matrix[m + 1]}")

        
        positions_now = list(drone_positions.values())
        # x_positions_now = realtime_x[:, step]
        # y_positions_now = realtime_y[:, step]
        # positions_now = [sol.get_city((x_pos, y_pos)) for x_pos, y_pos in zip(x_positions_now, y_positions_now)]
        if discrete_step > 0 and np.sum(positions_now == -1) == number_of_drones:
            timestep_drones_are_back_at_bs = step
            if step < timesteps - 1:
                # x.real_time_path_matrix = x.real_time_path_matrix[:, :step + 1]
                realtime_x = realtime_x[:, :step + 1]
                realtime_y = realtime_y[:, :step + 1]
            break


    detection_time = inform_time = time_at_least_one = np.inf
    # Final time metrics
    time_at_least_one = sum(x.time_elapsed_at_steps[:timestep_at_least_one_drone_knows_all_targets]) \
        if timestep_at_least_one_drone_knows_all_targets != np.inf else np.inf
    detection_time = sum(x.time_elapsed_at_steps[:timestep_all_targets_are_known]) \
        if timestep_all_targets_are_known != np.inf else np.inf
    inform_time = sum(x.time_elapsed_at_steps[timestep_all_targets_are_known : timestep_bs_knows_all_targets]) \
        if timestep_bs_knows_all_targets != np.inf else np.inf
    mission_time = sum(x.time_elapsed_at_steps[:timestep_drones_are_back_at_bs]) \
        if timestep_drones_are_back_at_bs != np.inf else np.inf
    

    return  {"search map": search_map, "occupancy status": occupancy_status, "detection time": detection_time, "inform time": inform_time, "mission time": mission_time, "time at least one drone knows all targets": time_at_least_one}, x


def sensing_and_discrete_info_sharing(sol: PathSolution, merging_strategy="onboard", target_locations=[12], B=0.9, p=0.9, q=0.2):
    x = deepcopy(sol)
    info = x.info
    final_search_steps = [len(q) - 2 for q in list(x.drone_dict.values())]
    drone_path_matrix = x.real_time_path_matrix[1:, :]
    number_of_drones, timesteps = drone_path_matrix.shape
    connectivity_matrix = x.connectivity_matrix
    cell_occupancy_probabilities = [ [] for _ in range(info.number_of_cells) ]

    default_obs = [{"n_obs": 0, "timestep": -1, "prob": 0.5}]
    search_map = np.full(
        (x.info.number_of_nodes, x.info.number_of_cells),
        fill_value=None,
        dtype=object
    )
    for i in range(x.info.number_of_nodes):
        for j in range(x.info.number_of_cells):
            search_map[i, j] = default_obs.copy()  # ensure each cell has its own list
    # search_map = np.array([[ [{"n_obs": 0, "timestep": -1, "prob": 0.5}]
    #                          for _ in range(info.number_of_cells)]
    #                          for _ in range(info.number_of_nodes)], dtype=object)
    

    drone_search_status = [True for _ in range(number_of_drones)]
    timestep_bs_knows_all_targets = np.inf
    timestep_at_least_one_drone_knows_all_targets = np.inf
    timestep_all_targets_are_known = np.inf
    timestep_drones_are_back_at_bs = np.inf

    x.mission_time = 0
    x.time_elapsed_at_steps = []

    x.target_detection_times = {target:None for target in target_locations}

    for step in range(timesteps):

        adj_mat = connectivity_matrix[step]
        conn_comp = connected_components(adj_mat)
        # search_map = merge_maps(conn_comp, search_map, merging_strategy)

        # Drones update probabilities
        for drone in range(number_of_drones):
            if step > final_search_steps[drone]:
                continue
            pos = drone_path_matrix[drone, step]
            if pos == -1:
                continue
            prior = search_map[drone + 1, pos][-1]["prob"]
            if pos in target_locations:
                new_prob = p * prior / (p * prior + q * (1 - prior))
            else:
                new_prob = (1 - p) * prior / ((1 - p) * prior + (1 - q) * (1 - prior))
            # n_obs = np.count_nonzero(drone_path_matrix[drone, :step + 1] == pos)
            n_obs = len(np.where(drone_path_matrix[drone, :step + 1] == pos)[0])
            search_map[drone + 1, pos].append({"n_obs": n_obs, "timestep": step, "prob": new_prob})

        # merging_strategy here is a merge topology ("none"/"onboard"/"gcs"); Task 11 renames it via SensingConfig
        search_map = merge_maps(conn_comp, search_map, merging_strategy)

        # Occupancy Status Check
        occupancy_status = np.zeros((info.number_of_nodes, info.number_of_cells), dtype=int)
        for col in range(info.number_of_cells):
            cell_probs = []
            for row in range(info.number_of_nodes):
                cell_probs.append(search_map[row, col][-1]["prob"])
                probs = [entry["prob"] for entry in search_map[row, col]]
                if any(prob > B for prob in probs):
                    occupancy_status[row, col] = 1
            max_cell_prob = max(cell_probs)
            cell_occupancy_probabilities[col].append(max_cell_prob)

        # for row in range(info.number_of_nodes):
        #     for col in range(info.number_of_cells):
        #         probs = [entry["prob"] for entry in search_map[row, col]]
        #         if any(prob > B for prob in probs):
        #             occupancy_status[row, col] = 1

        # Track detection/inform time steps
        if timestep_all_targets_are_known == np.inf:
            if len(np.unique(np.where(occupancy_status == 1)[1])) >= len(target_locations):
                timestep_all_targets_are_known = step

        if timestep_bs_knows_all_targets == np.inf:
            if np.sum(occupancy_status[0]) >= len(target_locations):
                timestep_bs_knows_all_targets = step

        if timestep_at_least_one_drone_knows_all_targets == np.inf:
            if np.any(np.sum(occupancy_status[1:], axis=1) >= len(target_locations)):
                timestep_at_least_one_drone_knows_all_targets = step

        # Compute time elapsed at step
        positions_now = x.real_time_path_matrix[1:, step]
        if step < timesteps - 1:
            positions_next = x.real_time_path_matrix[1:, step + 1]
            dists = [info.D[pos_now, pos_next] for pos_now, pos_next in zip(positions_now, positions_next)]
            max_dist = max(dists)
            time_elapsed = max_dist / info.max_drone_speed
            x.time_elapsed_at_steps.append(time_elapsed)
            x.mission_time += time_elapsed

        # Check if targets are detected and update target detection timesteps if detected
        missing_targets = [target for target in list(x.target_detection_times.keys()) if x.target_detection_times[target] is None]
        if len(missing_targets) != 0:
            if np.sum(occupancy_status[:,missing_targets]) != 0:
                for target in missing_targets:
                    if occupancy_status[:, target].any():
                        x.target_detection_times[target] = sum(x.time_elapsed_at_steps[:step])


        # Drones that know enough info return to base
        for m in range(number_of_drones):
            if step > 0 and x.real_time_path_matrix[m + 1, step] == -1:
                continue
            if drone_search_status[m]:
                knows_all = np.sum(occupancy_status[m + 1]) >= len(target_locations)
                is_connected_to_bs = m + 1 in get_connected_node_ids(adj_mat, 0)
                if timestep_bs_knows_all_targets != np.inf and is_connected_to_bs or knows_all:

                    drone_search_status[m] = False
                    current_pos = x.real_time_path_matrix[m + 1, step]
                    path_to_0 = interpolate_between_cities(x, current_pos, 0)
                    padded_path = path_to_0 + [-1] * (timesteps - len(path_to_0))
                    # print(padded_path)
                    # x.real_time_path_matrix[m + 1, step:] = padded_path
                    x.real_time_path_matrix[m + 1, step:] = padded_path[:timesteps - step]
                    # print(f"Shortened Drone Path: {x.real_time_path_matrix[m + 1]}")


        if step > 0 and np.sum(positions_now == -1) == number_of_drones:
            timestep_drones_are_back_at_bs = step
            if step < timesteps - 1:
                x.real_time_path_matrix = x.real_time_path_matrix[:, :step + 1]
            break


    detection_time = inform_time = time_at_least_one = np.inf
    # Final time metrics
    time_at_least_one = sum(x.time_elapsed_at_steps[:timestep_at_least_one_drone_knows_all_targets]) \
        if timestep_at_least_one_drone_knows_all_targets != np.inf else np.inf
    detection_time = sum(x.time_elapsed_at_steps[:timestep_all_targets_are_known]) \
        if timestep_all_targets_are_known != np.inf else np.inf
    inform_time = sum(x.time_elapsed_at_steps[timestep_all_targets_are_known : timestep_bs_knows_all_targets]) \
        if timestep_bs_knows_all_targets != np.inf else np.inf
    mission_time = sum(x.time_elapsed_at_steps[:timestep_drones_are_back_at_bs]) \
        if timestep_drones_are_back_at_bs != np.inf else np.inf


    return  {"cell occupancy probabilities": cell_occupancy_probabilities, "search map": search_map, "occupancy status": occupancy_status, "detection time": detection_time, "inform time": inform_time, "mission time": mission_time, "time at least one drone knows all targets": time_at_least_one}, x




# mtsp_sol = load_pickle("Results/Solutions/SOO_GA_MTSP_g_8_a_50_n_4_v_2.5_r_2_nvisits_3-SolutionObjects.pkl")[0]
# # print(sample_sol.percentage_connectivity)
# tct_sols = load_pickle("Results/Solutions/MOO_NSGA2_TCT_g_8_a_50_n_4_v_2.5_r_2_nvisits_3-SolutionObjects.pkl")
# tc_sols = load_pickle("Results/Solutions/MOO_NSGA2_TC_g_8_a_50_n_4_v_2.5_r_2_nvisits_3-SolutionObjects.pkl")
# tct_F = pd.read_pickle(f"{objective_values_filepath}MOO_NSGA2_TCT_g_8_a_50_n_4_v_2.5_r_2_nvisits_3-ObjectiveValues.pkl")
# tc_F = pd.read_pickle(f"{objective_values_filepath}MOO_NSGA2_TC_g_8_a_50_n_4_v_2.5_r_2_nvisits_3-ObjectiveValues.pkl")
# tct_best_time_sol, tct_best_conn_sol = tct_sols[tct_F["Mission Time"].idxmin()], tct_sols[tct_F["Percentage Connectivity"].idxmin()]
# tc_best_time_sol, tc_best_conn_sol = tc_sols[tc_F["Mission Time"].idxmin()], tc_sols[tc_F["Percentage Connectivity"].idxmin()]

# mtsp_metrics = sensing_and_info_sharing(mtsp_sol, merging_strategy="ondrone", target_locations=[12, 50, 63], B=0.9, p=0.8, q=0.2)
# tct_best_time_metrics = sensing_and_info_sharing(tct_best_time_sol, merging_strategy="ondrone", target_locations=[12, 50, 63], B=0.9, p=0.8, q=0.2)
# tc_best_time_metrics = sensing_and_info_sharing(tc_best_time_sol, merging_strategy="ondrone", target_locations=[12, 50, 63], B=0.9, p=0.8, q=0.2)
# tct_best_conn_metrics = sensing_and_info_sharing(tct_best_conn_sol, merging_strategy="ondrone", target_locations=[12, 50, 63], B=0.9, p=0.8, q=0.2)
# tc_best_conn_metrics = sensing_and_info_sharing(tc_best_conn_sol, merging_strategy="ondrone", target_locations=[12, 50, 63], B=0.9, p=0.8, q=0.2)
# print(f"MTSP Metrics:\n{mtsp_metrics}")
# print(f"TCT Best Time Metrics:\n{tct_best_time_metrics}")
# print(f"TC Best Time Metrics:\n{tc_best_time_metrics}")
# print(f"TCT Best Conn Metrics:\n{tct_best_conn_metrics}")
# print(f"TC Best Conn Metrics:\n{tc_best_conn_metrics}")