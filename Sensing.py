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


def _init_search_map(number_of_nodes, number_of_cells):
    default_obs = [{"n_obs": 0, "timestep": -1, "prob": 0.5}]
    search_map = np.full((number_of_nodes, number_of_cells), fill_value=None, dtype=object)
    for i in range(number_of_nodes):
        for j in range(number_of_cells):
            search_map[i, j] = default_obs.copy()
    return search_map


def _compute_occupancy_status(search_map, B, number_of_nodes, number_of_cells):
    """Occupancy flags (any historical prob > B) + per-cell max of LATEST probs."""
    occupancy_status = np.zeros((number_of_nodes, number_of_cells), dtype=int)
    per_cell_max_probs = []
    for col in range(number_of_cells):
        cell_probs = []
        for row in range(number_of_nodes):
            cell_probs.append(search_map[row, col][-1]["prob"])
            probs = [entry["prob"] for entry in search_map[row, col]]
            if any(prob > B for prob in probs):
                occupancy_status[row, col] = 1
        per_cell_max_probs.append(max(cell_probs))
    return occupancy_status, per_cell_max_probs


def _update_detection_timesteps(occupancy_status, target_locations, step,
                                t_all_known, t_bs_knows, t_one_knows):
    if t_all_known == np.inf:
        if len(np.unique(np.where(occupancy_status == 1)[1])) >= len(target_locations):
            t_all_known = step
    if t_bs_knows == np.inf:
        if np.sum(occupancy_status[0]) >= len(target_locations):
            t_bs_knows = step
    if t_one_knows == np.inf:
        if np.any(np.sum(occupancy_status[1:], axis=1) >= len(target_locations)):
            t_one_knows = step
    return t_all_known, t_bs_knows, t_one_knows


def _update_target_detection_times(x, occupancy_status, step):
    """Mutate x.target_detection_times in-place for any newly-detected targets."""
    missing_targets = [t for t in list(x.target_detection_times.keys())
                       if x.target_detection_times[t] is None]
    if len(missing_targets) != 0:
        if np.sum(occupancy_status[:, missing_targets]) != 0:
            for target in missing_targets:
                if occupancy_status[:, target].any():
                    x.target_detection_times[target] = sum(x.time_elapsed_at_steps[:step])


def _finalize_metrics(time_elapsed_at_steps, t_all_known, t_bs_knows, t_one_knows, t_back):
    time_at_least_one = sum(time_elapsed_at_steps[:t_one_knows]) if t_one_knows != np.inf else np.inf
    detection_time = sum(time_elapsed_at_steps[:t_all_known]) if t_all_known != np.inf else np.inf
    inform_time = sum(time_elapsed_at_steps[t_all_known:t_bs_knows]) if t_bs_knows != np.inf else np.inf
    mission_time = sum(time_elapsed_at_steps[:t_back]) if t_back != np.inf else np.inf
    return detection_time, inform_time, mission_time, time_at_least_one


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


def sensing_and_realtime_info_sharing(sol: PathSolution, merging_strategy="onboard",
                                      target_locations=[12], B=0.9, p=0.9, q=0.2):
    """Realtime sensing + merging: continuous positions, per-second connectivity,
    mid-flight merging. Sensing fires only on NEW grid arrivals (seam-deduped).
    Returns the same 7-key metrics dict as the discrete pipeline (contract AD1).
    """
    x = deepcopy(sol)
    info = x.info
    drone_path_matrix = x.real_time_path_matrix[1:, :]
    realtime_x, realtime_y = get_real_paths(x)

    number_of_nodes, timesteps = realtime_x.shape
    number_of_drones = number_of_nodes - 1

    connectivity_matrix = get_real_connectivity_matrix(realtime_x, realtime_y, sol)

    search_map = _init_search_map(info.number_of_nodes, info.number_of_cells)
    cell_occupancy_probabilities = [[] for _ in range(info.number_of_cells)]

    drone_search_status = [True for _ in range(number_of_drones)]
    timestep_bs_knows_all_targets = np.inf
    timestep_at_least_one_drone_knows_all_targets = np.inf
    timestep_all_targets_are_known = np.inf
    timestep_drones_are_back_at_bs = np.inf

    x.mission_time = 0
    x.time_elapsed_at_steps = []
    x.target_detection_times = {target: None for target in target_locations}

    drone_positions = {drone: -1 for drone in range(number_of_drones)}
    discrete_step = -1

    cell_0_x, cell_0_y = sol.get_coords(0)
    cell_bs_x, cell_bs_y = sol.get_coords(-1)

    for step in range(timesteps):

        # --- locate drones; detect grid alignment -------------------------------
        all_on_grid = True
        drone_on_grid = [False] * number_of_drones
        for drone in range(number_of_drones):
            pos_x = realtime_x[drone + 1, step]
            pos_y = realtime_y[drone + 1, step]
            drone_positions[drone] = sol.get_city((pos_x, pos_y))
            on_grid = isCoordinateDiscrete(pos_x, pos_y, sol)
            drone_on_grid[drone] = on_grid
            if not on_grid:
                all_on_grid = False

        # Seam dedup (AD2): get_real_paths uses endpoint-inclusive linspace, so the
        # last column of leg i duplicates the first column of leg i+1. A column only
        # counts as a NEW grid arrival if any coordinate changed since the previous
        # column; duplicates still merge/track time but never sense twice.
        is_new_grid_arrival = all_on_grid and (
            step == 0
            or not (np.array_equal(realtime_x[:, step], realtime_x[:, step - 1])
                    and np.array_equal(realtime_y[:, step], realtime_y[:, step - 1])))
        if is_new_grid_arrival:
            discrete_step += 1

        adj_mat = connectivity_matrix[step]
        conn_comp = connected_components(adj_mat)

        # --- sensing: only on new grid arrivals ---------------------------------
        if is_new_grid_arrival:
            for drone in range(number_of_drones):
                pos = drone_positions[drone]
                if pos == -1:
                    continue
                prior = search_map[drone + 1, pos][-1]["prob"]
                if pos in target_locations:
                    new_prob = p * prior / (p * prior + q * (1 - prior))
                else:
                    new_prob = (1 - p) * prior / ((1 - p) * prior + (1 - q) * (1 - prior))
                # n_obs counted from discrete-path arrivals, matching the discrete
                # pipeline's bookkeeping for the same physical visit (AD2).
                n_obs = len(np.where(drone_path_matrix[drone, :discrete_step + 1] == pos)[0])
                search_map[drone + 1, pos].append(
                    {"n_obs": n_obs, "timestep": discrete_step, "prob": new_prob})

        # --- merging: EVERY second, including mid-flight (AD5) ------------------
        # merging_strategy here is a merge topology ("none"/"onboard"/"gcs"); Task 11 renames it via SensingConfig
        search_map = merge_maps(conn_comp, search_map, merging_strategy)

        # --- occupancy + tracking (shared helpers) ------------------------------
        occupancy_status, per_cell_max = _compute_occupancy_status(
            search_map, B, info.number_of_nodes, info.number_of_cells)
        for col in range(info.number_of_cells):
            cell_occupancy_probabilities[col].append(per_cell_max[col])

        (timestep_all_targets_are_known, timestep_bs_knows_all_targets,
         timestep_at_least_one_drone_knows_all_targets) = _update_detection_timesteps(
            occupancy_status, target_locations, step,
            timestep_all_targets_are_known, timestep_bs_knows_all_targets,
            timestep_at_least_one_drone_knows_all_targets)

        # Realtime columns are ~1-second apart by construction of get_real_paths
        # (dt = ceil(dist/speed) per leg) — see spec Addendum A "not a defect".
        x.time_elapsed_at_steps.append(1)
        x.mission_time += 1

        _update_target_detection_times(x, occupancy_status, step)

        # --- early return-to-base (AD3) ------------------------------------------
        for m in range(number_of_drones):
            if step > 0 and drone_positions[m] == -1:
                continue
            if drone_search_status[m]:
                knows_all = np.sum(occupancy_status[m + 1]) >= len(target_locations)
                is_connected_to_bs = m + 1 in get_connected_node_ids(adj_mat, 0)
                if (timestep_bs_knows_all_targets != np.inf and is_connected_to_bs) or knows_all:
                    drone_search_status[m] = False
                    drone_x_pos = realtime_x[m + 1, step]
                    drone_y_pos = realtime_y[m + 1, step]
                    # continuous return: current pos -> cell 0 -> BS, explicit length
                    # reconciliation instead of the old swallowed try/except
                    ret_x1, ret_y1 = intp_between_coords(drone_x_pos, drone_y_pos,
                                                         cell_0_x, cell_0_y,
                                                         info.max_drone_speed)
                    ret_x2, ret_y2 = intp_between_coords(cell_0_x, cell_0_y,
                                                         cell_bs_x, cell_bs_y,
                                                         info.max_drone_speed)
                    ret_x = np.hstack((ret_x1, ret_x2))
                    ret_y = np.hstack((ret_y1, ret_y2))
                    remaining = timesteps - step
                    if len(ret_x) >= remaining:
                        ret_x, ret_y = ret_x[:remaining], ret_y[:remaining]
                    else:
                        pad = remaining - len(ret_x)
                        ret_x = np.hstack((ret_x, np.full(pad, cell_bs_x)))
                        ret_y = np.hstack((ret_y, np.full(pad, cell_bs_y)))
                    realtime_x[m + 1, step:] = ret_x
                    realtime_y[m + 1, step:] = ret_y
                    # Mirror into the DISCRETE path matrix so PathAnimation (which
                    # re-derives trajectories from it via get_real_paths) shows the
                    # early return (AD3). Adapted from the discrete pipeline's recipe,
                    # but indexed by discrete waypoint (leg), NOT the realtime step —
                    # realtime has ~dt× more columns; using step here would corrupt the mirror.
                    leg = max(discrete_step, 0)
                    current_cell = drone_positions[m] if drone_positions[m] != -1 else 0  # step-0 fallback; trigger can't actually fire there
                    path_to_0 = interpolate_between_cities(x, current_cell, 0)
                    n_cols = x.real_time_path_matrix.shape[1]
                    # Invariant: the return route always fits the remaining columns
                    # (slack is exactly 1 today because interpolate_between_cities
                    # produces Chebyshev-length paths and n_cols was sized with the
                    # same interpolator). Fail loudly if a future change breaks it,
                    # otherwise the mirrored path silently loses its BS arrival and
                    # replay animations break.
                    assert len(path_to_0) <= n_cols - leg, (
                        f"return route ({len(path_to_0)}) exceeds remaining mirror "
                        f"columns ({n_cols - leg})")
                    padded_path = path_to_0 + [-1] * (n_cols - leg - len(path_to_0))
                    x.real_time_path_matrix[m + 1, leg:] = padded_path[:n_cols - leg]

        # --- mission end: every drone home (defect A2 fix: ndarray, not list) ----
        # A drone counts as "home" only when it is at the BS (get_city == -1) AND
        # settled on-grid there. In the realtime pipeline get_city also returns -1
        # for a drone that is merely MID-FLIGHT (off-grid), so the bare
        # `positions == -1` test the discrete pipeline can safely use (its -1 only
        # ever means BS in the discrete matrix) would fire spuriously on step 1
        # when every drone has just left the BS. Requiring on-grid settling
        # restores the intended "all drones back at base" semantics. (deviation:
        # see report)
        positions_now = np.array(list(drone_positions.values()))
        drones_home = (positions_now == -1) & np.array(drone_on_grid, dtype=bool)
        if step > 0 and np.sum(drones_home) == number_of_drones:
            timestep_drones_are_back_at_bs = step
            if step < timesteps - 1:
                realtime_x = realtime_x[:, :step + 1]
                realtime_y = realtime_y[:, :step + 1]
                cell_occupancy_probabilities = [col_probs[:step + 1]
                                                for col_probs in cell_occupancy_probabilities]
            break

    # stash the exact realtime trajectory (incl. truncation) on the solution copy
    x.real_time_x_matrix = realtime_x
    x.real_time_y_matrix = realtime_y

    detection_time, inform_time, mission_time, time_at_least_one = _finalize_metrics(
        x.time_elapsed_at_steps, timestep_all_targets_are_known,
        timestep_bs_knows_all_targets, timestep_at_least_one_drone_knows_all_targets,
        timestep_drones_are_back_at_bs)

    return {"cell occupancy probabilities": cell_occupancy_probabilities,
            "search map": search_map,
            "occupancy status": occupancy_status,
            "detection time": detection_time,
            "inform time": inform_time,
            "mission time": mission_time,
            "time at least one drone knows all targets": time_at_least_one}, x


def sensing_and_discrete_info_sharing(sol: PathSolution, merging_strategy="onboard", target_locations=[12], B=0.9, p=0.9, q=0.2):
    x = deepcopy(sol)
    info = x.info
    final_search_steps = [len(q) - 2 for q in list(x.drone_dict.values())]
    drone_path_matrix = x.real_time_path_matrix[1:, :]
    number_of_drones, timesteps = drone_path_matrix.shape
    connectivity_matrix = x.connectivity_matrix
    cell_occupancy_probabilities = [ [] for _ in range(info.number_of_cells) ]

    search_map = _init_search_map(x.info.number_of_nodes, x.info.number_of_cells)

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
        occupancy_status, per_cell_max = _compute_occupancy_status(
            search_map, B, info.number_of_nodes, info.number_of_cells)
        for col in range(info.number_of_cells):
            cell_occupancy_probabilities[col].append(per_cell_max[col])

        # Track detection/inform time steps
        (timestep_all_targets_are_known, timestep_bs_knows_all_targets,
         timestep_at_least_one_drone_knows_all_targets) = _update_detection_timesteps(
            occupancy_status, target_locations, step,
            timestep_all_targets_are_known, timestep_bs_knows_all_targets,
            timestep_at_least_one_drone_knows_all_targets)

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
        _update_target_detection_times(x, occupancy_status, step)

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
                    x.real_time_path_matrix[m + 1, step:] = padded_path[:timesteps - step]


        if step > 0 and np.sum(positions_now == -1) == number_of_drones:
            timestep_drones_are_back_at_bs = step
            if step < timesteps - 1:
                x.real_time_path_matrix = x.real_time_path_matrix[:, :step + 1]
            break


    detection_time, inform_time, mission_time, time_at_least_one = _finalize_metrics(
        x.time_elapsed_at_steps, timestep_all_targets_are_known,
        timestep_bs_knows_all_targets, timestep_at_least_one_drone_knows_all_targets,
        timestep_drones_are_back_at_bs)

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