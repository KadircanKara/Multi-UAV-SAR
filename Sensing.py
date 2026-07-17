from math import atan, atan2, ceil
import numpy as np
import pandas as pd
# from PathInfo import *
# from FilePaths import *
from Connectivity import get_connected_node_ids, connected_components, PathSolution
# from PathOptimizationModel import *
# from Distance import interpolate_between_cities
from copy import deepcopy

from Time import get_real_connectivity_matrix, get_real_paths, isCoordinateDiscrete, intp_between_coords

# from matplotlib import pyplot as plt
# import seaborn as sns

from PathSolution import *


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


VALID_MERGE_TOPOLOGIES = ("none", "onboard", "gcs")


def _init_search_map(number_of_nodes, number_of_cells):
    """Per-(node, cell) list of the sensing EVENTS that node knows about.

    An empty list means "nothing observed yet" — the prior lives in
    _fused_odds, not in a sentinel entry. (The old prior sentinel carried no
    information any reader used, and one shallow-copied dict was aliased into
    every node and every cell.)
    """
    search_map = np.full((number_of_nodes, number_of_cells), fill_value=None, dtype=object)
    for i in range(number_of_nodes):
        for j in range(number_of_cells):
            search_map[i, j] = []
    return search_map


def _likelihood_ratio(positive, p, q):
    """Likelihood ratio of one sensing event: P(obs|target)/P(obs|no target)."""
    return (p / q) if positive else ((1.0 - p) / (1.0 - q))


def _fused_odds(observations, p, q, prior=0.5):
    """Posterior odds from the prior + every sensing EVENT in the list.
    An empty list is the prior. Order-independent (odds-form product of
    independent measurements). May overflow to inf for very long positive
    chains — callers treat inf as certainty."""
    odds = prior / (1.0 - prior)
    for obs in observations:
        odds *= _likelihood_ratio(obs["positive"], p, q)
    return odds


def _fused_belief(observations, p, q, prior=0.5):
    """Posterior probability from _fused_odds; inf odds → 1.0."""
    odds = _fused_odds(observations, p, q, prior)
    if np.isinf(odds):
        return 1.0
    return odds / (1.0 + odds)


def _matrix_column_arrival_steps(path_matrix, D, speed):
    """Realtime step at which each path-matrix column's positions are reached.

    Mirrors get_real_paths: leg j (col j -> j+1) spans dt = ceil(max-over-drones
    dist / speed) endpoint-inclusive realtime columns, so its arrival lands dt-1
    steps after the leg's start offset. dt=0 legs (every drone repeats its cell
    — e.g. the consecutive re-visit columns nvisits>1 matrices encode) add NO
    realtime columns; their arrival collapses onto the previous arrival step so
    the visits still sense (RT-1 fix)."""
    n_rows, n_cols = path_matrix.shape
    arrivals = [0]
    offset = 0
    for j in range(n_cols - 1):
        max_dist = max(D[path_matrix[r, j], path_matrix[r, j + 1]]
                       for r in range(1, n_rows))
        dt = ceil(max_dist / speed)
        if dt == 0:
            arrivals.append(arrivals[-1])
        else:
            arrivals.append(offset + dt - 1)
            offset += dt
    return arrivals


def _refresh_beliefs(search_map, beliefs, dirty, p, q):
    """Re-fold the cached belief of every (node, cell) whose event list changed
    this step, then clear the dirty set.

    Only sensing (an append) and merge_maps (a list rebuild) ever mutate
    search_map, and both record what they touched — so re-folding the whole
    nodes x cells grid every step, as this used to do, redid identical work for
    every cell that saw no event. That was the dominant cost of a pipeline that
    runs once per pymoo fitness evaluation.
    """
    for node, cell in dirty:
        beliefs[node, cell] = _fused_belief(search_map[node, cell], p, q)
    dirty.clear()


def _occupancy_from_beliefs(beliefs, B):
    """Occupancy flags + per-cell max belief. Target-cell beliefs are monotone
    non-decreasing (events only accumulate), so a crossed threshold stays
    crossed — the old any-historical-prob latch is implied."""
    return (beliefs > B).astype(int), beliefs.max(axis=0).tolist()


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


def _finalize_metrics(time_elapsed_at_steps, t_all_known, t_bs_knows, t_one_knows, t_back,
                      mission_time_at_steps=None):
    # time_elapsed_at_steps is the SEARCH clock (topology-independent in the
    # discrete pipeline; uniform 1s in realtime). mission_time_at_steps is the
    # MISSION clock (actual flown path incl. early-return legs); when omitted
    # (realtime, where the clock is uniform) it falls back to the search clock.
    if mission_time_at_steps is None:
        mission_time_at_steps = time_elapsed_at_steps
    time_at_least_one = sum(time_elapsed_at_steps[:t_one_knows]) if t_one_knows != np.inf else np.inf
    detection_time = sum(time_elapsed_at_steps[:t_all_known]) if t_all_known != np.inf else np.inf
    inform_time = sum(time_elapsed_at_steps[t_all_known:t_bs_knows]) if t_bs_knows != np.inf else np.inf
    mission_time = sum(mission_time_at_steps[:t_back]) if t_back != np.inf else np.inf
    return detection_time, inform_time, mission_time, time_at_least_one


def merge_maps(conn_comp, search_map, merge_topology="onboard", dirty=None):
    """Share sensing EVENTS within each connectivity clique.

    Every member of a clique ends up with the UNION of the clique's unique
    events per cell (dedup key: (drone, timestep) — one arrival per drone per
    discrete step). Older events are shared too; belief fusion happens
    downstream in _fused_belief — merging never picks a "winning" observation.
      "none": no sharing. "onboard": every clique. "gcs": only cliques with node 0.

    `dirty`, when given, collects the (node, cell) pairs whose event list this
    call replaced, so callers can refresh only those cached beliefs.
    """
    if merge_topology not in VALID_MERGE_TOPOLOGIES:
        raise ValueError(
            f"Unknown merge_topology {merge_topology!r}; valid: {VALID_MERGE_TOPOLOGIES}")
    if merge_topology == "none":
        return search_map
    number_of_cells = search_map.shape[1]

    for clique in conn_comp:
        if merge_topology == "gcs" and 0 not in clique:
            continue
        for cell in range(number_of_cells):
            union = {}
            for node in clique:
                for obs in search_map[node, cell]:
                    union.setdefault((obs["drone"], obs["timestep"]), obs)
            if not union:
                continue
            merged_events = sorted(union.values(),
                                   key=lambda o: (o["timestep"], o["drone"]))
            for node in clique:
                # every member's events are a subset of the union, so a length
                # match means the set already matches — skip the rebuild
                if len(search_map[node, cell]) != len(merged_events):
                    # a fresh list per node: the sensing loop appends to these
                    # in place, so sharing one list object would leak a drone's
                    # own observation into every clique member's map
                    search_map[node, cell] = list(merged_events)
                    if dirty is not None:
                        dirty.add((node, cell))
    return search_map


def sensing_and_realtime_info_sharing(sol: PathSolution, config):
    """Realtime sensing + merging: continuous positions, per-second connectivity,
    mid-flight merging. Sensing is driven by discrete path-matrix arrivals mapped
    onto realtime steps (_matrix_column_arrival_steps): one sensing event per
    (drone, matrix column), exactly like the discrete pipeline, so zero-distance
    repeat-visit legs still sense even though they add no realtime columns (RT-1).
    Returns the same 7-key metrics dict as the discrete pipeline (contract AD1).
    Beliefs are fused from the union of unique sensing events (see
    _fused_belief); merging shares events, not posteriors.
    """
    merge_topology = config.merge_topology
    target_locations = config.target_locations
    B, p, q = config.belief_threshold, config.detection_prob, config.false_alarm_prob
    x = deepcopy(sol)
    info = x.info
    # sensing source; a copy so early-return mirror rewrites cannot alter the
    # positions of columns that are still to be sensed
    drone_path_matrix = x.real_time_path_matrix[1:, :].copy()
    # tour-end sensing window (exact parity with the discrete pipeline): a drone
    # stops sensing once past its own tour — hovering on a cell is not a new
    # observation. Merging/relaying still runs every step for all drones.
    final_search_steps = [len(dpath) - 2 for dpath in list(x.drone_dict.values())]
    realtime_x, realtime_y = get_real_paths(x)

    number_of_nodes, timesteps = realtime_x.shape
    number_of_drones = number_of_nodes - 1

    connectivity_matrix = get_real_connectivity_matrix(realtime_x, realtime_y, sol)

    # matrix column -> realtime arrival step; group columns per step (dt=0 runs
    # make several columns share one step)
    col_arrivals = _matrix_column_arrival_steps(
        x.real_time_path_matrix, info.D, info.max_drone_speed)
    cols_at_step = {}
    for col, s in enumerate(col_arrivals):
        cols_at_step.setdefault(s, []).append(col)

    search_map = _init_search_map(info.number_of_nodes, info.number_of_cells)
    # cached fused belief per (node, cell); 0.5 == the prior (no events yet)
    beliefs = np.full((info.number_of_nodes, info.number_of_cells), 0.5)
    dirty = set()
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

    step = 0
    while step < timesteps:

        # --- locate drones (continuous positions; used for merging cadence,
        # early return and the home check) ---------------------------------------
        drone_on_grid = [False] * number_of_drones
        for drone in range(number_of_drones):
            pos_x = realtime_x[drone + 1, step]
            pos_y = realtime_y[drone + 1, step]
            drone_positions[drone] = sol.get_city((pos_x, pos_y))
            drone_on_grid[drone] = isCoordinateDiscrete(pos_x, pos_y, sol)

        adj_mat = connectivity_matrix[step]
        conn_comp = connected_components(adj_mat)

        # --- sensing: one event per (drone, matrix column) arrival (RT-1),
        #     within the drone's tour window only (no hovering — parity AD) -----
        for col in cols_at_step.get(step, ()):
            discrete_step = col
            for drone in range(number_of_drones):
                if not drone_search_status[drone]:
                    continue   # early-returned: row rewritten, search is over
                if col > final_search_steps[drone]:
                    continue   # tour over: hovering/returning, no more sensing
                pos = drone_path_matrix[drone, col]
                if pos == -1:
                    continue
                search_map[drone + 1, pos].append(
                    {"drone": drone + 1, "timestep": col,
                     "positive": pos in target_locations})
                dirty.add((drone + 1, pos))

        # --- merging: EVERY second, including mid-flight (AD5) ------------------
        search_map = merge_maps(conn_comp, search_map, merge_topology, dirty)

        # --- occupancy + tracking (shared helpers) ------------------------------
        _refresh_beliefs(search_map, beliefs, dirty, p, q)
        occupancy_status, per_cell_max = _occupancy_from_beliefs(beliefs, B)
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
                    # continuous return: current pos -> cell 0 -> BS
                    ret_x1, ret_y1 = intp_between_coords(drone_x_pos, drone_y_pos,
                                                         cell_0_x, cell_0_y,
                                                         info.max_drone_speed)
                    ret_x2, ret_y2 = intp_between_coords(cell_0_x, cell_0_y,
                                                         cell_bs_x, cell_bs_y,
                                                         info.max_drone_speed)
                    ret_x = np.hstack((ret_x1, ret_x2))
                    ret_y = np.hstack((ret_y1, ret_y2))
                    # intp_between_coords EXCLUDES its endpoint, so ret never
                    # contains the exact BS point — only the BS pad below does.
                    # Grow the horizon when the route (+ >= 1 exact-BS column)
                    # does not fit; the old truncation branch stranded the drone
                    # ~2 m off-grid so the all-home break never fired and
                    # mission time was inf (RT-2). Grown columns repeat each
                    # row's final position, which is the BS for every drone by
                    # construction of the original trajectory and of prior
                    # early-return pads.
                    remaining = timesteps - step
                    needed = len(ret_x) + 1
                    if needed > remaining:
                        grow = needed - remaining
                        realtime_x = np.hstack(
                            (realtime_x, np.repeat(realtime_x[:, -1:], grow, axis=1)))
                        realtime_y = np.hstack(
                            (realtime_y, np.repeat(realtime_y[:, -1:], grow, axis=1)))
                        timesteps += grow
                        connectivity_matrix = get_real_connectivity_matrix(
                            realtime_x, realtime_y, sol)
                        remaining = timesteps - step
                    pad = remaining - len(ret_x)
                    ret_x = np.hstack((ret_x, np.full(pad, cell_bs_x)))
                    ret_y = np.hstack((ret_y, np.full(pad, cell_bs_y)))
                    realtime_x[m + 1, step:] = ret_x
                    realtime_y[m + 1, step:] = ret_y
                    # Mirror into the DISCRETE path matrix so PathAnimation (which
                    # re-derives trajectories from it via get_real_paths) shows the
                    # early return (AD3). Indexed by discrete waypoint (leg), NOT
                    # the realtime step.
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
        # settled on-grid there; get_city also returns -1 mid-flight (off-grid),
        # so the on-grid conjunct prevents a spurious step-1 fire.
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

        step += 1

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


def sensing_and_discrete_info_sharing(sol: PathSolution, config):
    """Discrete sensing + merging: per-cell-step sensing on the discrete path matrix, clique merging each step, early return-to-base. Returns the same 7-key metrics dict as the realtime pipeline (contract AD1).
    Beliefs are fused from the union of unique sensing events (see
    _fused_belief); merging shares events, not posteriors.
    """
    merge_topology = config.merge_topology
    target_locations = config.target_locations
    B, p, q = config.belief_threshold, config.detection_prob, config.false_alarm_prob
    x = deepcopy(sol)
    info = x.info
    final_search_steps = [len(dpath) - 2 for dpath in list(x.drone_dict.values())]
    drone_path_matrix = x.real_time_path_matrix[1:, :]
    # Pristine planned trajectory, snapshotted before the loop mutates
    # x.real_time_path_matrix via early return. Drives the SEARCH clock so
    # detection/inform/time-at-least-one stay a pure function of the planned
    # path — independent of merge topology (see the clock block below).
    planned_path_matrix = x.real_time_path_matrix.copy()
    number_of_drones, timesteps = drone_path_matrix.shape
    connectivity_matrix = x.connectivity_matrix
    cell_occupancy_probabilities = [ [] for _ in range(info.number_of_cells) ]

    search_map = _init_search_map(x.info.number_of_nodes, x.info.number_of_cells)
    # cached fused belief per (node, cell); 0.5 == the prior (no events yet)
    beliefs = np.full((x.info.number_of_nodes, x.info.number_of_cells), 0.5)
    dirty = set()

    drone_search_status = [True for _ in range(number_of_drones)]
    timestep_bs_knows_all_targets = np.inf
    timestep_at_least_one_drone_knows_all_targets = np.inf
    timestep_all_targets_are_known = np.inf
    timestep_drones_are_back_at_bs = np.inf

    x.mission_time = 0
    x.time_elapsed_at_steps = []   # SEARCH clock (pristine path)
    mission_time_at_steps = []     # MISSION clock (actual flown path)

    x.target_detection_times = {target:None for target in target_locations}

    for step in range(timesteps):

        adj_mat = connectivity_matrix[step]
        conn_comp = connected_components(adj_mat)

        # Drones update probabilities
        for drone in range(number_of_drones):
            if not drone_search_status[drone]:
                continue   # early-returned: done searching, no more sensing (parity with realtime)
            if step > final_search_steps[drone]:
                continue
            pos = drone_path_matrix[drone, step]
            if pos == -1:
                continue
            search_map[drone + 1, pos].append(
                {"drone": drone + 1, "timestep": step,
                 "positive": pos in target_locations})
            dirty.add((drone + 1, pos))

        search_map = merge_maps(conn_comp, search_map, merge_topology, dirty)

        # Occupancy Status Check
        _refresh_beliefs(search_map, beliefs, dirty, p, q)
        occupancy_status, per_cell_max = _occupancy_from_beliefs(beliefs, B)
        for col in range(info.number_of_cells):
            cell_occupancy_probabilities[col].append(per_cell_max[col])

        # Track detection/inform time steps
        (timestep_all_targets_are_known, timestep_bs_knows_all_targets,
         timestep_at_least_one_drone_knows_all_targets) = _update_detection_timesteps(
            occupancy_status, target_locations, step,
            timestep_all_targets_are_known, timestep_bs_knows_all_targets,
            timestep_at_least_one_drone_knows_all_targets)

        # Time elapsed at this step, on two clocks:
        #   SEARCH clock (pristine planned path) -> detection / inform /
        #     time-at-least-one + per-target detection times. Built from the
        #     planned trajectory so early-return path rewrites (a merge-topology
        #     side effect) cannot warp the search timeline: onboard and gcs reach
        #     the same detection/inform STEPS, so they must report the same TIMES.
        #   MISSION clock (actual flown path incl. early-return legs) ->
        #     mission_time only, which must reflect the real return trajectory.
        positions_now = x.real_time_path_matrix[1:, step]   # mutated: all-home check below
        if step < timesteps - 1:
            planned_now = planned_path_matrix[1:, step]
            planned_next = planned_path_matrix[1:, step + 1]
            search_dt = max(info.D[a, b] for a, b in zip(planned_now, planned_next)) / info.max_drone_speed
            x.time_elapsed_at_steps.append(search_dt)

            positions_next = x.real_time_path_matrix[1:, step + 1]
            mission_dt = max(info.D[a, b] for a, b in zip(positions_now, positions_next)) / info.max_drone_speed
            mission_time_at_steps.append(mission_dt)
            x.mission_time += mission_dt

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
        timestep_drones_are_back_at_bs, mission_time_at_steps)

    return  {"cell occupancy probabilities": cell_occupancy_probabilities, "search map": search_map, "occupancy status": occupancy_status, "detection time": detection_time, "inform time": inform_time, "mission time": mission_time, "time at least one drone knows all targets": time_at_least_one}, x
# print(f"TC Best Conn Metrics:\n{tc_best_conn_metrics}")