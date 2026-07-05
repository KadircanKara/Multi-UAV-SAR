"""
Playback service: builds the per-step animation payload for the browser canvas.

Reuses prepare_replay_for_solution (Task 5 refactor) to get the live
ReplayResult, then extracts continuous trajectories, sparse connectivity
edges, per-cell belief heat, and targets-known curve — all aligned to a
single step axis.

Import-safety: Time.get_real_paths, Time.get_real_connectivity_matrix,
SensingReplay._targets_known_curve, numpy are safe.
PathAlgorithm / PathUnitTest / main are NEVER imported here.

Mode branching (discrete vs realtime):
- realtime: get_real_paths() interpolates each leg into sub-steps (~1107 total);
  get_real_connectivity_matrix() computes per-sub-step connectivity from x/y.
- discrete: positions come directly from real_time_path_matrix via get_coords()
  (one step per waypoint, ~78 total); connectivity is sol.connectivity_matrix
  (same waypoint granularity). This ensures trajectory, connectivity, belief,
  and targets_known are all at the same waypoint granularity so the min()
  alignment truncation does not silently discard 94% of the flight.

NOTE on build_playback (scenario-based): it fetches the RAW solution via
replay_service._solution_at (not via prepare_replay) and delegates to
build_playback_for_solution, which runs prepare_replay_for_solution exactly
once. replay()/the sensing pipeline mutates its returned solution copy
in-place (e.g. an early-returning drone's future real_time_path_matrix
columns are overwritten with its return-to-base path); feeding an
already-replayed solution into a second replay call measurably changes the
computed metrics (verified: effective_mission_time differed by ~17 time
units between a clean run and a run seeded from a once-replayed solution).
So the solution must be replayed exactly once per request — never round
tripped through prepare_replay first.
"""
from __future__ import annotations

import math
from typing import Optional

import app.rootpath  # side-effect: inserts repo root into sys.path

import numpy as np
from Time import get_real_paths, get_real_connectivity_matrix
from SensingReplay import _targets_known_curve

from app.replay_service import _solution_at, prepare_replay_for_solution


# ---------------------------------------------------------------------------
# JSON-safe helpers
# ---------------------------------------------------------------------------

def _safe_float(v) -> Optional[float]:
    """Cast to Python float; return None for inf/nan so JSON serialises cleanly."""
    f = float(v)
    return f if math.isfinite(f) else None


def _safe_float_list(arr) -> list:
    """Convert a 1-D array/list of floats to JSON-safe Python list."""
    return [_safe_float(v) for v in arr]


# ---------------------------------------------------------------------------
# Sparse connectivity helper
# ---------------------------------------------------------------------------

def _sparse_connectivity(conn_matrix: np.ndarray) -> list[list[list[int]]]:
    """
    Convert (T, nodes, nodes) symmetric 0/1 connectivity array to a sparse
    per-step edge list.

    Each step yields a list of [i, j] pairs (i < j) for connected node pairs.
    Upper-triangle only to avoid double-counting the symmetric matrix.
    """
    T, nodes, _ = conn_matrix.shape
    result: list[list[list[int]]] = []
    for t in range(T):
        edges: list[list[int]] = []
        for i in range(nodes):
            for j in range(i + 1, nodes):
                if conn_matrix[t, i, j] == 1:
                    edges.append([i, j])
        result.append(edges)
    return result


# ---------------------------------------------------------------------------
# Solution-accepting core
# ---------------------------------------------------------------------------

def build_playback_for_solution(
    solution,
    cfg_dict: dict,
    stride: int = 1,
) -> dict:
    """
    Build the animation playback payload for an already-obtained solution.

    Returns a JSON-safe dict with trajectories, connectivity edges, belief heat,
    and targets-known curve — all truncated to a single aligned step axis.

    Note: does NOT echo scenario/model_key/index (the caller doesn't have a
    scenario identity when replaying a reconstructed/uploaded solution); the
    scenario-based build_playback wrapper adds those keys after delegating here.

    Parameters
    ----------
    solution   : An already-obtained PathSolution (seeded or reconstructed).
    cfg_dict   : SensingConfig override dict (merge_topology, time_model, p, q, B, targets).
    stride     : Downsample every stride-th step (≥1).  Bounds payload size.

    Raises
    ------
    ValueError — bad config, degenerate replay, bad stride (→ 422).
    """
    stride = max(1, int(stride))  # clamp

    # 1. Run the replay (reuses Task 5 prepare_replay_for_solution).
    r = prepare_replay_for_solution(solution, cfg_dict)
    sol = r.solution
    info = sol.info

    # 2. Mode-specific position + connectivity sources.
    #
    # realtime: get_real_paths() interpolates each leg into ~dt sub-steps
    #   (continuous motion, ~1107 steps).  get_real_connectivity_matrix()
    #   computes per-sub-step connectivity from the same x/y arrays.
    #
    # discrete: positions map each waypoint cell id through get_coords() —
    #   one (x, y) per waypoint (~78 steps).  sol.connectivity_matrix is
    #   already computed at waypoint granularity (same ~78 steps).  Using
    #   get_real_paths() here would produce ~1225 interpolated points and
    #   the min() alignment would keep only the first 78 of them (first 6%
    #   of the flight), making drones appear nearly stationary.
    time_model = r.config.time_model

    if time_model == "discrete":
        # Build (nodes, time_slots) coordinate matrices directly from cell ids.
        # n_position_steps is the authoritative step count: real_time_path_matrix
        # is truncated on early exit while connectivity_matrix is not, so we must
        # derive the step count from the path matrix and slice connectivity to match.
        path_mat = sol.real_time_path_matrix        # (nodes, n_position_steps) — may be truncated
        nodes_count, n_position_steps = path_mat.shape
        x_list = np.zeros((nodes_count, n_position_steps))
        y_list = np.zeros((nodes_count, n_position_steps))
        for node in range(nodes_count):
            for t in range(n_position_steps):
                cell = int(path_mat[node, t])
                coords = sol.get_coords(cell)       # returns np.array([x, y])
                x_list[node, t] = coords[0]
                y_list[node, t] = coords[1]
        x_matrix = x_list   # (nodes, n_position_steps)
        y_matrix = y_list   # (nodes, n_position_steps)

        # Fix 1: slice connectivity to the position-step count BEFORE recording
        # raw_lengths.  On early-mission-exit, real_time_path_matrix is truncated
        # but connectivity_matrix is not — slicing here makes raw_lengths
        # internally consistent (connectivity beyond the early-return is
        # meaningless; drones are already home).
        if sol.connectivity_matrix is None:
            # Fallback: recompute from real_time_path_matrix (should not normally happen).
            sol.do_connectivity_calculations()
        conn_matrix = sol.connectivity_matrix[:n_position_steps]  # (n_position_steps, nodes, nodes)
    else:
        # Fix 2: reuse the trajectory already computed (and potentially truncated
        # on early exit) by sensing_and_realtime_info_sharing.  Fall back to
        # get_real_paths() only when the stored matrices are absent or empty.
        stored_x = getattr(sol, "real_time_x_matrix", None)
        stored_y = getattr(sol, "real_time_y_matrix", None)
        if (stored_x is not None and stored_x.size > 0
                and stored_y is not None and stored_y.size > 0):
            x_matrix = stored_x                                        # (nodes, T_traj)
            y_matrix = stored_y                                        # (nodes, T_traj)
        else:
            x_matrix, y_matrix = get_real_paths(sol)                  # (nodes, T_traj)
        conn_matrix = get_real_connectivity_matrix(x_matrix, y_matrix, sol)  # (T_traj, nodes, nodes)

    # 4. Belief and targets-known from the replay result.
    #    belief: list[cell][step]  — list of lists (Python, may vary in length)
    #    targets_known: list[step]
    belief = r.cell_occupancy_probabilities      # list[list[float]]
    targets_known = _targets_known_curve(r)      # list[int]

    # 5. Determine raw lengths for each array (debug / transparency).
    traj_len = x_matrix.shape[1]           # T_traj (authoritative step count)
    conn_len = conn_matrix.shape[0]        # T_traj (sliced to match traj in discrete; built from x/y in realtime)
    belief_len = len(belief[0]) if belief else 0
    tk_len = len(targets_known)

    raw_lengths = {
        "trajectory": traj_len,
        "connectivity": conn_len,
        "belief": belief_len,
        "targets_known": tk_len,
    }

    # 6. Align: truncate all arrays to the minimum length.
    steps = min(traj_len, conn_len, belief_len, tk_len)
    if steps == 0:
        raise ValueError(
            "Degenerate replay: one or more data arrays have zero length after "
            f"truncation. raw_lengths={raw_lengths}"
        )

    x_trunc = x_matrix[:, :steps]           # (nodes, steps)
    y_trunc = y_matrix[:, :steps]           # (nodes, steps)
    conn_trunc = conn_matrix[:steps]        # (steps, nodes, nodes)
    belief_trunc = [cell_series[:steps] for cell_series in belief]   # list[cells][steps]
    tk_trunc = targets_known[:steps]        # list[steps]

    # 7. Downsample by stride across ALL arrays uniformly.
    x_ds = x_trunc[:, ::stride]
    y_ds = y_trunc[:, ::stride]
    conn_ds = conn_trunc[::stride]
    belief_ds = [cell_series[::stride] for cell_series in belief_trunc]
    tk_ds = tk_trunc[::stride]

    aligned_steps = x_ds.shape[1]

    # 8. Build sparse connectivity (upper triangle only).
    connectivity_sparse = _sparse_connectivity(conn_ds)

    # 9. Assemble JSON-safe payload.
    #    Convert numpy arrays → Python lists; guard all floats.
    nodes = info.number_of_nodes   # includes base station (row 0 of x/y)
    grid_size = info.grid_size
    n_cells = info.number_of_cells

    trajectories = {
        "x": [_safe_float_list(x_ds[node]) for node in range(nodes)],
        "y": [_safe_float_list(y_ds[node]) for node in range(nodes)],
    }

    belief_payload = [
        _safe_float_list(belief_ds[cell]) for cell in range(n_cells)
    ]

    targets_known_payload = [int(v) for v in tk_ds]

    return {
        # Echo / identity
        "time_model": r.config.time_model,
        "merge_topology": r.config.merge_topology,
        # Grid / node metadata
        "grid_size": int(grid_size),
        "cell_side_length": float(info.cell_side_length),
        "number_of_nodes": int(nodes),
        # Sensing config echo
        "targets": [int(t) for t in r.config.target_locations],
        "belief_threshold": float(r.config.belief_threshold),
        # Step axis
        "steps": int(aligned_steps),
        "stride": int(stride),
        "raw_lengths": raw_lengths,
        # Animation data
        "trajectories": trajectories,
        "connectivity": connectivity_sparse,
        "belief": belief_payload,
        "targets_known": targets_known_payload,
    }


# ---------------------------------------------------------------------------
# Public service function (scenario-based; thin wrapper over the core)
# ---------------------------------------------------------------------------

def build_playback(
    scenario: str,
    model_key: Optional[str],
    index: int,
    cfg_dict: dict,
    stride: int = 1,
) -> dict:
    """
    Build the animation playback payload for one solution.

    Fetches the raw solution via the same seam replay_service uses
    (_solution_at) and delegates to build_playback_for_solution, which runs
    the replay exactly once — see module docstring for why this must not go
    through prepare_replay first (double-replay corrupts the metrics).

    Parameters
    ----------
    scenario   : Scenario identifier (path-safe; selector_service validates).
    model_key  : Optional model override (None → auto-detect).
    index      : Solution index within the Pareto front.
    cfg_dict   : SensingConfig override dict (merge_topology, time_model, p, q, B, targets).
    stride     : Downsample every stride-th step (≥1).  Bounds payload size.

    Raises
    ------
    _SelectorNotFound        → 404
    StrategyUnavailableError → 422
    ValueError               → 422  (bad config, degenerate replay, bad stride)
    """
    solution, _sel = _solution_at(scenario, model_key, index)
    payload = build_playback_for_solution(solution, cfg_dict, stride)
    payload["scenario"] = scenario
    payload["model_key"] = model_key
    payload["index"] = int(index)
    return payload
