"""
Playback service: builds the per-step animation payload for the browser canvas.

Reuses prepare_replay (Task 1.2 refactor) to get the live ReplayResult, then
extracts continuous trajectories, sparse connectivity edges, per-cell belief
heat, and targets-known curve — all aligned to a single step axis.

Import-safety: Time.get_real_paths, Time.get_real_connectivity_matrix,
SensingReplay._targets_known_curve, numpy are safe.
PathAlgorithm / PathUnitTest / main are NEVER imported here.
"""
from __future__ import annotations

import math
from typing import Optional

import app.rootpath  # side-effect: inserts repo root into sys.path

import numpy as np
from Time import get_real_paths, get_real_connectivity_matrix
from SensingReplay import _targets_known_curve

from app.replay_service import prepare_replay


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
# Main service function
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

    Returns a JSON-safe dict with trajectories, connectivity edges, belief heat,
    and targets-known curve — all truncated to a single aligned step axis.

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
    stride = max(1, int(stride))  # clamp

    # 1. Run the replay (reuses Task 1.2 prepare_replay).
    r = prepare_replay(scenario, model_key, index, cfg_dict)
    sol = r.solution
    info = sol.info

    # 2. Continuous trajectories (canonical coordinate space, meters).
    #    get_real_paths works for BOTH time models — shape (nodes, T_traj).
    x_matrix, y_matrix = get_real_paths(sol)          # (nodes, T_traj)

    # 3. Real connectivity from the same x/y so it is step-aligned with traj.
    #    Shape: (T_traj, nodes, nodes)
    conn_matrix = get_real_connectivity_matrix(x_matrix, y_matrix, sol)

    # 4. Belief and targets-known from the replay result.
    #    belief: list[cell][step]  — list of lists (Python, may vary in length)
    #    targets_known: list[step]
    belief = r.cell_occupancy_probabilities      # list[list[float]]
    targets_known = _targets_known_curve(r)      # list[int]

    # 5. Determine raw lengths for each array (debug / transparency).
    traj_len = x_matrix.shape[1]           # T_traj
    conn_len = conn_matrix.shape[0]        # T_traj (same pipeline → always equal)
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
        "scenario": scenario,
        "model_key": model_key,
        "index": int(index),
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
