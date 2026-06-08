"""Realtime connectivity matrix must be symmetric.

Regression test for the upper-triangular bug in get_real_connectivity_matrix:
connectivity is physically symmetric (if A reaches B within comm range, B
reaches A), but the matrix was filled only on the upper triangle, so the BFS/DFS
consumers (connected_components, get_connected_node_ids) fragmented any clique
whose connectivity wasn't index-monotonic — silently corrupting realtime merging.
"""
import numpy as np

from Time import get_real_connectivity_matrix
from Connectivity import get_connected_node_ids


def test_real_connectivity_matrix_is_symmetric(small_solution):
    info = small_solution.info
    # one timestep, 5 nodes; comm_range = 2 * 50 = 100 m
    real_x = np.array([[0.0], [130.0], [50.0], [1000.0], [1000.0]])
    real_y = np.array([[0.0], [0.0],   [0.0],  [1000.0], [1000.0]])
    m = get_real_connectivity_matrix(real_x, real_y, small_solution)
    assert np.allclose(m[0], m[0].T), "connectivity matrix must be symmetric"


def test_multihop_clique_not_fragmented(small_solution):
    # node 0 (BS) <-> node 2 (50 m), node 1 <-> node 2 (80 m), node 0 <-/-> node 1 (130 m).
    # True component containing the BS is {0, 1, 2}; the triangular bug returned only {2}.
    real_x = np.array([[0.0], [130.0], [50.0], [1000.0], [1000.0]])
    real_y = np.array([[0.0], [0.0],   [0.0],  [1000.0], [1000.0]])
    m = get_real_connectivity_matrix(real_x, real_y, small_solution)
    reached = set(get_connected_node_ids(m[0], 0))
    assert reached == {1, 2}, f"BS should reach drones 1 and 2 (multi-hop); got {reached}"
