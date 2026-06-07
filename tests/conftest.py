import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import pytest

from PathInfo import PathInfo
from PathSolution import PathSolution

GRID = 8
N_DRONES = 4
TARGET_CELL = 12          # visited early by drone 0 in the full-coverage path


def make_scenario(target_positions):
    return {
        'grid_size': GRID,
        'cell_side_length': 50,
        'number_of_drones': N_DRONES,
        'max_drone_speed': 2.5,
        'comm_cell_range': 2,
        'n_visits': 1,
        'target_positions': list(target_positions),
        'th': 0.9,
        'detection_probability': 0.7,
    }


@pytest.fixture(scope="session")
def small_solution():
    """Deterministic full-coverage solution: cells 0..63 in order, 4 equal subtours.

    session-scoped because building the path plan + connectivity matrix is the
    slow part; tests must NOT mutate it (the pipelines deepcopy internally).
    """
    info = PathInfo(make_scenario([TARGET_CELL]))
    path = np.arange(info.number_of_cells)          # 0..63
    start_points = np.array([0, 16, 32, 48])        # drone d starts at path index 16*d
    return PathSolution(path, start_points, info,
                        calculate_pathplan=True, calculate_connectivity=True)
