import numpy as np
import pytest

from Sensing import merge_maps


def _map(n_nodes=3, n_cells=4):
    m = np.full((n_nodes, n_cells), None, dtype=object)
    for i in range(n_nodes):
        for j in range(n_cells):
            m[i, j] = [{"n_obs": 0, "timestep": -1, "prob": 0.5}]
    return m


def test_unknown_topology_raises():
    with pytest.raises(ValueError, match=r"valid:"):
        merge_maps([[0, 1, 2]], _map(), "ondrone")   # legacy name must be rejected
    with pytest.raises(ValueError, match=r"valid:"):
        merge_maps([[0, 1, 2]], _map(), "discrete")  # the old vocabulary-collision value


def test_none_never_propagates():
    m = _map()
    m[1, 0].append({"n_obs": 1, "timestep": 5, "prob": 0.9})  # node 1 saw something
    out = merge_maps([[0, 1, 2]], m, "none")
    assert out[2, 0][-1]["prob"] == 0.5   # node 2 learned nothing
    assert out[0, 0][-1]["prob"] == 0.5   # BS learned nothing
    assert out is m


def test_onboard_merges_any_clique():
    m = _map()
    m[1, 0].append({"n_obs": 1, "timestep": 5, "prob": 0.9})
    out = merge_maps([[1, 2]], m, "onboard")          # clique WITHOUT the BS (node 0)
    assert out[2, 0][-1]["prob"] == 0.9               # drone 2 received it
    assert out[0, 0][-1]["prob"] == 0.5               # BS not in clique -> unchanged


def test_gcs_requires_bs_in_clique():
    m = _map()
    m[1, 0].append({"n_obs": 1, "timestep": 5, "prob": 0.9})
    out = merge_maps([[1, 2]], m, "gcs")              # no BS in clique
    assert out[2, 0][-1]["prob"] == 0.5               # nothing shared
    m2 = _map()
    m2[1, 0].append({"n_obs": 1, "timestep": 5, "prob": 0.9})
    out2 = merge_maps([[0, 1, 2]], m2, "gcs")         # BS present
    assert out2[2, 0][-1]["prob"] == 0.9
