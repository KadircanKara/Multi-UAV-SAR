import numpy as np
import pytest

from Sensing import merge_maps, _fused_belief

P, Q = 0.7, 0.2


def _event(drone, ts, positive=True, prob=0.9, n_obs=1):
    return {"drone": drone, "n_obs": n_obs, "timestep": ts,
            "prob": prob, "positive": positive}


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
    m[1, 0].append(_event(1, 5))
    out = merge_maps([[0, 1, 2]], m, "none")
    assert len(out[2, 0]) == 1        # node 2 learned nothing
    assert len(out[0, 0]) == 1        # BS learned nothing
    assert out is m


def test_onboard_merges_any_clique():
    m = _map()
    m[1, 0].append(_event(1, 5))
    out = merge_maps([[1, 2]], m, "onboard")          # clique WITHOUT the BS
    assert (1, 5) in {(o["drone"], o["timestep"]) for o in out[2, 0] if o["timestep"] >= 0}
    assert len(out[0, 0]) == 1                        # BS not in clique -> unchanged


def test_gcs_requires_bs_in_clique():
    m = _map()
    m[1, 0].append(_event(1, 5))
    out = merge_maps([[1, 2]], m, "gcs")              # no BS in clique
    assert len(out[2, 0]) == 1                        # nothing shared
    m2 = _map()
    m2[1, 0].append(_event(1, 5))
    out2 = merge_maps([[0, 1, 2]], m2, "gcs")         # BS present
    assert (1, 5) in {(o["drone"], o["timestep"]) for o in out2[2, 0] if o["timestep"] >= 0}


def test_merge_shares_old_events_not_just_latest():
    """The pre-fusion bug: only max-timestep observations propagated. The
    union merge must deliver OLDER events too."""
    m = _map()
    m[1, 0].append(_event(1, 3))
    m[1, 0].append(_event(1, 7))
    m[2, 0].append(_event(2, 5))
    out = merge_maps([[1, 2]], m, "onboard")
    for node in (1, 2):
        keys = {(o["drone"], o["timestep"]) for o in out[node, 0] if o["timestep"] >= 0}
        assert keys == {(1, 3), (1, 7), (2, 5)}


def test_merge_dedups_by_drone_timestep():
    m = _map()
    shared = _event(1, 5)
    m[1, 0].append(shared)
    m[2, 0].append(dict(shared))      # same event already delivered earlier
    out = merge_maps([[1, 2]], m, "onboard")
    for node in (1, 2):
        events = [o for o in out[node, 0] if o["timestep"] >= 0]
        assert len(events) == 1


def test_fused_belief_math():
    sentinel = [{"n_obs": 0, "timestep": -1, "prob": 0.5}]
    pos = lambda k: sentinel + [_event(d + 1, d) for d in range(k)]
    assert _fused_belief(sentinel, P, Q) == pytest.approx(0.5)
    assert _fused_belief(pos(1), P, Q) == pytest.approx(0.7777777777777778)
    assert _fused_belief(pos(2), P, Q) == pytest.approx(0.9245283018867925)
    assert _fused_belief(pos(3), P, Q) == pytest.approx(0.9772079772079773)
    mixed = sentinel + [_event(1, 0), _event(2, 1, positive=False)]
    assert _fused_belief(mixed, P, Q) == pytest.approx(0.5675675675675675)
    negs = sentinel + [_event(1, t, positive=False) for t in range(3)]
    assert _fused_belief(negs, P, Q) < 0.5


from Time import isCoordinateDiscrete


def test_isCoordinateDiscrete_tolerates_float_error(small_solution):
    x_exact, y_exact = small_solution.get_coords(12)
    assert isCoordinateDiscrete(x_exact, y_exact, small_solution)
    # interpolation-grade float error must still classify as on-grid
    assert isCoordinateDiscrete(x_exact + 1e-9, y_exact - 1e-9, small_solution)
    # mid-cell must NOT
    half = small_solution.info.cell_side_length / 2
    assert not isCoordinateDiscrete(x_exact + half, y_exact, small_solution)
