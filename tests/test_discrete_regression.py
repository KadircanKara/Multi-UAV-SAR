import random

import numpy as np
import pytest
from PathInfo import PathInfo
from PathSolution import PathSolution
from Sensing import sensing_and_discrete_info_sharing
from SensingReplay import SensingConfig

# Frozen from the evidence-fusion discrete pipeline (union-of-events merging,
# odds-form belief fold). If a behavior-preserving refactor changes ANY of
# these, the refactor is wrong.
#
# Re-frozen 2026-07-17. detection and time-at-least-one have never moved through
# any of it, structurally: early return cannot fire before detection, so the
# pre-detection legs are never rewritten and every clock agrees on them.
#
# "mission time" (1193.38 -> 1209.95 -> 1176.81): the clock used to read column
# step+1 BEFORE the early-return block rewrote it, so the leg on which a drone
# diverts home was timed against the route it had just abandoned. Then
# connectivity started being re-derived from the flown trajectory, which changes
# when drones are sent home and reshapes the flown path.
#
# "inform time" (532.55 -> 499.41 -> 201.42 -> 209.71): connectivity is now
# derived from the trajectory actually flown, so a drone sent home really
# reaches the BS and tells it, instead of relaying forever from the search
# pattern it abandoned -- that is the big drop. The final step (201.42 ->
# 209.71) withdrew the SEARCH clock: "inform time" is DEFINED as the time that
# elapses from all targets being detected to the BS knowing all targets, and the
# search clock priced that window with planned legs the drones never flew,
# reporting 201.42 s where 209.71 s had passed.
#
# The real guards are test_inform_time_is_real_elapsed_time and
# test_mission_time_matches_flown_path, which re-derive these from the returned
# path matrix without touching the pipeline's clock. This snapshot only pins drift.
SNAPSHOT = {
    "detection time": 632.5483399593904,
    "inform time": 209.7056274847714,
    "mission time": 1176.812408671319,
    "time at least one drone knows all targets": 632.5483399593904,
}
OCCUPANCY_SUM = 5
N_PROB_STEPS = 31


def test_discrete_metrics_unchanged(small_solution):
    m, _ = sensing_and_discrete_info_sharing(
        small_solution,
        SensingConfig(merge_topology="onboard", time_model="discrete", target_locations=[12],
                      belief_threshold=0.7, detection_prob=0.7, false_alarm_prob=0.2))
    for key, expected in SNAPSHOT.items():
        assert m[key] == expected, f"{key} drifted: {m[key]} != {expected}"
    assert int(np.sum(m["occupancy status"])) == OCCUPANCY_SUM
    assert len(m["cell occupancy probabilities"][0]) == N_PROB_STEPS


@pytest.fixture(scope="module")
def tight_return_solution():
    """A solution where a drone's route home EXACTLY fills the columns left.

    seed 271 of this generator: drone 0 diverts with len(path_to_0) == the
    remaining columns, so the old code's `padded_path[:timesteps - step]` sliced
    the trailing -1 away and parked it on cell 0 forever.
    """
    rng = random.Random(271)
    info = PathInfo({'grid_size': 5, 'cell_side_length': 50, 'number_of_drones': 3,
                     'max_drone_speed': 5.0, 'comm_cell_range': 2 * 2 ** 0.5,
                     'n_visits': 2, 'target_positions': [6, 20], 'th': 0.9,
                     'detection_probability': 0.7})
    path = list(range(info.number_of_cells)) * info.n_visits
    rng.shuffle(path)
    start_points = [0] + sorted(rng.sample(range(1, len(path)), 2))
    return PathSolution(np.array(path), np.array(start_points), info,
                        calculate_pathplan=True, calculate_connectivity=True)


@pytest.mark.parametrize("topo", ["onboard", "gcs", "none"])
def test_early_return_always_reaches_the_bs(tight_return_solution, topo):
    """Early return must never strand a drone short of the BS.

    The route home needs its own columns PLUS one for the BS arrival. When it
    exactly filled the columns left, the old code sliced the trailing -1 off and
    the drone parked on cell 0: the all-home check could never fire and mission
    time came back inf. The realtime pipeline already grew its horizon for this
    (RT-2); discrete now does too.
    """
    m, x = sensing_and_discrete_info_sharing(
        tight_return_solution,
        SensingConfig(merge_topology=topo, time_model="discrete", target_locations=[6, 20],
                      belief_threshold=0.9, detection_prob=0.7, false_alarm_prob=0.2))
    flown = x.real_time_path_matrix[1:, :]
    assert np.all(flown[:, -1] == -1), \
        f"{topo}: drones {list(np.where(flown[:, -1] != -1)[0])} are parked off the BS"
    assert np.isfinite(m["mission time"]), f"{topo}: mission time is inf"


@pytest.mark.parametrize("topo", ["onboard", "gcs", "none"])
def test_inform_time_is_real_elapsed_time(small_solution, topo):
    """"Inform time" is defined as the time that ELAPSES from all targets being
    detected to the BS knowing all targets. So it must equal the elapsed time
    over [t_all_known, t_bs_knows) on the path the drones actually flew.

    Re-derived here from the returned path matrix, never from the pipeline's own
    clock. This is what the withdrawn SEARCH clock got wrong: it priced that
    window with legs from the PLANNED path, which the drones abandon at the
    moment of the diversion, and reported 201.42 s where 209.71 s had passed.
    """
    m, x = sensing_and_discrete_info_sharing(
        small_solution,
        SensingConfig(merge_topology=topo, time_model="discrete", target_locations=[12],
                      belief_threshold=0.7, detection_prob=0.7, false_alarm_prob=0.2))
    if not np.isfinite(m["inform time"]):
        pytest.skip(f"{topo}: BS never learns, inform is inf by definition")
    D, speed = x.info.D, x.info.max_drone_speed
    flown = x.real_time_path_matrix[1:, :]
    legs = [max(D[flown[r, s], flown[r, s + 1]] for r in range(flown.shape[0])) / speed
            for s in range(flown.shape[1] - 1)]
    # Elapsed time on the flown path can only ever be a PREFIX SUM of its legs.
    # detection = elapsed(t_all_known) and detection + inform = elapsed(t_bs_knows),
    # so both must land exactly on a prefix -- no index recovery needed, and no
    # reuse of the pipeline's own clock. Under the withdrawn search clock,
    # detection still landed on a prefix (early return cannot fire before
    # detection) but detection + inform did NOT, because the diversion leg inside
    # the window was priced on the abandoned plan.
    prefixes = np.concatenate(([0.0], np.cumsum(legs)))
    on_prefix = lambda v: bool(np.any(np.isclose(prefixes, v, rtol=1e-9, atol=1e-9)))
    assert on_prefix(m["detection time"]), \
        f"{topo}: detection {m['detection time']} is not elapsed time on the flown path"
    assert on_prefix(m["detection time"] + m["inform time"]), (
        f"{topo}: detection+inform {m['detection time'] + m['inform time']} is not elapsed "
        f"time on the flown path -- inform is not a real duration")


@pytest.mark.parametrize("topo", ["onboard", "gcs", "none"])
def test_mission_time_matches_flown_path(small_solution, topo):
    """mission_time must be re-derivable from the path the drones ACTUALLY flew.

    The returned x.real_time_path_matrix is the flown trajectory, truncated at
    the step every drone is home; summing max-over-drones leg distance / speed
    across it must reproduce mission_time exactly. This is an independent check
    -- it never touches the pipeline's clock -- and it is what pins the ordering
    fix: the mission clock used to read column step+1 before the early-return
    block rewrote it, timing the diversion leg against the abandoned plan.
    """
    m, x = sensing_and_discrete_info_sharing(
        small_solution,
        SensingConfig(merge_topology=topo, time_model="discrete", target_locations=[12],
                      belief_threshold=0.7, detection_prob=0.7, false_alarm_prob=0.2))
    assert np.isfinite(m["mission time"])
    D, speed = x.info.D, x.info.max_drone_speed
    flown = x.real_time_path_matrix[1:, :]
    expected = sum(
        max(D[flown[r, s], flown[r, s + 1]] for r in range(flown.shape[0])) / speed
        for s in range(flown.shape[1] - 1))
    assert m["mission time"] == pytest.approx(expected)
