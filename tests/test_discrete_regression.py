import numpy as np
import pytest
from Sensing import sensing_and_discrete_info_sharing
from SensingReplay import SensingConfig

# Frozen 2026-07-06 from the evidence-fusion discrete pipeline (union-of-events
# merging, odds-form belief fold). If a behavior-preserving refactor changes
# ANY of these, the refactor is wrong.
#
# "inform time" re-frozen 2026-07-16 (532.55 -> 499.41): the discrete clock was
# split into a SEARCH clock (pristine planned path) for detection/inform/
# time-at-least-one and a MISSION clock (actual flown path) for mission_time.
# Early-return path rewrites used to leak long return legs into the inform
# window and inflate it; the search clock removes that artifact. detection,
# mission, and time-at-least-one are unchanged (early returns fire after
# detection, so the pre-detection clock never differed).
#
# All values re-frozen 2026-07-17 after three fixes; detection and
# time-at-least-one never moved, because early return cannot fire before
# detection and so the pre-detection legs are never rewritten.
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
