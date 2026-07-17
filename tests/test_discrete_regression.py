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
# "mission time" re-frozen 2026-07-17 (1193.38 -> 1209.95): the mission clock
# used to read column step+1 BEFORE the early-return block rewrote it, so the
# one leg on which a drone diverts home was timed against the planned route it
# abandoned -- undercounting the diversion. Moving the mission clock after the
# early-return block prices the leg actually flown, so the value rises.
# test_mission_time_matches_flown_path re-derives this number from the returned
# path matrix independently of the clock, and is the real guard; the snapshot
# only pins that it does not drift.
#
# "inform time" re-frozen 2026-07-17 (499.41 -> 201.42) and "mission time"
# (1209.95 -> 1176.81): connectivity is now re-derived from the trajectory the
# drones actually fly, instead of staying pinned to the planned path forever.
# A drone sent home used to keep relaying from the search pattern it had
# abandoned; now it relays from its route home, reaches the BS, and informs it
# far sooner -- hence the large inform drop. Detection is unchanged (it happens
# before any early return), and onboard/gcs now agree exactly: once the
# detecting drone physically flies to the BS, the merge topology stops mattering
# for this scenario.
SNAPSHOT = {
    "detection time": 632.5483399593904,
    "inform time": 201.4213562373095,
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


def test_search_clock_topology_independent(small_solution):
    """detection / inform / time-at-least-one must not depend on merge topology
    when the detection and inform STEPS are topology-independent (they are for
    this scenario). Guards the SEARCH-clock fix: early-return path rewrites are
    a merge-topology side effect and must never leak into the search timeline.
    mission_time legitimately differs (it rides the actual flown path)."""
    def run(topo):
        m, _ = sensing_and_discrete_info_sharing(
            small_solution,
            SensingConfig(merge_topology=topo, time_model="discrete", target_locations=[12],
                          belief_threshold=0.7, detection_prob=0.7, false_alarm_prob=0.2))
        return m
    onboard, gcs = run("onboard"), run("gcs")
    for key in ("detection time", "inform time",
                "time at least one drone knows all targets"):
        assert onboard[key] == gcs[key], (
            f"{key} is topology-dependent: onboard {onboard[key]} != gcs {gcs[key]}")


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
