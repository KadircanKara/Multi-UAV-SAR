import numpy as np
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
SNAPSHOT = {
    "detection time": 632.5483399593904,
    "inform time": 499.4112549695428,
    "mission time": 1193.3809511662428,
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
