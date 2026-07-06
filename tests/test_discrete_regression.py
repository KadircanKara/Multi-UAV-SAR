import numpy as np
from Sensing import sensing_and_discrete_info_sharing
from SensingReplay import SensingConfig

# Frozen 2026-07-06 from the evidence-fusion discrete pipeline (union-of-events
# merging, odds-form belief fold). If a behavior-preserving refactor changes
# ANY of these, the refactor is wrong.
SNAPSHOT = {
    "detection time": 632.5483399593904,
    "inform time": 532.5483399593904,
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
