import pytest

from SensingReplay import SensingConfig


def test_defaults_valid():
    cfg = SensingConfig()
    assert cfg.merge_topology == "onboard"
    assert cfg.time_model == "discrete"


@pytest.mark.parametrize("field,value", [
    ("merge_topology", "ondrone"),      # legacy name rejected
    ("merge_topology", "discrete"),     # old collision value rejected
    ("time_model", "continuous"),       # not a valid time model
    ("detection_prob", 0.0),
    ("detection_prob", 1.0),
    ("false_alarm_prob", -0.1),
    ("belief_threshold", 1.5),
])
def test_invalid_values_raise(field, value):
    with pytest.raises(ValueError):
        SensingConfig(**{field: value})


def test_from_info_defaults_and_overrides(small_solution):
    info = small_solution.info
    cfg = SensingConfig.from_info(info)
    assert cfg.detection_prob == info.detection_probability   # 0.7
    assert cfg.belief_threshold == info.th                    # 0.9
    assert cfg.target_locations == list(info.target_locations)
    cfg2 = SensingConfig.from_info(info, merge_topology="gcs", belief_threshold=0.8)
    assert cfg2.merge_topology == "gcs" and cfg2.belief_threshold == 0.8


def test_from_info_tolerates_old_pickled_pathinfo(small_solution):
    class OldInfo:                       # simulates pre-migration PathInfo pickle
        number_of_cells = 64
    cfg = SensingConfig.from_info(OldInfo())
    assert cfg.detection_prob == 0.7 and cfg.belief_threshold == 0.9


def test_from_info_rejects_out_of_grid_target(small_solution):
    with pytest.raises(ValueError):
        SensingConfig.from_info(small_solution.info, target_locations=[999])
