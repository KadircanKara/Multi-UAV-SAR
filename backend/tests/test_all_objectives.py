"""The -AllObjectives.pkl sibling artifact: strict extraction, atomic write,
cheap read.

The artifact exists so the Compare page can aggregate every objective for a
scenario without unpickling its ~160 MB SolutionObjects file. Its whole
correctness claim is that the numbers are the ones ALREADY cached on the seeded
solutions — never recomputed — because a recomputed Max Mean TBV diverges from
what the seeded pickles hold and would disagree with the model pages.
"""
import os

import pandas as pd
import pytest

from app import all_objectives, settings


class FakeSolution:
    """Stands in for a PathSolution: the artifact only ever reads attributes."""

    def __init__(self, **attrs):
        for k, v in attrs.items():
            setattr(self, k, v)


def _full_solution(**overrides):
    attrs = {
        "mission_time": 665.685424949238,
        "percentage_connectivity": 0.6157407407407407,
        "max_disconnected_time": 2.0,
        "mean_disconnected_time": 0.15384615384615385,
        "max_mean_tbv": 189.7056274847714,
    }
    attrs.update(overrides)
    return FakeSolution(**attrs)


@pytest.fixture
def results_root(tmp_path, monkeypatch):
    monkeypatch.setattr(settings, "RESULTS_ROOT", str(tmp_path))
    os.makedirs(tmp_path / "Objectives")
    return str(tmp_path)


def test_columns_are_the_canonical_five_in_order():
    assert all_objectives.COLUMNS == [
        "Mission Time",
        "Percentage Connectivity",
        "Max Disconnected Time",
        "Mean Disconnected Time",
        "Max Mean TBV",
    ]


def test_suffix_does_not_collide_with_the_objective_values_scan():
    # library_service scans with endswith("-ObjectiveValues.pkl"); the sibling
    # must not be picked up by it or every scenario would be counted twice.
    assert not f"X{all_objectives.SUFFIX}".endswith("-ObjectiveValues.pkl")


def test_rows_are_natural_unsigned_values(results_root):
    rows = all_objectives.rows_from_solutions([_full_solution()])
    assert rows == [{
        "Mission Time": 665.685424949238,
        "Percentage Connectivity": 0.6157407407407407,
        "Max Disconnected Time": 2.0,
        "Mean Disconnected Time": 0.15384615384615385,
        "Max Mean TBV": 189.7056274847714,
    }]


def test_missing_cached_objective_raises_instead_of_recomputing(results_root):
    sol = _full_solution()
    del sol.max_mean_tbv
    with pytest.raises(all_objectives.MissingCachedObjective) as exc:
        all_objectives.rows_from_solutions([sol])
    assert "Max Mean TBV" in str(exc.value)


def test_non_finite_becomes_none(results_root):
    rows = all_objectives.rows_from_solutions(
        [_full_solution(max_disconnected_time=float("inf"))])
    assert rows[0]["Max Disconnected Time"] is None


def test_write_then_read_roundtrips(results_root):
    n = all_objectives.write_all_objectives("SCEN", [_full_solution()])
    assert n == 1
    assert os.path.isfile(os.path.join(results_root, "Objectives",
                                       f"SCEN{all_objectives.SUFFIX}"))
    rows = all_objectives.read_all_objectives("SCEN")
    assert rows[0]["Mission Time"] == 665.685424949238


def test_written_frame_has_the_canonical_columns(results_root):
    all_objectives.write_all_objectives("SCEN", [_full_solution()])
    df = pd.read_pickle(os.path.join(results_root, "Objectives",
                                     f"SCEN{all_objectives.SUFFIX}"))
    assert list(df.columns) == all_objectives.COLUMNS


def test_nan_in_the_frame_reads_back_as_none(results_root):
    all_objectives.write_all_objectives(
        "SCEN", [_full_solution(max_mean_tbv=float("nan"))])
    rows = all_objectives.read_all_objectives("SCEN")
    assert rows[0]["Max Mean TBV"] is None


def test_read_returns_none_when_the_file_is_absent(results_root):
    assert all_objectives.read_all_objectives("NOPE") is None


def test_read_returns_none_on_row_count_mismatch(results_root):
    all_objectives.write_all_objectives("SCEN", [_full_solution()])
    assert all_objectives.read_all_objectives("SCEN", expected_rows=2) is None
    assert all_objectives.read_all_objectives("SCEN", expected_rows=1) is not None


def test_read_rejects_an_unsafe_scenario_name(results_root):
    assert all_objectives.read_all_objectives("../../etc/passwd") is None


def test_write_is_atomic_leaving_no_temp_file(results_root):
    all_objectives.write_all_objectives("SCEN", [_full_solution()])
    leftovers = [f for f in os.listdir(os.path.join(results_root, "Objectives"))
                 if not f.endswith(all_objectives.SUFFIX)]
    assert leftovers == []


def test_write_overwrites_in_place(results_root):
    all_objectives.write_all_objectives("SCEN", [_full_solution()])
    all_objectives.write_all_objectives(
        "SCEN", [_full_solution(mission_time=1.0), _full_solution()])
    rows = all_objectives.read_all_objectives("SCEN")
    assert len(rows) == 2
    assert rows[0]["Mission Time"] == 1.0
