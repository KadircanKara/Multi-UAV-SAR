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


def test_written_frame_is_float64(results_root):
    all_objectives.write_all_objectives(
        "SCEN", [_full_solution(max_mean_tbv=float("inf"))])
    df = pd.read_pickle(os.path.join(results_root, "Objectives",
                                     f"SCEN{all_objectives.SUFFIX}"))
    assert all(str(dt) == "float64" for dt in df.dtypes)
    assert df["Max Mean TBV"].isna().all()


def _write_seed_scenario(root, scenario, n_rows):
    """Lay down the two files the backfill walks: the objective table it
    discovers scenarios from, and the solutions it reads values off."""
    obj_dir = os.path.join(root, "Objectives")
    sol_dir = os.path.join(root, "Solutions")
    os.makedirs(obj_dir, exist_ok=True)
    os.makedirs(sol_dir, exist_ok=True)
    pd.DataFrame({"Mission Time": [1.0] * n_rows}).to_pickle(
        os.path.join(obj_dir, f"{scenario}-ObjectiveValues.pkl"))
    pd.to_pickle([_full_solution() for _ in range(n_rows)],
                 os.path.join(sol_dir, f"{scenario}-SolutionObjects.pkl"))


def test_backfill_writes_one_sibling_per_scenario(results_root):
    from scripts import backfill_all_objectives as bf
    _write_seed_scenario(results_root, "SCEN_A", 2)
    _write_seed_scenario(results_root, "SCEN_B", 1)
    assert bf.main(force=False, dry_run=False) == 2
    assert all_objectives.read_all_objectives("SCEN_A", expected_rows=2)
    assert all_objectives.read_all_objectives("SCEN_B", expected_rows=1)


def test_backfill_is_idempotent(results_root):
    from scripts import backfill_all_objectives as bf
    _write_seed_scenario(results_root, "SCEN_A", 1)
    assert bf.main(force=False, dry_run=False) == 1
    assert bf.main(force=False, dry_run=False) == 0


def test_backfill_force_rewrites(results_root):
    from scripts import backfill_all_objectives as bf
    _write_seed_scenario(results_root, "SCEN_A", 1)
    bf.main(force=False, dry_run=False)
    assert bf.main(force=True, dry_run=False) == 1


def test_backfill_dry_run_writes_nothing(results_root):
    from scripts import backfill_all_objectives as bf
    _write_seed_scenario(results_root, "SCEN_A", 1)
    assert bf.main(force=False, dry_run=True) == 1
    assert all_objectives.read_all_objectives("SCEN_A") is None


def test_backfill_aborts_when_an_objective_is_not_cached(results_root):
    from scripts import backfill_all_objectives as bf
    sol = _full_solution()
    del sol.max_mean_tbv
    os.makedirs(os.path.join(results_root, "Objectives"), exist_ok=True)
    os.makedirs(os.path.join(results_root, "Solutions"), exist_ok=True)
    pd.DataFrame({"Mission Time": [1.0]}).to_pickle(
        os.path.join(results_root, "Objectives", "SCEN_A-ObjectiveValues.pkl"))
    pd.to_pickle([sol], os.path.join(results_root, "Solutions",
                                     "SCEN_A-SolutionObjects.pkl"))
    with pytest.raises(all_objectives.MissingCachedObjective):
        bf.main(force=False, dry_run=False)


def test_backfill_skips_a_scenario_with_no_solutions_file(results_root):
    from scripts import backfill_all_objectives as bf
    os.makedirs(os.path.join(results_root, "Objectives"), exist_ok=True)
    pd.DataFrame({"Mission Time": [1.0]}).to_pickle(
        os.path.join(results_root, "Objectives", "SCEN_A-ObjectiveValues.pkl"))
    assert bf.main(force=False, dry_run=False) == 0


# ---------------------------------------------------------------------------
# Source stamp: binds the sibling to the -SolutionObjects.pkl it was derived
# from, so a same-length replacement of the source is detected instead of
# being served silently (see write_all_objectives / read_all_objectives).
# ---------------------------------------------------------------------------

def test_write_with_source_path_carries_the_stamp(results_root, tmp_path):
    src = tmp_path / "SCEN-SolutionObjects.pkl"
    pd.to_pickle([_full_solution()], src)
    all_objectives.write_all_objectives("SCEN", [_full_solution()], source_path=str(src))
    df = pd.read_pickle(os.path.join(results_root, "Objectives",
                                     f"SCEN{all_objectives.SUFFIX}"))
    st = os.stat(src)
    assert df.attrs["source"] == {"size": st.st_size, "mtime_ns": st.st_mtime_ns}


def test_read_with_matching_source_path_succeeds(results_root, tmp_path):
    src = tmp_path / "SCEN-SolutionObjects.pkl"
    pd.to_pickle([_full_solution()], src)
    all_objectives.write_all_objectives("SCEN", [_full_solution()], source_path=str(src))
    rows = all_objectives.read_all_objectives("SCEN", source_path=str(src))
    assert rows is not None
    assert rows[0]["Mission Time"] == 665.685424949238


def test_read_after_source_is_modified_returns_none(results_root, tmp_path):
    src = tmp_path / "SCEN-SolutionObjects.pkl"
    pd.to_pickle([_full_solution()], src)
    all_objectives.write_all_objectives("SCEN", [_full_solution()], source_path=str(src))
    # Rewrite the source with different content (and thus a different size),
    # simulating a same-length-or-not replacement of the scenario's solutions.
    pd.to_pickle([_full_solution(), _full_solution()], src)
    assert all_objectives.read_all_objectives("SCEN", source_path=str(src)) is None


def test_read_stamped_sibling_with_no_source_path_still_succeeds(results_root, tmp_path):
    src = tmp_path / "SCEN-SolutionObjects.pkl"
    pd.to_pickle([_full_solution()], src)
    all_objectives.write_all_objectives("SCEN", [_full_solution()], source_path=str(src))
    # Caller does not care about staleness (source_path omitted) -> still reads.
    rows = all_objectives.read_all_objectives("SCEN")
    assert rows is not None


def test_read_unstamped_sibling_with_source_path_still_succeeds(results_root, tmp_path):
    # Backward compatibility: every sibling written before this change (all
    # 216 already on disk) has no stamp at all. Supplying source_path must not
    # invalidate them.
    src = tmp_path / "SCEN-SolutionObjects.pkl"
    pd.to_pickle([_full_solution()], src)
    all_objectives.write_all_objectives("SCEN", [_full_solution()])  # no source_path
    rows = all_objectives.read_all_objectives("SCEN", source_path=str(src))
    assert rows is not None


def test_missing_source_file_does_not_invalidate_a_stamped_sibling(results_root, tmp_path):
    src = tmp_path / "SCEN-SolutionObjects.pkl"
    pd.to_pickle([_full_solution()], src)
    all_objectives.write_all_objectives("SCEN", [_full_solution()], source_path=str(src))
    os.unlink(src)
    rows = all_objectives.read_all_objectives("SCEN", source_path=str(src))
    assert rows is not None
