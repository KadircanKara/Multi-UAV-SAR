"""The decisive test: the sibling artifact must reproduce, exactly, the numbers
the selector-based path produces today.

The comparison endpoint is about to stop loading SolutionObjects entirely. That
is only safe if the precomputed file is numerically indistinguishable from
reading the solutions live — so this walks every seeded scenario and asserts
float equality (not approximate: the values are copies, so anything but exact
equality means something was recomputed or reordered).

Slow by design (it unpickles every solutions file once) and gated on the seeded
tree, so it does not run in CI.
"""
import glob
import os

import pytest

from app import settings
from app.all_objectives import read_all_objectives
from app.comparison_service import stats_from_objective_dicts
from app.library_service import parse_scenario_params, resolve_model_key
from app.selector_service import get_selector
from app import models_registry

from PathFuncDict import objective_values

_OBJ_SUFFIX = "-ObjectiveValues.pkl"


def _seeded_scenarios():
    obj_dir = os.path.join(settings.RESULTS_ROOT, "Objectives")
    names = []
    for path in sorted(glob.glob(os.path.join(obj_dir, f"*{_OBJ_SUFFIX}"))):
        scenario = os.path.basename(path)[: -len(_OBJ_SUFFIX)]
        sol = os.path.join(settings.RESULTS_ROOT, "Solutions",
                           f"{scenario}-SolutionObjects.pkl")
        if os.path.isfile(sol):
            names.append(scenario)
    return names


@pytest.mark.needs_seed_data
def test_every_seeded_scenario_has_a_sibling():
    scenarios = _seeded_scenarios()
    assert scenarios, "no seeded scenarios found"
    missing = [s for s in scenarios if read_all_objectives(s) is None]
    assert missing == [], (
        f"{len(missing)} scenario(s) have no readable -AllObjectives.pkl; "
        f"run backend/scripts/backfill_all_objectives.py. First few: "
        f"{missing[:5]}")


@pytest.mark.needs_seed_data
@pytest.mark.slow
def test_sibling_stats_match_the_selector_path_exactly():
    scenarios = _seeded_scenarios()
    assert scenarios, "no seeded scenarios found"

    mismatches = []
    for scenario in scenarios:
        params = parse_scenario_params(scenario)
        n_visits = (params.get("variant_value")
                    if params.get("variant") == "nvisits" else None)
        model = models_registry.get_model(resolve_model_key(scenario))

        selector = get_selector(scenario)
        live = stats_from_objective_dicts(
            [objective_values(sol) for sol in selector.solutions],
            model, n_visits)

        rows = read_all_objectives(scenario, expected_rows=len(selector.solutions))
        assert rows is not None, f"{scenario}: sibling missing or row-count mismatch"
        cheap = stats_from_objective_dicts(rows, model, n_visits)

        if cheap != live:
            mismatches.append((scenario, live, cheap))

        # Keep peak RSS flat: one ~160 MB selector at a time. Clearing the
        # module-global LRU here is a deliberate, accepted side effect for any
        # later test in the same pytest session (it just forces a cache miss).
        from app.selector_service import _load_selector
        _load_selector.cache_clear()

    assert mismatches == [], (
        f"{len(mismatches)} scenario(s) differ between the precomputed sibling "
        f"and the live selector read. First: {mismatches[0]}")
