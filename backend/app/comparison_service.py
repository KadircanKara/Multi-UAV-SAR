"""
Comparison service: compares precomputed scenarios across ALL objectives —
including objectives a given model did NOT optimise.

For each requested scenario it loads the (LRU-cached) SolutionSelector, reads
every objective from each solution via ``objective_values()`` — which fills any
uncached objective with ``compute_all_objectives()`` and is a no-op on the
already-complete seeded solutions — then aggregates min/max/mean/best per
objective across the front. ``best`` honours each objective's polarity
(max for maximize / -1, min for minimize / +1).

Import-safety: builds only on selector_service + library_service + PathFuncDict;
never imports PathAlgorithm / PathUnitTest / main.
"""
from __future__ import annotations

import math
import time
from typing import Optional

import app.rootpath  # side-effect: inserts repo root into sys.path

from app.library_service import (
    _parse_comm_range_value,
    _safe_float,
    parse_scenario_params,
    resolve_model_key,
)
from app import models_registry, replay_service, selector_service, settings
from app.model_aliases import to_display, to_storage
from app.selector_service import (
    _SelectorNotFound,
    StrategyUnavailableError,
    get_selector,
)

from PathOptimizationModel import (
    get_objectives_from_weighted_sum_model,
)
from PathFuncDict import model_metric_info, objective_values
from SensingReplay import METRIC_COLUMNS

# Only for TYPE_CHECKING-style hints in signatures; playground_reconstruct is
# imported lazily inside _label_and_params_for_result to avoid a hard
# import-time dependency from comparison_service on the playground module.
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from app.playground_schema import PlaygroundResult


# Canonical objective set + polarities (authoritative from PathFuncDict).
#   +1 → minimize (lower is better);  -1 → maximize (higher is better).
ALL_OBJECTIVES: list[str] = list(model_metric_info["Objectives"].keys())
_POLARITY: dict[str, int] = {
    name: int(info[1]) for name, info in model_metric_info["Objectives"].items()
}

# Max Mean TBV is undefined when each cell is visited once.
_TBV_OBJECTIVE = "Max Mean TBV"


def _optimized_objective_names(model: dict) -> list[str]:
    """The individual objective names a model actually optimises (WS expanded)."""
    if model.get("Type") == "WS":
        try:
            return list(get_objectives_from_weighted_sum_model(model))
        except Exception:
            return list(model.get("F", []))
    return list(model.get("F", []))


def stats_from_objective_dicts(
    obj_dicts: list[dict[str, Optional[float]]],
    model: dict,
    n_visits: Optional[int],
) -> dict[str, Optional[dict]]:
    """Aggregate a list of per-solution objective dicts into one scenario's
    ``objective_stats`` block.

    Each dict in *obj_dicts* holds the 5 canonical objectives, RAW/unsigned
    (the shape ``PathFuncDict.objective_values()`` returns). For every
    objective this computes min/max/mean across the list and ``best`` honours
    the objective's polarity (max for maximize / -1, min for minimize / +1).
    ``Max Mean TBV`` is forced to ``None`` when *n_visits* == 1 (undefined
    when every cell is visited exactly once), and an objective with no finite
    values anywhere in *obj_dicts* also reports ``None``.

    *model* is accepted (currently unused in the aggregation itself — polarity
    is read from the global, model-independent ``_POLARITY`` table) so this
    helper's signature stays stable if a future caller needs to special-case
    a model's own objective set.

    Shared by the seeded path (``_scenario_stats``, dicts built via
    ``objective_values(sol)``) and the playground path
    (``compare_objectives_from_results``, dicts read straight from the
    uploaded/stored ``PlaygroundResult.solutions[i].objectives``).
    """
    del model  # unused today; kept for signature symmetry — see docstring.

    per_obj: dict[str, list[float]] = {obj: [] for obj in ALL_OBJECTIVES}
    for vals in obj_dicts:
        for obj in ALL_OBJECTIVES:
            v = vals.get(obj)
            if v is not None and math.isfinite(v):
                per_obj[obj].append(v)

    objective_stats: dict[str, Optional[dict]] = {}
    for obj in ALL_OBJECTIVES:
        # TBV is meaningless at n_visits == 1 — surface as "no data".
        if obj == _TBV_OBJECTIVE and n_visits == 1:
            objective_stats[obj] = None
            continue
        vals = per_obj[obj]
        if not vals:
            objective_stats[obj] = None
            continue
        pol = _POLARITY.get(obj, 1)
        mn, mx = min(vals), max(vals)
        me = sum(vals) / len(vals)
        best = mx if pol == -1 else mn
        objective_stats[obj] = {
            "min": _safe_float(mn),
            "max": _safe_float(mx),
            "mean": _safe_float(me),
            "best": _safe_float(best),
        }
    return objective_stats


def _scenario_stats(scenario: str) -> Optional[dict]:
    """Load one scenario and aggregate ALL objectives across its front.

    Returns None if the scenario cannot be resolved / loaded (so the caller can
    skip it rather than fail the whole comparison).
    """
    # Normalize a display alias (…TCDV…) to storage so resolve/parse operate on
    # the real on-disk name; outputs are displayified again below.
    scenario = to_storage(scenario)

    try:
        selector = get_selector(scenario)
    except _SelectorNotFound:
        return None

    try:
        model_key = resolve_model_key(scenario)
    except Exception:
        return None
    model = models_registry.get_model(model_key)
    if model is None:
        return None

    params = parse_scenario_params(scenario)
    n_visits = (
        params.get("variant_value") if params.get("variant") == "nvisits" else None
    )

    # Collect each objective's values across every solution in the front, then
    # hand off to the shared aggregator.
    obj_dicts = [objective_values(sol) for sol in selector.solutions]
    objective_stats = stats_from_objective_dicts(obj_dicts, model, n_visits)

    comm_range_raw = params.get("comm_range")
    comm_range_value: Optional[float] = None
    if comm_range_raw is not None:
        try:
            comm_range_value = _parse_comm_range_value(comm_range_raw)
        except (ValueError, TypeError):
            comm_range_value = None

    return {
        "scenario": to_display(scenario),
        "model_key": to_display(model_key),
        "type": model["Type"],
        "algorithm": model["Alg"],
        "optimized_objectives": _optimized_objective_names(model),
        "number_of_drones": params.get("number_of_drones"),
        "comm_range": comm_range_raw,
        "comm_range_value": comm_range_value,
        "n_visits": n_visits,
        "n_solutions": int(len(selector.solutions)),
        "objective_stats": objective_stats,
    }


def compare_objectives(scenarios: list[str]) -> dict:
    """Compare the given scenarios across ALL objectives.

    Unknown / unloadable scenarios are skipped (returned in ``skipped``) so a
    single bad name does not fail the whole comparison. Input order is preserved
    and duplicates are collapsed.
    """
    # De-duplicate, preserving request order.
    seen: set[str] = set()
    ordered: list[str] = []
    for s in scenarios:
        if s not in seen:
            seen.add(s)
            ordered.append(s)

    # Bound the per-request heavy work: each distinct scenario is a ~160 MB
    # selector unpickle, so process at most COMPARISON_MAX_SCENARIOS of them and
    # surface the overflow in ``skipped`` (partial + honest, not silently cut).
    cap = max(1, settings.COMPARISON_MAX_SCENARIOS)
    overflow = ordered[cap:]
    ordered = ordered[:cap]

    # Wall-clock budget: the count cap bounds how many scenarios are attempted,
    # but a cold selector cache costs ~50x a warm one per scenario, so bound the
    # request's CPU by elapsed time too. Anything not reached is reported in
    # ``skipped`` (partial + honest), so it stays consistent with the overflow.
    budget = settings.COMPARISON_TIME_BUDGET_SECONDS
    start = time.monotonic()

    results: list[dict] = []
    skipped: list[str] = list(overflow)
    for i, scenario in enumerate(ordered):
        # Checked at the top of the loop, but skipped on the first iteration so at
        # least one scenario always runs (elapsed is ~0 there anyway) — the
        # all-skipped 404 then reflects genuinely unloadable input, not the budget.
        if i > 0 and time.monotonic() - start > budget:
            skipped.extend(ordered[i:])
            break
        stats = _scenario_stats(scenario)
        if stats is None:
            skipped.append(scenario)
        else:
            results.append(stats)

    return {
        "objectives": ALL_OBJECTIVES,
        "polarities": {o: _POLARITY.get(o, 1) for o in ALL_OBJECTIVES},
        "scenarios": results,
        "skipped": skipped,
    }


def _label_and_params_for_result(result: "PlaygroundResult") -> tuple[str, dict]:
    """Derive a stable scenario label + structured params for one uploaded
    result.

    Reuses ``PathInfo.__str__`` (the same canonical-name format seeded
    scenarios are stored under, e.g. ``MOO_NSGA2_TCDT_g_8_..._nvisits_2``) via
    ``playground_reconstruct.reconstruct_info``, then feeds that label through
    the same regex parser (``parse_scenario_params``) the seeded path uses —
    so a playground row's metadata (comm_range formatting, number_of_drones,
    n_visits, ...) has exactly the same shape as a seeded row's. Building a
    PathInfo is cheap (no path/connectivity computation, unlike
    ``reconstruct_solution(..., full=True)``).
    """
    from app.playground_reconstruct import reconstruct_info

    info = reconstruct_info(result.scenario.to_scenario_dict(), result.model)
    label = str(info)
    return label, parse_scenario_params(label)


def compare_objectives_from_results(results: "list[PlaygroundResult]") -> dict:
    """Cross-model comparison for uploaded Playground results.

    Unlike ``compare_objectives`` (which loads seeded selectors off disk and
    fills any missing objective via ``objective_values()``), this reads the
    objective values already STORED in each uploaded solution
    (``PlaygroundResult.solutions[i].objectives``) — the exact values the run
    was exported with. Reconstructed playground solutions are "light" (no
    cached path/connectivity/TBV attributes — see
    ``playground_reconstruct.reconstruct_solution``), so calling
    ``objective_values()`` on them would both be slow (full recompute) and
    could diverge from the values the file was actually exported with. No
    selector reconstruction is needed for this endpoint at all.

    Every uploaded result is already schema-validated (no disk / name lookup
    that can fail), so ``skipped`` is always empty here — it stays in the
    response purely to keep the shape identical to ``compare_objectives``.
    """
    scenario_rows: list[dict] = []
    for result in results:
        label, params = _label_and_params_for_result(result)
        n_visits = (
            params.get("variant_value") if params.get("variant") == "nvisits" else None
        )

        obj_dicts = [sol.objectives for sol in result.solutions]
        objective_stats = stats_from_objective_dicts(obj_dicts, result.model, n_visits)

        comm_range_raw = params.get("comm_range")
        comm_range_value: Optional[float] = None
        if comm_range_raw is not None:
            try:
                comm_range_value = _parse_comm_range_value(comm_range_raw)
            except (ValueError, TypeError):
                comm_range_value = None

        model_key = result.model.get("model_key") or result.model.get("Exp", "")

        scenario_rows.append({
            "scenario": label,
            "model_key": model_key,
            "type": result.model.get("Type", ""),
            "algorithm": result.model.get("Alg", ""),
            "optimized_objectives": _optimized_objective_names(result.model),
            "number_of_drones": params.get("number_of_drones"),
            "comm_range": comm_range_raw,
            "comm_range_value": comm_range_value,
            "n_visits": n_visits,
            "n_solutions": int(len(result.solutions)),
            "objective_stats": objective_stats,
        })

    return {
        "objectives": ALL_OBJECTIVES,
        "polarities": {o: _POLARITY.get(o, 1) for o in ALL_OBJECTIVES},
        "scenarios": scenario_rows,
        "skipped": [],
    }


def _time_metrics_for_scenario(
    scenario: str,
    cfg_dict: dict,
    strategy: str,
    objective_name: Optional[str],
    weights: Optional[dict],
) -> Optional[dict]:
    """Resolve one scenario's selected solution and read its four time metrics.

    Returns None if the scenario cannot be resolved / loaded (so the caller can
    skip it). A bad strategy / sensing config raises (StrategyUnavailableError /
    ValueError) and is allowed to propagate so the whole request fails 422.
    """
    try:
        sel = get_selector(scenario)
    except _SelectorNotFound:
        return None

    # Single-solution models (e.g. MTSP, CONN) have no Pareto front to select
    # from, so front strategies like 'balanced'/'knee' don't apply — use the one
    # solution. Front scenarios go through the requested strategy (a bad strategy
    # / missing arg raises StrategyUnavailableError → 422 for the whole request).
    if sel.result_kind == "single":
        idx = 0
    else:
        idx = selector_service.select(
            scenario,
            strategy,
            objective_name=objective_name,
            weights=weights,
        )["index"]
    r = replay_service.prepare_replay(scenario, None, idx, cfg_dict)

    metric_values = {
        name: _safe_float(getattr(r, attr))
        for name, attr in METRIC_COLUMNS.items()
    }

    model_key = resolve_model_key(scenario)
    model = models_registry.get_model(model_key)

    params = parse_scenario_params(scenario)
    n_visits = (
        params.get("variant_value") if params.get("variant") == "nvisits" else None
    )

    comm_range_raw = params.get("comm_range")
    comm_range_value: Optional[float] = None
    if comm_range_raw is not None:
        try:
            comm_range_value = _parse_comm_range_value(comm_range_raw)
        except (ValueError, TypeError):
            comm_range_value = None

    return {
        "scenario": scenario,
        "model_key": model_key,
        "type": model["Type"] if model else "",
        "algorithm": model["Alg"] if model else "",
        "number_of_drones": params.get("number_of_drones"),
        "comm_range": comm_range_raw,
        "comm_range_value": comm_range_value,
        "n_visits": n_visits,
        "selected_index": int(idx),
        "metric_values": metric_values,
    }


def compare_time_metrics(
    scenarios: list[str],
    cfg_dict: dict,
    strategy: str = "balanced",
    objective_name: Optional[str] = None,
    weights: Optional[dict[str, float]] = None,
) -> dict:
    """Compare sensing-replay time metrics across scenarios for one shared config.

    For each scenario a single solution is picked via the given selection
    *strategy*, replayed under *cfg_dict*, and its four time metrics
    (``SensingReplay.METRIC_COLUMNS``) are recorded. Unknown / unloadable
    scenarios are skipped (returned in ``skipped``); a bad strategy / sensing
    config (StrategyUnavailableError / ValueError) propagates so the whole
    request fails. Input order is preserved and duplicates are collapsed.
    """
    # De-duplicate, preserving request order.
    seen: set[str] = set()
    ordered: list[str] = []
    for s in scenarios:
        if s not in seen:
            seen.add(s)
            ordered.append(s)

    # Bound the per-request heavy work: each distinct scenario is a full sensing
    # replay, so process at most COMPARISON_MAX_SCENARIOS of them and surface the
    # overflow in ``skipped`` (partial + honest, not silently cut).
    cap = max(1, settings.COMPARISON_MAX_SCENARIOS)
    overflow = ordered[cap:]
    ordered = ordered[:cap]

    # Wall-clock budget across the per-scenario replays (see compare_objectives).
    # Each scenario here is a full sensing replay, so the elapsed-time bound is
    # the one that actually keeps a cold-cache request from pinning a core for
    # minutes; the unreached remainder is reported in ``skipped``.
    budget = settings.COMPARISON_TIME_BUDGET_SECONDS
    start = time.monotonic()

    results: list[dict] = []
    skipped: list[str] = list(overflow)
    for i, scenario in enumerate(ordered):
        # Top-of-loop check, skipped on the first iteration so at least one
        # scenario always runs (keeps the all-skipped 404 tied to unloadable
        # input, not to the budget).
        if i > 0 and time.monotonic() - start > budget:
            skipped.extend(ordered[i:])
            break
        try:
            row = _time_metrics_for_scenario(
                scenario, cfg_dict, strategy, objective_name, weights
            )
        except StrategyUnavailableError:
            # 'best' on an objective THIS scenario's model didn't optimize → skip
            # it rather than fail the whole comparison. A malformed request (e.g.
            # 'best' with no objective_name at all) still propagates → 422.
            if strategy == "best" and objective_name:
                skipped.append(scenario)
                continue
            raise
        if row is None:
            skipped.append(scenario)
        else:
            results.append(row)

    return {
        "metrics": list(METRIC_COLUMNS.keys()),
        "scenarios": results,
        "skipped": skipped,
        "strategy": strategy,
    }
