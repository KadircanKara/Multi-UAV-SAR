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
from typing import Optional

import app.rootpath  # side-effect: inserts repo root into sys.path

from app.library_service import (
    _parse_comm_range_value,
    _safe_float,
    parse_scenario_params,
    resolve_model_key,
)
from app import models_registry, replay_service, selector_service
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


def _scenario_stats(scenario: str) -> Optional[dict]:
    """Load one scenario and aggregate ALL objectives across its front.

    Returns None if the scenario cannot be resolved / loaded (so the caller can
    skip it rather than fail the whole comparison).
    """
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

    # Collect each objective's values across every solution in the front.
    per_obj: dict[str, list[float]] = {obj: [] for obj in ALL_OBJECTIVES}
    for sol in selector.solutions:
        vals = objective_values(sol)
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

    results: list[dict] = []
    skipped: list[str] = []
    for scenario in ordered:
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

    results: list[dict] = []
    skipped: list[str] = []
    for scenario in ordered:
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
