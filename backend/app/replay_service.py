"""
Replay service: builds SensingConfigs, runs sensing replays via SensingReplay,
and assembles the compare table.  Consumed by the /api/replay and /api/compare
routers.

Import-safety: SensingReplay, SolutionSelection, PathOptimizationModel, numpy
are used here.  PathAlgorithm / PathUnitTest / main are NEVER imported.
"""
from __future__ import annotations

import app.rootpath  # side-effect: inserts repo root into sys.path

from typing import Optional

from SensingReplay import (
    SensingConfig,
    METRIC_COLUMNS,
    _dedupe_labels,
    replay,
)
from SolutionSelection import StrategyUnavailableError

from app.selector_service import _SelectorNotFound, get_selector


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _solution_at(scenario: str, model_key: Optional[str], index: int):
    """
    Fetch (solution, selector) for a given scenario + index.

    Raises:
        _SelectorNotFound  — bad scenario / missing pickles (→ 404).
        StrategyUnavailableError — index out of range (→ 422).
    """
    sel = get_selector(scenario, model_key)
    _idx, solution, _label = sel.by_index(index)  # raises StrategyUnavailableError if OOB
    return solution, sel


def _build_config(solution, cfg_dict: dict) -> SensingConfig:
    """
    Build a SensingConfig from a solution's info + caller overrides.

    Applies grid-bounds validation and the p>q rule via SensingConfig.from_info.
    A ValueError here (p<=q, target outside grid, etc.) should map to 422.
    """
    return SensingConfig.from_info(solution.info, **cfg_dict)


def _assemble_compare_result(results: list) -> dict:
    """
    Shared table/rows assembly for run_compare and run_compare_for_solution.

    Returns::

        {
            "table": [
                {"label": <str>, "Effective Mission Time": <float|None>, ...},
                ...
            ],
            "rows": [<each replay's to_dict()>, ...]
        }
    """
    replay_dicts = [r.to_dict() for r in results]

    # Build table from the already-serialized dicts (inf already → None).
    table = []
    for rd in replay_dicts:
        row: dict = {"label": rd["label"]}
        for display_col, attr_key in METRIC_COLUMNS.items():
            row[display_col] = rd[attr_key]
        table.append(row)

    return {"table": table, "rows": replay_dicts}


# ---------------------------------------------------------------------------
# Solution-accepting cores
#
# These operate on an already-obtained `solution` object (either fetched via
# _solution_at from a seeded scenario, or reconstructed from uploaded
# playground JSON). The scenario-based functions below are thin wrappers that
# fetch the solution then delegate here.
# ---------------------------------------------------------------------------

def prepare_replay_for_solution(solution, cfg_dict: dict, label: Optional[str] = None):
    """
    Run a single sensing replay for an already-obtained solution and return
    the live ReplayResult object.

    Raises:
        ValueError — bad sensing config (p<=q, grid bounds, …) (→ 422).
    """
    config = _build_config(solution, cfg_dict)
    return replay(solution, config, label=label)


def run_replay_for_solution(solution, cfg_dict: dict, label: Optional[str] = None) -> dict:
    """Run a single sensing replay for an already-obtained solution and return its to_dict() payload."""
    return prepare_replay_for_solution(solution, cfg_dict, label=label).to_dict()


def run_compare_for_solution(
    solution,
    cfg_dicts: list[dict],
    labels: Optional[list[str]] = None,
) -> dict:
    """
    Run a replay per config for an already-obtained solution and return a
    comparison payload (see _assemble_compare_result).

    Raises:
        ValueError — empty cfg_dicts, or bad sensing config (→ 422).
    """
    if not cfg_dicts:
        raise ValueError("configs must not be empty")

    # Build configs once up front so _dedupe_labels can inspect merge_topology
    # (mirrors run_compare's dedup behavior exactly). SensingConfig.from_info
    # is pure/side-effect-free, so rebuilding it inside prepare_replay_for_solution
    # below is redundant but harmless.
    configs = [_build_config(solution, cd) for cd in cfg_dicts]
    deduped_labels = _dedupe_labels(configs, labels)

    results = [
        prepare_replay_for_solution(solution, cd, label=lbl)
        for cd, lbl in zip(cfg_dicts, deduped_labels)
    ]
    return _assemble_compare_result(results)


# ---------------------------------------------------------------------------
# Public service functions (scenario-based; thin wrappers over the cores)
# ---------------------------------------------------------------------------

def prepare_replay(
    scenario: str,
    model_key: Optional[str],
    index: int,
    cfg_dict: dict,
    label: Optional[str] = None,
):
    """
    Run a single sensing replay and return the live ReplayResult object.

    This is the internal workhorse reused by run_replay and playback_service.
    The caller gets r.solution, r.cell_occupancy_probabilities, r.config, etc.

    Raises:
        _SelectorNotFound      — bad scenario / missing pickles (→ 404).
        StrategyUnavailableError — index out of range (→ 422).
        ValueError             — bad sensing config (p<=q, grid bounds, …) (→ 422).
    """
    solution, _sel = _solution_at(scenario, model_key, index)
    return prepare_replay_for_solution(solution, cfg_dict, label=label)


def run_replay(
    scenario: str,
    model_key: Optional[str],
    index: int,
    cfg_dict: dict,
    label: Optional[str] = None,
) -> dict:
    """
    Run a single sensing replay and return its to_dict() payload.

    Raises:
        _SelectorNotFound      — bad scenario / missing pickles (→ 404).
        StrategyUnavailableError — index out of range (→ 422).
        ValueError             — bad sensing config (p<=q, grid bounds, …) (→ 422).
    """
    return prepare_replay(scenario, model_key, index, cfg_dict, label=label).to_dict()


def run_compare(
    scenario: str,
    model_key: Optional[str],
    index: int,
    cfg_dicts: list[dict],
    labels: Optional[list[str]] = None,
) -> dict:
    """
    Run a replay per config and return a comparison payload.

    Returns::

        {
            "table": [
                {"label": <str>, "Effective Mission Time": <float|None>, ...},
                ...
            ],
            "rows": [<each replay's to_dict()>, ...]
        }

    Raises:
        ValueError             — empty cfg_dicts (→ 422).
        _SelectorNotFound      — bad scenario / missing pickles (→ 404).
        StrategyUnavailableError — index out of range (→ 422).
        ValueError             — bad sensing config (→ 422).
    """
    if not cfg_dicts:
        raise ValueError("configs must not be empty")

    solution, _sel = _solution_at(scenario, model_key, index)
    return run_compare_for_solution(solution, cfg_dicts, labels=labels)
