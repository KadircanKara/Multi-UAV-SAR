"""Rebuild PathInfo / PathSolution / SolutionSelector from an uploaded
PlaygroundResult, mirroring optimizer_service.scenario_name_for's construction
(PathInfo(scenario_dict) then info.model = model_dict). No disk, no pickle.
"""
from __future__ import annotations

import app.rootpath  # noqa: F401

import pandas as pd

from app.playground_schema import PlaygroundResult, PlaygroundSolution
from PathInfo import PathInfo
from PathSolution import PathSolution
from SolutionSelection import SolutionSelector


def reconstruct_info(scenario_dict: dict, model: dict) -> PathInfo:
    info = PathInfo(scenario_dict)   # builds grid/D/comm_dist/etc from the dict
    info.model = model               # patch model on after construction
    return info


def reconstruct_solution(sol_json: PlaygroundSolution, info: PathInfo, *, full: bool) -> PathSolution:
    return PathSolution(
        list(sol_json.path), list(sol_json.start_points), info,
        calculate_pathplan=full,
        calculate_connectivity=full,
        calculate_disconnectivity=full,
        calculate_tbv=(full and info.n_visits > 1),
    )


def reconstruct_selector(result: PlaygroundResult) -> SolutionSelector:
    scenario_dict = result.scenario.to_scenario_dict()
    info = reconstruct_info(scenario_dict, result.model)
    # Light solutions: front/select/compare-objectives never read PathSolution
    # attributes (they use F + model), so skip the expensive path compute.
    solutions = [reconstruct_solution(s, info, full=False) for s in result.solutions]
    F = pd.DataFrame([s.f_row for s in result.solutions], columns=list(result.model["F"]))
    return SolutionSelector(F, solutions, result.model)
