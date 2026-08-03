"""Pydantic schema for the Playground result JSON (schema_version 1).

This is the ONLY accepted upload format. Parsed with Pydantic (no pickle),
so it cannot execute code. Bounds guard against oversized-array DoS.
"""
from __future__ import annotations

import math
from typing import Optional

from pydantic import BaseModel, Field, model_validator

from app import settings
from app.schemas import ScenarioConfig

MAX_SOLUTIONS = 2000
# A legal total path is at most grid_size**2 * n_visits * number_of_drones cells
# (grid 8 -> 64*3*16 ~ 3072). 5000 sits well above any legal path yet 20x below
# the old ceiling, which let a ~400KB upload force reconstruct_solution(full=True)
# to allocate a time_slots x nodes x nodes connectivity matrix (~231 MB) plus long
# O(path) loops in PathSolution.do_connectivity_calculations — a cheap OOM/CPU DoS.
MAX_PATH_LEN = 5000

# Combined ceiling on path cells summed across ALL solutions. The per-solution
# MAX_PATH_LEN (5000) x MAX_SOLUTIONS (2000) worst case is 10M cells — a schema-
# valid upload that would drive the per-cell validation scan (and downstream
# reconstruction) into tens of millions of iterations on the event loop. A legit
# 2000-solution export is ~400k cells total (2000 x ~200), so 1M is ~2.5x headroom
# while blocking the 10M pathological case. Checked EARLY, before the per-cell loop.
MAX_TOTAL_PATH_CELLS = 1_000_000


class PlaygroundSolution(BaseModel):
    index: int = Field(ge=0)
    # Raw objective_values(sol) output: unsigned magnitudes for all 5 objectives.
    # Polarity (sign) is applied downstream by the comparison endpoint,
    # mirroring the seeded pipeline. Only f_row is signed.
    # Max Mean TBV serializes as 0.0 (not None) here when n_visits == 1;
    # the comparison endpoint nulls it out downstream for stats purposes.
    objectives: dict[str, Optional[float]]
    # Signed values for the model["F"] columns, in order (mirrors Objectives.pkl row).
    f_row: list[float]
    path: list[int] = Field(max_length=MAX_PATH_LEN)
    start_points: list[int] = Field(max_length=MAX_PATH_LEN)


class PlaygroundResult(BaseModel):
    schema_version: int = Field(ge=1, le=1)
    scenario: ScenarioConfig
    model: dict
    polarities: dict[str, int] = Field(default_factory=dict)
    run_config: dict = Field(default_factory=dict)
    solutions: list[PlaygroundSolution] = Field(min_length=1, max_length=MAX_SOLUTIONS)

    @model_validator(mode="after")
    def _check_model_and_frow_shape(self) -> "PlaygroundResult":
        """Reject schema-valid-but-semantically-broken uploads with a clean 422
        here, BEFORE reconstruction, so a bad file never reaches PathInfo /
        PathSolution (which would either 500 or -- worse -- silently succeed on
        garbage). ``model`` stays a free-form dict (downstream relies on
        ``result.model["F"]`` subscripting), so its shape is enforced here rather
        than via a nested schema.
        """
        required_keys = ("F", "Type", "Alg", "Exp")
        missing = [k for k in required_keys if k not in self.model]
        if missing:
            raise ValueError(f"model is missing required key(s): {missing}")

        # Resource caps. The playground path does NOT go through OptimizeRequest,
        # so the optimizer's ceilings are re-applied here: PathInfo eagerly builds
        # a distance matrix of size number_of_cells^2 == grid_size^4, so an
        # unbounded grid_size is an OOM DoS.
        if self.scenario.grid_size > settings.MAX_GRID_SIZE:
            raise ValueError(
                f"grid_size {self.scenario.grid_size} exceeds the cap of "
                f"{settings.MAX_GRID_SIZE}")
        if self.scenario.number_of_drones > settings.MAX_DRONES:
            raise ValueError(
                f"number_of_drones {self.scenario.number_of_drones} exceeds the "
                f"cap of {settings.MAX_DRONES}")
        if self.scenario.n_visits > settings.MAX_N_VISITS:
            raise ValueError(
                f"n_visits {self.scenario.n_visits} exceeds the cap of "
                f"{settings.MAX_N_VISITS}")
        # cell_side_length x (1 / max_drone_speed) drives the realtime sub-sample
        # count in Time.get_real_paths; a huge cell or a near-zero speed can OOM the
        # main process on a /replay|/compare|/playback with time_model="realtime".
        # get_real_paths' cumulative bound is the real backstop; this fails fast.
        if self.scenario.cell_side_length > settings.MAX_CELL_SIDE_LENGTH:
            raise ValueError(
                f"cell_side_length {self.scenario.cell_side_length} exceeds the "
                f"cap of {settings.MAX_CELL_SIDE_LENGTH}")
        if self.scenario.max_drone_speed < settings.MIN_DRONE_SPEED:
            raise ValueError(
                f"max_drone_speed {self.scenario.max_drone_speed} is below the "
                f"floor of {settings.MIN_DRONE_SPEED}")

        # Combined path-cell cap, checked BEFORE the per-cell loop below so a
        # pathological upload (every solution near MAX_PATH_LEN) short-circuits
        # here instead of driving tens of millions of validation iterations on the
        # event loop. Summing len() is O(n_solutions) and cheap.
        total_path_cells = sum(len(sol.path) for sol in self.solutions)
        if total_path_cells > MAX_TOTAL_PATH_CELLS:
            raise ValueError(
                f"total path length across all solutions ({total_path_cells}) "
                f"exceeds the cap of {MAX_TOTAL_PATH_CELLS}")

        n_objectives = len(self.model["F"])
        number_of_cells = self.scenario.grid_size ** 2
        n_drones = self.scenario.number_of_drones
        for sol in self.solutions:
            if len(sol.f_row) != n_objectives:
                raise ValueError(
                    f"solution {sol.index}: f_row has {len(sol.f_row)} values, "
                    f"expected {n_objectives} to match model['F']")
            # f_row feeds SolutionSelector's [0,1] normalization; a NaN/Inf
            # poisons every min/max and the balanced/knee distances.
            for v in sol.f_row:
                if not math.isfinite(v):
                    raise ValueError(
                        f"solution {sol.index}: f_row contains a non-finite value ({v})")
            # path: non-empty, every entry a real grid cell (or the -1 BS marker).
            # PathSolution indexes it as path % number_of_cells, so an
            # out-of-range value would silently WRAP to a wrong cell rather than
            # error -- the reconstruction would "succeed" on nonsense.
            if not sol.path:
                raise ValueError(f"solution {sol.index}: path is empty")
            for c in sol.path:
                if not (-1 <= c < number_of_cells):
                    raise ValueError(
                        f"solution {sol.index}: path cell {c} out of range "
                        f"[-1, {number_of_cells})")
            # start_points: one subtour-start index per drone, into the path.
            if len(sol.start_points) != n_drones:
                raise ValueError(
                    f"solution {sol.index}: {len(sol.start_points)} start_points, "
                    f"expected {n_drones} (one per drone)")
            for sp in sol.start_points:
                if not (0 <= sp < len(sol.path)):
                    raise ValueError(
                        f"solution {sol.index}: start_point {sp} out of range "
                        f"[0, {len(sol.path)})")
            # start_points slice the flat path into one contiguous sub-tour per
            # drone (path[sp[i]:sp[i+1]]), so they must begin at 0 and strictly
            # increase. Out of order or not starting at 0 (e.g. [1, 0]) yields an
            # empty sub-tour; reconstruction then indexes drone_path[0] and raises
            # IndexError deep in PathSolution -> uncaught 500. Reject cleanly here.
            if sol.start_points[0] != 0:
                raise ValueError(
                    f"solution {sol.index}: start_points must begin at 0 "
                    f"(got {sol.start_points[0]})")
            for a, b in zip(sol.start_points, sol.start_points[1:]):
                if b <= a:
                    raise ValueError(
                        f"solution {sol.index}: start_points must be strictly "
                        f"increasing (got {sol.start_points})")
        return self
