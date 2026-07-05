"""Pydantic schema for the Playground result JSON (schema_version 1).

This is the ONLY accepted upload format. Parsed with Pydantic (no pickle),
so it cannot execute code. Bounds guard against oversized-array DoS.
"""
from __future__ import annotations

from typing import Optional

from pydantic import BaseModel, Field

from app.schemas import ScenarioConfig

MAX_SOLUTIONS = 2000
MAX_PATH_LEN = 100_000


class PlaygroundSolution(BaseModel):
    index: int = Field(ge=0)
    # Raw objective_values(sol) output: unsigned magnitudes for all 5 objectives.
    # Polarity (sign) and Max-Mean-TBV nulling are applied downstream by the
    # comparison endpoint, mirroring the seeded pipeline. Only f_row is signed.
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
