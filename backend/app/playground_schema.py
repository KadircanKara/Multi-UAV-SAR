"""Pydantic schema for the Playground result JSON (schema_version 1).

This is the ONLY accepted upload format. Parsed with Pydantic (no pickle),
so it cannot execute code. Bounds guard against oversized-array DoS.
"""
from __future__ import annotations

from typing import Optional

from pydantic import BaseModel, Field, model_validator

from app.schemas import ScenarioConfig

MAX_SOLUTIONS = 2000
MAX_PATH_LEN = 100_000


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
        """Guard against schema-valid-but-malformed uploads that would
        otherwise crash reconstruction with a 500 (KeyError / pandas
        ValueError) instead of failing cleanly with a 422. ``model`` stays a
        free-form dict (downstream code relies on ``result.model["F"]``
        subscript access), so these checks are enforced here instead of via
        a nested schema.
        """
        required_keys = ("F", "Type", "Alg", "Exp")
        missing = [k for k in required_keys if k not in self.model]
        if missing:
            raise ValueError(f"model is missing required key(s): {missing}")

        n_objectives = len(self.model["F"])
        for sol in self.solutions:
            if len(sol.f_row) != n_objectives:
                raise ValueError(
                    f"solution {sol.index}: f_row has {len(sol.f_row)} values, "
                    f"expected {n_objectives} to match model['F']"
                )
        return self
