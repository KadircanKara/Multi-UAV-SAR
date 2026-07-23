"""Bounds on the scenario parameters that drive run cost.

Two separate guards live here:

* ``n_visits`` gets an env-tunable deploy cap like drones/grid, because path
  length scales with it and it is by far the largest cost multiplier a caller
  can reach — an uncapped n_visits turns a one-minute run into a multi-day one.
* grid size, cell side length and drone speed must be *finite* and positive.
  ``gt=0`` alone still admits ``inf``, which propagates into the objective
  computation as inf/NaN rather than failing cleanly at the edge.
"""
import pytest

from app import settings


def _cfg(**over):
    body = {
        "optimization_type": "MOO",
        "method": "NSGA2",
        "objectives": ["Mission Time", "Percentage Connectivity"],
        "pop_size": 12,
        "n_gen": 5,
        "scenario": {"number_of_drones": 4, "n_visits": 2},
    }
    body.update(over)
    return body


def _scenario(**over):
    base = {"grid_size": 8, "cell_side_length": 50, "number_of_drones": 4,
            "max_drone_speed": 2.5, "comm_cell_range": 2, "n_visits": 2,
            "target_positions": [0], "th": 0.9, "detection_probability": 0.7}
    base.update(over)
    return base


# ─── n_visits deploy cap ──────────────────────────────────────────────────────

def test_n_visits_at_default_cap_is_accepted(client):
    r = client.post("/api/optimize/check",
                    json=_cfg(scenario={"number_of_drones": 4,
                                        "n_visits": settings.MAX_N_VISITS}))
    assert r.status_code == 200


def test_n_visits_over_default_cap_is_rejected(client):
    r = client.post("/api/optimize/check",
                    json=_cfg(scenario={"number_of_drones": 4,
                                        "n_visits": settings.MAX_N_VISITS + 1}))
    assert r.status_code == 422


def test_n_visits_cap_is_enforced_when_starting_a_run(client):
    """The cap has to bite on the endpoint that actually spends the CPU, not
    just on /check."""
    r = client.post("/api/optimize",
                    json=_cfg(scenario={"number_of_drones": 4,
                                        "n_visits": settings.MAX_N_VISITS + 1}))
    assert r.status_code == 422


def test_n_visits_cap_is_enforced_by_scenario_validation(client):
    r = client.post("/api/scenarios/validate",
                    json={"scenario": _scenario(n_visits=settings.MAX_N_VISITS + 1)})
    assert r.status_code == 422


def test_n_visits_cap_is_tunable_by_env(client, monkeypatch):
    monkeypatch.setattr(settings, "MAX_N_VISITS", 5)
    ok = client.post("/api/optimize/check",
                     json=_cfg(scenario={"number_of_drones": 4, "n_visits": 5}))
    too_big = client.post("/api/optimize/check",
                          json=_cfg(scenario={"number_of_drones": 4, "n_visits": 6}))
    assert ok.status_code == 200
    assert too_big.status_code == 422


# ─── the caps are sized for a public deploy ───────────────────────────────────

def test_n_visits_stays_tightly_capped():
    """The one cap here that is a safety bound rather than a tuning choice: it
    is the largest cost multiplier reachable from a request, and uncapped it
    turned a single run into a week of CPU."""
    assert settings.MAX_N_VISITS <= 3


def test_default_caps_match_the_documented_deploy_defaults():
    """Pinned exactly because the UI sliders mirror these numbers; drift between
    the two means the UI offers values the API answers with a 422.

    pop/gen are sized for CONVERGENCE, not latency — with the constraints
    active, runs much below these rarely converge at all — so they are
    deliberately generous and the worst case is tens of minutes, not minutes.
    """
    assert settings.MAX_POP_SIZE == 300
    assert settings.MAX_N_GEN == 1000
    assert settings.MAX_N_VISITS == 3


def _schema_max(model, field: str) -> int:
    """The field's hard `le` ceiling, wherever pydantic filed it in metadata."""
    for constraint in model.model_fields[field].metadata:
        if hasattr(constraint, "le"):
            return constraint.le
    raise AssertionError(f"{model.__name__}.{field} has no le ceiling")


def test_schema_ceilings_are_not_below_the_deploy_caps():
    """A cap the schema rejects first would make the env var unusable upward."""
    from app.schemas import OptimizeConfig, ScenarioConfig

    assert _schema_max(ScenarioConfig, "n_visits") >= settings.MAX_N_VISITS
    assert _schema_max(OptimizeConfig, "pop_size") >= settings.MAX_POP_SIZE
    assert _schema_max(OptimizeConfig, "n_gen") >= settings.MAX_N_GEN


# ─── finite positivity ────────────────────────────────────────────────────────

@pytest.mark.parametrize("field", ["max_drone_speed", "cell_side_length"])
@pytest.mark.parametrize("bad", [0, -1])
def test_zero_and_negative_are_rejected(client, field, bad):
    r = client.post("/api/scenarios/validate",
                    json={"scenario": _scenario(**{field: bad})})
    assert r.status_code == 422


@pytest.mark.parametrize("field", ["max_drone_speed", "cell_side_length",
                                   "comm_cell_range"])
def test_infinity_is_rejected(client, field):
    """gt=0 admits inf, which poisons the objective computation downstream
    instead of failing at the request edge.

    Sent as a raw body: `Infinity` is not valid JSON so json.dumps refuses to
    emit it, but Python's parser accepts it on the way in — which is exactly
    how it would arrive from a hand-rolled client.
    """
    import json

    body = json.dumps({"scenario": _scenario(**{field: 1})})
    body = body.replace(f'"{field}": 1', f'"{field}": Infinity')
    r = client.post("/api/scenarios/validate", content=body,
                    headers={"Content-Type": "application/json"})
    assert r.status_code == 422


@pytest.mark.parametrize("field", ["max_drone_speed", "cell_side_length",
                                   "comm_cell_range"])
def test_infinity_is_rejected_by_the_schema(field):
    """The guard belongs to the model, not one endpoint's hand-written checks."""
    from app.schemas import ScenarioConfig

    with pytest.raises(Exception):
        ScenarioConfig(**{field: float("inf")})


@pytest.mark.parametrize("bad", [0, -1])
def test_grid_size_zero_and_negative_are_rejected(client, bad):
    r = client.post("/api/scenarios/validate",
                    json={"scenario": _scenario(grid_size=bad)})
    assert r.status_code == 422


def test_absurd_grid_size_is_rejected_by_the_schema(client):
    """Not just by the deploy cap: a grid of a billion cells must never reach
    PathInfo, whatever SAR_MAX_GRID_SIZE is set to."""
    from app.schemas import ScenarioConfig

    with pytest.raises(Exception):
        ScenarioConfig(grid_size=10**9)
