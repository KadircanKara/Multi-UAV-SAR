from app.playground_schema import PlaygroundResult

def _min_payload():
    return {
        "schema_version": 1,
        "scenario": {"number_of_drones": 4, "n_visits": 2},
        "model": {"model_key": "TCD_MOO_NSGA2", "Type": "MOO", "Alg": "NSGA2",
                  "Exp": "TCD",
                  "F": ["Mission Time", "Percentage Connectivity",
                        "Mean Disconnected Time", "Max Disconnected Time"],
                  "G": [], "H": []},
        "polarities": {"Mission Time": 1, "Percentage Connectivity": -1},
        "run_config": {"pop_size": 100, "n_gen": 300, "seed": 1},
        "solutions": [{
            "index": 0,
            "objectives": {"Mission Time": 123.0, "Percentage Connectivity": -0.8,
                           "Max Disconnected Time": 5.0, "Mean Disconnected Time": 2.0,
                           "Max Mean TBV": None},
            "f_row": [123.0, -0.8, 2.0, 5.0],
            "path": [0, 1, 2, 3], "start_points": [0],
        }],
    }

def test_valid_payload_parses():
    r = PlaygroundResult.model_validate(_min_payload())
    assert r.schema_version == 1
    assert r.solutions[0].path == [0, 1, 2, 3]

def test_unknown_schema_version_rejected():
    import pytest
    from pydantic import ValidationError
    bad = _min_payload(); bad["schema_version"] = 999
    with pytest.raises(ValidationError):
        PlaygroundResult.model_validate(bad)

def test_too_many_solutions_rejected():
    import pytest
    from pydantic import ValidationError
    bad = _min_payload(); bad["solutions"] = bad["solutions"] * 2001
    with pytest.raises(ValidationError):
        PlaygroundResult.model_validate(bad)


def _seeded_selector():
    from app.selector_service import get_selector

    # A scenario known to exist in Results/ (EC2 sweep).
    scenario = "MOO_NSGA2_TCD_g_8_a_50_n_4_v_2.5_r_2_nvisits_1"
    return scenario, get_selector(scenario)


def test_serialize_run_matches_schema():
    from app.playground_export import serialize_run

    _sc, sel = _seeded_selector()
    payload = serialize_run(sel.solutions, sel.F, sel.model, run_config={"seed": 1})
    r = PlaygroundResult.model_validate(payload)  # must validate
    assert len(r.solutions) == len(sel.solutions)
    # f_row length equals number of F columns
    assert len(r.solutions[0].f_row) == len(sel.model["F"])
    # all 5 canonical objectives present as keys
    assert set(r.solutions[0].objectives.keys()) == {
        "Mission Time",
        "Percentage Connectivity",
        "Max Disconnected Time",
        "Mean Disconnected Time",
        "Max Mean TBV",
    }


def test_objectives_equal_raw_objective_values():
    """The serialized `objectives` dict is exactly PathFuncDict.objective_values(sol)
    output (unsigned); polarity/TBV-nulling is applied downstream, not here."""
    from app.playground_export import serialize_run
    from PathFuncDict import objective_values
    _sc, sel = _seeded_selector()
    payload = serialize_run(sel.solutions, sel.F, sel.model, run_config={})
    for i, sol in enumerate(sel.solutions):
        assert payload["solutions"][i]["objectives"] == objective_values(sol)


def test_reconstruct_selector_shapes():
    from app.playground_export import serialize_run
    from app.playground_schema import PlaygroundResult
    from app.playground_reconstruct import reconstruct_selector

    _sc, sel = _seeded_selector()
    result = PlaygroundResult.model_validate(serialize_run(sel.solutions, sel.F, sel.model, {}))
    rebuilt = reconstruct_selector(result)
    assert rebuilt.F.columns.tolist() == sel.model["F"]
    assert rebuilt.n_solutions == len(result.solutions)
    assert rebuilt.solutions[0].info.number_of_drones == sel.solutions[0].info.number_of_drones


def test_reconstructed_front_matches_seeded():
    """The reconstructed (upload -> rebuild) front must match the seeded
    build_front output byte-for-byte on objectives + objectives_signed."""
    from app.playground_export import serialize_run
    from app.playground_schema import PlaygroundResult
    from app.playground_reconstruct import reconstruct_selector
    from app.selector_service import build_front, build_front_from_selector

    scenario, sel = _seeded_selector()
    seeded_front = build_front(scenario)

    result = PlaygroundResult.model_validate(
        serialize_run(sel.solutions, sel.F, sel.model, {})
    )
    rebuilt = reconstruct_selector(result)
    rebuilt_front = build_front_from_selector(rebuilt)

    assert rebuilt_front["objectives"] == seeded_front["objectives"]
    seeded_signed = [s["objectives_signed"] for s in seeded_front["solutions"]]
    rebuilt_signed = [s["objectives_signed"] for s in rebuilt_front["solutions"]]
    assert rebuilt_signed == seeded_signed


def test_replay_for_reconstructed_solution():
    from app.playground_export import serialize_run
    from app.playground_schema import PlaygroundResult
    from app.playground_reconstruct import reconstruct_info, reconstruct_solution
    from app.replay_service import run_replay_for_solution

    _sc, sel = _seeded_selector()
    result = PlaygroundResult.model_validate(serialize_run(sel.solutions, sel.F, sel.model, {}))
    info = reconstruct_info(result.scenario.to_scenario_dict(), result.model)
    sol = reconstruct_solution(result.solutions[0], info, full=True)   # heavy: sim-ready

    cfg = {"merge_topology": "onboard", "time_model": "discrete",
           "detection_prob": 0.8, "false_alarm_prob": 0.1,
           "belief_threshold": 0.9, "target_locations": [12]}
    out = run_replay_for_solution(sol, cfg)
    assert set(["effective_mission_time", "detection_time", "inform_time",
                "time_at_least_one_drone_knows_all",
                "cell_occupancy_probabilities"]).issubset(out.keys())


def test_serialize_run_uses_explicit_model_key():
    from app.playground_export import serialize_run
    _sc, sel = _seeded_selector()  # sel.model is an AVAILABLE_MODELS dict (no "model_key")
    payload = serialize_run(sel.solutions, sel.F, sel.model, {}, model_key="TCD_MOO_NSGA2")
    assert payload["model"]["model_key"] == "TCD_MOO_NSGA2"
