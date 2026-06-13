"""Unit tests for the RunConfig builders (backend/app/run_config.py)."""


def test_build_run_config_moo_shape():
    from app.run_config import build_run_config

    model = {"Type": "MOO", "Exp": "TC", "Alg": "NSGA2",
             "F": ["Mission Time", "Percentage Connectivity"], "G": [], "H": []}
    cfg = build_run_config(
        model_dict=model,
        objectives=["Mission Time", "Percentage Connectivity"],
        weights=None, pop_size=120, n_gen=300, seed=7,
        gen_strategy="fixed", early_stop_patience=10, early_stop_threshold=0.10,
        max_mission_time=3600.0, min_connectivity=0.5, max_mean_tbv=None,
        scenario={"number_of_drones": 4, "comm_cell_range": 2, "n_visits": 2,
                  "max_drone_speed": 2.5, "grid_size": 8, "cell_side_length": 50},
        n_solutions=12, cancelled=False, early_stopped=False, stopped_at_gen=300,
        source="optimizer",
    )
    assert cfg["schema_version"] == 1
    assert cfg["optimization_type"] == "MOO" and cfg["method"] == "NSGA2"
    assert cfg["pop_size"] == 120 and cfg["n_gen"] == 300 and cfg["seed"] == 7
    assert cfg["gen_strategy"] == "fixed"
    # early-stop fields null in fixed mode
    assert cfg["early_stop_patience"] is None and cfg["early_stop_threshold"] is None
    assert cfg["constraints"] == {
        "speed_feasibility": True, "max_mission_time": 3600.0,
        "min_connectivity": 0.5, "max_mean_tbv": None}
    assert cfg["scenario"]["comm_range"] == 2 and cfg["scenario"]["grid_size"] == 8
    assert cfg["outcome"] == {"n_solutions": 12, "cancelled": False,
                              "early_stopped": False, "stopped_at_gen": 300}
    assert cfg["source"] == "optimizer" and cfg["weights"] is None


def test_build_run_config_max_mode_records_early_stop():
    from app.run_config import build_run_config

    model = {"Type": "MOO", "Exp": "TC", "Alg": "NSGA2",
             "F": ["Mission Time", "Percentage Connectivity"], "G": [], "H": []}
    cfg = build_run_config(
        model_dict=model, objectives=["Mission Time", "Percentage Connectivity"],
        weights=None, pop_size=50, n_gen=200, seed=1,
        gen_strategy="max", early_stop_patience=15, early_stop_threshold=0.05,
        max_mission_time=None, min_connectivity=None, max_mean_tbv=None,
        scenario={}, n_solutions=3, cancelled=False, early_stopped=True,
        stopped_at_gen=84, source="optimizer",
    )
    assert cfg["early_stop_patience"] == 15 and cfg["early_stop_threshold"] == 0.05
    assert cfg["outcome"]["early_stopped"] is True
    assert cfg["outcome"]["stopped_at_gen"] == 84


def test_build_run_config_ws_maps_type_and_weights():
    from app.run_config import build_run_config

    model = {"Type": "WS", "Exp": "TC", "Alg": "GA",
             "F": ["Mission Time & Percentage Connectivity Weighted Sum"],
             "G": [], "H": []}
    cfg = build_run_config(
        model_dict=model, objectives=["Mission Time", "Percentage Connectivity"],
        weights={"Mission Time": 0.5, "Percentage Connectivity": 0.5},
        pop_size=300, n_gen=800, seed=1, gen_strategy="fixed",
        early_stop_patience=10, early_stop_threshold=0.10,
        max_mission_time=3600.0, min_connectivity=0.5, max_mean_tbv=None,
        scenario={}, n_solutions=1, cancelled=False, early_stopped=False,
        stopped_at_gen=800, source="optimizer",
    )
    assert cfg["optimization_type"] == "SOO" and cfg["method"] == "WS"
    assert cfg["weights"] == {"Mission Time": 0.5, "Percentage Connectivity": 0.5}


def test_seed_config_mtsp_has_no_mission_time_constraint():
    from app.run_config import seed_config_for

    cfg = seed_config_for("SOO_GA_MTSP_g_8_a_50_n_12_v_2.5_r_2_nvisits_1", n_solutions=5)
    assert cfg is not None
    assert cfg["source"] == "seed"
    assert cfg["pop_size"] == 300 and cfg["n_gen"] == 800 and cfg["seed"] == 1
    assert cfg["gen_strategy"] == "fixed"
    assert cfg["constraints"]["speed_feasibility"] is True
    assert cfg["constraints"]["max_mission_time"] is None      # MTSP declares none
    assert cfg["constraints"]["min_connectivity"] == 0.5       # 12 drones > 2
    assert cfg["constraints"]["max_mean_tbv"] is None
    assert cfg["objectives"] == ["Mission Time"]
    assert cfg["weights"] is None
    assert cfg["scenario"]["number_of_drones"] == 12
    assert cfg["scenario"]["n_visits"] == 1


def test_seed_config_tcdt_has_all_constraints():
    from app.run_config import seed_config_for

    cfg = seed_config_for("MOO_NSGA2_TCDT_g_8_a_50_n_12_v_2.5_r_2_nvisits_1", n_solutions=9)
    assert cfg is not None
    assert cfg["constraints"]["max_mission_time"] == 3600.0     # n_visits 1 < 4
    assert cfg["constraints"]["min_connectivity"] == 0.5
    assert cfg["constraints"]["max_mean_tbv"] is None
    assert "Mission Time" in cfg["objectives"] and len(cfg["objectives"]) == 5
    assert cfg["optimization_type"] == "MOO" and cfg["method"] == "NSGA2"


def test_seed_config_ws_uses_equal_weights():
    from app.run_config import seed_config_for

    cfg = seed_config_for("WS_GA_TC_g_8_a_50_n_12_v_2.5_r_2_nvisits_1", n_solutions=1)
    assert cfg is not None
    assert cfg["optimization_type"] == "SOO" and cfg["method"] == "WS"
    assert cfg["objectives"] == ["Mission Time", "Percentage Connectivity"]
    assert cfg["weights"] == {"Mission Time": 0.5, "Percentage Connectivity": 0.5}
    assert cfg["constraints"]["max_mission_time"] == 3600.0
    assert cfg["constraints"]["min_connectivity"] == 0.5


def test_seed_config_unknown_name_returns_none():
    from app.run_config import seed_config_for

    assert seed_config_for("MOO_NSGA2_ZZZ_g_8_a_50_n_4_v_2.5_r_2_nvisits_1") is None
