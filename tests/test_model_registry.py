import pandas as pd

from PathOptimizationModel import AVAILABLE_MODELS, list_models


def test_registry_complete():
    assert len(AVAILABLE_MODELS) == 20
    assert AVAILABLE_MODELS["TCDT_MOO_NSGA2"]["Exp"] == "TCDT"
    assert AVAILABLE_MODELS["MTSP"]["Type"] == "SOO"


def test_list_models_dropdown_ready():
    df = list_models()
    assert isinstance(df, pd.DataFrame)
    assert set(df.columns) == {"name", "type", "algorithm", "objectives", "constraints"}
    assert len(df) == 20
    row = df[df.name == "TC_MOO_NSGA2"].iloc[0]
    assert row.type == "MOO" and row.algorithm == "NSGA2"
    assert "Mission Time" in row.objectives
