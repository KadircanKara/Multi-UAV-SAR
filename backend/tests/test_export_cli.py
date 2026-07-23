import json
import os
import subprocess
import sys

import pytest

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SEEDED = "MOO_NSGA2_TCD_g_8_a_50_n_4_v_2.5_r_2_nvisits_1"

pytestmark = pytest.mark.needs_seed_data


def test_cli_exports_valid_playground_json(tmp_path):
    out = tmp_path / "run.json"
    env = dict(os.environ, PYTHONPATH=f"{os.path.join(REPO, 'backend')}:{REPO}")
    r = subprocess.run(
        [sys.executable, "backend/scripts/export_run_json.py",
         "--scenario", SEEDED, "--out", str(out)],
        cwd=REPO, env=env, capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    payload = json.load(open(out))
    # Validates against the schema and carries the full resolved model key.
    sys.path[:0] = [os.path.join(REPO, "backend"), REPO]
    from app.playground_schema import PlaygroundResult
    result = PlaygroundResult.model_validate(payload)
    assert result.model["model_key"] == "TCD_MOO_NSGA2"
    assert len(result.solutions) > 0
