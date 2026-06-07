# Merging & Visual-Analysis Layer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix and complete both sensing/merging pipelines (discrete + realtime), wrap them behind a `SensingConfig`-driven `replay()`/`compare()` API, and add a model-aware `SolutionSelector` + model registry — per the approved spec `docs/superpowers/specs/2026-06-07-merging-analysis-layer-design.md`.

**Architecture:** Merging parameters live only in the analysis layer (`SensingConfig`), never in the scenario/`PathInfo`/filenames. Both `Sensing.py` pipelines satisfy one 7-key return contract so `SensingReplay.replay()` dispatches blindly on `time_model`. Pareto-front navigation is by intent through a self-describing `SolutionSelector` gated by `capabilities()`. Flat module layout preserved.

**Tech Stack:** Python 3, pymoo 0.6.1 (incl. `pymoo.mcdm.pseudo_weights.PseudoWeights` → returns int index; `pymoo.mcdm.high_tradeoff.HighTradeoffPoints` → returns index array **or `None`** — verified in the project venv), numpy 1.26, pandas, matplotlib (animations saved as GIF via `PillowWriter` — no ffmpeg dependency), pytest.

**Branch:** all work on `dev` (already checked out). Run tests with `.venv/bin/python -m pytest`.

**Verified codebase facts the plan relies on** (do not re-derive; line numbers refer to current `dev`):
- `ObjectiveValues.pkl` stores **signed** F (`F = pd.DataFrame(res.F, columns=model['F'])`, PathUnitTest.py:100; connectivity is negative because PathProblem multiplies by polarity −1). Therefore **`idxmin` per column = best for every objective**. `ObjectiveValuesAbs.pkl` = `abs(F)` is for display only.
- `SolutionObjects.pkl` = `res.X.flatten()`; elements may be 1-element arrays (PathUnitTest.py:104-110 unwraps with `row[0]`).
- `PathSolution(path, start_points, info, calculate_pathplan=False, calculate_tbv=False, calculate_connectivity=False, calculate_disconnectivity=False)` (PathSolution.py:50). `calculate_pathplan=True` builds `real_time_path_matrix`; `calculate_connectivity=True` builds `connectivity_matrix`.
- `get_real_paths` (Time.py:227-259): per leg `dt = ceil(max_dist/speed)`, `np.linspace(..., dt)` endpoint-INCLUSIVE → duplicate grid columns at leg seams; returns matrices including BS row 0; also stashes `sol.real_time_x_matrix/_y_matrix`.
- `intp_between_coords(cx, cy, nx, ny, v)` (Time.py:205-224): endpoint-EXCLUSIVE linspace; returns `(np.array([]), np.array([]))` when distance is 0.
- `interpolate_between_cities(sol, city_prev, city)` is defined locally in Sensing.py:34 (no import needed inside Sensing.py).
- `PathAnimation(sol, fig, ax, target_locations=None, cell_occupancy_probabilities=None, B=0.9, p0=0.5)` (PathAnimation.py:10) re-derives trajectories via `get_real_paths(self.sol)` — early-return truncation MUST be written into `sol.real_time_path_matrix` to show up in animations. Animation driving pattern (Results.py:100-104): `FuncAnimation(fig, anim_object.update, frames=anim_object.paths[0].shape[1], init_func=anim_object.initialize_figure, blit=False, interval=0)`.
- `Sensing.py` imports: line 13-14 `from copy import copy` / `import copy` shadow each other and `deepcopy` is NOT in the module namespace (verified: `'deepcopy' in dir(Sensing)` → False) → both pipelines crash on entry.
- `PathInfo.__init__` reads module-level `model`, `pop_size`, `n_gen` via `from PathInput import *` (PathInfo.py:6,24-26).

---

## File structure (created/modified)

| File | Responsibility |
|---|---|
| `tests/conftest.py` *(new)* | Deterministic small `PathSolution` fixtures |
| `tests/test_merge_maps.py` *(new)* | Topology semantics unit tests |
| `tests/test_sensing_contract.py` *(new)* | Cross-pipeline contract + realtime invariants |
| `tests/test_discrete_regression.py` *(new)* | Snapshot lock on discrete behavior |
| `tests/test_sensing_config.py` *(new)* | `SensingConfig` validation/defaulting |
| `tests/test_replay.py` *(new)* | `replay()`/`compare()` artifacts |
| `tests/test_solution_selection.py` *(new)* | Selector capability matrix + strategies |
| `tests/test_model_registry.py` *(new)* | `AVAILABLE_MODELS`/`list_models()` |
| `Sensing.py` *(modify)* | Import fix; `merge_maps` topology rework; 4 shared helpers; realtime rewrite; config-based signatures |
| `Time.py` *(modify)* | Tolerance-based `isCoordinateDiscrete` |
| `SensingReplay.py` *(new)* | `SensingConfig`, `ReplayResult`, `replay()`, `compare()` |
| `SolutionSelection.py` *(new)* | `SolutionSelector`, `StrategyUnavailableError` |
| `PathOptimizationModel.py` *(modify)* | `AVAILABLE_MODELS`, `list_models()` |
| `PathInput.py` *(modify)* | Model resolution by registry name |
| `Analysis.py` *(modify)* | Call sites → `SensingConfig` |
| `requirements.txt` *(modify)* | Add pytest (dev) |
| `README.md` *(modify)* | Document new API |

---

# Phase 1 — Sensing foundations & fixes

### Task 1: Test infrastructure

**Files:**
- Modify: `requirements.txt`
- Create: `tests/conftest.py` (NO `tests/__init__.py` — pytest's prepend import
  mode must treat `tests/` as a plain directory so `from conftest import ...`
  works in test modules)

- [ ] **Step 1: Add pytest to requirements**

Append to `requirements.txt`:

```
pytest>=8.0          # dev/test dependency
```

Run: `.venv/bin/pip install pytest`
Expected: `Successfully installed pytest-...`

- [ ] **Step 2: Create the fixtures**

Create `tests/conftest.py`:

```python
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import pytest

from PathInfo import PathInfo
from PathSolution import PathSolution

GRID = 8
N_DRONES = 4
TARGET_CELL = 12          # visited early by drone 0 in the full-coverage path


def make_scenario(target_positions):
    return {
        'grid_size': GRID,
        'cell_side_length': 50,
        'number_of_drones': N_DRONES,
        'max_drone_speed': 2.5,
        'comm_cell_range': 2,
        'n_visits': 1,
        'target_positions': list(target_positions),
        'th': 0.9,
        'detection_probability': 0.7,
    }


@pytest.fixture(scope="session")
def small_solution():
    """Deterministic full-coverage solution: cells 0..63 in order, 4 equal subtours.

    session-scoped because building the path plan + connectivity matrix is the
    slow part; tests must NOT mutate it (the pipelines deepcopy internally).
    """
    info = PathInfo(make_scenario([TARGET_CELL]))
    path = np.arange(info.number_of_cells)          # 0..63
    start_points = np.array([0, 16, 32, 48])        # drone d starts at path index 16*d
    return PathSolution(path, start_points, info,
                        calculate_pathplan=True, calculate_connectivity=True)
```

- [ ] **Step 3: Verify the fixture builds**

Run: `.venv/bin/python -m pytest tests/ --collect-only -q`
Expected: `no tests ran` (collection succeeds, no import errors). If `PathSolution` construction fails on `start_points` shape, check `PathSampling.py` for the exact representation and adjust the fixture (start_points are sorted path indices, first one 0, length = number_of_drones).

- [ ] **Step 4: Commit**

```bash
git add requirements.txt tests/
git commit -m "test: add pytest infrastructure and deterministic solution fixture"
```

---

### Task 2: Failing sensing-contract tests (red)

**Files:**
- Create: `tests/test_sensing_contract.py`

These encode the spec's Addendum-A contract. They MUST fail now (both pipelines crash on `deepcopy`); that is the red state.

- [ ] **Step 1: Write the failing tests**

Create `tests/test_sensing_contract.py`:

```python
import numpy as np
import pytest

from Sensing import sensing_and_discrete_info_sharing, sensing_and_realtime_info_sharing

# NOTE: until Task 11 flips signatures to SensingConfig, call with the current
# loose kwargs. Task 11 updates ONLY the call shape in this file, not the asserts.
KW = dict(target_locations=[12], B=0.7, p=0.7, q=0.2)
KW_STRICT = dict(target_locations=[12], B=0.999, p=0.7, q=0.2)  # B unreachable with n_visits=1

EXPECTED_KEYS = {
    "cell occupancy probabilities", "search map", "occupancy status",
    "detection time", "inform time", "mission time",
    "time at least one drone knows all targets",
}


def test_realtime_runs_without_nameerror(small_solution):
    metrics, x = sensing_and_realtime_info_sharing(small_solution, merging_strategy="onboard", **KW)
    assert isinstance(metrics, dict)


def test_discrete_runs_without_nameerror(small_solution):
    metrics, x = sensing_and_discrete_info_sharing(small_solution, merging_strategy="onboard", **KW)
    assert isinstance(metrics, dict)


def test_both_pipelines_return_identical_keys(small_solution):
    m_d, _ = sensing_and_discrete_info_sharing(small_solution, merging_strategy="onboard", **KW)
    m_r, _ = sensing_and_realtime_info_sharing(small_solution, merging_strategy="onboard", **KW)
    assert set(m_d.keys()) == EXPECTED_KEYS
    assert set(m_r.keys()) == EXPECTED_KEYS


def test_realtime_mission_time_finite(small_solution):
    metrics, _ = sensing_and_realtime_info_sharing(small_solution, merging_strategy="onboard", **KW)
    assert np.isfinite(metrics["mission time"]) and metrics["mission time"] > 0


def test_realtime_early_return_shortens_mission(small_solution):
    # Favorable: low threshold + onboard merging -> drones learn fast, go home early.
    # Unfavorable: unreachable threshold + no merging -> full path is flown.
    favorable, _ = sensing_and_realtime_info_sharing(small_solution, merging_strategy="onboard", **KW)
    unfavorable, _ = sensing_and_realtime_info_sharing(small_solution, merging_strategy="none", **KW_STRICT)
    assert np.isfinite(favorable["mission time"])
    assert favorable["mission time"] < unfavorable["mission time"]


def test_realtime_detection_finite_when_detectable(small_solution):
    metrics, _ = sensing_and_realtime_info_sharing(small_solution, merging_strategy="onboard", **KW)
    assert np.isfinite(metrics["detection time"])


def test_realtime_detection_inf_when_threshold_unreachable(small_solution):
    metrics, _ = sensing_and_realtime_info_sharing(small_solution, merging_strategy="onboard", **KW_STRICT)
    assert metrics["detection time"] == np.inf


def test_onboard_le_none_detection_within_realtime(small_solution):
    onboard, _ = sensing_and_realtime_info_sharing(small_solution, merging_strategy="onboard", **KW)
    none_, _ = sensing_and_realtime_info_sharing(small_solution, merging_strategy="none", **KW)
    assert onboard["detection time"] <= none_["detection time"]


def test_onboard_le_none_detection_within_discrete(small_solution):
    onboard, _ = sensing_and_discrete_info_sharing(small_solution, merging_strategy="onboard", **KW)
    none_, _ = sensing_and_discrete_info_sharing(small_solution, merging_strategy="none", **KW)
    assert onboard["detection time"] <= none_["detection time"]
```

- [ ] **Step 2: Run and verify they fail with NameError**

Run: `.venv/bin/python -m pytest tests/test_sensing_contract.py -x -q`
Expected: FAIL — `NameError: name 'deepcopy' is not defined` (and `merge_maps` rejects nothing yet — `"onboard"` currently behaves as merging because only `"gcs"` is special-cased; that becomes correct semantics in Task 5).

- [ ] **Step 3: Commit**

```bash
git add tests/test_sensing_contract.py
git commit -m "test: add failing sensing-pipeline contract tests (red)"
```

---

### Task 3: Fix the deepcopy import (defect A1)

**Files:**
- Modify: `Sensing.py:13-14`

- [ ] **Step 1: Fix the imports**

In `Sensing.py`, replace lines 13-14:

```python
from copy import copy
import copy
```

with:

```python
from copy import copy, deepcopy
```

(Nothing in `Sensing.py` uses the `copy` *module*; only bare `copy(...)` at most and `deepcopy(...)` at lines 124 and 386 — verify with `grep -n "copy\." Sensing.py` → expect no `copy.something` hits.)

- [ ] **Step 2: Verify the NameError tests now get past entry**

Run: `.venv/bin/python -m pytest tests/test_sensing_contract.py::test_discrete_runs_without_nameerror -x -q`
Expected: PASS (discrete pipeline is otherwise intact). Realtime tests may still fail on later defects — that is expected until Task 7.

- [ ] **Step 3: Commit**

```bash
git add Sensing.py
git commit -m "fix: import deepcopy correctly in Sensing (both pipelines crashed on entry)"
```

---

### Task 4: Discrete regression snapshot (locks behavior before refactors)

**Files:**
- Create: `tests/test_discrete_regression.py`

- [ ] **Step 1: Capture the current discrete output as the snapshot**

Run this ONCE to print the values to freeze:

```bash
.venv/bin/python - <<'EOF'
import sys; sys.path.insert(0, "tests")
from conftest import make_scenario
import numpy as np
from PathInfo import PathInfo
from PathSolution import PathSolution
from Sensing import sensing_and_discrete_info_sharing

info = PathInfo(make_scenario([12]))
sol = PathSolution(np.arange(64), np.array([0, 16, 32, 48]), info,
                   calculate_pathplan=True, calculate_connectivity=True)
m, _ = sensing_and_discrete_info_sharing(sol, merging_strategy="onboard",
                                         target_locations=[12], B=0.7, p=0.7, q=0.2)
for k in ["detection time", "inform time", "mission time",
          "time at least one drone knows all targets"]:
    print(f"{k!r}: {m[k]!r},")
print("occupancy sum:", int(np.sum(m["occupancy status"])))
print("n occupancy prob steps:", len(m["cell occupancy probabilities"][0]))
EOF
```

- [ ] **Step 2: Write the snapshot test with the printed literals**

Create `tests/test_discrete_regression.py`, substituting `<...>` with the literals printed in Step 1:

```python
import numpy as np
from Sensing import sensing_and_discrete_info_sharing

# Frozen 2026-06-07 from the pre-refactor discrete pipeline (Task 4 Step 1).
# If a behavior-preserving refactor changes ANY of these, the refactor is wrong.
SNAPSHOT = {
    "detection time": <detection time literal>,
    "inform time": <inform time literal>,
    "mission time": <mission time literal>,
    "time at least one drone knows all targets": <literal>,
}
OCCUPANCY_SUM = <occupancy sum literal>
N_PROB_STEPS = <n occupancy prob steps literal>


def test_discrete_metrics_unchanged(small_solution):
    m, _ = sensing_and_discrete_info_sharing(
        small_solution, merging_strategy="onboard",
        target_locations=[12], B=0.7, p=0.7, q=0.2)
    for key, expected in SNAPSHOT.items():
        assert m[key] == expected, f"{key} drifted: {m[key]} != {expected}"
    assert int(np.sum(m["occupancy status"])) == OCCUPANCY_SUM
    assert len(m["cell occupancy probabilities"][0]) == N_PROB_STEPS
```

(Task 11 changes only the call shape to `SensingConfig`; the literals stay.)

- [ ] **Step 3: Run, verify green**

Run: `.venv/bin/python -m pytest tests/test_discrete_regression.py -q`
Expected: PASS

- [ ] **Step 4: Commit**

```bash
git add tests/test_discrete_regression.py
git commit -m "test: freeze discrete pipeline behavior snapshot before refactors"
```

---

### Task 5: `merge_maps` topology rework (D3/D4/D5)

**Files:**
- Create: `tests/test_merge_maps.py`
- Modify: `Sensing.py:62-114` (merge_maps), plus its two call sites `Sensing.py:244` and `:441`

- [ ] **Step 1: Write the failing topology tests**

Create `tests/test_merge_maps.py`:

```python
import numpy as np
import pytest

from Sensing import merge_maps


def _map(n_nodes=3, n_cells=4):
    m = np.full((n_nodes, n_cells), None, dtype=object)
    for i in range(n_nodes):
        for j in range(n_cells):
            m[i, j] = [{"n_obs": 0, "timestep": -1, "prob": 0.5}]
    return m


def test_unknown_topology_raises():
    with pytest.raises(ValueError):
        merge_maps([[0, 1, 2]], _map(), "ondrone")   # legacy name must be rejected
    with pytest.raises(ValueError):
        merge_maps([[0, 1, 2]], _map(), "discrete")  # the old vocabulary-collision value


def test_none_never_propagates():
    m = _map()
    m[1, 0].append({"n_obs": 1, "timestep": 5, "prob": 0.9})  # node 1 saw something
    out = merge_maps([[0, 1, 2]], m, "none")
    assert out[2, 0][-1]["prob"] == 0.5   # node 2 learned nothing
    assert out[0, 0][-1]["prob"] == 0.5   # BS learned nothing


def test_onboard_merges_any_clique():
    m = _map()
    m[1, 0].append({"n_obs": 1, "timestep": 5, "prob": 0.9})
    out = merge_maps([[1, 2]], m, "onboard")          # clique WITHOUT the BS (node 0)
    assert out[2, 0][-1]["prob"] == 0.9               # drone 2 received it
    assert out[0, 0][-1]["prob"] == 0.5               # BS not in clique -> unchanged


def test_gcs_requires_bs_in_clique():
    m = _map()
    m[1, 0].append({"n_obs": 1, "timestep": 5, "prob": 0.9})
    out = merge_maps([[1, 2]], m, "gcs")              # no BS in clique
    assert out[2, 0][-1]["prob"] == 0.5               # nothing shared
    m2 = _map()
    m2[1, 0].append({"n_obs": 1, "timestep": 5, "prob": 0.9})
    out2 = merge_maps([[0, 1, 2]], m2, "gcs")         # BS present
    assert out2[2, 0][-1]["prob"] == 0.9
```

Run: `.venv/bin/python -m pytest tests/test_merge_maps.py -q`
Expected: FAIL (`merge_maps` currently treats every non-"gcs" string as merging; no ValueError, no "none").

- [ ] **Step 2: Rework merge_maps**

In `Sensing.py`, change the function signature and add the guard (line 62; the loop body from `for clique in conn_comp:` onward is UNCHANGED except the topology comparison):

```python
VALID_MERGE_TOPOLOGIES = ("none", "onboard", "gcs")

def merge_maps(conn_comp, search_map, merge_topology="onboard"):
    if merge_topology not in VALID_MERGE_TOPOLOGIES:
        raise ValueError(
            f"Unknown merge_topology {merge_topology!r}; valid: {VALID_MERGE_TOPOLOGIES}")
    if merge_topology == "none":
        return search_map
    number_of_nodes, number_of_cells = search_map.shape

    for clique in conn_comp:
        if merge_topology == "gcs" and 0 not in clique:
            continue
        # ... existing body from line 70 onward, byte-identical ...
```

Update the two call sites to pass through the parameter (they currently pass `merging_strategy`, which Task 11 renames at the function-signature level; for now keep the variable name but it now must hold a VALID topology):
- `Sensing.py:244`: `search_map = merge_maps(conn_comp, search_map, merging_strategy)` — unchanged text, but callers must now pass `"none"/"onboard"/"gcs"`.
- `Sensing.py:441`: same.

Also update `Analysis.py:368-370` minimally so it doesn't pass the colliding values (this is the bug fix; full config migration is Task 11): replace

```python
if merging_strategy == "discrete":
    time_metrics, updated_sol = sensing_and_discrete_info_sharing(sol=sol, merging_strategy=merging_strategy, target_locations=target_locations, B=B, p=p, q=q)
elif merging_strategy == "realtime":
    time_metrics, updated_sol = sensing_and_realtime_info_sharing(sol=sol, merging_strategy=merging_strategy, target_locations=target_locations, B=B, p=p, q=q)
```

with:

```python
if merging_strategy == "discrete":
    time_metrics, updated_sol = sensing_and_discrete_info_sharing(sol=sol, merging_strategy="onboard", target_locations=target_locations, B=B, p=p, q=q)
elif merging_strategy == "realtime":
    time_metrics, updated_sol = sensing_and_realtime_info_sharing(sol=sol, merging_strategy="onboard", target_locations=target_locations, B=B, p=p, q=q)
```

and the other two discrete call sites (`Analysis.py:71` and `:164`) likewise get `merging_strategy="onboard"` instead of forwarding the time-model string.

- [ ] **Step 3: Run merge_maps tests + regression**

Run: `.venv/bin/python -m pytest tests/test_merge_maps.py tests/test_discrete_regression.py -q`
Expected: PASS (regression still green: snapshot used `"onboard"`, which behaves exactly as the old fallthrough did).

- [ ] **Step 4: Commit**

```bash
git add Sensing.py Analysis.py tests/test_merge_maps.py
git commit -m "feat: explicit merge topologies (none/onboard/gcs) with validation in merge_maps"
```

---

### Task 6: Tolerance-based `isCoordinateDiscrete` (defect A4)

**Files:**
- Modify: `Time.py:9-15`
- Test: append to `tests/test_merge_maps.py` (small, sensing-adjacent unit test)

- [ ] **Step 1: Write the failing test**

Append to `tests/test_merge_maps.py`:

```python
from Time import isCoordinateDiscrete


def test_isCoordinateDiscrete_tolerates_float_error(small_solution):
    x_exact, y_exact = small_solution.get_coords(12)
    assert isCoordinateDiscrete(x_exact, y_exact, small_solution)
    # interpolation-grade float error must still classify as on-grid
    assert isCoordinateDiscrete(x_exact + 1e-9, y_exact - 1e-9, small_solution)
    # mid-cell must NOT
    half = small_solution.info.cell_side_length / 2
    assert not isCoordinateDiscrete(x_exact + half, y_exact, small_solution)
```

Run: `.venv/bin/python -m pytest tests/test_merge_maps.py::test_isCoordinateDiscrete_tolerates_float_error -q`
Expected: FAIL on the `+1e-9` assertion (exact `in` comparison).

- [ ] **Step 2: Replace the implementation**

Replace `Time.py:9-15` with:

```python
def isCoordinateDiscrete(x, y, sol: PathSolution, atol=None):
    """True iff (x, y) lies on a grid cell center, within float tolerance.

    atol defaults to a millionth of a cell side: far above linspace float
    error, far below half a cell, so misclassification is impossible.
    """
    if atol is None:
        atol = sol.info.cell_side_length * 1e-6
    base = sol.get_coords(-1)[0]
    discrete_coords = np.array(
        [base + sol.info.cell_side_length * i for i in range(sol.info.grid_size + 1)])
    return bool(np.min(np.abs(discrete_coords - x)) <= atol
                and np.min(np.abs(discrete_coords - y)) <= atol)
```

- [ ] **Step 3: Run the test + full suite**

Run: `.venv/bin/python -m pytest tests/ -q`
Expected: tolerance test PASS; realtime contract tests still failing (Task 7 fixes them); everything else green.

- [ ] **Step 4: Commit**

```bash
git add Time.py tests/test_merge_maps.py
git commit -m "fix: tolerance-based isCoordinateDiscrete (exact float equality dropped grid hits)"
```

---

### Task 7: Shared helpers + discrete refactor (AD4)

**Files:**
- Modify: `Sensing.py` (add 4 module-level helpers; refactor `sensing_and_discrete_info_sharing` to use them)

- [ ] **Step 1: Add the helpers** (place directly above `merge_maps`)

```python
def _init_search_map(number_of_nodes, number_of_cells):
    default_obs = [{"n_obs": 0, "timestep": -1, "prob": 0.5}]
    search_map = np.full((number_of_nodes, number_of_cells), fill_value=None, dtype=object)
    for i in range(number_of_nodes):
        for j in range(number_of_cells):
            search_map[i, j] = default_obs.copy()
    return search_map


def _compute_occupancy_status(search_map, B, number_of_nodes, number_of_cells):
    """Occupancy flags (any historical prob > B) + per-cell max of LATEST probs."""
    occupancy_status = np.zeros((number_of_nodes, number_of_cells), dtype=int)
    per_cell_max_probs = []
    for col in range(number_of_cells):
        cell_probs = []
        for row in range(number_of_nodes):
            cell_probs.append(search_map[row, col][-1]["prob"])
            probs = [entry["prob"] for entry in search_map[row, col]]
            if any(prob > B for prob in probs):
                occupancy_status[row, col] = 1
        per_cell_max_probs.append(max(cell_probs))
    return occupancy_status, per_cell_max_probs


def _update_detection_timesteps(occupancy_status, target_locations, step,
                                t_all_known, t_bs_knows, t_one_knows):
    if t_all_known == np.inf:
        if len(np.unique(np.where(occupancy_status == 1)[1])) >= len(target_locations):
            t_all_known = step
    if t_bs_knows == np.inf:
        if np.sum(occupancy_status[0]) >= len(target_locations):
            t_bs_knows = step
    if t_one_knows == np.inf:
        if np.any(np.sum(occupancy_status[1:], axis=1) >= len(target_locations)):
            t_one_knows = step
    return t_all_known, t_bs_knows, t_one_knows


def _update_target_detection_times(x, occupancy_status, step):
    missing_targets = [t for t in list(x.target_detection_times.keys())
                       if x.target_detection_times[t] is None]
    if len(missing_targets) != 0:
        if np.sum(occupancy_status[:, missing_targets]) != 0:
            for target in missing_targets:
                if occupancy_status[:, target].any():
                    x.target_detection_times[target] = sum(x.time_elapsed_at_steps[:step])


def _finalize_metrics(time_elapsed_at_steps, t_all_known, t_bs_knows, t_one_knows, t_back):
    time_at_least_one = sum(time_elapsed_at_steps[:t_one_knows]) if t_one_knows != np.inf else np.inf
    detection_time = sum(time_elapsed_at_steps[:t_all_known]) if t_all_known != np.inf else np.inf
    inform_time = sum(time_elapsed_at_steps[t_all_known:t_bs_knows]) if t_bs_knows != np.inf else np.inf
    mission_time = sum(time_elapsed_at_steps[:t_back]) if t_back != np.inf else np.inf
    return detection_time, inform_time, mission_time, time_at_least_one
```

- [ ] **Step 2: Refactor the discrete function to call them**

In `sensing_and_discrete_info_sharing`:
- Replace the search-map init block (Sensing.py:394-405) with `search_map = _init_search_map(x.info.number_of_nodes, x.info.number_of_cells)`.
- Replace the occupancy block (Sensing.py:443-453) with:

```python
        occupancy_status, per_cell_max = _compute_occupancy_status(
            search_map, B, info.number_of_nodes, info.number_of_cells)
        for col in range(info.number_of_cells):
            cell_occupancy_probabilities[col].append(per_cell_max[col])
```

- Replace the three tracking `if`s (Sensing.py:461-472) with:

```python
        (timestep_all_targets_are_known, timestep_bs_knows_all_targets,
         timestep_at_least_one_drone_knows_all_targets) = _update_detection_timesteps(
            occupancy_status, target_locations, step,
            timestep_all_targets_are_known, timestep_bs_knows_all_targets,
            timestep_at_least_one_drone_knows_all_targets)
```

- Replace the target-detection block (Sensing.py:484-490) with `_update_target_detection_times(x, occupancy_status, step)`.
- Replace the four final-metric lines (Sensing.py:519-528) with:

```python
    detection_time, inform_time, mission_time, time_at_least_one = _finalize_metrics(
        x.time_elapsed_at_steps, timestep_all_targets_are_known,
        timestep_bs_knows_all_targets, timestep_at_least_one_drone_knows_all_targets,
        timestep_drones_are_back_at_bs)
```

- [ ] **Step 3: The regression snapshot is the gate**

Run: `.venv/bin/python -m pytest tests/test_discrete_regression.py tests/test_merge_maps.py -q`
Expected: PASS, identical snapshot values. If any value drifts, the refactor changed behavior — fix the refactor, never the snapshot.

- [ ] **Step 4: Commit**

```bash
git add Sensing.py
git commit -m "refactor: extract shared sensing helpers; discrete pipeline locked by snapshot"
```

---

### Task 8: Rewrite the realtime pipeline (defects A2, A3, A5, A6; AD2, AD3)

**Files:**
- Modify: `Sensing.py:117-382` (`sensing_and_realtime_info_sharing` — full body replacement)

- [ ] **Step 1: Replace the function body**

Replace the entire `sensing_and_realtime_info_sharing` function (current lines 117-382) with:

```python
def sensing_and_realtime_info_sharing(sol: PathSolution, merging_strategy="onboard",
                                      target_locations=[12], B=0.9, p=0.9, q=0.2):
    """Realtime sensing + merging: continuous positions, per-second connectivity,
    mid-flight merging. Sensing fires only on NEW grid arrivals (seam-deduped).
    Returns the same 7-key metrics dict as the discrete pipeline (contract AD1).
    """
    x = deepcopy(sol)
    info = x.info
    drone_path_matrix = x.real_time_path_matrix[1:, :]
    realtime_x, realtime_y = get_real_paths(x)

    number_of_nodes, timesteps = realtime_x.shape
    number_of_drones = number_of_nodes - 1

    connectivity_matrix = get_real_connectivity_matrix(realtime_x, realtime_y, sol)

    search_map = _init_search_map(info.number_of_nodes, info.number_of_cells)
    cell_occupancy_probabilities = [[] for _ in range(info.number_of_cells)]

    drone_search_status = [True for _ in range(number_of_drones)]
    timestep_bs_knows_all_targets = np.inf
    timestep_at_least_one_drone_knows_all_targets = np.inf
    timestep_all_targets_are_known = np.inf
    timestep_drones_are_back_at_bs = np.inf

    x.mission_time = 0
    x.time_elapsed_at_steps = []
    x.target_detection_times = {target: None for target in target_locations}

    drone_positions = {drone: -1 for drone in range(number_of_drones)}
    discrete_step = -1

    cell_0_x, cell_0_y = sol.get_coords(0)
    cell_bs_x, cell_bs_y = sol.get_coords(-1)

    for step in range(timesteps):

        # --- locate drones; detect grid alignment -------------------------------
        all_on_grid = True
        for drone in range(number_of_drones):
            pos_x = realtime_x[drone + 1, step]
            pos_y = realtime_y[drone + 1, step]
            drone_positions[drone] = sol.get_city((pos_x, pos_y))
            if not isCoordinateDiscrete(pos_x, pos_y, sol):
                all_on_grid = False

        # Seam dedup (AD2): get_real_paths uses endpoint-inclusive linspace, so the
        # last column of leg i duplicates the first column of leg i+1. A column only
        # counts as a NEW grid arrival if any coordinate changed since the previous
        # column; duplicates still merge/track time but never sense twice.
        is_new_grid_arrival = all_on_grid and (
            step == 0
            or not (np.array_equal(realtime_x[:, step], realtime_x[:, step - 1])
                    and np.array_equal(realtime_y[:, step], realtime_y[:, step - 1])))
        if is_new_grid_arrival:
            discrete_step += 1

        adj_mat = connectivity_matrix[step]
        conn_comp = connected_components(adj_mat)

        # --- sensing: only on new grid arrivals ---------------------------------
        if is_new_grid_arrival:
            for drone in range(number_of_drones):
                pos = drone_positions[drone]
                if pos == -1:
                    continue
                prior = search_map[drone + 1, pos][-1]["prob"]
                if pos in target_locations:
                    new_prob = p * prior / (p * prior + q * (1 - prior))
                else:
                    new_prob = (1 - p) * prior / ((1 - p) * prior + (1 - q) * (1 - prior))
                # n_obs counted from discrete-path arrivals, matching the discrete
                # pipeline's bookkeeping for the same physical visit (AD2).
                n_obs = len(np.where(drone_path_matrix[drone, :discrete_step + 1] == pos)[0])
                search_map[drone + 1, pos].append(
                    {"n_obs": n_obs, "timestep": discrete_step, "prob": new_prob})

        # --- merging: EVERY second, including mid-flight (AD5) ------------------
        search_map = merge_maps(conn_comp, search_map, merging_strategy)

        # --- occupancy + tracking (shared helpers) ------------------------------
        occupancy_status, per_cell_max = _compute_occupancy_status(
            search_map, B, info.number_of_nodes, info.number_of_cells)
        for col in range(info.number_of_cells):
            cell_occupancy_probabilities[col].append(per_cell_max[col])

        (timestep_all_targets_are_known, timestep_bs_knows_all_targets,
         timestep_at_least_one_drone_knows_all_targets) = _update_detection_timesteps(
            occupancy_status, target_locations, step,
            timestep_all_targets_are_known, timestep_bs_knows_all_targets,
            timestep_at_least_one_drone_knows_all_targets)

        # Realtime columns are ~1-second apart by construction of get_real_paths
        # (dt = ceil(dist/speed) per leg) — see spec Addendum A "not a defect".
        x.time_elapsed_at_steps.append(1)
        x.mission_time += 1

        _update_target_detection_times(x, occupancy_status, step)

        # --- early return-to-base (AD3) ------------------------------------------
        for m in range(number_of_drones):
            if step > 0 and drone_positions[m] == -1:
                continue
            if drone_search_status[m]:
                knows_all = np.sum(occupancy_status[m + 1]) >= len(target_locations)
                is_connected_to_bs = m + 1 in get_connected_node_ids(adj_mat, 0)
                if (timestep_bs_knows_all_targets != np.inf and is_connected_to_bs) or knows_all:
                    drone_search_status[m] = False
                    drone_x_pos = realtime_x[m + 1, step]
                    drone_y_pos = realtime_y[m + 1, step]
                    # continuous return: current pos -> cell 0 -> BS, explicit length
                    # reconciliation instead of the old swallowed try/except
                    ret_x1, ret_y1 = intp_between_coords(drone_x_pos, drone_y_pos,
                                                         cell_0_x, cell_0_y,
                                                         info.max_drone_speed)
                    ret_x2, ret_y2 = intp_between_coords(cell_0_x, cell_0_y,
                                                         cell_bs_x, cell_bs_y,
                                                         info.max_drone_speed)
                    ret_x = np.hstack((ret_x1, ret_x2))
                    ret_y = np.hstack((ret_y1, ret_y2))
                    remaining = timesteps - step
                    if len(ret_x) >= remaining:
                        ret_x, ret_y = ret_x[:remaining], ret_y[:remaining]
                    else:
                        pad = remaining - len(ret_x)
                        ret_x = np.hstack((ret_x, np.full(pad, cell_bs_x)))
                        ret_y = np.hstack((ret_y, np.full(pad, cell_bs_y)))
                    realtime_x[m + 1, step:] = ret_x
                    realtime_y[m + 1, step:] = ret_y
                    # Mirror into the DISCRETE path matrix so PathAnimation (which
                    # re-derives trajectories from it via get_real_paths) shows the
                    # early return (AD3). Same recipe as the discrete pipeline.
                    leg = max(discrete_step, 0)
                    current_cell = drone_positions[m] if drone_positions[m] != -1 else 0
                    path_to_0 = interpolate_between_cities(x, current_cell, 0)
                    n_cols = x.real_time_path_matrix.shape[1]
                    padded_path = path_to_0 + [-1] * (n_cols - leg - len(path_to_0))
                    x.real_time_path_matrix[m + 1, leg:] = padded_path[:n_cols - leg]

        # --- mission end: every drone home (defect A2 fix: ndarray, not list) ----
        positions_now = np.array(list(drone_positions.values()))
        if step > 0 and np.sum(positions_now == -1) == number_of_drones:
            timestep_drones_are_back_at_bs = step
            if step < timesteps - 1:
                realtime_x = realtime_x[:, :step + 1]
                realtime_y = realtime_y[:, :step + 1]
                cell_occupancy_probabilities = [col_probs[:step + 1]
                                                for col_probs in cell_occupancy_probabilities]
            break

    # stash the exact realtime trajectory (incl. truncation) on the solution copy
    x.real_time_x_matrix = realtime_x
    x.real_time_y_matrix = realtime_y

    detection_time, inform_time, mission_time, time_at_least_one = _finalize_metrics(
        x.time_elapsed_at_steps, timestep_all_targets_are_known,
        timestep_bs_knows_all_targets, timestep_at_least_one_drone_knows_all_targets,
        timestep_drones_are_back_at_bs)

    return {"cell occupancy probabilities": cell_occupancy_probabilities,
            "search map": search_map,
            "occupancy status": occupancy_status,
            "detection time": detection_time,
            "inform time": inform_time,
            "mission time": mission_time,
            "time at least one drone knows all targets": time_at_least_one}, x
```

This deletes all the dead blocks (old lines 119-121, 130-131, 219-242, 271-278, 310-354) by replacement.

- [ ] **Step 2: Run the full contract suite**

Run: `.venv/bin/python -m pytest tests/test_sensing_contract.py tests/test_discrete_regression.py -q`
Expected: ALL PASS. Debug notes if not:
- `test_realtime_early_return_shortens_mission` failing → check the early-return branch actually fires (drop into `pytest --pdb`, inspect `occupancy_status` after the target cell's first visit with B=0.7: posterior = 0.7·0.5/(0.7·0.5+0.2·0.5) ≈ 0.778 > 0.7 must flag).
- Key-parity failing → discrete dict still built inline somewhere; re-check Task 7 Step 2.

- [ ] **Step 3: Commit**

```bash
git add Sensing.py
git commit -m "feat: complete realtime sensing pipeline (early return, occupancy parity, seam dedup)"
```

---

### Task 9: Pin the mid-flight merging distinction (AD5)

**Files:**
- Test: append to `tests/test_sensing_contract.py`

- [ ] **Step 1: Write the test**

Append to `tests/test_sensing_contract.py`:

```python
def test_realtime_merges_midflight(small_solution):
    """The realtime pipeline must exchange beliefs BETWEEN grid cells.

    Evidence accepted: with onboard merging, some drone's search_map holds an
    observation of the target cell that its own discrete path could not have
    produced before its first own visit — i.e. it was received via a merge.
    Strict mid-flight verification: rerun with merging disabled ('none') and
    assert the receive disappears while per-drone self-observations are equal.
    """
    onboard, x_on = sensing_and_realtime_info_sharing(
        small_solution, merging_strategy="onboard", target_locations=[12], B=0.999, p=0.7, q=0.2)
    none_, x_off = sensing_and_realtime_info_sharing(
        small_solution, merging_strategy="none", target_locations=[12], B=0.999, p=0.7, q=0.2)
    # B=0.999 disables early return so both runs fly identical full paths.
    target = 12
    received_on = sum(len(onboard["search map"][node, target]) for node in range(5))
    received_off = sum(len(none_["search map"][node, target]) for node in range(5))
    assert received_on > received_off  # merging propagated target observations
```

- [ ] **Step 2: Run it**

Run: `.venv/bin/python -m pytest tests/test_sensing_contract.py::test_realtime_merges_midflight -q`
Expected: PASS (comm_cell_range=2 → 100 m range on 50 m cells guarantees contact in the fixture).

- [ ] **Step 3: Commit**

```bash
git add tests/test_sensing_contract.py
git commit -m "test: pin mid-flight merging as the realtime pipeline's distinguishing behavior"
```

---

# Phase 2 — Config & replay API

### Task 10: `SensingConfig`

**Files:**
- Create: `SensingReplay.py` (config part), `tests/test_sensing_config.py`

- [ ] **Step 1: Write the failing tests**

Create `tests/test_sensing_config.py`:

```python
import pytest

from SensingReplay import SensingConfig


def test_defaults_valid():
    cfg = SensingConfig()
    assert cfg.merge_topology == "onboard"
    assert cfg.time_model == "discrete"


@pytest.mark.parametrize("field,value", [
    ("merge_topology", "ondrone"),      # legacy name rejected
    ("merge_topology", "discrete"),     # old collision value rejected
    ("time_model", "continuous"),       # not a valid time model
    ("detection_prob", 0.0),
    ("detection_prob", 1.0),
    ("false_alarm_prob", -0.1),
    ("belief_threshold", 1.5),
])
def test_invalid_values_raise(field, value):
    with pytest.raises(ValueError):
        SensingConfig(**{field: value})


def test_from_info_defaults_and_overrides(small_solution):
    info = small_solution.info
    cfg = SensingConfig.from_info(info)
    assert cfg.detection_prob == info.detection_probability   # 0.7
    assert cfg.belief_threshold == info.th                    # 0.9
    assert cfg.target_locations == list(info.target_locations)
    cfg2 = SensingConfig.from_info(info, merge_topology="gcs", belief_threshold=0.8)
    assert cfg2.merge_topology == "gcs" and cfg2.belief_threshold == 0.8


def test_from_info_tolerates_old_pickled_pathinfo(small_solution):
    class OldInfo:                       # simulates pre-migration PathInfo pickle
        number_of_cells = 64
    cfg = SensingConfig.from_info(OldInfo())
    assert cfg.detection_prob == 0.7 and cfg.belief_threshold == 0.9


def test_from_info_rejects_out_of_grid_target(small_solution):
    with pytest.raises(ValueError):
        SensingConfig.from_info(small_solution.info, target_locations=[999])
```

Run: `.venv/bin/python -m pytest tests/test_sensing_config.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'SensingReplay'`

- [ ] **Step 2: Implement**

Create `SensingReplay.py`:

```python
"""Analysis-layer sensing/merging configuration and replay API (spec section 1-3).

Merging parameters live HERE, never in the scenario dict / PathInfo / filenames
(spec D1). This module is also the future web view-section's parameter schema.
"""
from dataclasses import dataclass, field

VALID_MERGE_TOPOLOGIES = ("none", "onboard", "gcs")
VALID_TIME_MODELS = ("discrete", "realtime")


@dataclass
class SensingConfig:
    merge_topology: str = "onboard"
    time_model: str = "discrete"
    detection_prob: float = 0.7        # p
    false_alarm_prob: float = 0.2      # q
    belief_threshold: float = 0.9      # B
    target_locations: list = field(default_factory=lambda: [12])

    def __post_init__(self):
        if self.merge_topology not in VALID_MERGE_TOPOLOGIES:
            raise ValueError(f"merge_topology {self.merge_topology!r} not in {VALID_MERGE_TOPOLOGIES}")
        if self.time_model not in VALID_TIME_MODELS:
            raise ValueError(f"time_model {self.time_model!r} not in {VALID_TIME_MODELS}")
        for name in ("detection_prob", "false_alarm_prob", "belief_threshold"):
            v = getattr(self, name)
            if not (0.0 < v < 1.0):
                raise ValueError(f"{name} must be in (0, 1), got {v}")
        if not self.target_locations:
            raise ValueError("target_locations must not be empty")

    @classmethod
    def from_info(cls, info, **overrides):
        """Build a config defaulting from PathInfo. The ONLY place old-pickle
        defaulting lives (spec: 'centralized, not scattered getattrs')."""
        defaults = dict(
            detection_prob=getattr(info, "detection_probability", 0.7),
            belief_threshold=getattr(info, "th", 0.9),
            target_locations=list(getattr(info, "target_locations", [12])),
        )
        defaults.update(overrides)
        cfg = cls(**defaults)
        n_cells = getattr(info, "number_of_cells", None)
        if n_cells is not None:
            bad = [t for t in cfg.target_locations if not (0 <= t < n_cells)]
            if bad:
                raise ValueError(f"target_locations {bad} outside grid (0..{n_cells - 1})")
        return cfg
```

- [ ] **Step 3: Run, verify green**

Run: `.venv/bin/python -m pytest tests/test_sensing_config.py -q`
Expected: PASS

- [ ] **Step 4: Commit**

```bash
git add SensingReplay.py tests/test_sensing_config.py
git commit -m "feat: SensingConfig — validated analysis-layer parameter schema"
```

---

### Task 11: Config-based pipeline signatures + Analysis call sites

**Files:**
- Modify: `Sensing.py` (both pipeline signatures), `Analysis.py:1,71,164,368-370`, `tests/test_sensing_contract.py`, `tests/test_discrete_regression.py`

- [ ] **Step 1: Flip the two signatures**

In `Sensing.py`, change

```python
def sensing_and_discrete_info_sharing(sol: PathSolution, merging_strategy="onboard", target_locations=[12], B=0.9, p=0.9, q=0.2):
```

to

```python
def sensing_and_discrete_info_sharing(sol: PathSolution, config):
    merge_topology = config.merge_topology
    target_locations = config.target_locations
    B, p, q = config.belief_threshold, config.detection_prob, config.false_alarm_prob
```

and identically for `sensing_and_realtime_info_sharing`. Inside both bodies, replace the `merge_maps(conn_comp, search_map, merging_strategy)` calls with `merge_maps(conn_comp, search_map, merge_topology)`. No other body changes.

- [ ] **Step 2: Update Analysis.py call sites**

Add to `Analysis.py` imports: `from SensingReplay import SensingConfig`. Replace the three call sites:

`Analysis.py:71` (and `:164`, identical shape):

```python
    cfg = SensingConfig.from_info(sol.info, merge_topology="onboard", time_model="discrete",
                                  target_locations=target_locations,
                                  detection_prob=p, false_alarm_prob=q, belief_threshold=B)
    time_metrics, new_sol = sensing_and_discrete_info_sharing(sol=sol, config=cfg)
```

`Analysis.py:367-372`:

```python
    cfg = SensingConfig.from_info(sol.info, merge_topology="onboard",
                                  time_model=merging_strategy,
                                  target_locations=target_locations,
                                  detection_prob=p, false_alarm_prob=q, belief_threshold=B)
    if cfg.time_model == "discrete":
        time_metrics, updated_sol = sensing_and_discrete_info_sharing(sol=sol, config=cfg)
    else:
        time_metrics, updated_sol = sensing_and_realtime_info_sharing(sol=sol, config=cfg)
```

(The old `else: print("Incorrect merging strategy...")` branch is dead — `SensingConfig` validation now raises instead.)

- [ ] **Step 3: Update the test call shapes (asserts unchanged)**

In `tests/test_sensing_contract.py` replace the KW constants and every call:

```python
from SensingReplay import SensingConfig

CFG = lambda topo, **kw: SensingConfig(merge_topology=topo, target_locations=[12],
                                       belief_threshold=kw.get("B", 0.7),
                                       detection_prob=0.7, false_alarm_prob=0.2)
# calls become e.g.:
#   sensing_and_realtime_info_sharing(small_solution, CFG("onboard"))
#   sensing_and_realtime_info_sharing(small_solution, CFG("none", B=0.999))
```

In `tests/test_discrete_regression.py` the call becomes
`sensing_and_discrete_info_sharing(small_solution, CFG("onboard"))` with the same `CFG` helper (import it or redefine inline). **Snapshot literals unchanged.**

- [ ] **Step 4: Full suite green**

Run: `.venv/bin/python -m pytest tests/ -q`
Expected: ALL PASS, snapshot identical.

- [ ] **Step 5: Commit**

```bash
git add Sensing.py Analysis.py tests/
git commit -m "feat: pipelines take SensingConfig; Analysis call sites migrated"
```

---

### Task 12: `replay()` + `ReplayResult`

**Files:**
- Modify: `SensingReplay.py`
- Create: `tests/test_replay.py`

- [ ] **Step 1: Write the failing tests**

Create `tests/test_replay.py`:

```python
import numpy as np
import pytest

from SensingReplay import SensingConfig, replay, compare


def cfg(topo="onboard", time_model="discrete", B=0.7):
    return SensingConfig(merge_topology=topo, time_model=time_model,
                         target_locations=[12], belief_threshold=B,
                         detection_prob=0.7, false_alarm_prob=0.2)


def test_replay_dispatches_discrete(small_solution):
    r = replay(small_solution, cfg(time_model="discrete"))
    assert np.isfinite(r.effective_mission_time)
    assert r.label == "onboard"


def test_replay_dispatches_realtime(small_solution):
    r = replay(small_solution, cfg(time_model="realtime"))
    assert np.isfinite(r.effective_mission_time)
    assert len(r.cell_occupancy_probabilities) == small_solution.info.number_of_cells


def test_replay_result_carries_metrics_and_solution(small_solution):
    r = replay(small_solution, cfg())
    assert r.detection_time <= r.effective_mission_time
    assert r.solution is not small_solution          # deep copy, original untouched
    assert r.config.merge_topology == "onboard"
```

Run: `.venv/bin/python -m pytest tests/test_replay.py -q`
Expected: FAIL — `ImportError: cannot import name 'replay'`

- [ ] **Step 2: Implement**

Append to `SensingReplay.py`:

```python
from Sensing import sensing_and_discrete_info_sharing, sensing_and_realtime_info_sharing


@dataclass
class ReplayResult:
    config: "SensingConfig"
    label: str
    effective_mission_time: float
    detection_time: float
    inform_time: float
    time_at_least_one_drone_knows_all: float
    cell_occupancy_probabilities: list
    occupancy_status: object        # np.ndarray (nodes x cells)
    search_map: object              # np.ndarray of per-node observation lists
    solution: object                # truncated PathSolution copy (for animation)


def replay(solution, config, label=None):
    """Run one sensing replay. Dispatch is the ONLY thing that looks at
    time_model — the pipelines satisfy one return contract (spec AD1)."""
    if config.time_model == "discrete":
        metrics, x = sensing_and_discrete_info_sharing(solution, config)
    else:
        metrics, x = sensing_and_realtime_info_sharing(solution, config)
    return ReplayResult(
        config=config,
        label=label if label is not None else config.merge_topology,
        effective_mission_time=metrics["mission time"],
        detection_time=metrics["detection time"],
        inform_time=metrics["inform time"],
        time_at_least_one_drone_knows_all=metrics["time at least one drone knows all targets"],
        cell_occupancy_probabilities=metrics["cell occupancy probabilities"],
        occupancy_status=metrics["occupancy status"],
        search_map=metrics["search map"],
        solution=x,
    )
```

- [ ] **Step 3: Run, verify green**

Run: `.venv/bin/python -m pytest tests/test_replay.py -q`
Expected: first 3 tests PASS (`compare` import fails → temporarily comment the `compare` import in the test file, restore it in Task 13; or implement Task 13 immediately after).

- [ ] **Step 4: Commit**

```bash
git add SensingReplay.py tests/test_replay.py
git commit -m "feat: replay() — uniform dispatch over both sensing pipelines"
```

---

### Task 13: `compare()` — metrics table + time-series plots + animations

**Files:**
- Modify: `SensingReplay.py`, `tests/test_replay.py`

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_replay.py`:

```python
def test_compare_table_and_artifacts(small_solution, tmp_path):
    configs = [cfg("none"), cfg("onboard"), cfg("gcs")]
    result = compare(small_solution, configs, output_dir=str(tmp_path),
                     scenario_label="testscn", animations=False)
    assert list(result.table.index) == ["none", "onboard", "gcs"]
    assert "Effective Mission Time" in result.table.columns
    assert "Detection Time" in result.table.columns
    # onboard merging never detects later than none (append-only invariant)
    assert (result.table.loc["onboard", "Detection Time"]
            <= result.table.loc["none", "Detection Time"])
    assert (tmp_path / "testscn-comparison.csv").exists()
    assert len(result.plot_paths) == 2          # targets-over-time + belief evolution
    for p in result.plot_paths:
        import os; assert os.path.exists(p)


def test_compare_duplicate_topology_labels_deduped(small_solution, tmp_path):
    configs = [cfg("onboard", B=0.7), cfg("onboard", B=0.85)]
    result = compare(small_solution, configs, output_dir=str(tmp_path),
                     scenario_label="dup", animations=False)
    assert list(result.table.index) == ["onboard-0", "onboard-1"]


@pytest.mark.slow
def test_compare_animations_render(small_solution, tmp_path):
    result = compare(small_solution, [cfg("onboard")], output_dir=str(tmp_path),
                     scenario_label="anim", animations=True)
    import os
    assert len(result.animation_paths) == 1
    assert os.path.exists(result.animation_paths[0])
    assert result.animation_paths[0].endswith(".gif")
```

Run: `.venv/bin/python -m pytest tests/test_replay.py -q -m "not slow"`
Expected: FAIL — `compare` not defined.

- [ ] **Step 2: Implement compare()**

Append to `SensingReplay.py`:

```python
import os
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt

METRIC_COLUMNS = {
    "Effective Mission Time": "effective_mission_time",
    "Detection Time": "detection_time",
    "Inform Time": "inform_time",
    "Time At Least One Drone Knows All Targets": "time_at_least_one_drone_knows_all",
}

DEFAULT_PLOT_DIR = "Figures/Sensing/Merging Comparisons"
DEFAULT_ANIM_DIR = "Results/Animations"


@dataclass
class ComparisonResult:
    table: pd.DataFrame
    csv_path: str
    plot_paths: list
    animation_paths: list
    replays: list


def _dedupe_labels(configs, labels):
    if labels is None:
        labels = [c.merge_topology for c in configs]
    if len(set(labels)) != len(labels):
        labels = [f"{lab}-{i}" for i, lab in enumerate(labels)]
    return labels


def _targets_known_curve(r):
    """targets-known count per step, derived from per-cell max-prob history."""
    cfg, probs = r.config, r.cell_occupancy_probabilities
    n_steps = len(probs[0])
    curve = []
    for step in range(n_steps):
        known = sum(1 for t in cfg.target_locations
                    if any(p > cfg.belief_threshold for p in probs[t][:step + 1]))
        curve.append(known)
    return curve


def compare(solution, configs, labels=None, scenario_label="scenario",
            output_dir=None, plot_dir=None, anim_dir=None, animations=True):
    """Replay one solution under each config; emit table + plots (+ animations).

    output_dir, when given, overrides plot_dir/anim_dir/csv location (used by
    tests); otherwise artifacts land in the standard Figures/Results trees.
    """
    labels = _dedupe_labels(configs, labels)
    replays = [replay(solution, c, label=l) for c, l in zip(configs, labels)]

    plot_dir = output_dir or plot_dir or DEFAULT_PLOT_DIR
    anim_dir = output_dir or anim_dir or DEFAULT_ANIM_DIR
    csv_dir = output_dir or plot_dir
    for d in (plot_dir, anim_dir, csv_dir):
        os.makedirs(d, exist_ok=True)

    # 1) metrics table -------------------------------------------------------
    table = pd.DataFrame(
        {col: [getattr(r, attr) for r in replays] for col, attr in METRIC_COLUMNS.items()},
        index=labels)
    csv_path = os.path.join(csv_dir, f"{scenario_label}-comparison.csv")
    table.to_csv(csv_path)

    # 2) time-series plots ---------------------------------------------------
    plot_paths = []
    fig, ax = plt.subplots(figsize=(8, 5))
    for r in replays:
        curve = _targets_known_curve(r)
        ax.plot(range(len(curve)), curve, label=r.label)
    ax.set_xlabel("step"); ax.set_ylabel("targets known")
    ax.set_title(f"Targets known over time — {scenario_label}")
    ax.legend()
    p1 = os.path.join(plot_dir, f"{scenario_label}-targets-over-time.png")
    fig.savefig(p1, dpi=150, bbox_inches="tight"); plt.close(fig)
    plot_paths.append(p1)

    fig, ax = plt.subplots(figsize=(8, 5))
    first_target = replays[0].config.target_locations[0]
    for r in replays:
        probs = r.cell_occupancy_probabilities[first_target]
        ax.plot(range(len(probs)), probs, label=r.label)
    ax.axhline(replays[0].config.belief_threshold, linestyle="--", color="grey",
               label=f"B = {replays[0].config.belief_threshold}")
    ax.set_xlabel("step"); ax.set_ylabel(f"max belief, cell {first_target}")
    ax.set_title(f"Belief evolution — {scenario_label}")
    ax.legend()
    p2 = os.path.join(plot_dir, f"{scenario_label}-belief-evolution.png")
    fig.savefig(p2, dpi=150, bbox_inches="tight"); plt.close(fig)
    plot_paths.append(p2)

    # 3) animations ----------------------------------------------------------
    animation_paths = []
    if animations:
        from matplotlib.animation import FuncAnimation, PillowWriter
        from PathAnimation import PathAnimation
        for r in replays:
            fig, ax = plt.subplots(figsize=(6, 6))
            anim_obj = PathAnimation(
                r.solution, fig, ax,
                target_locations=r.config.target_locations,
                cell_occupancy_probabilities=r.cell_occupancy_probabilities,
                B=r.config.belief_threshold)
            frames = anim_obj.paths[0].shape[1]
            anim = FuncAnimation(fig, anim_obj.update, frames=frames,
                                 init_func=anim_obj.initialize_figure,
                                 blit=False, interval=0)
            path = os.path.join(anim_dir, f"{scenario_label}-{r.label}-replay.gif")
            anim.save(path, writer=PillowWriter(fps=10))
            plt.close(fig)
            animation_paths.append(path)

    return ComparisonResult(table=table, csv_path=csv_path, plot_paths=plot_paths,
                            animation_paths=animation_paths, replays=replays)
```

Also register the slow marker — create `pytest.ini`:

```ini
[pytest]
markers =
    slow: long-running tests (animations, end-to-end)
```

- [ ] **Step 3: Run fast tests, then the slow animation test once**

Run: `.venv/bin/python -m pytest tests/test_replay.py -q -m "not slow"`
Expected: PASS
Run: `.venv/bin/python -m pytest tests/test_replay.py -q -m slow`
Expected: PASS (GIF rendered). If `PathAnimation.update` raises an index error on occupancy columns, the fix belongs in Task 8's occupancy/truncation bookkeeping (lengths must satisfy `len(probs[cell]) >= number of executed steps`) — not in `PathAnimation`.

- [ ] **Step 4: Commit**

```bash
git add SensingReplay.py tests/test_replay.py pytest.ini
git commit -m "feat: compare() — metrics table, time-series plots, replay animations"
```

---

# Phase 3 — Selection & registry

### Task 14: Model registry

**Files:**
- Modify: `PathOptimizationModel.py` (append at end), `PathInput.py`
- Create: `tests/test_model_registry.py`

- [ ] **Step 1: Write the failing tests**

Create `tests/test_model_registry.py`:

```python
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
```

Run: `.venv/bin/python -m pytest tests/test_model_registry.py -q`
Expected: FAIL — `ImportError`

- [ ] **Step 2: Implement**

Append to `PathOptimizationModel.py`:

```python
# --- Model registry (spec section 5): the ready-made, tried-and-tested models.
AVAILABLE_MODELS = {
    "MTSP": MTSP, "CONN": CONN,
    "TC_WS": TC_WS, "TC_MOO_NSGA2": TC_MOO_NSGA2, "TC_MOO_NSGA3": TC_MOO_NSGA3,
    "TT_WS": TT_WS, "TT_MOO_NSGA2": TT_MOO_NSGA2, "TT_MOO_NSGA3": TT_MOO_NSGA3,
    "TCT_WS": TCT_WS, "TCT_MOO_NSGA2": TCT_MOO_NSGA2, "TCT_MOO_NSGA3": TCT_MOO_NSGA3,
    "TCDT_WS": TCDT_WS, "TCDT_MOO_NSGA2": TCDT_MOO_NSGA2, "TCDT_MOO_NSGA3": TCDT_MOO_NSGA3,
    "TCD_WS": TCD_WS, "TCD_MOO_NSGA2": TCD_MOO_NSGA2, "TCD_MOO_NSGA3": TCD_MOO_NSGA3,
    "CD_WS": CD_WS, "CD_MOO_NSGA2": CD_MOO_NSGA2, "CD_MOO_NSGA3": CD_MOO_NSGA3,
}


def list_models():
    """Dropdown-ready model metadata (one row per ready-made model)."""
    rows = [{"name": name, "type": m["Type"], "algorithm": m["Alg"],
             "objectives": list(m["F"]), "constraints": list(m["G"]) + list(m["H"])}
            for name, m in AVAILABLE_MODELS.items()]
    return pd.DataFrame(rows)
```

Replace `PathInput.py` content:

```python
from PathOptimizationModel import *

# Select the optimization model by registry name (see list_models()).
model_name = "TCDT_MOO_NSGA2"
model = AVAILABLE_MODELS[model_name]
pop_size = 250
n_gen = 800
```

(Note: the spec said "17 ready-made models"; the actual count in `PathOptimizationModel.py` is 20 — the registry is the source of truth.)

- [ ] **Step 3: Run, verify green, plus import smoke**

Run: `.venv/bin/python -m pytest tests/test_model_registry.py -q && .venv/bin/python -c "import PathUnitTest; print('imports OK')"`
Expected: PASS + `imports OK`

- [ ] **Step 4: Commit**

```bash
git add PathOptimizationModel.py PathInput.py tests/test_model_registry.py
git commit -m "feat: AVAILABLE_MODELS registry + list_models(); PathInput resolves by name"
```

---

### Task 15: `SolutionSelector` — kind, capabilities, `the_solution()`, `by_index()`

**Files:**
- Create: `SolutionSelection.py`, `tests/test_solution_selection.py`

- [ ] **Step 1: Write the failing tests**

Create `tests/test_solution_selection.py`:

```python
import numpy as np
import pandas as pd
import pytest

from PathOptimizationModel import TC_MOO_NSGA2, TC_WS, MTSP
from SolutionSelection import SolutionSelector, StrategyUnavailableError


def front_selector(n=5):
    """5-point 2-objective front; F is SIGNED as stored on disk
    (Mission Time positive-minimize, Percentage Connectivity negative-minimize)."""
    F = pd.DataFrame({
        "Mission Time":            [100.0, 200.0, 300.0, 400.0, 500.0],
        "Percentage Connectivity": [-0.10, -0.30, -0.50, -0.70, -0.90],
    })
    solutions = [f"sol{i}" for i in range(n)]      # selector never inspects solutions
    return SolutionSelector(F, solutions, TC_MOO_NSGA2)


def single_selector(model):
    F = pd.DataFrame({"Mission Time": [123.0]}) if model is MTSP \
        else pd.DataFrame({"Mission Time & Percentage Connectivity Weighted Sum": [0.5]})
    return SolutionSelector(F, ["only"], model)


def test_result_kind():
    assert front_selector().result_kind == "front"
    assert single_selector(MTSP).result_kind == "single"
    assert single_selector(TC_WS).result_kind == "single"


def test_degenerate_moo_front_downgrades_with_warning():
    F = pd.DataFrame({"Mission Time": [100.0], "Percentage Connectivity": [-0.5]})
    with pytest.warns(UserWarning):
        sel = SolutionSelector(F, ["only"], TC_MOO_NSGA2)
    assert sel.result_kind == "single"


def test_capabilities_front():
    caps = front_selector().capabilities()
    assert caps["best"] == ["Mission Time", "Percentage Connectivity"]
    assert caps["balanced"] and caps["knee"] and caps["by_weights"]
    assert caps["by_index"] == 4 and caps["the_solution"] is False


def test_capabilities_single():
    caps = single_selector(MTSP).capabilities()
    assert caps["best"] == [] and not caps["balanced"] and not caps["knee"]
    assert not caps["by_weights"] and caps["the_solution"] is True


def test_the_solution_only_for_single():
    idx, sol, label = single_selector(MTSP).the_solution()
    assert idx == 0 and sol == "only"
    with pytest.raises(StrategyUnavailableError):
        front_selector().the_solution()


def test_by_index_bounds():
    sel = front_selector()
    idx, sol, label = sel.by_index(3)
    assert idx == 3 and sol == "sol3"
    with pytest.raises(StrategyUnavailableError):
        sel.by_index(99)


def test_ws_error_message_teaches():
    with pytest.raises(StrategyUnavailableError, match="MOO variant"):
        single_selector(TC_WS).by_weights({"Mission Time": 1.0})
```

Run: `.venv/bin/python -m pytest tests/test_solution_selection.py -q`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 2: Implement the core**

Create `SolutionSelection.py`:

```python
"""Model-aware Pareto-front navigation (spec section 4).

The selector is self-describing via capabilities(); UIs must consult it instead
of hardcoding which strategies exist for which model type.
"""
import warnings

import numpy as np
import pandas as pd

from FilePaths import objective_values_filepath, solutions_filepath
from PathFileManagement import load_pickle


class StrategyUnavailableError(Exception):
    """Raised when a selection strategy does not exist for this result shape."""


class SolutionSelector:

    def __init__(self, F: pd.DataFrame, solutions, model: dict):
        self.F = F
        # SolutionObjects.pkl rows can be 1-element arrays (PathUnitTest.py:104-110)
        self.solutions = [s[0] if isinstance(s, np.ndarray) else s for s in solutions]
        self.model = model
        if model["Type"] == "MOO" and len(self.solutions) > 1:
            self.result_kind = "front"
        else:
            self.result_kind = "single"
            if model["Type"] == "MOO":
                warnings.warn(
                    "MOO front collapsed to a single non-dominated solution — "
                    "treating as 'single'. This is a convergence signal worth checking.")

    @classmethod
    def from_scenario(cls, scenario: str, model: dict):
        F = pd.read_pickle(f"{objective_values_filepath}{scenario}-ObjectiveValues.pkl")
        solutions = load_pickle(f"{solutions_filepath}{scenario}-SolutionObjects.pkl")
        return cls(F, list(solutions), model)

    # --- capability discovery ------------------------------------------------
    def capabilities(self):
        if self.result_kind == "single":
            return {"best": [], "balanced": False, "knee": False, "by_weights": False,
                    "by_index": len(self.solutions) - 1, "the_solution": True}
        return {"best": list(self.model["F"]),
                "balanced": True,
                "knee": self.F.shape[1] >= 2 and len(self.solutions) >= 3,
                "by_weights": True,
                "by_index": len(self.solutions) - 1,
                "the_solution": False}

    def _require_front(self, name):
        if self.result_kind == "front":
            return
        if self.model["Type"] == "WS":
            raise StrategyUnavailableError(
                f"'{name}' unavailable: WS models bake objective weights in before the "
                f"run, producing a single solution. Re-run with different WS weights, or "
                f"use the MOO variant to explore trade-offs interactively. "
                f"Use the_solution() for this result.")
        raise StrategyUnavailableError(
            f"'{name}' unavailable for single-solution results; use the_solution().")

    # --- strategies -----------------------------------------------------------
    def the_solution(self):
        if self.result_kind != "single":
            raise StrategyUnavailableError(
                "the_solution() is for single-solution results; this is a Pareto front — "
                "use best()/balanced()/knee()/by_weights()/by_index().")
        return 0, self.solutions[0], "The solution"

    def by_index(self, i):
        if not (0 <= i < len(self.solutions)):
            raise StrategyUnavailableError(
                f"index {i} out of range (0..{len(self.solutions) - 1})")
        return i, self.solutions[i], f"Solution #{i}"
```

- [ ] **Step 3: Run — the core tests pass, strategy tests still red**

Run: `.venv/bin/python -m pytest tests/test_solution_selection.py -q`
Expected: kind/capabilities/the_solution/by_index/WS-message tests PASS (the WS message test passes because `by_weights` doesn't exist yet → add a stub raising via `_require_front`):

```python
    def by_weights(self, weights):
        self._require_front("by_weights")
        raise NotImplementedError  # completed in Task 17
```

- [ ] **Step 4: Commit**

```bash
git add SolutionSelection.py tests/test_solution_selection.py
git commit -m "feat: model-aware SolutionSelector core (kind, capabilities, the_solution, by_index)"
```

---

### Task 16: `best()` + `balanced()`

**Files:**
- Modify: `SolutionSelection.py`, `tests/test_solution_selection.py`

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_solution_selection.py`:

```python
def test_best_polarity_aware():
    sel = front_selector()
    idx, sol, label = sel.best("Mission Time")
    assert idx == 0                       # 100 s is fastest
    idx, sol, label = sel.best("Percentage Connectivity")
    assert idx == 4                       # -0.90 signed == 90% connectivity, the most


def test_best_rejects_non_model_objective():
    with pytest.raises(StrategyUnavailableError, match="Max Mean TBV"):
        front_selector().best("Max Mean TBV")   # TC never optimized TBV


def test_best_unavailable_for_single():
    with pytest.raises(StrategyUnavailableError):
        single_selector(MTSP).best("Mission Time")


def test_balanced_centroid_nearest():
    # normalized F: MT = [0,.25,.5,.75,1], PC = [1,.75,.5,.25,0] (after min-max);
    # centroid = (.5,.5) -> nearest is index 2 exactly.
    idx, sol, label = front_selector().balanced()
    assert idx == 2
```

Run: `.venv/bin/python -m pytest tests/test_solution_selection.py -q`
Expected: new tests FAIL — methods missing.

- [ ] **Step 2: Implement**

Append to `SolutionSelector`:

```python
    def _normalized_F(self):
        F_norm = (self.F - self.F.min(axis=0)) / (self.F.max(axis=0) - self.F.min(axis=0))
        return F_norm.fillna(0.5)    # zero-range column -> all solutions equal

    def best(self, objective_name):
        self._require_front("best")
        if objective_name not in self.model["F"]:
            raise StrategyUnavailableError(
                f"{objective_name!r} was not optimized by this model "
                f"(valid: {list(self.model['F'])}). Best-of-a-non-optimized metric "
                f"would be a sampling accident, not an answer.")
        # ObjectiveValues stores SIGNED values (polarity already applied by
        # PathProblem), so min is best for every objective.
        idx = int(self.F[objective_name].idxmin())
        return idx, self.solutions[idx], f"Best {objective_name}"

    def balanced(self):
        self._require_front("balanced")
        F_norm = self._normalized_F()
        centroid = F_norm.mean(axis=0)
        dists = np.linalg.norm(F_norm.values - centroid.values, axis=1)
        idx = int(np.argmin(dists))
        return idx, self.solutions[idx], "Balanced"
```

(`balanced()` reimplements `get_median_index_of_scenario`'s normalize-centroid-argmin formula (PathOptimizationModel.py:43-56) on the in-memory F instead of re-reading pickles from disk; the `fillna(0.5)` guard additionally protects degenerate zero-range columns.)

- [ ] **Step 3: Run, verify green**

Run: `.venv/bin/python -m pytest tests/test_solution_selection.py -q`
Expected: PASS

- [ ] **Step 4: Commit**

```bash
git add SolutionSelection.py tests/test_solution_selection.py
git commit -m "feat: best() and balanced() selection strategies (signed-F aware)"
```

---

### Task 17: `knee()` + `by_weights()`

**Files:**
- Modify: `SolutionSelection.py`, `tests/test_solution_selection.py`

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_solution_selection.py`:

```python
def test_by_weights_one_hot_equals_best():
    sel = front_selector()
    idx_w, _, _ = sel.by_weights({"Mission Time": 1.0})
    idx_b, _, _ = sel.best("Mission Time")
    assert idx_w == idx_b


def test_by_weights_validates_keys_and_zeroes():
    sel = front_selector()
    with pytest.raises(StrategyUnavailableError):
        sel.by_weights({"Nonexistent Objective": 1.0})
    with pytest.raises(StrategyUnavailableError):
        sel.by_weights({"Mission Time": 0.0})


def test_knee_on_kneed_front():
    # Convex front with a pronounced knee at index 1.
    F = pd.DataFrame({"Mission Time": [100.0, 120.0, 300.0, 500.0],
                      "Percentage Connectivity": [-0.20, -0.80, -0.85, -0.90]})
    sel = SolutionSelector(F, list("abcd"), TC_MOO_NSGA2)
    idx, sol, label = sel.knee()
    assert idx == 1


def test_knee_falls_back_to_balanced_with_warning():
    # Perfectly linear front: HighTradeoffPoints().do returns None (verified
    # against pymoo 0.6.1.6 in this venv).
    F = pd.DataFrame({"Mission Time": [100.0, 200.0, 300.0],
                      "Percentage Connectivity": [-0.9, -0.5, -0.1]})
    sel = SolutionSelector(F, list("abc"), TC_MOO_NSGA2)
    with pytest.warns(UserWarning, match="balanced"):
        idx, sol, label = sel.knee()
    assert label == "Balanced (knee fallback)"
```

Run: `.venv/bin/python -m pytest tests/test_solution_selection.py -q`
Expected: new tests FAIL.

- [ ] **Step 2: Implement** (replace the Task 15 `by_weights` stub)

```python
    def by_weights(self, weights: dict):
        self._require_front("by_weights")
        from pymoo.mcdm.pseudo_weights import PseudoWeights
        unknown = set(weights) - set(self.model["F"])
        if unknown:
            raise StrategyUnavailableError(
                f"Unknown objective(s) {sorted(unknown)}; valid: {list(self.model['F'])}")
        w = np.array([float(weights.get(name, 0.0)) for name in self.model["F"]])
        if w.sum() <= 0:
            raise StrategyUnavailableError("at least one weight must be positive")
        w = w / w.sum()
        idx = int(PseudoWeights(w).do(self.F.values))
        pretty = {name: round(float(wi), 3) for name, wi in zip(self.model["F"], w)}
        return idx, self.solutions[idx], f"Weights {pretty}"

    def knee(self):
        self._require_front("knee")
        if self.F.shape[1] < 2 or len(self.solutions) < 3:
            raise StrategyUnavailableError(
                "knee() needs >= 2 objectives and >= 3 solutions on the front")
        from pymoo.mcdm.high_tradeoff import HighTradeoffPoints
        try:
            idxs = HighTradeoffPoints().do(self.F.values)
        except Exception:
            idxs = None    # numerically degenerate fronts: treat as no knee
        if idxs is None or len(np.atleast_1d(idxs)) == 0:
            warnings.warn("No high-tradeoff point found; falling back to balanced().")
            idx, sol, _ = self.balanced()
            return idx, sol, "Balanced (knee fallback)"
        idxs = np.atleast_1d(idxs)
        if len(idxs) > 1:    # spec: nearest-to-centroid among knee candidates
            F_norm = self._normalized_F()
            centroid = F_norm.mean(axis=0)
            d = np.linalg.norm(F_norm.values[idxs] - centroid.values, axis=1)
            idx = int(idxs[int(np.argmin(d))])
        else:
            idx = int(idxs[0])
        return idx, self.solutions[idx], "Knee"
```

- [ ] **Step 3: Run, verify green**

Run: `.venv/bin/python -m pytest tests/test_solution_selection.py -q`
Expected: PASS. If `test_knee_on_kneed_front` picks a different index, print `HighTradeoffPoints().do(F.values)` for that front and adjust the FIXTURE front (not the implementation) until it has one unambiguous knee — the assertion's purpose is "knee() returns a HighTradeoffPoints pick", not a specific geometry.

- [ ] **Step 4: Commit**

```bash
git add SolutionSelection.py tests/test_solution_selection.py
git commit -m "feat: knee() with balanced() fallback and by_weights() pseudo-weight selection"
```

---

### Task 18: End-to-end integration + docs + final gate

**Files:**
- Create: `tests/test_integration.py`
- Modify: `README.md`

- [ ] **Step 1: Write the integration test** (hand-built mini result set — deterministic and fast; a real NSGA-II run stays a manual check because hyperparameters are module-level globals)

Create `tests/test_integration.py`:

```python
"""End-to-end: result set -> SolutionSelector -> SensingConfig -> compare()."""
import numpy as np
import pandas as pd
import pytest

from PathInfo import PathInfo
from PathSolution import PathSolution
from PathOptimizationModel import TC_MOO_NSGA2
from SolutionSelection import SolutionSelector
from SensingReplay import SensingConfig, compare
from conftest import make_scenario


@pytest.fixture(scope="module")
def mini_result_set():
    """Three hand-built solutions standing in for a small Pareto front."""
    info = PathInfo(make_scenario([12]))
    paths = [np.arange(64), np.roll(np.arange(64), 7), np.roll(np.arange(64), 31)]
    sols = [PathSolution(p, np.array([0, 16, 32, 48]), info,
                         calculate_pathplan=True, calculate_connectivity=True)
            for p in paths]
    F = pd.DataFrame({
        "Mission Time": [s.mission_time for s in sols],
        "Percentage Connectivity": [-abs(s.percentage_connectivity or 0.5) for s in sols],
    })
    return F, sols


def test_select_then_compare(mini_result_set, tmp_path):
    F, sols = mini_result_set
    selector = SolutionSelector(F, sols, TC_MOO_NSGA2)
    assert selector.result_kind == "front"
    idx, solution, label = selector.best("Mission Time")

    configs = [SensingConfig.from_info(solution.info, merge_topology=t,
                                       belief_threshold=0.7)
               for t in ("none", "onboard", "gcs")]
    result = compare(solution, configs, output_dir=str(tmp_path),
                     scenario_label="integration", animations=False)

    assert (result.table.loc["onboard", "Detection Time"]
            <= result.table.loc["none", "Detection Time"])
    assert np.isfinite(result.table.loc["onboard", "Effective Mission Time"])
    assert len(result.plot_paths) == 2


def test_realtime_compare_end_to_end(mini_result_set, tmp_path):
    F, sols = mini_result_set
    solution = SolutionSelector(F, sols, TC_MOO_NSGA2).balanced()[1]
    configs = [SensingConfig.from_info(solution.info, merge_topology=t,
                                       time_model="realtime", belief_threshold=0.7)
               for t in ("none", "onboard")]
    result = compare(solution, configs, output_dir=str(tmp_path),
                     scenario_label="rt-integration", animations=False)
    assert np.isfinite(result.table.loc["onboard", "Effective Mission Time"])
```

- [ ] **Step 2: Run the FULL suite including slow**

Run: `.venv/bin/python -m pytest tests/ -q`
Expected: ALL PASS.

- [ ] **Step 3: Update README**

In `README.md`, replace the `## Implemented` section body with:

```markdown
## Implemented

- Information merging between connected drones (`Sensing.py`): Bayesian belief
  updates fused via `merge_maps()` per connectivity clique. Merge topologies:
  `none`, `onboard` (drone-to-drone), `gcs` (base-station-relayed). Time
  models: `discrete` (per cell-step) and `realtime` (continuous positions,
  per-second connectivity, mid-flight merging).
- Analysis layer (`SensingReplay.py`): `SensingConfig` parameter schema,
  `replay()` for a single run, `compare()` for side-by-side merging-strategy
  comparisons (metrics CSV, time-series plots, GIF replay animations).
- Pareto-front navigation (`SolutionSelection.py`): model-aware
  `SolutionSelector` with `best()/balanced()/knee()/by_weights()/by_index()`
  gated by `capabilities()`; single-solution models use `the_solution()`.
- Model registry (`PathOptimizationModel.AVAILABLE_MODELS`, `list_models()`).

Run tests: `python -m pytest tests/ -q`
```

- [ ] **Step 4: Manual sanity check (not automated): one real optimizer run**

Optionally edit `PathInput.py` to `pop_size = 50`, `n_gen = 20` and `main.py`'s scenario loop, run `.venv/bin/python main.py`, then on the produced scenario pickles run `SolutionSelector.from_scenario(...)` + `compare(...)`. Revert hyperparameters afterward. This validates the disk path (`from_scenario`) against real optimizer output.

- [ ] **Step 5: Final commit**

```bash
git add tests/test_integration.py README.md
git commit -m "test: end-to-end select-then-compare integration; document new analysis API"
```

---

## Self-review notes (resolved during planning)

- **Spec coverage:** D1/D3/D4/D5 → Tasks 5,10,11; AD1 contract → Tasks 2,8,12; AD2 → Task 8 (seam dedup + n_obs); AD3 → Task 8 + Task 13 animation test; AD4 → Tasks 4,7; AD5 → Task 9; AD6 honored (no realtime≤discrete assertion anywhere); A1-A6 defects → Tasks 3,5,6,8; selector capability matrix incl. WS teaching error → Tasks 15-17; registry/D6 → Task 14; three compare() artifacts → Task 13; spec's "17 models" corrected to the actual 20 (noted in Task 14).
- **Known judgment calls:** integration test uses a hand-built mini front instead of monkeypatching the module-level `model/pop_size/n_gen` globals across 3 namespaces (fragile); a manual real-run check is Task 18 Step 4. `balanced()` is formula-equivalent to `get_median_index_of_scenario` but operates in memory (that function asserts on disk files; equivalence is by construction, tested via the exact-centroid fixture in Task 16).
- **Type consistency check:** `SensingConfig` field names (`merge_topology`, `time_model`, `detection_prob`, `false_alarm_prob`, `belief_threshold`, `target_locations`) used identically in Tasks 10-13 and 18; selector returns `(index, solution, label)` tuples everywhere; `ReplayResult.cell_occupancy_probabilities` consumed by `_targets_known_curve` and `PathAnimation` with the same list-of-lists-per-cell shape produced by `_compute_occupancy_status` appends.
