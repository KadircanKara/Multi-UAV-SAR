"""Manual validation: real NSGA-II run -> on-disk pickles -> SolutionSelector.from_scenario -> compare().

Deferred check from the 2026-06-07 round (plan Task 18 Step 4). Run with the
TEMP PathInput hyperparameters (TC_MOO_NSGA2, pop 50, gen 20), then revert.
"""
import os
import time

import numpy as np

from PathInfo import PathInfo
from PathInput import model
from PathUnitTest import PathUnitTest
from SolutionSelection import SolutionSelector
from SensingReplay import SensingConfig, compare
from FilePaths import solutions_filepath, objective_values_filepath, runtimes_filepath

scenario = {
    'grid_size': 8,
    'cell_side_length': 50,
    'number_of_drones': 4,
    'max_drone_speed': 2.5,
    'comm_cell_range': 2,
    'n_visits': 2,                 # >1 skips the n_tour generation block
    'target_positions': [12],
    'th': 0.9,
    'detection_probability': 0.7,
}

print("=" * 72)
print("STEP 1: real NSGA-II run via PathUnitTest")
print("=" * 72)
info = PathInfo(scenario)
scenario_str = str(info)
print(f"scenario string: {scenario_str}")
t0 = time.time()
test = PathUnitTest(scenario)
test(save_results=True, animation=False, copy_to_drive=False)
print(f"optimization wall time: {time.time() - t0:.1f}s")

print("=" * 72)
print("STEP 2: verify the pickles landed where from_scenario expects them")
print("=" * 72)
expected = [
    f"{solutions_filepath}{scenario_str}-SolutionObjects.pkl",
    f"{objective_values_filepath}{scenario_str}-ObjectiveValues.pkl",
    f"{objective_values_filepath}{scenario_str}-ObjectiveValuesAbs.pkl",
    f"{runtimes_filepath}{scenario_str}-Runtime.pkl",
]
for p in expected:
    status = "OK " if os.path.isfile(p) else "MISSING"
    print(f"  [{status}] {p}")
missing = [p for p in expected if not os.path.isfile(p)]
if missing:
    raise SystemExit(f"FAIL: missing pickles: {missing}")

print("=" * 72)
print("STEP 3: SolutionSelector.from_scenario on the real pickles")
print("=" * 72)
selector = SolutionSelector.from_scenario(scenario_str, model)
print(f"result_kind: {selector.result_kind}")
print(f"front size:  {len(selector.solutions)}")
print(f"F shape:     {selector.F.shape}, columns: {list(selector.F.columns)}")
print(f"F index:     {type(selector.F.index).__name__}")
caps = selector.capabilities()
print(f"capabilities: {caps}")

print("-" * 72)
results = {}
for name, call in [
    ("best(Mission Time)", lambda: selector.best("Mission Time")),
    ("best(Percentage Connectivity)", lambda: selector.best("Percentage Connectivity")),
    ("balanced()", lambda: selector.balanced()),
    ("knee()", lambda: selector.knee()),
    ("by_weights(70/30)", lambda: selector.by_weights(
        {"Mission Time": 0.7, "Percentage Connectivity": 0.3})),
    ("by_index(0)", lambda: selector.by_index(0)),
]:
    idx, sol, label = call()
    mt = selector.F["Mission Time"].iloc[idx]
    pc = selector.F["Percentage Connectivity"].iloc[idx]
    results[name] = idx
    print(f"  {name:34s} -> idx {idx:3d}  label={label!r:30s} "
          f"MT={mt:8.1f}  PC={pc:+.3f}")
    # sanity: returned solution object is a PathSolution, not an ndarray wrapper
    assert type(sol).__name__ == "PathSolution", f"unexpected solution type: {type(sol)}"

# cross-checks
assert results["best(Mission Time)"] == int(selector.F["Mission Time"].idxmin())
assert results["best(Percentage Connectivity)"] == int(
    selector.F["Percentage Connectivity"].idxmin())

print("=" * 72)
print("STEP 4: compare() on the best-Mission-Time solution, default output dirs")
print("=" * 72)
_, best_sol, _ = selector.best("Mission Time")
configs = [SensingConfig.from_info(best_sol.info, merge_topology=t, belief_threshold=0.7)
           for t in ("none", "onboard", "gcs")]
cmp_result = compare(best_sol, configs, scenario_label=scenario_str, animations=False)
print(cmp_result.table.to_string())
print(f"\ncsv:   {cmp_result.csv_path}  exists={os.path.isfile(cmp_result.csv_path)}")
for p in cmp_result.plot_paths:
    print(f"plot:  {p}  exists={os.path.isfile(p)}")

onb = cmp_result.table.loc["onboard", "Detection Time"]
non = cmp_result.table.loc["none", "Detection Time"]
assert onb <= non, f"merging-helps invariant violated on real data: {onb} > {non}"
assert np.isfinite(cmp_result.table["Effective Mission Time"]).all(), \
    "non-finite Effective Mission Time on real data"

print("\nALL VALIDATION CHECKS PASSED")
