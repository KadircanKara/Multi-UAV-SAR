# Multi-UAV-SAR

Multi-objective path optimization for multi-UAV Search-and-Rescue swarms
(pymoo: NSGA-II/III, MOEA/D, GA, PSO). Migrated from SAR_DETECTION_REALTIME_MERGING.

## Install

    python -m venv .venv && source .venv/bin/activate
    pip install -r requirements.txt

## Run

    python main.py

Results are written to `Results/` and figures to `Figures/` (relative paths, see `FilePaths.py`).

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

## Roadmap

- More realistic sensing model (current one is probabilistic: detection
  probability `p`, false-alarm rate `q`, confirmation threshold `B`)
- Reward-based dynamic path planning
- AWS-deployed interactive, explainable web tool: users set up UAV swarm
  scenarios, watch the multi-objective optimizer run live, explore Pareto
  fronts, and tweak constraints.
