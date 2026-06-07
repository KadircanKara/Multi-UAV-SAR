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

- Information merging / sharing between connected drones (`Sensing.py`):
  Bayesian belief updates fused via `merge_maps()` per connectivity clique,
  with `"ondrone"` (drone-to-drone) and `"gcs"` (base-station-relayed)
  strategies, in both discrete and real-time variants.

## Roadmap

- More realistic sensing model (current one is probabilistic: detection
  probability `p`, false-alarm rate `q`, confirmation threshold `B`)
- Reward-based dynamic path planning
- AWS-deployed interactive, explainable web tool: users set up UAV swarm
  scenarios, watch the multi-objective optimizer run live, explore Pareto
  fronts, and tweak constraints.
