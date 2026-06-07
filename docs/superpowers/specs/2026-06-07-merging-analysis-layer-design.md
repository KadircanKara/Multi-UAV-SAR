# Merging & Visual-Analysis Layer — Design

**Date:** 2026-06-07
**Status:** Approved (pending spec review)
**Scope:** Script-level parameter layer and analysis API. The AWS web tool consumes
this later; it is not built in this round.

## Goal

Let a user take a converged optimization result, pick a solution from the Pareto
front by intent, and compare merging strategies (`none` / `onboard` / `gcs`) on
that solution — producing a metrics table, time-series plots, and mission-replay
animations that show the value of information merging (e.g., shorter Effective
Mission Time because shared beliefs let drones return to base earlier).

## Verified facts this design rests on

1. **The optimizer never sees merging.** Every objective/constraint resolves
   through `model_metric_info` (`PathFuncDict.py:7-20`) to plain accessors of
   `PathSolution` attributes computed from geometry only
   (`PathSolution.py:214-237`; no Sensing imports anywhere in
   `Distance/Connectivity/Time/TimeBetweenVisits/PathSolution`).
   Therefore: **one optimization run serves all merging comparisons.** Merging
   parameters do not enter the scenario dict, `PathInfo`, or result filenames.
2. **The sensing replay rewrites the mission.** Drones that know enough return
   to base early (`Sensing.py:493-516`), so the replayed mission time differs
   from the optimizer's planned mission time. Merging accelerates belief
   convergence and therefore shortens the *effective* mission.
3. **Existing vocabulary collision (bug).** `Analysis.py:495` passes
   `merging_strategy="discrete"` (a time model) into a parameter that
   `merge_maps()` (`Sensing.py:62`) compares against `"gcs"` (a topology), so
   `"discrete"` silently behaves as onboard merging and the gcs path is
   effectively never exercised. This design separates the two concepts.

## Decisions

| # | Decision | Rationale |
|---|----------|-----------|
| D1 | Merging/sensing parameters live **only in the analysis layer** (`SensingConfig`), not in the scenario dict / `PathInfo`. | Verified fact 1. Scenario stays geometry-focused; optimizer outputs stay shareable across merging experiments. |
| D2 | **Future rule (documented, not implemented):** when a sensing-aware objective (e.g., Effective Mission Time, Expected Detection Time) is added to `model_metric_info` and used in a model's F/G/H, merging parameters must graduate into the scenario and its filename identity. | Prevents result collisions the day the optimizer can exploit merging. |
| D3 | Two separate parameters: `merge_topology ∈ {none, onboard, gcs}` and `time_model ∈ {discrete, realtime}`. | Fixes verified fact 3. |
| D4 | Standardize on **`onboard`** (replaces code's `ondrone`; matches `Analysis.py` and the owner's terminology). Unknown values raise `ValueError` — no silent fallthrough. | One canonical vocabulary. |
| D5 | `'none'` becomes a first-class topology (short-circuit in `merge_maps`). | The baseline is currently inexpressible; comparisons need it. |
| D6 | Users choose among the **17 ready-made models** via a registry; custom model authoring is future work. | Models are tried and tested; convergence tuning remains the engineer's job (out of scope here). |
| D7 | Pareto-front navigation is **by intent**, not by exhaustive listing: per-objective extremes, balanced (centroid), knee, importance weights, direct index. | Fronts hold hundreds of non-dominated solutions; users navigate via named strategies. |
| D8 | Flat module layout preserved. | Consistent with migration decision; package restructuring belongs to the web-API round. |

## Components

### 1. `SensingConfig` (in new `SensingReplay.py`)

Validated dataclass:

```python
SensingConfig(
    merge_topology   = 'onboard',   # 'none' | 'onboard' | 'gcs'
    time_model       = 'discrete',  # 'discrete' | 'realtime'
    detection_prob   = 0.7,         # p
    false_alarm_prob = 0.2,         # q
    belief_threshold = 0.9,         # B
    target_locations = [12],
)
```

- `SensingConfig.from_info(info, **overrides)` — single, centralized defaulting
  from `PathInfo`'s existing fields (`detection_probability`, `th`,
  `target_locations`), tolerant of old pickled `PathInfo` objects lacking
  attributes (defaulting access lives here and nowhere else).
- Validation in `__post_init__`: enum membership; probabilities and threshold in
  (0, 1); `target_locations` within grid bounds when grid size is known.
- This dataclass is the future web view-section's JSON parameter schema.

### 2. `Sensing.py` changes

- `merge_maps(conn_comp, search_map, merge_topology)`:
  explicit topology parameter; `'none'` returns the map untouched; `'gcs'`
  merges only cliques containing node 0; `'onboard'` merges any clique;
  anything else raises `ValueError`.
- `sensing_and_discrete_info_sharing(sol, config)` and
  `sensing_and_realtime_info_sharing(sol, config)` take a `SensingConfig`
  instead of five loose kwargs. Behavior otherwise unchanged.

### 3. `replay()` and `compare()` (in `SensingReplay.py`)

```python
replay(solution, config)  -> ReplayResult
compare(solution, configs, labels=None) -> ComparisonResult
```

`labels` defaults to each config's `merge_topology` (deduplicated with the
config index when topologies repeat, e.g. same topology at different `p`).

`ReplayResult` carries:
- **Metrics:** Effective Mission Time (replay `mission_time`, with early
  return), Detection Time (all targets known anywhere), Inform Time (gap until
  BS knows), Time-at-least-one-drone-knows-all. Undetected targets stay `inf`
  (not masked). Display labels may render Detection Time as "Expected Detection
  Time" in the UI; the replay itself is deterministic given (solution, config).
- **Per-step artifacts** for animation/plots: belief snapshots
  (`cell_occupancy_probabilities`), occupancy status, and the truncated
  `real_time_path_matrix` (what the early-return actually flew).

`compare()` replays one solution under each config and emits the three
artifacts:
1. **Metrics table** — `pandas.DataFrame`, one row per config, plus CSV export.
2. **Time-series plots** — targets-known-over-time and belief-evolution curves,
   one line per config, saved under `Figures/Sensing/Merging Comparisons/`.
3. **Animations** — one mission playback per config (reusing `PathAnimation`),
   saved under `Results/Animations/`, filenames embedding scenario string +
   selection label + topology.

`time_model` selects which pipeline runs; topology is forwarded to
`merge_maps` — the two never share a parameter again.

### 4. `SolutionSelector` (new `SolutionSelection.py`)

Constructed from a scenario's saved results (ObjectiveValues DataFrame +
SolutionObjects list) or passed them directly.

| Method | Returns | Notes |
|---|---|---|
| `best(objective_name)` | extreme solution for that objective | Polarity-aware via `model_metric_info` (connectivity is maximized). |
| `balanced()` | centroid-nearest solution | Delegates to existing `get_median_index_of_scenario` logic (`PathOptimizationModel.py:43-56`). |
| `knee()` | high-tradeoff solution | `pymoo.mcdm.high_tradeoff`. Zero candidates → fall back to `balanced()` with a warning. Multiple → nearest to centroid among them. |
| `by_weights({name: w, ...})` | pseudo-weight pick | `pymoo.mcdm.pseudo_weights` on polarity-normalized F. Keys validated against the model's objectives; weights normalized to sum 1. Maps to web-UI importance sliders. |
| `by_index(i)` | direct pick | Bounds-checked. Maps to clicking a Pareto-front point. |

All methods return `(index, solution, label)`. WS/SOO models yield a single
solution: the selector returns index 0 for every strategy (graceful
degradation; the future view section skips front navigation for these).

Front visualization guidance (for the later web round, recorded here):
2 objectives → scatter; 3 → 3-D scatter; 4-5 (TCD/TCDT) → parallel coordinates.

### 5. Model registry (in `PathOptimizationModel.py`)

```python
AVAILABLE_MODELS: dict[str, dict]   # all 17 ready-made models by name
list_models() -> pandas.DataFrame   # name, type, algorithm, objectives, constraints
```

`PathInput.py` resolves `model` by registry name (string) as an alternative to
the current direct import. Custom model authoring: future work — the registry
makes a user-defined model "just another entry".

### 6. `Analysis.py`

Call sites updated to the new signatures (this removes the vocabulary-collision
bug). No structural refactor in this round.

## Data flow

```
scenario dict ──> PathUnitTest ──> Results/{Solutions,Objectives,...}/<scenario>-*.pkl
                                            │
                       SolutionSelector(scenario) ── best()/balanced()/knee()/by_weights()/by_index()
                                            │  (index, solution, label)
        SensingConfig.from_info(info, merge_topology=..., ...)  × N configs
                                            │
                          compare(solution, configs, labels)
                                            │
        ┌───────────────────────────────────┼─────────────────────────────┐
   metrics DataFrame + CSV      curves → Figures/Sensing/        animations → Results/Animations/
                                 Merging Comparisons/
```

## Error handling

- `SensingConfig` validation errors are raised at construction, not mid-replay.
- `merge_maps` raises `ValueError` on unknown topology (replacing today's
  silent anything-that-isn't-gcs-behaves-as-onboard).
- `knee()` fallback to `balanced()` emits a warning naming the fallback.
- `by_weights()` rejects weight keys not in the model's F.
- Undetected targets yield `inf` metrics, rendered as `inf` in tables (and
  "not detected" in plots/legends), never silently dropped.
- Old pickled solutions (pre-migration `PathInfo` without newer attributes)
  are handled exclusively inside `SensingConfig.from_info`.

## Testing (pytest, dev dependency)

1. **`merge_maps` topology semantics:** `'none'` never propagates across
   drones; `'gcs'` merges only cliques containing node 0; `'onboard'` merges
   any clique; unknown topology raises.
2. **Replay properties on a small fixed solution:** onboard Detection Time ≤
   none Detection Time; Effective Mission Time ≤ planned `sol.mission_time`.
3. **Selector:** `best()` = polarity-aware argmin/argmax per F column;
   `balanced()` agrees with `get_median_index_of_scenario`; one-hot
   `by_weights()` picks the same solution as `best()` of that objective;
   `by_index` bounds-checked; single-solution (WS) degradation returns index 0.
4. **Integration smoke:** tiny scenario (grid 8, 4 drones, few generations)
   end-to-end: optimize → select → `compare()` over the three topologies →
   assert table, plot files, and animation files exist.

## Out of scope (this round)

- Web UI / FastAPI / AWS deployment (consumes this API later).
- Custom (user-authored) optimization models.
- Convergence diagnostics and parameter auto-tuning (engineer's judgment for
  now; revisit later).
- Sensing-aware objectives inside the optimizer (D2 documents the rule for
  when they arrive).
- `Analysis.py` structural refactor.

## Future considerations

- The FastAPI view-section endpoint wraps `compare()`; importance sliders wrap
  `by_weights()`; clicking a front point wraps `by_index()`.
- Per-generation pymoo callbacks → stream the evolving front over
  WebSocket/SSE during optimization.
- `PathSolution.to_dict()` JSON serialization for the browser (pickle stays
  for the batch pipeline).
- When sensing-aware objectives land (D2): merging params join the scenario
  and `PathInfo.__str__`, and per-strategy optimization runs become necessary.
