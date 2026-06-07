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

### 4. `SolutionSelector` (new `SolutionSelection.py`) — model-aware

Constructed from a scenario's saved results (ObjectiveValues DataFrame +
SolutionObjects list) **and the model dict**. The model and result shape
determine which strategies exist; the selector is self-describing so UIs
never hardcode these rules.

```python
selector.result_kind     # 'front' | 'single'
selector.capabilities()  # machine-readable: which strategies are valid,
                         # which objective names best() accepts, max index
```

`result_kind` is `'single'` when the model Type is SOO or WS, **or** when a
MOO front collapsed to one non-dominated point (degenerate front — also
surfaced as a warning, since it is a convergence signal for the engineer).

| Method | Availability | Notes |
|---|---|---|
| `best(objective_name)` | `'front'` only; `objective_name` must be in the model's F | Polarity-aware via `model_metric_info` (connectivity is maximized). Best-of-a-non-optimized-metric (e.g. `best('Max Mean TBV')` on a TC run) is rejected — the front was never shaped by it, so the answer would be a sampling accident. Error lists valid names. |
| `balanced()` | `'front'` only | Delegates to existing centroid logic (`PathOptimizationModel.py:43-56`), which already asserts MOO. |
| `knee()` | `'front'` only, ≥ 2 objectives, ≥ 3 points | `pymoo.mcdm.high_tradeoff`. Zero candidates → fall back to `balanced()` with a warning. Multiple → nearest to centroid among them. |
| `by_weights({name: w, ...})` | `'front'` only | `pymoo.mcdm.pseudo_weights` on polarity-normalized F. Keys ⊆ model's F (missing = weight 0, at least one nonzero); normalized to sum 1. Maps to web-UI importance sliders. For WS models the error explains that weights were fixed at optimization time: re-run with different WS weights, or use the MOO variant to explore trade-offs interactively. |
| `by_index(i)` | always | Bounds-checked against result size. Maps to clicking a Pareto-front point. |
| `the_solution()` | `'single'` only | The only meaningful accessor for SOO/WS/degenerate results; the view section skips front navigation and goes straight to the merging comparison. |

All methods return `(index, solution, label)`. Calling an unavailable
strategy raises `StrategyUnavailableError` naming the reason and the valid
alternatives; UIs are expected to consult `capabilities()` first.

Front visualization guidance (for the later web round, recorded here):
2 objectives → scatter; 3 → 3-D scatter; 4-5 (TCD/TCDT) → parallel
coordinates; `'single'` results → solution card, no front plot.

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
                       SolutionSelector(scenario, model) ── capabilities()-gated:
                              best()/balanced()/knee()/by_weights()/by_index()/the_solution()
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
- Unavailable selection strategies raise `StrategyUnavailableError` with the
  reason and valid alternatives (e.g., `by_weights` on a WS model explains
  that WS weights are fixed pre-run and points to the MOO variant).
- `by_weights()` rejects weight keys not in the model's F and all-zero weights.
- Degenerate MOO fronts (single non-dominated point) downgrade the selector to
  `'single'` kind with a warning — a convergence signal for the engineer.
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
   `by_index` bounds-checked.
4. **Capability matrix:** SOO/WS selectors expose only
   `the_solution()`/`by_index()` (no weights, knee, balanced, best); MOO
   selectors expose all strategies; `best()` rejects objectives outside the
   model's F (e.g. `Max Mean TBV` on TC); degenerate single-point MOO front
   downgrades to `'single'` with a warning; unavailable strategies raise
   `StrategyUnavailableError`.
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

## Addendum A (2026-06-07): Realtime pipeline completion

Investigation (Explore + Plan agents, claims verified against code) answered:
*does `time_model` genuinely affect sensing calculations?* **Yes — and the
realtime pipeline is genuinely distinct but was abandoned mid-implementation.**
Realtime uses continuous interpolated positions (`get_real_paths`,
`Time.py:227-259`, ~1-second resolution by construction), per-second dynamic
connectivity (`get_real_connectivity_matrix`, `Time.py:493-513`) so drones can
merge **mid-flight between cells** (the value proposition of realtime merging),
and sparse Bayesian sensing only at on-grid positions. It is never exercised
by current experiment code and cannot currently produce correct metrics.
Completing it is **added to this round's scope** — `time_model='realtime'`
must work, not just dispatch.

### Verified defects

| # | Defect | Evidence |
|---|---|---|
| A1 | `deepcopy` never imported (`from copy import copy` then `import copy` shadowing, `Sensing.py:13-14`) — **both pipelines crash on entry today** | `'deepcopy' in dir(Sensing)` → False; `Sensing.py:124,386` |
| A2 | Realtime early-return detection never fires (Python `list == int` → `False`, not elementwise) → Effective Mission Time always `inf` | `Sensing.py:357,361` |
| A3 | Realtime return dict omits `"cell occupancy probabilities"` (required by ReplayResult for belief plots/animations) | `Sensing.py:382` vs `:531` |
| A4 | `isCoordinateDiscrete` uses exact float equality on interpolated coords → silent under-sensing risk | `Time.py:9-15` |
| A5 | ~100 lines of abandoned/commented code inside the realtime function, incl. an unfinished identifier | `Sensing.py:219-242,271-278,310-354` |
| A6 | Leg-seam double-sensing: `get_real_paths` uses endpoint-inclusive `linspace`, so leg i's last column == leg i+1's first column → the same physical cell arrival is sensed twice, inflating `n_obs` | `Time.py:242-252` |
| — | **Not** a defect: realtime's 1-second-per-step mission time is correct by construction (interpolation density is `ceil(dist/speed)`); only `ceil` rounding makes it slightly coarser than discrete | `Sensing.py:267-270` |

### Decisions

| # | Decision | Rationale |
|---|---|---|
| AD1 | Return-dict contract: both pipelines return the same 7 keys (`cell occupancy probabilities`, `search map`, `occupancy status`, `detection time`, `inform time`, `mission time`, `time at least one drone knows all targets`) plus the mutated solution copy. `replay()` must not know which pipeline ran. | Uniform dispatch in `SensingReplay`. |
| AD2 | Keep the all-drones-aligned `discrete_step` concept (geometrically sound: `get_real_paths` gives every drone the same `dt` per leg) but gate sensing on *cell change since previous column* (kills the A6 seam double-count) and compute `n_obs` from discrete-path arrivals exactly as the discrete pipeline does — same arrival ⇒ same `n_obs` in both pipelines. | Cross-pipeline parity must be testable. |
| AD3 | Realtime early-return truncation also rewrites `x.real_time_path_matrix` (discrete recipe, `Sensing.py:503-508`) and stashes truncated realtime arrays on the solution — because `PathAnimation` re-derives trajectories via `get_real_paths(sol)`, this is what makes animations show the early return. The bare `try/except` around the path rewrite becomes explicit length reconciliation. | Animations are a primary spec artifact. |
| AD4 | Extract only four provably-identical helpers shared by both loops (search-map init, occupancy status + per-cell max probs, detection-time update, metric finalization); do **not** merge the two loops. Discrete behavior locked by a regression snapshot test before refactoring. | Spec requires discrete behavior preservation; full merge is high-risk. |
| AD5 | Preserve `merge_maps` being called every interpolated second in realtime (only sensing is on-grid-gated; merging is not). A dedicated test pins mid-flight merging — two drones within comm range between cells must exchange beliefs. | This is the realtime-vs-discrete behavioral distinction. |
| AD6 | No `realtime ≤ discrete` detection-time assertion: realtime senses sparser per wall-clock second but shares earlier — the effects fight, the sign is solution-dependent. Test only provable invariants. | Honest tests. |

### Additional tests (extends the Testing section)

5. **Contract:** both pipelines return identical metric key sets; realtime
   runs without `NameError`; realtime Effective Mission Time finite (and ≤
   planned `sol.mission_time`) on a fixture that triggers early return;
   detection time finite when a target lies on a visited cell, `inf` when not.
6. **Within-time-model property:** onboard detection ≤ none detection holds
   inside realtime as well (append-only merging argument unchanged).
7. **Mid-flight merge distinction (AD5):** belief transfer observed at a step
   where neither drone is on a grid coordinate.
8. **Discrete regression snapshot (AD4):** discrete metrics byte-identical
   before/after helper extraction.
9. **Animation-shape consistency (AD3):** after a realtime early return,
   `get_real_paths(x)` yields the truncated trajectory and
   `cell occupancy probabilities` column counts line up with it.

## Addendum B (2026-06-07): post-validation decisions

A real NSGA-II run (TC_MOO_NSGA2, pop 200 × gen 300, 4-drone mutation tuning
per main.py annotations) validated `SolutionSelector.from_scenario` against
on-disk pickles end-to-end. Two findings, two decisions:

| # | Decision | Rationale |
|---|---|---|
| B1 | `from_scenario` raises a clear `FileNotFoundError` ("Did the optimization run converge? Infeasible runs save nothing...") when a scenario has no saved results. | Infeasible runs (constraint violation never reaches 0) save no pickles; the raw pandas path error was unhelpful. The future web tool needs this distinction ("no results" vs "bad path"). |
| B2 | **Owner-confirmed doctrine:** drones that know all target locations return to the GCS by going to **cell 0 first, then to the GCS position, to avoid going out of the map**. This containment detour is intended even though it can lengthen the discrete-replay Effective Mission Time relative to `none` (observed on real data: onboard 1178.8s vs none 1130.5s) while Inform Time improves dramatically (∞ → 76.6s). Not a defect; no straight-home option planned. | Map containment is a hard operational constraint of the mission model. |

## Future considerations

- The FastAPI view-section endpoint wraps `compare()`; importance sliders wrap
  `by_weights()`; clicking a front point wraps `by_index()`.
- Per-generation pymoo callbacks → stream the evolving front over
  WebSocket/SSE during optimization.
- `PathSolution.to_dict()` JSON serialization for the browser (pickle stays
  for the batch pipeline).
- When sensing-aware objectives land (D2): merging params join the scenario
  and `PathInfo.__str__`, and per-strategy optimization runs become necessary.
