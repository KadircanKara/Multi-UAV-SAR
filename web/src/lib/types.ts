/**
 * TypeScript types mirroring the backend Pydantic schemas (backend/app/schemas.py).
 * Field names match exactly.
 */

// ─── Model Info ──────────────────────────────────────────────────────────────

export interface ModelInfo {
  name: string;
  type: string;
  algorithm: string;
  objectives: string[];
  constraints: string[];
}

// ─── Scenario Config & Derived ───────────────────────────────────────────────

export interface ScenarioConfig {
  grid_size: number;
  cell_side_length: number;
  number_of_drones: number;
  max_drone_speed: number;
  comm_cell_range: number;
  n_visits: number;
  target_positions: number[];
  th: number;
  detection_probability: number;
}

export interface ScenarioDerived {
  number_of_cells: number;
  number_of_nodes: number;
  comm_dist: number;
  miss_probability: number;
}

export interface ScenarioValidateResponse {
  valid: boolean;
  derived: ScenarioDerived;
  scenario_str: string | null;
}

// ─── Library ─────────────────────────────────────────────────────────────────

export interface ScenarioSummary {
  scenario: string;
  model_key: string;
  type: string;
  algorithm: string;
  objectives: string[];
  n_solutions: number;
  result_kind: string;
  grid_size: number | null;
  cell_side_length: number | null;
  max_drone_speed: number | null;
  number_of_drones: number | null;
  comm_range: string | null;
  variant: string | null;
  variant_value: number | null;
  has_solutions: boolean;
}

export interface ScenarioDetail {
  scenario: string;
  model: ModelInfo;
  n_solutions: number;
  result_kind: string;
  params: Record<string, unknown>;
}

// ─── Pareto Front / Solution Selection ───────────────────────────────────────

export interface ParetoSolution {
  index: number;
  objectives_signed: Record<string, number>;
  objectives_abs: Record<string, number>;
}

export interface ParetoFront {
  scenario: string;
  model_key: string;
  objectives: string[];
  polarities: Record<string, number>;
  result_kind: string;
  n_solutions: number;
  solutions: ParetoSolution[];
  capabilities: Record<string, unknown>;
}

export interface SelectRequest {
  model_key?: string | null;
  strategy: string;
  objective_name?: string | null;
  weights?: Record<string, number> | null;
  index?: number | null;
}

export interface SolutionDetail {
  index: number;
  label: string;
  objectives_abs: Record<string, number>;
}

export interface SelectResponse {
  index: number;
  label: string;
  detail: SolutionDetail;
}

// ─── Optimizer (run your own optimisation) ────────────────────────────────────

export interface OptimizeConfig {
  optimization_type: "SOO" | "MOO";
  method: string;
  objectives: string[];
  weights?: Record<string, number> | null;
  pop_size: number;
  n_gen: number;
  seed?: number;
  /** Constraint thresholds — null disables the constraint. Omitting them makes
   *  the backend apply its defaults (3600 s / 0.5), so callers should send them
   *  explicitly to reflect the user's toggles. */
  max_mission_time?: number | null;
  min_connectivity?: number | null;
  /** Max Mean TBV ceiling (seconds); null disables it. Only bites at n_visits ≥ 2. */
  max_mean_tbv?: number | null;
  /** "fixed" runs all n_gen; "max" treats n_gen as a cap and stops early once
   *  the feasible objective optima converge. Patience/threshold use backend
   *  defaults unless provided. */
  gen_strategy?: "fixed" | "max";
  early_stop_patience?: number;
  early_stop_threshold?: number;
  scenario: ScenarioConfig;
}

export interface OptimizeFrontSolution {
  index: number;
  objectives_signed: Record<string, number | null>;
  objectives_abs: Record<string, number | null>;
}

export interface OptimizeFront {
  scenario: string;
  model_key: string;
  objectives: string[];
  polarities: Record<string, number>;
  result_kind: "front" | "single";
  n_solutions: number;
  solutions: OptimizeFrontSolution[];
  /** True when the user stopped the run early; the front is the best-so-far. */
  cancelled?: boolean;
  /** True when "Max Generations" converged and stopped before n_gen. */
  early_stopped?: boolean;
  stopped_at_gen?: number | null;
}

export interface OptimizeStartResponse {
  run_id: string;
  scenario_name: string;
  model_key: string;
  exists: boolean;
}

export interface OptimizeCheckResponse {
  scenario_name: string;
  model_key: string;
  exists: boolean;
}

export interface OptimizeStatus {
  state: "running" | "done" | "failed";
  gen?: number;
  n_gen?: number;
  front?: OptimizeFront;
  error?: string;
  exists_in_library?: boolean;
  /** Live progress while running. */
  best?: Record<string, number>;
  live_front?: Record<string, number>[];
}

export interface OptimizeStopResponse {
  run_id: string;
  stopping: boolean;
}

export interface OptimizeSaveResponse {
  scenario_name: string;
  model_key: string;
}

// ─── Sensing Config ───────────────────────────────────────────────────────────

export interface SensingConfig {
  merge_topology: "none" | "onboard" | "gcs";
  time_model: "discrete" | "realtime";
  detection_prob: number;
  false_alarm_prob: number;
  belief_threshold: number;
  target_locations: number[];
}

// ─── Replay / Compare / Playback ─────────────────────────────────────────────

export interface ReplayRequest {
  model_key?: string | null;
  index: number;
  config: SensingConfig;
  label?: string | null;
}

export interface CompareRequest {
  model_key?: string | null;
  index: number;
  configs: SensingConfig[];
  labels?: string[] | null;
}

export interface PlaybackRequest {
  model_key?: string | null;
  index: number;
  config: SensingConfig;
  stride?: number;
}

// Response payloads from replay/compare/playback are dynamic;
// typed as structured but open records.
export type ReplayResponse   = Record<string, unknown>;
export type CompareResponse  = Record<string, unknown>;
export type PlaybackResponse = Record<string, unknown>;

// ─── Model Grid (parameter-effect analysis) ──────────────────────────────────

export interface ObjectiveStat {
  min: number | null;
  max: number | null;
  mean: number | null;
  best: number | null;
}

export interface ModelGridScenario {
  scenario: string;
  number_of_drones: number | null;
  comm_range: string | null;
  comm_range_value: number | null;
  n_visits: number | null;
  n_solutions: number;
  result_kind: string;
  objective_stats: Record<string, ObjectiveStat>;
}

export interface ModelGrid {
  model_key: string;
  type: string;
  algorithm: string;
  objectives: string[];
  polarities: Record<string, number>;
  scenarios: ModelGridScenario[];
}

// ─── Cross-model objective comparison (/api/comparison) ──────────────────────

export interface ComparisonScenario {
  scenario: string;
  model_key: string;
  type: string;
  algorithm: string;
  /** Objectives this model actually optimised (the rest are computed for compare). */
  optimized_objectives: string[];
  number_of_drones: number | null;
  comm_range: string | null;
  comm_range_value: number | null;
  n_visits: number | null;
  n_solutions: number;
  /** Per-objective stats; null for an objective with no data (e.g. TBV at n_visits=1). */
  objective_stats: Record<string, ObjectiveStat | null>;
}

export interface ComparisonResponse {
  objectives: string[];
  polarities: Record<string, number>;
  scenarios: ComparisonScenario[];
  skipped: string[];
}

// ─── Cross-model time-metric comparison (/api/comparison/time) ───────────────

export interface TimeComparisonScenario {
  scenario: string;
  model_key: string;
  type: string;
  algorithm: string;
  number_of_drones: number | null;
  comm_range: string | null;
  comm_range_value: number | null;
  n_visits: number | null;
  selected_index: number;
  /** Keyed by metric name (the four sensing time-metrics). */
  metric_values: Record<string, number | null>;
}

export interface TimeComparisonResponse {
  metrics: string[];
  scenarios: TimeComparisonScenario[];
  skipped: string[];
  strategy: string;
}

// ─── Playback payload (typed from /api/playback response) ────────────────────

export interface PlaybackPayload {
  scenario: string;
  model_key: string;
  index: number;
  time_model: "discrete" | "realtime";
  merge_topology: "none" | "onboard" | "gcs";
  grid_size: number;
  cell_side_length: number;
  number_of_nodes: number;
  targets: number[];
  belief_threshold: number;
  steps: number;
  stride: number;
  raw_lengths: Record<string, unknown>;
  /** shape [number_of_nodes][steps], positions in METERS */
  trajectories: { x: number[][]; y: number[][] };
  /** [step] -> list of [i,j] connected node-pairs (i<j) */
  connectivity: number[][][];
  /** [cell][step], 0..1 (or null); length = grid_size^2 */
  belief: (number | null)[][];
  /** [step] count of known targets */
  targets_known: number[];
}
