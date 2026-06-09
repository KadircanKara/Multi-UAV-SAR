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
