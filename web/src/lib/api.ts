/**
 * Typed API client for the Multi-UAV-SAR backend.
 * Base URL: NEXT_PUBLIC_API_BASE env var (defaults to http://localhost:8000).
 * All fetchers are async and throw an Error with the response detail on non-2xx.
 */

import type {
  ModelInfo,
  ScenarioConfig,
  ScenarioValidateResponse,
  ScenarioSummary,
  ScenarioDetail,
  ParetoFront,
  SelectRequest,
  SelectResponse,
  ReplayRequest,
  ReplayResponse,
  CompareRequest,
  CompareResponse,
  PlaybackRequest,
  PlaybackResponse,
  ModelGrid,
  ComparisonResponse,
  TimeComparisonResponse,
  SensingConfig,
  OptimizeConfig,
  OptimizeCheckResponse,
  OptimizeStartResponse,
  OptimizeStatus,
  OptimizeStopResponse,
  OptimizeSaveResponse,
  MissionConfigResponse,
} from "@/lib/types";

const BASE =
  process.env.NEXT_PUBLIC_API_BASE ?? "http://localhost:8000";

// ─── Internal helper ─────────────────────────────────────────────────────────

async function request<T>(
  path: string,
  options?: RequestInit
): Promise<T> {
  const res = await fetch(`${BASE}${path}`, {
    headers: { "Content-Type": "application/json", ...(options?.headers ?? {}) },
    ...options,
  });
  if (!res.ok) {
    let detail: string;
    try {
      const body = await res.json();
      detail = body?.detail ?? JSON.stringify(body);
    } catch {
      detail = res.statusText;
    }
    throw new Error(`API ${res.status}: ${detail}`);
  }
  return res.json() as Promise<T>;
}

// ─── Endpoints ───────────────────────────────────────────────────────────────

/** GET /api/models — list available optimisation models. */
export function getModels(): Promise<ModelInfo[]> {
  return request<ModelInfo[]>("/api/models");
}

/** GET /api/scenarios/default — fetch the default scenario config. */
export function getDefaultScenario(): Promise<ScenarioConfig> {
  return request<ScenarioConfig>("/api/scenarios/default");
}

/** POST /api/scenarios/validate — validate + derive a scenario config. */
export function validateScenario(
  body: { scenario: ScenarioConfig; model_key?: string | null }
): Promise<ScenarioValidateResponse> {
  return request<ScenarioValidateResponse>("/api/scenarios/validate", {
    method: "POST",
    body: JSON.stringify(body),
  });
}

/** GET /api/library — list all precomputed scenario summaries. */
export function getLibrary(): Promise<ScenarioSummary[]> {
  return request<ScenarioSummary[]>("/api/library");
}

/** GET /api/models/{model_key}/grid — parameter-effect grid for one model. */
export function getModelGrid(modelKey: string): Promise<ModelGrid> {
  return request<ModelGrid>(
    `/api/models/${encodeURIComponent(modelKey)}/grid`
  );
}

/** GET /api/library/{scenario} — full detail for one scenario. */
export function getScenarioDetail(scenario: string): Promise<ScenarioDetail> {
  return request<ScenarioDetail>(
    `/api/library/${encodeURIComponent(scenario)}`
  );
}

/** GET /api/library/{scenario}/config — persisted optimizer run-config. */
export function getMissionConfig(
  scenario: string
): Promise<MissionConfigResponse> {
  return request<MissionConfigResponse>(
    `/api/library/${encodeURIComponent(scenario)}/config`
  );
}

/** GET /api/fronts/{scenario}[?model_key=] — Pareto front (+ capabilities). */
export function getFront(
  scenario: string,
  modelKey?: string
): Promise<ParetoFront> {
  const qs = modelKey ? `?model_key=${encodeURIComponent(modelKey)}` : "";
  return request<ParetoFront>(
    `/api/fronts/${encodeURIComponent(scenario)}${qs}`
  );
}

/** GET /api/fronts/{scenario}/capabilities[?model_key=] — model capabilities. */
export function getCapabilities(
  scenario: string,
  modelKey?: string
): Promise<Record<string, unknown>> {
  const qs = modelKey ? `?model_key=${encodeURIComponent(modelKey)}` : "";
  return request<Record<string, unknown>>(
    `/api/fronts/${encodeURIComponent(scenario)}/capabilities${qs}`
  );
}

/** POST /api/fronts/{scenario}/select — select a solution from the Pareto front. */
export function selectSolution(
  scenario: string,
  body: SelectRequest
): Promise<SelectResponse> {
  return request<SelectResponse>(
    `/api/fronts/${encodeURIComponent(scenario)}/select`,
    { method: "POST", body: JSON.stringify(body) }
  );
}

/** POST /api/replay/{scenario} — run a sensing replay. */
export function replay(
  scenario: string,
  body: ReplayRequest
): Promise<ReplayResponse> {
  return request<ReplayResponse>(
    `/api/replay/${encodeURIComponent(scenario)}`,
    { method: "POST", body: JSON.stringify(body) }
  );
}

/** POST /api/compare/{scenario} — compare sensing configs. */
export function compare(
  scenario: string,
  body: CompareRequest
): Promise<CompareResponse> {
  return request<CompareResponse>(
    `/api/compare/${encodeURIComponent(scenario)}`,
    { method: "POST", body: JSON.stringify(body) }
  );
}

/** POST /api/playback/{scenario} — step-wise playback. */
export function playback(
  scenario: string,
  body: PlaybackRequest
): Promise<PlaybackResponse> {
  return request<PlaybackResponse>(
    `/api/playback/${encodeURIComponent(scenario)}`,
    { method: "POST", body: JSON.stringify(body) }
  );
}

/**
 * POST /api/comparison — compare scenarios across ALL objectives (including
 * objectives a model did not optimise, computed from the solution objects).
 */
export function compareObjectives(
  scenarios: string[]
): Promise<ComparisonResponse> {
  return request<ComparisonResponse>("/api/comparison", {
    method: "POST",
    body: JSON.stringify({ scenarios }),
  });
}

/**
 * POST /api/comparison/time — compare sensing time-metrics across scenarios.
 * Runs one replay per scenario (one solution chosen by `strategy`, shared config).
 */
export function compareTimeMetrics(body: {
  scenarios: string[];
  config: SensingConfig;
  strategy?: string;
  objective_name?: string | null;
  weights?: Record<string, number> | null;
}): Promise<TimeComparisonResponse> {
  return request<TimeComparisonResponse>("/api/comparison/time", {
    method: "POST",
    body: JSON.stringify(body),
  });
}

// ─── Optimizer ───────────────────────────────────────────────────────────────

/** POST /api/optimize/check — does a run for this config already exist? */
export function checkOptimize(
  body: OptimizeConfig
): Promise<OptimizeCheckResponse> {
  return request<OptimizeCheckResponse>("/api/optimize/check", {
    method: "POST",
    body: JSON.stringify(body),
  });
}

/** POST /api/optimize — start an optimisation run (409 if one is in progress). */
export function startOptimize(
  body: OptimizeConfig
): Promise<OptimizeStartResponse> {
  return request<OptimizeStartResponse>("/api/optimize", {
    method: "POST",
    body: JSON.stringify(body),
  });
}

/** GET /api/optimize/{run_id} — poll the status of a running optimisation. */
export function getOptimizeStatus(runId: string): Promise<OptimizeStatus> {
  return request<OptimizeStatus>(
    `/api/optimize/${encodeURIComponent(runId)}`
  );
}

/** POST /api/optimize/{run_id}/stop — cooperatively stop a running optimisation
 *  (it finishes within a generation or two with its best-so-far front). */
export function stopOptimize(runId: string): Promise<OptimizeStopResponse> {
  return request<OptimizeStopResponse>(
    `/api/optimize/${encodeURIComponent(runId)}/stop`,
    { method: "POST" }
  );
}

/** POST /api/optimize/{run_id}/save — persist a finished run (409 if it exists). */
export function saveOptimizeRun(
  runId: string,
  overwrite: boolean
): Promise<OptimizeSaveResponse> {
  return request<OptimizeSaveResponse>(
    `/api/optimize/${encodeURIComponent(runId)}/save`,
    { method: "POST", body: JSON.stringify({ overwrite }) }
  );
}
