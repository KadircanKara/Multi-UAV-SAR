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
