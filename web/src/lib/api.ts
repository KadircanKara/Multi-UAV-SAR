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
  PlaygroundResult,
} from "@/lib/types";

// Empty means SAME ORIGIN: requests become relative (/api/...), which is what
// production serves — Caddy puts the API and the UI on one hostname, so there
// is no CORS and no domain baked into the image.
//
// The dev fallback is deliberately gated on NODE_ENV rather than written as a
// plain `?? "http://localhost:8000"`. Next inlines an *empty* env var as
// undefined, so a bare `??` fallback silently baked localhost:8000 into
// production bundles — a deployed site would then call the visitor's own
// machine. Local dev still works with no setup; `web/.env.local` can override.
const BASE =
  process.env.NEXT_PUBLIC_API_BASE ??
  (process.env.NODE_ENV === "development" ? "http://localhost:8000" : "");

// ─── Errors ──────────────────────────────────────────────────────────────────

/**
 * Error thrown by every fetcher on failure. `.message` is already
 * human-readable (safe to drop straight into a toast); `.status` carries the
 * HTTP status (0 = the request never reached the server) for callers that need
 * to branch on it — prefer `err.status === 404` over string-matching the message.
 */
export class ApiError extends Error {
  status: number;
  constructor(message: string, status: number) {
    super(message);
    this.name = "ApiError";
    this.status = status;
  }
}

/** A friendly message for statuses that carry no useful server detail. */
function fallbackMessage(status: number): string {
  if (status === 0) return "Can't reach the server. Check that the backend is running and try again.";
  if (status === 413) return "That request is too large to process.";
  if (status === 429) return "You're going a bit fast — wait a moment and try again.";
  if (status === 404) return "We couldn't find what you asked for.";
  // 503 is the backend's "busy right now" signal (every heavy read and the
  // comparison fan-out take a concurrency slot and refuse fast when it is full),
  // not a crash. Its own detail is suppressed with the rest of the 5xx bodies,
  // so the retry advice has to live here.
  if (status === 503) return "The server is busy right now. Wait a moment and try again.";
  if (status >= 500) return "The server ran into a problem. Please try again in a moment.";
  return "Something went wrong. Please try again.";
}

/** Pull a readable message out of an error response body (any shape). */
function messageFromBody(body: unknown, status: number): string {
  const detail = (body as { detail?: unknown })?.detail;
  // FastAPI HTTPException: detail is a plain string.
  if (typeof detail === "string" && detail.trim()) return detail.trim();
  // Pydantic/validation: detail is a list of error objects. The backend
  // normalizes these to a friendly string, so this is a defensive fallback.
  if (Array.isArray(detail) && detail.length) {
    const first = detail[0] as { msg?: string; loc?: unknown[] };
    const msg = (first?.msg ?? "").replace(/^Value error,\s*/, "");
    if (msg) return msg;
  }
  return fallbackMessage(status);
}

// ─── Internal helper ─────────────────────────────────────────────────────────

// Ceiling on how long any single request may run before the client gives up.
// Generous, because the synchronous compare/replay calls can legitimately take a
// while on a cold cache — this is here to end an indefinite hang (a wedged
// server, a dropped connection), not to bound normal latency. Per-call override
// via `timeoutMs`.
const DEFAULT_TIMEOUT_MS = 120_000;

async function request<T>(
  path: string,
  options?: RequestInit & { timeoutMs?: number }
): Promise<T> {
  const { timeoutMs, ...init } = options ?? {};
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), timeoutMs ?? DEFAULT_TIMEOUT_MS);
  // The timer stays armed until the BODY is read, not just until fetch()
  // resolves — fetch settles as soon as the headers land, so clearing it there
  // would leave `res.json()` unbounded, which is exactly the wedged-server hang
  // the timeout exists to end.
  try {
    let res: Response;
    try {
      res = await fetch(`${BASE}${path}`, {
        headers: { "Content-Type": "application/json", ...(init.headers ?? {}) },
        ...init,
        // Applied after `init` so the timeout always wins. No caller passes its
        // own signal today; one that needs to should use `timeoutMs` instead,
        // since a signal in `init` would be silently overridden here.
        signal: controller.signal,
      });
    } catch (err) {
      // A timeout aborts the fetch; distinguish it from a plain reachability
      // failure so the user knows to retry rather than assume the server is down.
      if (err instanceof DOMException && err.name === "AbortError") {
        throw new ApiError(
          "The request took too long and was cancelled. Please try again.",
          0,
        );
      }
      // Network-level failure (server down, DNS, CORS, offline) — fetch rejects
      // before any response. Present it as a reachability problem, not a crash.
      throw new ApiError(fallbackMessage(0), 0);
    }
    if (!res.ok) {
      // Never surface a raw server-side (5xx) detail to the user — it can carry
      // stack traces or internal paths. Only trust 4xx detail, which describes
      // something the user can act on.
      if (res.status >= 500) throw new ApiError(fallbackMessage(res.status), res.status);
      let body: unknown = null;
      try {
        body = await res.json();
      } catch {
        /* non-JSON error body — fall back to a status-based message */
      }
      throw new ApiError(messageFromBody(body, res.status), res.status);
    }
    return (await res.json()) as T;
  } catch (err) {
    // A body read aborted by the timer surfaces here, not in the fetch catch.
    if (err instanceof DOMException && err.name === "AbortError") {
      throw new ApiError(
        "The request took too long and was cancelled. Please try again.",
        0,
      );
    }
    throw err;
  } finally {
    clearTimeout(timer);
  }
}

/**
 * fetch() against the API origin with the same abort timeout and error shape as
 * `request()`, but handing back the raw Response — for the callers that need the
 * body as a blob or as text (the optimizer export download) rather than parsed
 * JSON. Consume the body inside `read` so the timeout covers it.
 */
export async function requestRaw<T>(
  url: string,
  read: (res: Response) => Promise<T>,
  timeoutMs = DEFAULT_TIMEOUT_MS,
): Promise<T> {
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), timeoutMs);
  try {
    const res = await fetch(url, { signal: controller.signal });
    if (!res.ok) throw new ApiError(fallbackMessage(res.status), res.status);
    return await read(res);
  } catch (err) {
    if (err instanceof DOMException && err.name === "AbortError") {
      throw new ApiError(
        "The request took too long and was cancelled. Please try again.",
        0,
      );
    }
    if (err instanceof ApiError) throw err;
    throw new ApiError(fallbackMessage(0), 0);
  } finally {
    clearTimeout(timer);
  }
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

/** GET /api/optimize/{run_id}/export — download a finished run as JSON
 *  (the `/optimize` page is memoryless: this is the only way to keep a run). */
export function optimizeExportUrl(runId: string): string {
  return `${BASE}/api/optimize/${encodeURIComponent(runId)}/export`;
}

// ─── Playground (stateless analysis of an uploaded/exported run JSON) ────────
// These mirror their seeded siblings above but POST the whole PlaygroundResult
// instead of addressing a scenario by name. Playground has no scenario, so
// `model_key` is never sent — only the fields the backend's `_SelectReq` /
// `_ReplayReq` / `_CompareReq` / `_PlaybackReq` models accept are forwarded.

/** POST /api/playground/front — Pareto front for an uploaded result. */
export function playgroundFront(result: PlaygroundResult): Promise<ParetoFront> {
  return request<ParetoFront>("/api/playground/front", {
    method: "POST",
    body: JSON.stringify({ result }),
  });
}

/** POST /api/playground/select — select a solution from an uploaded result. */
export function playgroundSelect(
  result: PlaygroundResult,
  body: SelectRequest
): Promise<SelectResponse> {
  const { strategy, objective_name, weights, index } = body;
  return request<SelectResponse>("/api/playground/select", {
    method: "POST",
    body: JSON.stringify({ result, strategy, objective_name, weights, index }),
  });
}

/** POST /api/playground/replay — run a sensing replay for an uploaded result. */
export function playgroundReplay(
  result: PlaygroundResult,
  body: ReplayRequest
): Promise<ReplayResponse> {
  const { index, config, label } = body;
  return request<ReplayResponse>("/api/playground/replay", {
    method: "POST",
    body: JSON.stringify({ result, index, config, label }),
  });
}

/** POST /api/playground/compare — compare sensing configs for an uploaded result. */
export function playgroundCompare(
  result: PlaygroundResult,
  body: CompareRequest
): Promise<CompareResponse> {
  const { index, configs, labels } = body;
  return request<CompareResponse>("/api/playground/compare", {
    method: "POST",
    body: JSON.stringify({ result, index, configs, labels }),
  });
}

/** POST /api/playground/playback — step-wise playback for an uploaded result. */
export function playgroundPlayback(
  result: PlaygroundResult,
  body: PlaybackRequest
): Promise<PlaybackResponse> {
  const { index, config, stride } = body;
  return request<PlaybackResponse>("/api/playground/playback", {
    method: "POST",
    body: JSON.stringify({ result, index, config, stride }),
  });
}

// NOTE: POST /api/playground/comparison (multi-run objective comparison) still
// exists on the backend but has no UI caller since the compare-playground page
// was removed (2026-07 route re-architecture).
