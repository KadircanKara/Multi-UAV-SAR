/**
 * ExplorerSource — dispatches Pareto-front/select/replay/compare/playback
 * calls to either the seeded (named scenario) or playground (uploaded
 * PlaygroundResult) backend, so explorer UI code can be written once against
 * a single source-agnostic interface.
 */

import {
  getFront,
  selectSolution,
  replay,
  compare,
  playback,
  playgroundFront,
  playgroundSelect,
  playgroundReplay,
  playgroundCompare,
  playgroundPlayback,
} from "./api";
import type {
  ParetoFront,
  SelectRequest,
  SelectResponse,
  ReplayRequest,
  ReplayResponse,
  CompareRequest,
  CompareResponse,
  PlaybackRequest,
  PlaybackResponse,
  PlaygroundResult,
} from "./types";

export type ExplorerSource =
  | { mode: "seeded"; scenario: string }
  | { mode: "playground"; result: PlaygroundResult };

/**
 * A stable string identifying WHICH run a source points at, for use as a React
 * `key`.
 *
 * Components that fetch derived results for a source (the merging comparison,
 * the playback) hold those results in local state and have no effect that
 * clears them when the source changes underneath. Keying them on this string
 * unmounts the stale instance, which is the only thing that guarantees a
 * result on screen belongs to the run named beside it. Do not replace it with
 * the source object's identity: callers build `{ mode: "seeded", scenario }`
 * inline, so that is fresh on every render and would remount continuously.
 */
export function sourceKey(s: ExplorerSource): string {
  if (s.mode === "seeded") return `seeded:${s.scenario}`;
  // A playground result has no server-side name. Model key plus solution count
  // separates the runs a user actually switches between; /optimize additionally
  // remounts its whole Analysis block per run, so this never stands alone there.
  const key = s.result.model.model_key ?? s.result.model.Exp;
  return `playground:${key}:${s.result.solutions.length}`;
}

export function sourceFront(s: ExplorerSource): Promise<ParetoFront> {
  return s.mode === "seeded" ? getFront(s.scenario) : playgroundFront(s.result);
}

export function sourceSelect(
  s: ExplorerSource,
  body: SelectRequest
): Promise<SelectResponse> {
  return s.mode === "seeded"
    ? selectSolution(s.scenario, body)
    : playgroundSelect(s.result, body);
}

export function sourceReplay(
  s: ExplorerSource,
  body: ReplayRequest
): Promise<ReplayResponse> {
  return s.mode === "seeded"
    ? replay(s.scenario, body)
    : playgroundReplay(s.result, body);
}

export function sourceCompare(
  s: ExplorerSource,
  body: CompareRequest
): Promise<CompareResponse> {
  return s.mode === "seeded"
    ? compare(s.scenario, body)
    : playgroundCompare(s.result, body);
}

export function sourcePlayback(
  s: ExplorerSource,
  body: PlaybackRequest
): Promise<PlaybackResponse> {
  return s.mode === "seeded"
    ? playback(s.scenario, body)
    : playgroundPlayback(s.result, body);
}
