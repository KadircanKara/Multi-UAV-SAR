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
