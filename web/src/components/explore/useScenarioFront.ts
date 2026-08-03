"use client";

/**
 * useScenarioFront — loads the Pareto front behind one ExplorerSource.
 *
 * Extracted verbatim from ScenarioExplorer's front-fetch effect: same
 * `sourceFront` dispatch (getFront for a seeded scenario, playgroundFront for
 * an uploaded result), same cancelled-flag single-flight, same error text.
 *
 * The only change is the dependency array. The effect used to depend on
 * `[sourceKey, onFrontLoaded]` behind an eslint-disable, which made an
 * unmemoised `onFrontLoaded` refetch in a loop. Fetching now depends on the
 * request payload and nothing else; notifying the parent is the caller's
 * concern (see useScenarioSections), so no callback identity can reach the
 * network.
 */

import { useEffect, useState } from "react";
import { sourceFront, type ExplorerSource } from "@/lib/source";
import type { ParetoFront } from "@/lib/types";

export interface ScenarioFrontState {
  front: ParetoFront | null;
  loading: boolean;
  error: string | null;
}

export function useScenarioFront(source: ExplorerSource): ScenarioFrontState {
  const [front, setFront] = useState<ParetoFront | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  // Narrow the union down to the two values the request is actually built
  // from. Callers write `source={{ mode: "seeded", scenario }}` inline, so the
  // object identity is fresh on every render and depending on it would refetch
  // forever; `scenario` is a string and `result` is the caller's own state
  // object, both stable. Listing these lets react-hooks/exhaustive-deps verify
  // the effect instead of us silencing it.
  const scenario = source.mode === "seeded" ? source.scenario : null;
  const result = source.mode === "playground" ? source.result : null;

  useEffect(() => {
    // A seeded source whose scenario name hasn't resolved yet (route params
    // still loading) has nothing to fetch: stay in the initial loading state
    // and hit no endpoint, exactly as before.
    const request: ExplorerSource | null = scenario
      ? { mode: "seeded", scenario }
      : result
        ? { mode: "playground", result }
        : null;
    if (!request) return;

    let cancelled = false;
    setLoading(true);
    setError(null);

    sourceFront(request)
      .then((data) => {
        if (!cancelled) {
          setFront(data);
          setLoading(false);
        }
      })
      .catch((err: unknown) => {
        if (!cancelled) {
          setError(err instanceof Error ? err.message : String(err));
          setLoading(false);
        }
      });

    return () => {
      cancelled = true;
    };
  }, [scenario, result]);

  return { front, loading, error };
}
