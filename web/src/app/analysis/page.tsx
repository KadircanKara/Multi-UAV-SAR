"use client";

/**
 * /analysis — the deep-dive for ONE exported run, split out of /optimize.
 *
 * Nothing here is seeded: the run arrives as a JSON file the reader uploads,
 * or as a `?run=<id>` handed over by /optimize's "Analyze this result". That
 * parameter is a RUN ID, not the result — the export endpoint already serves
 * the exact bytes an upload would carry, so the handover is a refetch rather
 * than a copy through sessionStorage or a store. It costs one request and buys
 * three things: no size ceiling on a front with thousands of solutions, a URL
 * that survives a reload, and one parse path (`parseRunJson`) for both entry
 * points instead of two that can drift.
 *
 * Runs are not persisted server-side, so a link can go stale once the job is
 * pruned. That reads as "couldn't load", with the upload still there — the
 * reader always has the downloaded JSON as the durable copy.
 */

import { Suspense, useCallback, useEffect, useState } from "react";
import { useSearchParams } from "next/navigation";
import { toast } from "sonner";
import { optimizeExportUrl, requestRaw } from "@/lib/api";
import type { ParetoFront, PlaygroundResult } from "@/lib/types";
import { UploadResult, parseRunJson } from "@/components/playground/UploadResult";
import RunSummary from "@/components/optimize/RunSummary";
import BestValues from "@/components/optimize/BestValues";
import ScenarioExplorer from "@/components/explore/ScenarioExplorer";
import { Skeleton } from "@/components/ui/skeleton";
import { useElementHeight } from "@/hooks/useElementHeight";

function AnalysisPage() {
  const searchParams = useSearchParams();
  const runId = searchParams.get("run");

  const [result, setResult] = useState<PlaygroundResult | null>(null);
  // Bumped per loaded run so the explorer subtree is rebuilt rather than
  // reused — it caches per-run state that has no business surviving a switch.
  const [nonce, setNonce] = useState(0);
  // Handed over by the explorer once it loads, so the cards below need no
  // second fetch of the same front.
  const [front, setFront] = useState<ParetoFront | null>(null);
  const [loadingRun, setLoadingRun] = useState(false);

  // The page's own pinned header. The explorer's control panel pins directly
  // below it — see SectionPanelLayout's `stickyOffset`.
  const { ref: headerRef, height: headerHeight } = useElementHeight();

  const load = useCallback((r: PlaygroundResult) => {
    setResult(r);
    // Drop the previous run's cards so they can't linger over a new upload
    // while the explorer refetches.
    setFront(null);
    setNonce((n) => n + 1);
  }, []);

  // Fetch the handed-over run. Keyed on the id alone, so returning to the same
  // URL does not refetch on every render.
  useEffect(() => {
    if (!runId) return;
    let cancelled = false;
    setLoadingRun(true);
    requestRaw(optimizeExportUrl(runId), (r) => r.text())
      .then((text) => {
        if (cancelled) return;
        load(parseRunJson(text));
      })
      .catch((e: unknown) => {
        if (cancelled) return;
        toast.error("Could not load that run", {
          description:
            e instanceof Error
              ? e.message
              : "It may have expired — upload the downloaded JSON instead.",
        });
      })
      .finally(() => {
        if (!cancelled) setLoadingRun(false);
      });
    return () => {
      cancelled = true;
    };
  }, [runId, load]);

  const handleFront = useCallback((f: ParetoFront) => setFront(f), []);

  return (
    <div className="mx-auto flex max-w-7xl flex-col gap-6 px-4 py-6">
      {/* Sticky: which run this is, and its headline numbers, stay on screen
          while the reader scrolls the charts underneath. */}
      <div
        ref={headerRef}
        className="sticky top-14 z-30 lg:top-0 flex flex-col gap-3 rounded-xl border border-border bg-background px-4 py-3"
      >
        <div className="flex flex-col gap-1">
          <h1 className="text-2xl font-bold tracking-tight text-foreground">
            Analysis
          </h1>
          <p className="text-[15px] text-muted-foreground">
            Analyze any exported run — upload a result JSON, or use “Analyze
            this result” after a run finishes on the Optimizer. Results are not
            stored; everything runs from the file.
          </p>
        </div>
        <div className="flex flex-wrap items-center gap-x-6 gap-y-3">
          <div className="shrink-0">
            <UploadResult onLoaded={load} />
          </div>
          {result && <RunSummary result={result} front={front} />}
        </div>
      </div>

      {/* Outside the sticky panel: the tiles are a reading of the front, not
          the identity of it, and pinning them costs a third of the viewport
          that the charts below need. */}
      {front && <BestValues front={front} />}

      {loadingRun && !result && (
        <div className="flex flex-col gap-4">
          <Skeleton className="h-8 w-64" />
          <Skeleton className="h-64 w-full" />
        </div>
      )}

      {result ? (
        <ScenarioExplorer
          key={nonce}
          source={{ mode: "playground", result }}
          showTitle={false}
          showSummary={false}
          onFrontLoaded={handleFront}
          stickyOffset={headerHeight}
        />
      ) : (
        !loadingRun && (
          <p className="rounded border border-dashed border-border px-4 py-6 text-center text-sm text-muted-foreground">
            No run loaded. Upload a result JSON above, or run an optimisation
            and choose “Analyze this result”.
          </p>
        )
      )}
    </div>
  );
}

export default function AnalysisRoute() {
  // useSearchParams needs a Suspense boundary to avoid opting the whole route
  // into client-side rendering at build time.
  return (
    <Suspense fallback={null}>
      <AnalysisPage />
    </Suspense>
  );
}
