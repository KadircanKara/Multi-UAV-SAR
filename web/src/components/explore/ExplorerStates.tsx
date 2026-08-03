"use client";

/**
 * The two non-content states a scenario deep-dive can be in: still fetching its
 * front, or unable to.
 *
 * These lived inside ScenarioExplorer, which could short-circuit its whole
 * render on `loading` / `error` because it owned the entire block. A page that
 * interleaves its own sections with the explorer's (the model route) cannot do
 * that — the parameter-effect charts and the combination table must keep
 * rendering when the front fails — so the states have to be renderable as ONE
 * section's content. Hence a file of their own, importable by both without
 * ScenarioExplorer and useScenarioSections importing each other.
 */

import { Skeleton } from "@/components/ui/skeleton";

export function ExplorerSkeleton() {
  return (
    <div className="flex flex-col gap-4">
      <div className="flex gap-2">
        <Skeleton className="h-6 w-24" />
        <Skeleton className="h-6 w-32" />
      </div>
      <div className="flex gap-2">
        <Skeleton className="h-9 w-24" />
        <Skeleton className="h-9 w-24" />
      </div>
      <Skeleton className="h-72 w-full" />
    </div>
  );
}

export function OfflinePanel({ message }: { message: string }) {
  return (
    <div className="rounded border border-destructive bg-destructive/10 px-4 py-4 font-mono">
      <p className="text-sm font-semibold tracking-widest text-destructive uppercase">
        BACKEND OFFLINE
      </p>
      <p className="text-sm text-muted-foreground mt-1">
        Start the API on :8000 then reload.
      </p>
      {message && (
        <p className="mt-2 text-xs text-muted-foreground break-all">
          {message}
        </p>
      )}
    </div>
  );
}
