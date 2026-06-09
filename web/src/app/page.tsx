"use client";

import { useEffect, useState } from "react";
import { getLibrary } from "@/lib/api";
import type { ScenarioSummary } from "@/lib/types";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { Skeleton } from "@/components/ui/skeleton";
import { Input } from "@/components/ui/input";
import { cn } from "@/lib/utils";

// ─── Loading skeleton ─────────────────────────────────────────────────────────

function MissionCardSkeleton() {
  return (
    <div className="flex flex-col gap-3 rounded border border-border bg-card p-4">
      <Skeleton className="h-5 w-40" />
      <Skeleton className="h-4 w-full" />
      <Skeleton className="h-4 w-3/4" />
      <div className="flex gap-2">
        <Skeleton className="h-5 w-16" />
        <Skeleton className="h-5 w-16" />
      </div>
    </div>
  );
}

// ─── Single mission card ──────────────────────────────────────────────────────

function MissionCard({ s }: { s: ScenarioSummary }) {
  return (
    <Card
      className={cn(
        "cursor-pointer transition-all duration-150",
        "hover:ring-1 hover:ring-primary hover:shadow-[0_0_14px_hsl(var(--primary)/0.3)]",
        !s.has_solutions && "opacity-60"
      )}
    >
      <CardHeader>
        <CardTitle
          className="text-sm font-semibold tracking-widest uppercase text-primary truncate font-display"
          style={{ fontFamily: "var(--font-display)" }}
        >
          {s.model_key}
        </CardTitle>
      </CardHeader>

      <CardContent className="flex flex-col gap-2">
        {/* Objectives as badges */}
        <div className="flex flex-wrap gap-1">
          {s.objectives.map((obj) => (
            <Badge key={obj} variant="outline" className="text-xs tracking-wide">
              {obj}
            </Badge>
          ))}
        </div>

        {/* Tactical mono readouts */}
        <dl className="grid grid-cols-2 gap-x-4 gap-y-0.5 text-xs font-mono tabular-nums">
          {s.grid_size != null && (
            <>
              <dt className="text-muted-foreground/70">GRID</dt>
              <dd className="text-foreground">
                {s.grid_size}×{s.grid_size}
              </dd>
            </>
          )}
          {s.number_of_drones != null && (
            <>
              <dt className="text-muted-foreground/70">DRONES</dt>
              <dd className="text-foreground">{s.number_of_drones}</dd>
            </>
          )}
          {s.comm_range != null && (
            <>
              <dt className="text-muted-foreground/70">COMM</dt>
              <dd className="text-foreground">{s.comm_range}</dd>
            </>
          )}
          <dt className="text-muted-foreground/70">SOLUTIONS</dt>
          <dd
            className={cn(
              "font-semibold",
              s.has_solutions ? "text-accent" : "text-muted-foreground"
            )}
          >
            {s.n_solutions}
          </dd>
          <dt className="text-muted-foreground/70">KIND</dt>
          <dd className="text-foreground uppercase tracking-wide">
            {s.result_kind}
          </dd>
          <dt className="text-muted-foreground/70">TYPE</dt>
          <dd className="text-foreground uppercase tracking-wide">{s.type}</dd>
        </dl>

        {/* Scenario ID */}
        <p className="mt-1 text-xs text-muted-foreground/60 font-mono truncate">
          {s.scenario}
        </p>
      </CardContent>
    </Card>
  );
}

// ─── Offline / error banner ───────────────────────────────────────────────────

function OfflineBanner({ message }: { message: string }) {
  return (
    <div className="rounded border border-destructive bg-destructive/10 px-4 py-3 font-mono">
      <span className="text-sm font-semibold tracking-wide text-destructive">
        BACKEND OFFLINE
      </span>
      <span className="text-sm text-muted-foreground">
        {" — "}start the API on :8000
      </span>
      {message && (
        <p className="mt-1 text-xs text-muted-foreground/70 truncate">
          {message}
        </p>
      )}
    </div>
  );
}

// ─── Page ─────────────────────────────────────────────────────────────────────

export default function MissionSelectPage() {
  const [scenarios, setScenarios] = useState<ScenarioSummary[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [query, setQuery] = useState("");

  useEffect(() => {
    let cancelled = false;
    setLoading(true);
    setError(null);

    getLibrary()
      .then((data) => {
        if (!cancelled) {
          setScenarios(data);
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
  }, []);

  const filtered = scenarios.filter((s) => {
    if (!query.trim()) return true;
    const q = query.toLowerCase();
    return (
      s.scenario.toLowerCase().includes(q) ||
      s.model_key.toLowerCase().includes(q) ||
      s.type.toLowerCase().includes(q) ||
      s.objectives.some((o) => o.toLowerCase().includes(q))
    );
  });

  return (
    <div className="mx-auto flex max-w-7xl flex-col gap-6 px-4 py-6">
      {/* Page header */}
      <div className="flex flex-col gap-1">
        <h1
          className="text-lg font-semibold tracking-widest uppercase text-primary"
          style={{ fontFamily: "var(--font-display)" }}
        >
          MISSION SELECT
        </h1>
        <p className="text-xs tracking-wide text-muted-foreground">
          SELECT A PRECOMPUTED SCENARIO TO INSPECT ITS PARETO FRONT AND REPLAY
          SENSING
        </p>
      </div>

      {/* Filter input */}
      <div className="max-w-sm">
        <Input
          type="search"
          placeholder="FILTER MISSIONS…"
          value={query}
          onChange={(e) => setQuery(e.target.value)}
          className="text-xs tracking-widest uppercase placeholder:tracking-widest"
        />
      </div>

      {/* Error state */}
      {error && <OfflineBanner message={error} />}

      {/* Loading skeletons */}
      {loading && !error && (
        <div className="grid grid-cols-1 gap-4 sm:grid-cols-2 lg:grid-cols-3 xl:grid-cols-4">
          {Array.from({ length: 8 }).map((_, i) => (
            <MissionCardSkeleton key={i} />
          ))}
        </div>
      )}

      {/* Empty filter result */}
      {!loading && !error && filtered.length === 0 && (
        <p className="font-mono text-sm tracking-wide text-muted-foreground">
          NO MISSIONS MATCH FILTER.
        </p>
      )}

      {/* Mission cards */}
      {!loading && !error && filtered.length > 0 && (
        <>
          <p className="font-mono text-xs tabular-nums text-muted-foreground">
            {filtered.length}/{scenarios.length} MISSIONS
          </p>
          <div className="grid grid-cols-1 gap-4 sm:grid-cols-2 lg:grid-cols-3 xl:grid-cols-4">
            {filtered.map((s) => (
              <MissionCard key={`${s.scenario}::${s.model_key}`} s={s} />
            ))}
          </div>
        </>
      )}
    </div>
  );
}
