"use client";

import { useEffect, useState, useMemo } from "react";
import {
  useInitialSearchParams,
  useUrlSync,
  readString,
  scalarParam,
} from "@/hooks/useUrlState";
import Link from "next/link";
import { getLibrary } from "@/lib/api";
import type { ScenarioSummary } from "@/lib/types";
import { Skeleton } from "@/components/ui/skeleton";
import { cn } from "@/lib/utils";

// ─── Icons ────────────────────────────────────────────────────────────────────

function SearchIcon({ className }: { className?: string }) {
  return (
    <svg viewBox="0 0 24 24" className={className} fill="none" stroke="currentColor" strokeWidth={1.8} strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
      <circle cx="11" cy="11" r="7" />
      <line x1="21" y1="21" x2="16.65" y2="16.65" />
    </svg>
  );
}

function ArrowIcon({ className }: { className?: string }) {
  return (
    <svg viewBox="0 0 24 24" className={className} fill="none" stroke="currentColor" strokeWidth={1.8} strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
      <line x1="5" y1="12" x2="19" y2="12" />
      <polyline points="12 5 19 12 12 19" />
    </svg>
  );
}

// ─── Model group (derived from ScenarioSummary[]) ────────────────────────────

interface ModelGroup {
  model_key: string;
  type: string;
  algorithm: string;
  objectives: string[];
  scenarios: ScenarioSummary[];
  droneRange: [number, number] | null;
  nVisitsRange: [number, number] | null;
  combinationCount: number;
}

function buildModelGroups(scenarios: ScenarioSummary[]): ModelGroup[] {
  const map = new Map<string, ScenarioSummary[]>();
  for (const s of scenarios) {
    const arr = map.get(s.model_key) ?? [];
    arr.push(s);
    map.set(s.model_key, arr);
  }

  return Array.from(map.entries()).map(([model_key, items]) => {
    const first = items[0]!;
    const drones = items
      .map((s) => s.number_of_drones)
      .filter((d): d is number => d != null);
    const nVisits = items
      .map((s) => s.variant_value)
      .filter((v): v is number => v != null);

    return {
      model_key,
      type: first.type,
      algorithm: first.algorithm,
      objectives: first.objectives,
      scenarios: items,
      droneRange: drones.length > 0 ? [Math.min(...drones), Math.max(...drones)] : null,
      nVisitsRange: nVisits.length > 0 ? [Math.min(...nVisits), Math.max(...nVisits)] : null,
      combinationCount: items.length,
    };
  });
}

// ─── Pieces ───────────────────────────────────────────────────────────────────

function ModelCardSkeleton() {
  return (
    <div className="flex flex-col gap-3 rounded-xl border border-border bg-card p-5">
      <Skeleton className="h-5 w-40" />
      <div className="flex gap-1.5">
        <Skeleton className="h-5 w-12 rounded-full" />
        <Skeleton className="h-5 w-24 rounded-full" />
        <Skeleton className="h-5 w-20 rounded-full" />
      </div>
      <Skeleton className="h-4 w-3/4" />
    </div>
  );
}

function OfflineBanner({ message }: { message: string }) {
  return (
    <div className="rounded-xl border border-destructive/30 bg-destructive/5 px-4 py-3">
      <p className="text-sm font-medium text-destructive">Backend offline</p>
      <p className="text-sm text-muted-foreground">
        Start the API on port 8000, then reload.
      </p>
      {message && (
        <p className="mt-1 truncate text-xs text-muted-foreground">{message}</p>
      )}
    </div>
  );
}

function ModelCard({ group }: { group: ModelGroup }) {
  const summaryParts: string[] = [
    `${group.combinationCount} combination${group.combinationCount !== 1 ? "s" : ""}`,
  ];
  if (group.droneRange) {
    const [lo, hi] = group.droneRange;
    summaryParts.push(lo === hi ? `${lo} drone${lo !== 1 ? "s" : ""}` : `${lo}–${hi} drones`);
  }
  if (group.nVisitsRange) {
    const [lo, hi] = group.nVisitsRange;
    summaryParts.push(lo === hi ? `n_visits ${lo}` : `n_visits ${lo}–${hi}`);
  }

  return (
    <Link
      href={"/missions/" + encodeURIComponent(group.model_key)}
      className="group flex flex-col gap-3 rounded-xl border border-border bg-card p-5 transition-all duration-200 hover:-translate-y-0.5 hover:border-foreground/20 hover:shadow-md hover:shadow-foreground/5"
    >
      <div className="flex items-start justify-between gap-2">
        <h3 className="font-semibold tracking-tight text-foreground">
          {group.model_key}
        </h3>
        <ArrowIcon className="size-4 shrink-0 text-muted-foreground transition-all duration-200 group-hover:translate-x-0.5 group-hover:text-foreground" />
      </div>

      <div className="flex flex-wrap gap-1.5">
        <span className="rounded-full bg-chart-1/10 px-2.5 py-0.5 text-xs font-medium text-chart-1">
          {group.type}
        </span>
        {group.objectives.map((obj) => (
          <span
            key={obj}
            className="rounded-full bg-muted px-2.5 py-0.5 text-xs font-medium text-muted-foreground"
          >
            {obj}
          </span>
        ))}
      </div>

      <p className="text-sm text-muted-foreground">{summaryParts.join(" · ")}</p>
    </Link>
  );
}

// ─── Page ─────────────────────────────────────────────────────────────────────

export default function MissionSelectPage() {
  const [scenarios, setScenarios] = useState<ScenarioSummary[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  // Search term round-trips through ?q= so a filtered list can be shared and
  // survives a reload.
  const initialParams = useInitialSearchParams();
  const [query, setQuery] = useState(() => readString(initialParams, "q", ""));
  useUrlSync({ q: scalarParam(query.trim(), "") });

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

  const allGroups = useMemo(() => buildModelGroups(scenarios), [scenarios]);

  const filteredGroups = useMemo(() => {
    if (!query.trim()) return allGroups;
    const q = query.toLowerCase();
    return allGroups.filter(
      (g) =>
        g.model_key.toLowerCase().includes(q) ||
        g.type.toLowerCase().includes(q) ||
        g.objectives.some((o) => o.toLowerCase().includes(q))
    );
  }, [allGroups, query]);

  return (
    <div className="mx-auto flex max-w-6xl flex-col gap-8 px-6 py-10">
      {/* Header */}
      <div className="flex flex-col gap-1.5">
        <h1 className="text-2xl font-bold tracking-tight text-foreground">
          Mission Explorer
        </h1>
        <p className="text-[15px] text-muted-foreground">
          Pick a model to browse its parameter combinations and objective-effect
          analysis.
        </p>
      </div>

      {/* Search + count */}
      <div className="flex flex-wrap items-center justify-between gap-4">
        <div className="relative w-full max-w-sm">
          <SearchIcon className="pointer-events-none absolute left-3 top-1/2 size-4 -translate-y-1/2 text-muted-foreground" />
          <input
            type="search"
            placeholder="Search models, objectives…"
            value={query}
            onChange={(e) => setQuery(e.target.value)}
            className={cn(
              "h-10 w-full rounded-lg border border-input bg-background pl-9 pr-3 text-sm",
              "placeholder:text-muted-foreground focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-ring/60"
            )}
          />
        </div>
        {!loading && !error && (
          <p className="text-sm text-muted-foreground tabular-nums">
            {filteredGroups.length} of {allGroups.length} models ·{" "}
            {scenarios.length} combinations
          </p>
        )}
      </div>

      {error && <OfflineBanner message={error} />}

      {loading && !error && (
        <div className="grid grid-cols-1 gap-4 sm:grid-cols-2 lg:grid-cols-3">
          {Array.from({ length: 6 }).map((_, i) => (
            <ModelCardSkeleton key={i} />
          ))}
        </div>
      )}

      {!loading && !error && filteredGroups.length === 0 && (
        <p className="text-sm text-muted-foreground">No models match your search.</p>
      )}

      {!loading && !error && filteredGroups.length > 0 && (
        <div className="grid grid-cols-1 gap-4 sm:grid-cols-2 lg:grid-cols-3">
          {filteredGroups.map((g) => (
            <ModelCard key={g.model_key} group={g} />
          ))}
        </div>
      )}
    </div>
  );
}
