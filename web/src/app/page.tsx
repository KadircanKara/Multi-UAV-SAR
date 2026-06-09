"use client";

import { useEffect, useState, useMemo } from "react";
import { useRouter } from "next/navigation";
import { getLibrary } from "@/lib/api";
import type { ScenarioSummary } from "@/lib/types";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { Skeleton } from "@/components/ui/skeleton";
import { Input } from "@/components/ui/input";
import { cn } from "@/lib/utils";

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
      droneRange:
        drones.length > 0
          ? [Math.min(...drones), Math.max(...drones)]
          : null,
      nVisitsRange:
        nVisits.length > 0
          ? [Math.min(...nVisits), Math.max(...nVisits)]
          : null,
      combinationCount: items.length,
    };
  });
}

// ─── Loading skeleton ─────────────────────────────────────────────────────────

function ModelCardSkeleton() {
  return (
    <div className="flex flex-col gap-3 rounded border border-border bg-card p-4">
      <Skeleton className="h-5 w-40" />
      <div className="flex gap-2">
        <Skeleton className="h-5 w-12" />
        <Skeleton className="h-5 w-20" />
        <Skeleton className="h-5 w-20" />
      </div>
      <Skeleton className="h-4 w-full" />
      <Skeleton className="h-4 w-3/4" />
    </div>
  );
}

// ─── Single model card ────────────────────────────────────────────────────────

function ModelCard({ group }: { group: ModelGroup }) {
  const router = useRouter();

  function handleClick() {
    router.push("/model/" + encodeURIComponent(group.model_key));
  }

  const summaryParts: string[] = [
    `${group.combinationCount} parameter combination${group.combinationCount !== 1 ? "s" : ""}`,
  ];
  if (group.droneRange) {
    const [lo, hi] = group.droneRange;
    summaryParts.push(lo === hi ? `${lo} drone${lo !== 1 ? "s" : ""}` : `drones ${lo}–${hi}`);
  }
  if (group.nVisitsRange) {
    const [lo, hi] = group.nVisitsRange;
    summaryParts.push(lo === hi ? `n_visits ${lo}` : `n_visits ${lo}–${hi}`);
  }

  return (
    <Card
      onClick={handleClick}
      role="button"
      tabIndex={0}
      onKeyDown={(e) => {
        if (e.key === "Enter" || e.key === " ") handleClick();
      }}
      aria-label={`Open model ${group.model_key}`}
      className={cn(
        "cursor-pointer transition-all duration-150",
        "hover:ring-1 hover:ring-primary hover:shadow-[0_0_14px_hsl(var(--primary)/0.3)]"
      )}
    >
      <CardHeader>
        <CardTitle
          className="text-sm font-semibold tracking-widest uppercase text-primary truncate font-display"
          style={{ fontFamily: "var(--font-display)" }}
        >
          {group.model_key}
        </CardTitle>
      </CardHeader>

      <CardContent className="flex flex-col gap-3">
        {/* Type + objectives badges */}
        <div className="flex flex-wrap gap-1">
          <Badge className="text-xs font-mono tracking-widest bg-secondary text-secondary-foreground">
            {group.type}
          </Badge>
          {group.objectives.map((obj) => (
            <Badge key={obj} variant="outline" className="text-xs tracking-wide">
              {obj}
            </Badge>
          ))}
        </div>

        {/* Summary line */}
        <p className="text-xs font-mono text-muted-foreground">
          {summaryParts.join(" · ")}
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
          SELECT A MODEL TO BROWSE ITS PARAMETER COMBINATIONS AND OBJECTIVE-EFFECT ANALYSIS
        </p>
      </div>

      {/* Filter input */}
      <div className="max-w-sm">
        <Input
          type="search"
          placeholder="FILTER MODELS…"
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
            <ModelCardSkeleton key={i} />
          ))}
        </div>
      )}

      {/* Empty filter result */}
      {!loading && !error && filteredGroups.length === 0 && (
        <p className="font-mono text-sm tracking-wide text-muted-foreground">
          NO MODELS MATCH FILTER.
        </p>
      )}

      {/* Model cards */}
      {!loading && !error && filteredGroups.length > 0 && (
        <>
          <p className="font-mono text-xs tabular-nums text-muted-foreground">
            {filteredGroups.length}/{allGroups.length} MODELS · {scenarios.length} TOTAL COMBINATIONS
          </p>
          <div className="grid grid-cols-1 gap-4 sm:grid-cols-2 lg:grid-cols-3 xl:grid-cols-4">
            {filteredGroups.map((g) => (
              <ModelCard key={g.model_key} group={g} />
            ))}
          </div>
        </>
      )}
    </div>
  );
}
