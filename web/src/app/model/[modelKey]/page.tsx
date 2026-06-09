"use client";

/**
 * /model/[modelKey] — Parameter-effect analysis + combination table for one model.
 */

import { useEffect, useState, useMemo } from "react";
import dynamic from "next/dynamic";
import Link from "next/link";
import { useParams, useRouter } from "next/navigation";
import { getModelGrid } from "@/lib/api";
import type { ModelGrid, ModelGridScenario } from "@/lib/types";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { Skeleton } from "@/components/ui/skeleton";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from "@/components/ui/table";
import { cn } from "@/lib/utils";
import type { ParameterEffectChartProps, EffectPoint } from "@/components/viz/ParameterEffectChart";

// ─── Dynamic (SSR-off) chart import ──────────────────────────────────────────

const ParameterEffectChart = dynamic<ParameterEffectChartProps>(
  () => import("@/components/viz/ParameterEffectChart"),
  { ssr: false, loading: () => <ChartSkeleton /> }
);

// ─── Small utility components (defined at module scope) ───────────────────────

function ChartSkeleton() {
  return <Skeleton className="h-48 w-full rounded" />;
}

function PageSkeleton() {
  return (
    <div className="flex flex-col gap-4">
      <Skeleton className="h-8 w-64" />
      <Skeleton className="h-4 w-40" />
      <div className="flex gap-2">
        <Skeleton className="h-6 w-16" />
        <Skeleton className="h-6 w-24" />
      </div>
      <Skeleton className="h-48 w-full" />
    </div>
  );
}

function OfflinePanel({ message }: { message: string }) {
  return (
    <div className="rounded border border-destructive bg-destructive/10 px-4 py-4 font-mono">
      <p className="text-sm font-semibold tracking-widest text-destructive uppercase">
        BACKEND OFFLINE / MODEL NOT FOUND
      </p>
      <p className="text-sm text-muted-foreground mt-1">
        Check the API on :8000 or verify the model key.
      </p>
      {message && (
        <p className="mt-2 text-xs text-muted-foreground/70 break-all">
          {message}
        </p>
      )}
    </div>
  );
}

// ─── Sweep parameter type ─────────────────────────────────────────────────────

type SweepParam = "drones" | "comm_range" | "n_visits";

const SWEEP_LABELS: Record<SweepParam, string> = {
  drones: "Number of Drones",
  comm_range: "Comm Range",
  n_visits: "n_visits",
};

// ─── Helper: collect unique values of a field ─────────────────────────────────

function uniqueDrones(scenarios: ModelGridScenario[]): number[] {
  const values: number[] = [];
  for (const s of scenarios) {
    if (s.number_of_drones != null) values.push(s.number_of_drones);
  }
  return Array.from(new Set(values)).sort((a, b) => a - b);
}

function uniqueCommRanges(scenarios: ModelGridScenario[]): string[] {
  const seen = new Map<string, number>();
  for (const s of scenarios) {
    if (s.comm_range != null && s.comm_range_value != null) {
      seen.set(s.comm_range, s.comm_range_value);
    }
  }
  return Array.from(seen.entries())
    .sort((a, b) => a[1] - b[1])
    .map(([k]) => k);
}

function uniqueNVisits(scenarios: ModelGridScenario[]): (number | null)[] {
  return Array.from(new Set(scenarios.map((s) => s.n_visits))).sort((a, b) => {
    if (a == null) return 1;
    if (b == null) return -1;
    return a - b;
  });
}

// ─── Format objective value ───────────────────────────────────────────────────

function fmtObj(v: number | null | undefined): string {
  if (v == null) return "—";
  return v.toFixed(2);
}

// Max Mean TBV (mean time between visits) is undefined when each cell is
// visited only once — there is no interval "between visits", so the optimiser
// reports a spurious 0.00. Exclude/blank it for n_visits === 1 so it doesn't
// distort the combination table or the parameter-effect plots.
function tbvMeaningless(objective: string, nVisits: number | null): boolean {
  return objective.includes("TBV") && nVisits === 1;
}

// ─── Plain-language model description ────────────────────────────────────────

function modelDescription(grid: ModelGrid): string {
  const objList = grid.objectives.join(" and ");
  const algo = grid.algorithm ? ` using ${grid.algorithm}` : "";
  if (grid.type === "MOO") {
    return `Multi-objective optimisation${algo} — simultaneously optimises ${objList}.`;
  }
  if (grid.type === "WS") {
    return `Weighted-sum scalarisation${algo} — balances ${objList} via a scalar weight.`;
  }
  if (grid.type === "SOO") {
    return `Single-objective optimisation${algo} — minimises/maximises ${objList}.`;
  }
  return `Optimises ${objList}${algo}.`;
}

// ─── Sweep controls ───────────────────────────────────────────────────────────

interface SweepControlsProps {
  grid: ModelGrid;
  sweep: SweepParam;
  onSweep: (s: SweepParam) => void;
  fixedDrones: string;
  onFixedDrones: (v: string) => void;
  fixedComm: string;
  onFixedComm: (v: string) => void;
  fixedNVisits: string;
  onFixedNVisits: (v: string) => void;
}

function SweepControls({
  grid,
  sweep,
  onSweep,
  fixedDrones,
  onFixedDrones,
  fixedComm,
  onFixedComm,
  fixedNVisits,
  onFixedNVisits,
}: SweepControlsProps) {
  const allDrones = uniqueDrones(grid.scenarios);
  const allComms = uniqueCommRanges(grid.scenarios);
  const allNVisits = uniqueNVisits(grid.scenarios);
  const hasNVisits = allNVisits.some((v) => v != null);

  return (
    <div className="flex flex-wrap items-center gap-3">
      {/* Sweep selector */}
      <div className="flex items-center gap-2">
        <span className="text-xs font-mono tracking-widest text-muted-foreground uppercase">
          Sweep
        </span>
        <Select
          value={sweep}
          onValueChange={(v) => onSweep(v as SweepParam)}
        >
          <SelectTrigger className="h-7 w-36 text-xs font-mono">
            <SelectValue />
          </SelectTrigger>
          <SelectContent>
            <SelectItem value="drones" className="text-xs font-mono">
              Drones
            </SelectItem>
            <SelectItem value="comm_range" className="text-xs font-mono">
              Comm Range
            </SelectItem>
            {hasNVisits && (
              <SelectItem value="n_visits" className="text-xs font-mono">
                n_visits
              </SelectItem>
            )}
          </SelectContent>
        </Select>
      </div>

      {/* Fixed: Drones (when not sweeping) */}
      {sweep !== "drones" && allDrones.length > 1 && (
        <div className="flex items-center gap-2">
          <span className="text-xs font-mono tracking-widest text-muted-foreground uppercase">
            Drones
          </span>
          <Select value={fixedDrones} onValueChange={onFixedDrones}>
            <SelectTrigger className="h-7 w-24 text-xs font-mono">
              <SelectValue />
            </SelectTrigger>
            <SelectContent>
              {allDrones.map((d) => (
                <SelectItem key={d} value={String(d)} className="text-xs font-mono">
                  {d}
                </SelectItem>
              ))}
            </SelectContent>
          </Select>
        </div>
      )}

      {/* Fixed: Comm range (when not sweeping) */}
      {sweep !== "comm_range" && allComms.length > 1 && (
        <div className="flex items-center gap-2">
          <span className="text-xs font-mono tracking-widest text-muted-foreground uppercase">
            Comm
          </span>
          <Select value={fixedComm} onValueChange={onFixedComm}>
            <SelectTrigger className="h-7 w-28 text-xs font-mono">
              <SelectValue />
            </SelectTrigger>
            <SelectContent>
              {allComms.map((c) => (
                <SelectItem key={c} value={c} className="text-xs font-mono">
                  {c}
                </SelectItem>
              ))}
            </SelectContent>
          </Select>
        </div>
      )}

      {/* Fixed: n_visits (when not sweeping) */}
      {sweep !== "n_visits" && hasNVisits && allNVisits.length > 1 && (
        <div className="flex items-center gap-2">
          <span className="text-xs font-mono tracking-widest text-muted-foreground uppercase">
            n_visits
          </span>
          <Select value={fixedNVisits} onValueChange={onFixedNVisits}>
            <SelectTrigger className="h-7 w-24 text-xs font-mono">
              <SelectValue />
            </SelectTrigger>
            <SelectContent>
              {allNVisits.map((v) => (
                <SelectItem
                  key={String(v)}
                  value={String(v)}
                  className="text-xs font-mono"
                >
                  {v ?? "—"}
                </SelectItem>
              ))}
            </SelectContent>
          </Select>
        </div>
      )}
    </div>
  );
}

// ─── Page ─────────────────────────────────────────────────────────────────────

export default function ModelPage() {
  const params = useParams();
  const router = useRouter();
  const rawKey = params?.modelKey;
  const modelKey = decodeURIComponent(
    Array.isArray(rawKey) ? (rawKey[0] ?? "") : (rawKey ?? "")
  );

  const [grid, setGrid] = useState<ModelGrid | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  // Sweep state
  const [sweep, setSweep] = useState<SweepParam>("drones");
  const [fixedDrones, setFixedDrones] = useState<string>("");
  const [fixedComm, setFixedComm] = useState<string>("");
  const [fixedNVisits, setFixedNVisits] = useState<string>("");

  useEffect(() => {
    if (!modelKey) return;
    let cancelled = false;
    setLoading(true);
    setError(null);

    getModelGrid(modelKey)
      .then((data) => {
        if (!cancelled) {
          setGrid(data);
          // Set default fixed values from the first scenario with non-null fields
          const first = data.scenarios[0];
          if (first) {
            setFixedDrones(first.number_of_drones != null ? String(first.number_of_drones) : "");
            setFixedComm(first.comm_range ?? "");
            setFixedNVisits(first.n_visits != null ? String(first.n_visits) : "");
          }
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
  }, [modelKey]);

  // Filter scenarios for the current sweep + fixed values
  const sweepScenarios = useMemo(() => {
    if (!grid) return [];
    let filtered = grid.scenarios;

    // When a dimension is being held fixed, exclude scenarios where that
    // dimension is null (can't match any fixed value) and only keep those
    // whose value matches the chosen fixed value.
    if (sweep !== "drones") {
      // Always exclude null-drones when drones is a fixed dimension
      filtered = filtered.filter((s) => s.number_of_drones != null);
      if (fixedDrones) {
        filtered = filtered.filter(
          (s) => String(s.number_of_drones) === fixedDrones
        );
      }
    }
    if (sweep !== "comm_range") {
      // Always exclude null-comm_range when comm_range is a fixed dimension
      filtered = filtered.filter((s) => s.comm_range != null);
      if (fixedComm) {
        filtered = filtered.filter((s) => s.comm_range === fixedComm);
      }
    }
    if (sweep !== "n_visits" && fixedNVisits !== "") {
      filtered = filtered.filter(
        (s) => String(s.n_visits ?? "") === fixedNVisits
      );
    }

    // When sweeping a dimension, exclude scenarios where that dimension is
    // null — they have no meaningful x position on the chart.
    if (sweep === "drones") {
      filtered = filtered.filter((s) => s.number_of_drones != null);
      filtered = [...filtered].sort(
        (a, b) => (a.number_of_drones as number) - (b.number_of_drones as number)
      );
    } else if (sweep === "comm_range") {
      filtered = filtered.filter((s) => s.comm_range_value != null);
      filtered = [...filtered].sort(
        (a, b) => (a.comm_range_value as number) - (b.comm_range_value as number)
      );
    } else {
      // n_visits sweep: null sorts last, then ascending
      filtered = [...filtered].sort((a, b) => {
        if (a.n_visits == null && b.n_visits == null) return 0;
        if (a.n_visits == null) return 1;
        if (b.n_visits == null) return -1;
        return a.n_visits - b.n_visits;
      });
    }
    return filtered;
  }, [grid, sweep, fixedDrones, fixedComm, fixedNVisits]);

  // Build EffectPoint[] for each objective
  const effectPointsByObj = useMemo((): Record<string, EffectPoint[]> => {
    if (!grid) return {};
    const result: Record<string, EffectPoint[]> = {};
    for (const obj of grid.objectives) {
      result[obj] = sweepScenarios
        .filter((s) => !tbvMeaningless(obj, s.n_visits))
        .map((s) => {
        const stats = s.objective_stats[obj];
        return {
          xLabel:
            sweep === "drones"
              ? (s.number_of_drones != null ? String(s.number_of_drones) : "—")
              : sweep === "comm_range"
              ? (s.comm_range ?? "—")
              : String(s.n_visits ?? "—"),
          xNum:
            sweep === "drones"
              ? (s.number_of_drones ?? 0)
              : sweep === "comm_range"
              ? (s.comm_range_value ?? 0)
              : (s.n_visits ?? 0),
          best: stats?.best ?? null,
          min: stats?.min ?? null,
          max: stats?.max ?? null,
        };
      });
    }
    return result;
  }, [grid, sweepScenarios, sweep]);

  if (!modelKey) return null;

  return (
    <div className="mx-auto flex max-w-7xl flex-col gap-6 px-4 py-6">
      {/* Back link */}
      <Link
        href="/"
        className="inline-flex items-center gap-1 text-xs font-mono tracking-widest text-muted-foreground hover:text-primary transition-colors uppercase"
      >
        ← MISSIONS
      </Link>

      {loading ? (
        <PageSkeleton />
      ) : error ? (
        <>
          <h1
            className="text-sm font-semibold tracking-widest uppercase text-primary font-display"
            style={{ fontFamily: "var(--font-display)" }}
          >
            {modelKey}
          </h1>
          <OfflinePanel message={error} />
        </>
      ) : grid ? (
        <>
          {/* Header */}
          <div className="flex flex-col gap-2">
            <h1
              className="text-lg font-semibold tracking-widest uppercase text-primary font-display"
              style={{ fontFamily: "var(--font-display)" }}
            >
              {grid.model_key}
            </h1>
            <div className="flex flex-wrap items-center gap-2">
              <Badge className="text-xs font-mono tracking-widest bg-secondary text-secondary-foreground">
                {grid.type}
              </Badge>
              {grid.algorithm && (
                <Badge
                  variant="outline"
                  className="text-xs font-mono tracking-widest"
                >
                  {grid.algorithm}
                </Badge>
              )}
              {grid.objectives.map((obj) => (
                <Badge
                  key={obj}
                  variant="outline"
                  className="text-xs tracking-wide"
                >
                  {obj}
                  {grid.polarities[obj] === -1 && (
                    <span className="ml-1 text-muted-foreground">(max)</span>
                  )}
                </Badge>
              ))}
            </div>
            <p className="text-xs text-muted-foreground font-mono max-w-xl">
              {modelDescription(grid)}
            </p>
          </div>

          {/* ── Parameter-effect analysis ─────────────────────────────────── */}
          <Card>
            <CardHeader>
              <CardTitle
                className="text-xs font-semibold tracking-widest uppercase text-primary font-display"
                style={{ fontFamily: "var(--font-display)" }}
              >
                PARAMETER-EFFECT ANALYSIS
              </CardTitle>
            </CardHeader>
            <CardContent className="flex flex-col gap-5">
              {/* Sweep controls */}
              <SweepControls
                grid={grid}
                sweep={sweep}
                onSweep={setSweep}
                fixedDrones={fixedDrones}
                onFixedDrones={setFixedDrones}
                fixedComm={fixedComm}
                onFixedComm={setFixedComm}
                fixedNVisits={fixedNVisits}
                onFixedNVisits={setFixedNVisits}
              />

              {/* One chart per objective, or a note if the filter yields nothing */}
              {sweepScenarios.length === 0 ? (
                <p className="text-xs font-mono text-muted-foreground border border-dashed border-border rounded px-4 py-3">
                  No data for this combination. Try a different fixed-parameter selection.
                </p>
              ) : (
                <div
                  className={cn(
                    "grid gap-6",
                    grid.objectives.length === 1
                      ? "grid-cols-1"
                      : grid.objectives.length === 2
                      ? "grid-cols-1 md:grid-cols-2"
                      : "grid-cols-1 md:grid-cols-2 xl:grid-cols-3"
                  )}
                >
                  {grid.objectives.map((obj, idx) => {
                    const pts = effectPointsByObj[obj] ?? [];
                    if (pts.length === 0) {
                      return (
                        <div key={obj} className="flex flex-col gap-1">
                          <p className="text-xs font-mono tracking-wide text-foreground">
                            {obj}
                          </p>
                          <p className="text-xs font-mono text-muted-foreground border border-dashed border-border rounded px-3 py-6 text-center">
                            {obj.includes("TBV")
                              ? "Requires n_visits > 1 — undefined when each cell is visited once."
                              : "No data for the current selection."}
                          </p>
                        </div>
                      );
                    }
                    return (
                      <ParameterEffectChart
                        key={obj}
                        objective={obj}
                        polarity={grid.polarities[obj] ?? 1}
                        sweepLabel={SWEEP_LABELS[sweep]}
                        points={pts}
                        colorIndex={idx}
                      />
                    );
                  })}
                </div>
              )}

              {/* Caption */}
              <p className="text-xs text-muted-foreground font-mono">
                Each point is the best achievable value of that objective for
                the given parameters.
                {sweepScenarios.length > 0
                  ? ` Showing ${sweepScenarios.length} combination${sweepScenarios.length !== 1 ? "s" : ""}.`
                  : " No combinations match the current filter."}
              </p>
            </CardContent>
          </Card>

          {/* ── Parameter-combination table ───────────────────────────────── */}
          <div className="flex flex-col gap-3">
            <div>
              <h2
                className="text-xs font-semibold tracking-widest uppercase text-primary font-display"
                style={{ fontFamily: "var(--font-display)" }}
              >
                ALL PARAMETER COMBINATIONS
              </h2>
              <p className="text-xs text-muted-foreground font-mono mt-0.5">
                Click a row to open the full Pareto / merging / animation analysis.
              </p>
            </div>

            <div className="rounded border border-border overflow-hidden">
              <Table>
                <TableHeader>
                  <TableRow>
                    <TableHead className="text-xs font-mono tracking-widest uppercase text-muted-foreground">
                      Drones
                    </TableHead>
                    <TableHead className="text-xs font-mono tracking-widest uppercase text-muted-foreground">
                      Comm Range
                    </TableHead>
                    <TableHead className="text-xs font-mono tracking-widest uppercase text-muted-foreground">
                      n_visits
                    </TableHead>
                    <TableHead className="text-xs font-mono tracking-widest uppercase text-muted-foreground">
                      # Solutions
                    </TableHead>
                    {grid.objectives.map((obj) => (
                      <TableHead
                        key={obj}
                        className="text-xs font-mono tracking-widest uppercase text-muted-foreground"
                      >
                        {obj}
                        <span className="ml-1 text-muted-foreground/60 normal-case tracking-normal font-normal">
                          (best)
                        </span>
                      </TableHead>
                    ))}
                  </TableRow>
                </TableHeader>
                <TableBody>
                  {grid.scenarios.map((s) => (
                    <TableRow
                      key={s.scenario}
                      onClick={() =>
                        router.push(
                          "/explore/" + encodeURIComponent(s.scenario)
                        )
                      }
                      className="cursor-pointer hover:bg-primary/5 transition-colors"
                      role="button"
                      tabIndex={0}
                      onKeyDown={(e) => {
                        if (e.key === "Enter" || e.key === " ") {
                          router.push(
                            "/explore/" + encodeURIComponent(s.scenario)
                          );
                        }
                      }}
                      aria-label={`Open scenario ${s.scenario}`}
                    >
                      <TableCell className="font-mono text-xs tabular-nums">
                        {s.number_of_drones}
                      </TableCell>
                      <TableCell className="font-mono text-xs">
                        {s.comm_range}
                      </TableCell>
                      <TableCell className="font-mono text-xs tabular-nums">
                        {s.n_visits ?? "—"}
                      </TableCell>
                      <TableCell className="font-mono text-xs tabular-nums text-accent">
                        {s.n_solutions}
                      </TableCell>
                      {grid.objectives.map((obj) => (
                        <TableCell
                          key={obj}
                          className="font-mono text-xs tabular-nums"
                        >
                          {tbvMeaningless(obj, s.n_visits)
                            ? "—"
                            : fmtObj(s.objective_stats[obj]?.best)}
                        </TableCell>
                      ))}
                    </TableRow>
                  ))}
                </TableBody>
              </Table>
            </div>
          </div>
        </>
      ) : null}
    </div>
  );
}
