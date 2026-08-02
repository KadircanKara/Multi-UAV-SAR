"use client";

/**
 * /missions/[modelKey] — Parameter-effect analysis and a scenario deep-dive
 * for one model, merged into ONE SectionPanelLayout (one sticky panel, one
 * scrollspy) instead of this page's own card plus an embedded ScenarioExplorer
 * running a second one.
 *
 * Section order: parameter-effect (this file's own sweep charts) → pareto /
 * merging / animation (useScenarioSections, called directly — no
 * <ScenarioExplorer> wrapper) → all-combinations (the full table; click a row
 * to pick that combination and scroll up to Pareto). The combination picker
 * (dependent Drones/Comm/n_visits dropdowns) is passed to the hook as
 * `combinationControls` and renders inside the Pareto section's panel.
 *
 * useScenarioSections is called once, here, for the page's lifetime: a
 * combination change updates its `source` prop rather than remounting a keyed
 * <ScenarioExplorer> subtree. That old wrapper key was doing real work — three
 * things below it (SolutionSelectorPanel's seeded weights, MergingContent's
 * fetched comparison, GridPlayback's fetched replay) cache per-run state with
 * no effect that clears it — but the replacement belongs INSIDE the hook,
 * keyed on the run, not out here: keying from this file would take the
 * parameter-effect charts, the combination table and the scrollspy's
 * seen/active state with it on every combination change.
 */

import { useEffect, useState, useMemo, useCallback } from "react";
import dynamic from "next/dynamic";
import Link from "next/link";
import { useParams } from "next/navigation";
import { getModelGrid } from "@/lib/api";
import type { ModelGrid, ModelGridScenario } from "@/lib/types";
import { useScenarioSections } from "@/components/explore/useScenarioSections";
import SectionPanelLayout from "@/components/layout/SectionPanelLayout";
import { useElementHeight } from "@/hooks/useElementHeight";
import type { PanelSection } from "@/components/layout/PanelSection";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import { toast } from "sonner";
import { Skeleton } from "@/components/ui/skeleton";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import { ToggleGroup, ToggleGroupItem } from "@/components/ui/toggle-group";
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from "@/components/ui/table";
import { cn } from "@/lib/utils";
import { commLabel } from "@/lib/comm";
import { isPercentObjective, percentString } from "@/lib/objective-format";
import type {
  ParameterEffectChartProps,
  EffectPoint,
  EffectSeries,
} from "@/components/viz/ParameterEffectChart";

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
        <p className="mt-2 text-xs text-muted-foreground break-all">
          {message}
        </p>
      )}
    </div>
  );
}

// ─── Parameter dimensions ─────────────────────────────────────────────────────

type SweepParam = "drones" | "comm_range" | "n_visits";

const SWEEP_LABELS: Record<SweepParam, string> = {
  drones: "Number of Drones",
  comm_range: "Comm Range",
  n_visits: "n_visits",
};

const DIM_SHORT: Record<SweepParam, string> = {
  drones: "Drones",
  comm_range: "Comm",
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

// String key for an n_visits value (null → ""), used to match a scenario across
// the three combination dropdowns.
function nvKey(v: number | null | undefined): string {
  return v == null ? "" : String(v);
}

// Find the scenario in this model with the exact (drones, comm, n_visits) combo.
function findScenario(
  scenarios: ModelGridScenario[],
  drones: number | string | null,
  comm: string | null,
  nvisitsKey: string
): ModelGridScenario | undefined {
  return scenarios.find(
    (s) =>
      String(s.number_of_drones) === String(drones) &&
      s.comm_range === comm &&
      nvKey(s.n_visits) === nvisitsKey
  );
}

// ─── Format objective value ───────────────────────────────────────────────────

function fmtObj(obj: string, v: number | null | undefined): string {
  if (v == null) return "—";
  if (isPercentObjective(obj)) return percentString(v);
  return v.toFixed(2);
}

// Max Mean TBV (mean time between visits) is undefined when each cell is
// visited only once — there is no interval "between visits", so the optimiser
// reports a spurious 0.00. Exclude/blank it for n_visits === 1 so it doesn't
// distort the combination table or the parameter-effect plots.
function tbvMeaningless(objective: string, nVisits: number | null): boolean {
  return objective.includes("TBV") && nVisits === 1;
}

// Fixed scenario params (grid, cell side, max speed) encoded in a scenario name:
// "..._g_8_a_50_n_4_v_2.5_r_2_nvisits_2" → grid 8, cell 50 m, max speed 2.5 m/s.
// These are constant for a given model, so they're shown as read-only context.
function scenarioParams(
  name: string
): { grid: number; cell: number; speed: number } | null {
  const m = /_g_(\d+)_a_([0-9.]+)_n_\d+_v_([0-9.]+)_/.exec(name);
  if (!m) return null;
  return { grid: Number(m[1]), cell: Number(m[2]), speed: Number(m[3]) };
}

// ─── Parameter-effect series construction ─────────────────────────────────────

function availableDimValues(
  grid: ModelGrid,
  dim: SweepParam
): { value: string; label: string }[] {
  if (dim === "drones") {
    return uniqueDrones(grid.scenarios).map((d) => ({ value: String(d), label: String(d) }));
  }
  if (dim === "comm_range") {
    return uniqueCommRanges(grid.scenarios).map((c) => ({
      value: c,
      label: commLabel(c),
    }));
  }
  return uniqueNVisits(grid.scenarios)
    .filter((v): v is number => v != null)
    .map((v) => ({ value: String(v), label: String(v) }));
}

function dimValueLabel(dim: SweepParam, value: string): string {
  if (dim === "drones") return `${value} drones`;
  if (dim === "comm_range") return commLabel(value, 50, { short: true });
  return `${value} visits`;
}

function matchDim(s: ModelGridScenario, dim: SweepParam, value: string): boolean {
  if (dim === "drones") return String(s.number_of_drones) === value;
  if (dim === "comm_range") return s.comm_range === value;
  return nvKey(s.n_visits) === value;
}

function sweepHasValue(s: ModelGridScenario, sweep: SweepParam): boolean {
  if (sweep === "drones") return s.number_of_drones != null;
  if (sweep === "comm_range") return s.comm_range_value != null;
  return s.n_visits != null;
}

function sweepNum(s: ModelGridScenario, sweep: SweepParam): number {
  if (sweep === "drones") return s.number_of_drones ?? 0;
  if (sweep === "comm_range") return s.comm_range_value ?? 0;
  return s.n_visits ?? 0;
}

// The selectable-value key of a scenario along the sweep dimension (matches the
// value strings produced by availableDimValues).
function sweepValueKey(s: ModelGridScenario, sweep: SweepParam): string {
  if (sweep === "drones") return String(s.number_of_drones);
  if (sweep === "comm_range") return s.comm_range ?? "";
  return nvKey(s.n_visits);
}

function pointForScenario(
  s: ModelGridScenario,
  sweep: SweepParam,
  obj: string
): EffectPoint {
  const stats = s.objective_stats[obj];
  return {
    xLabel:
      sweep === "drones"
        ? s.number_of_drones != null
          ? String(s.number_of_drones)
          : "—"
        : sweep === "comm_range"
        ? s.comm_range ?? "—"
        : String(s.n_visits ?? "—"),
    xNum: sweepNum(s, sweep),
    best: stats?.best ?? null,
    min: stats?.min ?? null,
    max: stats?.max ?? null,
  };
}

function cartesian<T>(arrays: T[][]): T[][] {
  return arrays.reduce<T[][]>(
    (acc, arr) => acc.flatMap((combo) => arr.map((x) => [...combo, x])),
    [[]]
  );
}

// Build one EffectSeries[] per objective. Each series is a line for one
// combination of the non-swept ("series") dimensions; the label names only the
// dimensions that actually vary (>1 selected value), so a single line has no
// label and the chart omits its legend.
function buildSeriesByObjective(
  grid: ModelGrid,
  sweep: SweepParam,
  selByDim: Record<SweepParam, string[]>,
  sweepValues: string[],
  objectives: string[]
): Record<string, EffectSeries[]> {
  const seriesDims = (["drones", "comm_range", "n_visits"] as SweepParam[])
    .filter((d) => d !== sweep && (selByDim[d]?.length ?? 0) > 0)
    .map((d) => ({ dim: d, values: selByDim[d]! }));

  const varying = new Set(
    seriesDims.filter((sd) => sd.values.length > 1).map((sd) => sd.dim)
  );

  const combos = cartesian(
    seriesDims.map((sd) => sd.values.map((v) => ({ dim: sd.dim, value: v })))
  );

  // Restrict the x-axis to the selected sweep values (empty ⇒ show all).
  const sweepSet = sweepValues.length > 0 ? new Set(sweepValues) : null;

  const result: Record<string, EffectSeries[]> = {};
  for (const obj of objectives) {
    const list: EffectSeries[] = [];
    for (const combo of combos) {
      const scs = grid.scenarios
        .filter((s) => combo.every((c) => matchDim(s, c.dim, c.value)))
        .filter((s) => sweepHasValue(s, sweep))
        .filter((s) => !sweepSet || sweepSet.has(sweepValueKey(s, sweep)))
        .filter((s) => !tbvMeaningless(obj, s.n_visits))
        .sort((a, b) => sweepNum(a, sweep) - sweepNum(b, sweep));
      const points = scs.map((s) => pointForScenario(s, sweep, obj));
      const label = combo
        .filter((c) => varying.has(c.dim))
        .map((c) => dimValueLabel(c.dim, c.value))
        .join(" · ");
      const key = combo.map((c) => `${c.dim}=${c.value}`).join("|") || "all";
      list.push({ key, label, points });
    }
    result[obj] = list;
  }
  return result;
}

// ─── Combination selector (dependent dropdowns) ───────────────────────────────

interface CombinationSelectProps {
  grid: ModelGrid;
  selected: ModelGridScenario | null;
  onSelectName: (name: string) => void;
}

function CombinationSelect({ grid, selected, onSelectName }: CombinationSelectProps) {
  if (!selected) return null;
  const scenarios = grid.scenarios;
  const hasNVisits = scenarios.some((s) => s.n_visits != null);

  // Each dropdown's options depend on the OTHER two current selections, so every
  // option always resolves to a real scenario (no dead-end combinations).
  const droneOptions = uniqueDrones(
    scenarios.filter(
      (s) =>
        s.comm_range === selected.comm_range &&
        nvKey(s.n_visits) === nvKey(selected.n_visits)
    )
  );
  const commOptions = uniqueCommRanges(
    scenarios.filter(
      (s) =>
        s.number_of_drones === selected.number_of_drones &&
        nvKey(s.n_visits) === nvKey(selected.n_visits)
    )
  );
  const nVisitsOptions = uniqueNVisits(
    scenarios.filter(
      (s) =>
        s.number_of_drones === selected.number_of_drones &&
        s.comm_range === selected.comm_range
    )
  ).filter((v): v is number => v != null);

  function selectDrones(d: string) {
    const sc = findScenario(scenarios, d, selected!.comm_range, nvKey(selected!.n_visits));
    if (sc) onSelectName(sc.scenario);
  }
  function selectComm(c: string) {
    const sc = findScenario(scenarios, selected!.number_of_drones, c, nvKey(selected!.n_visits));
    if (sc) onSelectName(sc.scenario);
  }
  function selectNVisits(k: string) {
    const sc = findScenario(scenarios, selected!.number_of_drones, selected!.comm_range, k);
    if (sc) onSelectName(sc.scenario);
  }

  return (
    // One row, not three stacked. Fixed widths (w-24/w-44) wrapped to three
    // rows inside the 360px panel and cost ~130px of the height the strategy
    // controls below need; the columns are proportional instead, with Comm
    // given the extra because its label carries both cells and metres.
    <div className="grid grid-cols-[1fr_1.75fr_1fr] items-end gap-2">
      {/* Drones */}
      <div className="flex min-w-0 flex-col gap-1.5">
        <span className="text-xs font-mono tracking-widest text-muted-foreground uppercase">
          Drones
        </span>
        <Select value={String(selected.number_of_drones)} onValueChange={selectDrones}>
          <SelectTrigger className="h-8 w-full min-w-0 text-xs font-mono">
            <SelectValue />
          </SelectTrigger>
          <SelectContent>
            {droneOptions.map((d) => (
              <SelectItem key={d} value={String(d)} className="text-xs font-mono">
                {d}
              </SelectItem>
            ))}
          </SelectContent>
        </Select>
      </div>

      {/* Comm range */}
      <div className="flex min-w-0 flex-col gap-1.5">
        <span className="text-xs font-mono tracking-widest text-muted-foreground uppercase">
          Comm
        </span>
        <Select value={selected.comm_range ?? ""} onValueChange={selectComm}>
          <SelectTrigger
            className="h-8 w-full min-w-0 text-xs font-mono"
            title={selected.comm_range ? commLabel(selected.comm_range) : undefined}
          >
            <SelectValue />
          </SelectTrigger>
          <SelectContent>
            {commOptions.map((c) => (
              <SelectItem key={c} value={c} className="text-xs font-mono">
                {commLabel(c)}
              </SelectItem>
            ))}
          </SelectContent>
        </Select>
      </div>

      {/* n_visits */}
      {hasNVisits && (
        <div className="flex min-w-0 flex-col gap-1.5">
          <span className="text-xs font-mono tracking-widest text-muted-foreground uppercase">
            Visits
          </span>
          <Select value={nvKey(selected.n_visits)} onValueChange={selectNVisits}>
            <SelectTrigger className="h-8 w-full min-w-0 text-xs font-mono">
              <SelectValue />
            </SelectTrigger>
            <SelectContent>
              {nVisitsOptions.map((v) => (
                <SelectItem key={v} value={String(v)} className="text-xs font-mono">
                  {v}
                </SelectItem>
              ))}
            </SelectContent>
          </Select>
        </div>
      )}
    </div>
  );
}

// ─── Sweep controls (x-axis selector + multi-value overlays) ──────────────────

interface SweepControlsProps {
  grid: ModelGrid;
  sweep: SweepParam;
  onSweep: (s: SweepParam) => void;
  selByDim: Record<SweepParam, string[]>;
  onToggleDim: (dim: SweepParam, values: string[]) => void;
  sweepValues: string[];
  onToggleSweepValues: (values: string[]) => void;
}

function SweepControls({
  grid,
  sweep,
  onSweep,
  selByDim,
  onToggleDim,
  sweepValues,
  onToggleSweepValues,
}: SweepControlsProps) {
  const hasNVisits = grid.scenarios.some((s) => s.n_visits != null);
  const nonSwept = (["drones", "comm_range", "n_visits"] as SweepParam[]).filter(
    (d) => d !== sweep && (d !== "n_visits" || hasNVisits)
  );
  const sweepOptions = availableDimValues(grid, sweep);

  return (
    <div className="flex flex-col gap-4">
      {/* Sweep (x-axis) selector + which values appear on the axis */}
      <div className="flex flex-wrap items-center gap-x-4 gap-y-2">
        <div className="flex items-center gap-2">
          <span className="text-xs font-mono tracking-widest text-muted-foreground uppercase">
            Sweep (x-axis)
          </span>
          <Select value={sweep} onValueChange={(v) => onSweep(v as SweepParam)}>
            <SelectTrigger className="h-7 w-40 text-xs font-mono">
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

        {/* X-axis value filter for the swept dimension */}
        {sweepOptions.length > 1 && (
          <div className="flex flex-wrap items-center gap-2">
            <span className="text-xs font-mono tracking-widest text-muted-foreground uppercase">
              X values
            </span>
            <ToggleGroup
              type="multiple"
              value={sweepValues}
              onValueChange={(v: string[]) => onToggleSweepValues(v)}
              className="flex-wrap justify-start gap-1"
            >
              {sweepOptions.map((o) => (
                <ToggleGroupItem
                  key={o.value}
                  value={o.value}
                  className="h-7 px-2.5 text-xs font-mono"
                >
                  {o.label}
                </ToggleGroupItem>
              ))}
            </ToggleGroup>
          </div>
        )}
      </div>

      {/* Multi-value overlays for each non-swept dimension */}
      {nonSwept.map((dim) => {
        const opts = availableDimValues(grid, dim);
        if (opts.length <= 1) return null;
        const sel = selByDim[dim] ?? [];
        return (
          <div key={dim} className="flex flex-wrap items-center gap-2">
            <span className="w-16 text-xs font-mono tracking-widest text-muted-foreground uppercase">
              {DIM_SHORT[dim]}
            </span>
            <ToggleGroup
              type="multiple"
              value={sel}
              onValueChange={(v: string[]) => onToggleDim(dim, v)}
              className="flex-wrap justify-start gap-1"
            >
              {opts.map((o) => (
                <ToggleGroupItem
                  key={o.value}
                  value={o.value}
                  className="h-7 px-2.5 text-xs font-mono"
                >
                  {o.label}
                </ToggleGroupItem>
              ))}
            </ToggleGroup>
          </div>
        );
      })}

      <p className="text-xs text-muted-foreground font-mono">
        Pick which <span className="text-foreground">X values</span> appear on
        the axis, and toggle overlay values to draw multiple trend lines — the
        legend appears once more than one line is shown.
      </p>
    </div>
  );
}

// Reserved scroll height for the CONTENT COLUMN before it mounts, px — the
// same idea as PARETO_HEIGHT/MERGING_HEIGHT/ANIMATION_HEIGHT in
// useScenarioSections. Parameter-effect is the sweep Card (header, up to a
// 3-column chart grid, and the caption); all-combinations is the hint, the
// table header row and one row per combination — 36 of them for every seeded
// model. Both are measured-ish rather than conservative: `containIntrinsicSize`
// is a fixed length, so a section snaps back to its estimate every time it
// leaves the viewport, and an estimate half the real height makes the document
// height lurch by that difference on each scroll past.
const PARAM_EFFECT_HEIGHT = 1200;
const ALL_COMBINATIONS_HEIGHT = 1400;

// ─── Page ─────────────────────────────────────────────────────────────────────

export default function ModelPage() {
  const params = useParams();
  const rawKey = params?.modelKey;
  const modelKey = decodeURIComponent(
    Array.isArray(rawKey) ? (rawKey[0] ?? "") : (rawKey ?? "")
  );

  const [grid, setGrid] = useState<ModelGrid | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  // The page's own pinned header. The control panel pins directly below it, so
  // the panel needs its live height — see SectionPanelLayout's `stickyOffset`.
  const { ref: headerRef, height: headerHeight } = useElementHeight();

  // Sweep state: x-axis dimension, which of its values appear on the axis, and
  // the selected overlay values per non-swept dimension.
  const [sweep, setSweep] = useState<SweepParam>("drones");
  const [sweepValueSel, setSweepValueSel] = useState<string[]>([]);
  const [seriesDrones, setSeriesDrones] = useState<string[]>([]);
  const [seriesComm, setSeriesComm] = useState<string[]>([]);
  const [seriesNVisits, setSeriesNVisits] = useState<string[]>([]);

  // Scenario-parameter filters (speed / grid / cell). Constant for seeded models,
  // but can vary across custom saved runs that share a model key — toggleable
  // like the overlay dims, filtering which scenarios the analysis + table use.
  const [selSpeed, setSelSpeed] = useState<string[]>([]);
  const [selGrid, setSelGrid] = useState<string[]>([]);
  const [selCell, setSelCell] = useState<string[]>([]);

  // Selected combination driving the Pareto/Merging/Animation sections.
  const [selName, setSelName] = useState<string>("");

  useEffect(() => {
    if (!modelKey) return;
    let cancelled = false;
    setLoading(true);
    setError(null);

    getModelGrid(modelKey)
      .then((data) => {
        if (!cancelled) {
          setGrid(data);
          // Default the Pareto/Merging/Animation sections to the first combination.
          setSelName(data.scenarios[0]?.scenario ?? "");
          // Seed the overlay selections (one value each ⇒ a single line) from
          // the first scenario.
          const first = data.scenarios[0];
          if (first) {
            setSeriesDrones(
              first.number_of_drones != null ? [String(first.number_of_drones)] : []
            );
            setSeriesComm(first.comm_range != null ? [first.comm_range] : []);
            // The swept dimension always defaults to Drones. Max Mean TBV is
            // undefined at n_visits=1, so for TBV models seed the n_visits overlay
            // with the smallest value > 1 — that keeps the TBV plot populated even
            // though the x-axis sweeps Drones; other models seed it from the first
            // scenario.
            const hasTbv = data.objectives.some((o) => o.includes("TBV"));
            const nVisitsAboveOne = Array.from(
              new Set(
                data.scenarios
                  .map((s) => s.n_visits)
                  .filter((v): v is number => v != null && v > 1)
              )
            ).sort((a, b) => a - b);
            setSeriesNVisits(
              hasTbv && nVisitsAboveOne.length > 0
                ? [String(nVisitsAboveOne[0])]
                : first.n_visits != null
                ? [String(first.n_visits)]
                : []
            );
            setSweep("drones");
            // Default the x-axis to ALL Drone values.
            setSweepValueSel(
              availableDimValues(data, "drones").map((o) => o.value)
            );
            // Seed the scenario-parameter filters to all present values.
            const sp = new Set<number>();
            const gr = new Set<number>();
            const ce = new Set<number>();
            for (const s of data.scenarios) {
              const p = scenarioParams(s.scenario);
              if (p) {
                sp.add(p.speed);
                gr.add(p.grid);
                ce.add(p.cell);
              }
            }
            // Single-select: default each to its smallest present value.
            const smallest = (s: Set<number>): string[] => {
              const v = Array.from(s).sort((a, b) => a - b)[0];
              return v != null ? [String(v)] : [];
            };
            setSelSpeed(smallest(sp));
            setSelGrid(smallest(gr));
            setSelCell(smallest(ce));
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

  // Currently-selected scenario object for the explorer dropdowns.
  const selectedScenario = useMemo(
    () => grid?.scenarios.find((s) => s.scenario === selName) ?? null,
    [grid, selName]
  );

  // Pareto / Merging / Animation sections for the selected combination.
  // Called unconditionally (grid may still be loading; selName is then "",
  // which useScenarioFront treats as "nothing to fetch yet") so the hook's own
  // state never has to survive a remount of its own. It always returns all
  // three sections — showing a skeleton or the offline panel as content when
  // the front is unavailable — so a failed front can neither silently delete
  // the deep-dive from this page nor shift the table under the reader when it
  // finally arrives.
  const { sections: explorerSections } = useScenarioSections({
    source: { mode: "seeded", scenario: selName },
    combinationControls: grid ? (
      <CombinationSelect
        grid={grid}
        selected={selectedScenario}
        onSelectName={setSelName}
      />
    ) : undefined,
  });

  // Distinct scenario-parameter values (parsed from scenario names), with units.
  const extraDimOpts = useMemo(() => {
    const sp = new Set<number>();
    const gr = new Set<number>();
    const ce = new Set<number>();
    for (const s of grid?.scenarios ?? []) {
      const p = scenarioParams(s.scenario);
      if (p) {
        sp.add(p.speed);
        gr.add(p.grid);
        ce.add(p.cell);
      }
    }
    const num = (a: number, b: number) => a - b;
    return {
      speed: Array.from(sp).sort(num).map((v) => ({ value: String(v), label: `${v} m/s` })),
      grid: Array.from(gr).sort(num).map((v) => ({ value: String(v), label: `${v} × ${v}` })),
      cell: Array.from(ce).sort(num).map((v) => ({ value: String(v), label: `${v} m` })),
    };
  }, [grid]);

  // Scenario set after applying the speed/grid/cell filters (lenient: an empty
  // selection means "all", so nothing is hidden during the initial seed window).
  const filteredGrid = useMemo<ModelGrid | null>(() => {
    if (!grid) return null;
    const sp = new Set(selSpeed);
    const gr = new Set(selGrid);
    const ce = new Set(selCell);
    const ok = (set: Set<string>, v: string) => set.size === 0 || set.has(v);
    const scenarios = grid.scenarios.filter((s) => {
      const p = scenarioParams(s.scenario);
      if (!p) return true;
      return (
        ok(sp, String(p.speed)) && ok(gr, String(p.grid)) && ok(ce, String(p.cell))
      );
    });
    return { ...grid, scenarios };
  }, [grid, selSpeed, selGrid, selCell]);

  // Toggleable scenario-parameter filter rows (rendered like the overlay dims).
  const filterRows: {
    label: string;
    opts: { value: string; label: string }[];
    sel: string[];
    set: (v: string[]) => void;
  }[] = [
    { label: "Speed", opts: extraDimOpts.speed, sel: selSpeed, set: setSelSpeed },
    { label: "Grid", opts: extraDimOpts.grid, sel: selGrid, set: setSelGrid },
    { label: "Cell", opts: extraDimOpts.cell, sel: selCell, set: setSelCell },
  ];

  // Select a combination and bring the Pareto section into view (used by
  // rows in ALL COMBINATIONS, which sits below it). Targets the section's DOM
  // id directly rather than a ref — its <section scroll-mt-20> already clears
  // the sticky header, and the id is stable across combination changes.
  const selectCombination = useCallback((name: string) => {
    setSelName(name);
    requestAnimationFrame(() => {
      document
        .getElementById("pareto")
        ?.scrollIntoView({ behavior: "smooth", block: "start" });
    });
  }, []);

  // Shared table builder for both export formats: raw numeric bests (kept as
  // numbers so XLSX cells are numeric), TBV blanked at n_visits === 1 to mirror
  // the on-screen "—".
  const buildCombinationsTable = useCallback((): {
    headers: string[];
    rows: (string | number)[][];
  } | null => {
    if (!filteredGrid) return null;
    const headers = [
      "Drones",
      "Comm Range",
      "n_visits",
      "# Solutions",
      ...filteredGrid.objectives.map((o) => `${o} (best)`),
    ];
    const rows: (string | number)[][] = filteredGrid.scenarios.map((s) => [
      s.number_of_drones ?? "",
      s.comm_range != null ? commLabel(s.comm_range) : "",
      s.n_visits ?? "",
      s.n_solutions,
      ...filteredGrid.objectives.map((obj) => {
        const v = s.objective_stats[obj]?.best;
        return tbvMeaningless(obj, s.n_visits) || v == null ? "" : v;
      }),
    ]);
    return { headers, rows };
  }, [filteredGrid]);

  const exportCombinationsCsv = useCallback(() => {
    const table = buildCombinationsTable();
    if (!table) return;
    const esc = (v: string | number) => {
      const s = String(v);
      return /[",\n]/.test(s) ? `"${s.replace(/"/g, '""')}"` : s;
    };
    const csv = [table.headers, ...table.rows]
      .map((r) => r.map(esc).join(","))
      .join("\r\n");
    const blob = new Blob([csv], { type: "text/csv;charset=utf-8;" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `${modelKey}-parameter-combinations.csv`;
    document.body.appendChild(a);
    a.click();
    a.remove();
    URL.revokeObjectURL(url);
  }, [buildCombinationsTable, modelKey]);

  // XLSX uses SheetJS, dynamically imported so it stays out of the initial
  // bundle (only fetched when the user actually exports an .xlsx).
  const exportCombinationsXlsx = useCallback(async () => {
    const table = buildCombinationsTable();
    if (!table) return;
    try {
      const XLSX = await import("xlsx");
      const ws = XLSX.utils.aoa_to_sheet([table.headers, ...table.rows]);
      const wb = XLSX.utils.book_new();
      XLSX.utils.book_append_sheet(wb, ws, "Combinations");
      XLSX.writeFile(wb, `${modelKey}-parameter-combinations.xlsx`);
    } catch (err: unknown) {
      toast.error("XLSX export failed", {
        description: err instanceof Error ? err.message : String(err),
      });
    }
  }, [buildCombinationsTable, modelKey]);

  // Toggle overlay values for a dimension, keeping at least one selected.
  const onToggleDim = useCallback((dim: SweepParam, values: string[]) => {
    if (values.length === 0) return; // never allow an empty (zero-line) state
    if (dim === "drones") setSeriesDrones(values);
    else if (dim === "comm_range") setSeriesComm(values);
    else setSeriesNVisits(values);
  }, []);

  // Change the swept dimension and reset its x-axis to all available values.
  const handleSweepChange = useCallback(
    (next: SweepParam) => {
      setSweep(next);
      if (filteredGrid) {
        setSweepValueSel(availableDimValues(filteredGrid, next).map((o) => o.value));
      }
    },
    [filteredGrid]
  );

  // Toggle which sweep values appear on the x-axis, keeping at least one.
  const onToggleSweepValues = useCallback((values: string[]) => {
    if (values.length === 0) return;
    setSweepValueSel(values);
  }, []);

  // Build one EffectSeries[] per objective, with overlay values ordered by the
  // natural parameter order (so legends read 2 → sqrt(8) → 4, etc.).
  const seriesByObj = useMemo<Record<string, EffectSeries[]>>(() => {
    if (!filteredGrid) return {};
    const hasNVisits = filteredGrid.scenarios.some((s) => s.n_visits != null);
    const orderSel = (dim: SweepParam, sel: string[]): string[] => {
      const order = availableDimValues(filteredGrid, dim).map((o) => o.value);
      return [...sel].sort((a, b) => order.indexOf(a) - order.indexOf(b));
    };
    const selByDim: Record<SweepParam, string[]> = {
      drones: orderSel("drones", seriesDrones),
      comm_range: orderSel("comm_range", seriesComm),
      n_visits: hasNVisits ? orderSel("n_visits", seriesNVisits) : [],
    };
    return buildSeriesByObjective(
      filteredGrid,
      sweep,
      selByDim,
      sweepValueSel,
      filteredGrid.objectives
    );
  }, [filteredGrid, sweep, seriesDrones, seriesComm, seriesNVisits, sweepValueSel]);

  // Largest line count across objectives — drives plot height + grid columns.
  const lineCount = useMemo(() => {
    if (!grid) return 1;
    let m = 1;
    for (const obj of grid.objectives) {
      const n = (seriesByObj[obj] ?? []).filter((s) => s.points.length > 0).length;
      if (n > m) m = n;
    }
    return m;
  }, [grid, seriesByObj]);

  const hasAnyData = useMemo(() => {
    if (!grid) return false;
    return grid.objectives.some((obj) =>
      (seriesByObj[obj] ?? []).some((s) => s.points.length > 0)
    );
  }, [grid, seriesByObj]);

  if (!modelKey) return null;

  // Plot sizing grows with the number of overlaid lines.
  const heightClass =
    lineCount >= 7 ? "h-80" : lineCount >= 4 ? "h-72" : lineCount >= 2 ? "h-60" : "h-48";
  const objCount = grid?.objectives.length ?? 1;
  const gridColsClass =
    lineCount >= 7
      ? "grid-cols-1"
      : lineCount >= 4
      ? "grid-cols-1 lg:grid-cols-2"
      : objCount === 1
      ? "grid-cols-1"
      : objCount === 2
      ? "grid-cols-1 md:grid-cols-2"
      : "grid-cols-1 md:grid-cols-2 xl:grid-cols-3";

  // One SectionPanelLayout for the whole page: this file's own
  // parameter-effect analysis, then the explorer's Pareto/Merging/Animation,
  // then this file's own all-combinations table. Nothing here is keyed —
  // everything this file owns updates from props alone, and the per-run
  // remounts the explorer sections need are keyed inside the hook that builds
  // them (see the file doc comment).
  const sections: PanelSection[] = grid
    ? [
        {
          id: "parameter-effect",
          label: "PARAMETER-EFFECT ANALYSIS",
          estimatedHeight: PARAM_EFFECT_HEIGHT,
          controls: (
            <div className="flex flex-col gap-5">
              <SweepControls
                grid={filteredGrid ?? grid}
                sweep={sweep}
                onSweep={handleSweepChange}
                selByDim={{
                  drones: seriesDrones,
                  comm_range: seriesComm,
                  n_visits: seriesNVisits,
                }}
                onToggleDim={onToggleDim}
                sweepValues={sweepValueSel}
                onToggleSweepValues={onToggleSweepValues}
              />

              {/* Scenario-parameter filters (speed / grid / cell) — toggleable. */}
              {extraDimOpts.speed.length > 0 && (
                <div className="flex flex-col gap-2">
                  {filterRows.map(({ label, opts, sel, set }) => (
                    <div key={label} className="flex flex-wrap items-center gap-2">
                      <span className="w-16 text-xs font-mono tracking-widest text-muted-foreground uppercase">
                        {label}
                      </span>
                      <ToggleGroup
                        type="single"
                        value={sel[0] ?? ""}
                        onValueChange={(v: string) => {
                          if (v) set([v]); // single-select; ignore deselect
                        }}
                        className="flex-wrap justify-start gap-1"
                      >
                        {opts.map((o) => (
                          <ToggleGroupItem
                            key={o.value}
                            value={o.value}
                            className="h-7 px-2.5 text-xs font-mono"
                          >
                            {o.label}
                          </ToggleGroupItem>
                        ))}
                      </ToggleGroup>
                    </div>
                  ))}
                </div>
              )}
            </div>
          ),
          content: (
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
                {/* One chart per objective, or a note if the filter yields nothing */}
                {filteredGrid && filteredGrid.scenarios.length === 0 ? (
                  <p className="rounded-lg border border-amber-500/40 bg-amber-500/10 px-4 py-2.5 text-xs font-mono text-amber-700 dark:text-amber-400">
                    ⚠ No saved mission matches the selected Speed / Grid / Cell —
                    this combination may not have been run yet.
                  </p>
                ) : !hasAnyData ? (
                  <p className="text-xs font-mono text-muted-foreground border border-dashed border-border rounded px-4 py-3">
                    No data for this selection. Try different overlay values.
                  </p>
                ) : (
                  <div className={cn("grid gap-6", gridColsClass)}>
                    {grid.objectives.map((obj, idx) => {
                      const objSeries = (seriesByObj[obj] ?? []).filter(
                        (s) => s.points.length > 0
                      );
                      if (objSeries.length === 0) {
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
                          series={objSeries}
                          colorIndex={idx}
                          heightClass={heightClass}
                        />
                      );
                    })}
                  </div>
                )}

                {/* Caption */}
                <p className="text-xs text-muted-foreground font-mono">
                  Each point is the best achievable value of that objective for the
                  given parameters.
                  {lineCount > 1
                    ? ` Overlaying ${lineCount} trend lines.`
                    : ""}
                </p>
              </CardContent>
            </Card>
          ),
        },
        // Spliced in unchanged. The remounting that a combination switch needs
        // is arranged inside these sections (useScenarioSections keys the
        // merging/playback content and the solution selector on the run), so
        // there is nothing for this page to key — and keying from out here
        // would take the parameter-effect charts and the scroll position with
        // it, which is exactly what this layout exists to avoid.
        ...explorerSections,
        {
          id: "all-combinations",
          label: "ALL COMBINATIONS",
          estimatedHeight: ALL_COMBINATIONS_HEIGHT,
          // No panel for this section: the exports sit on the table they act
          // on, and everything else here is the table itself. Dropping the
          // controls is what stops the sticky panel trailing down past the
          // last section that had any use for it.
          controls: null,
          content: (
            <div className="flex flex-col gap-2">
              <div className="flex flex-wrap items-end justify-between gap-3">
                {/* The rows are the page's other way in to a combination, and
                    nothing else says so: they look like a read-only table, and
                    clicking one scrolls several sections UP to the Pareto
                    front rather than to a block directly below. */}
                <p
                  id="all-combinations-hint"
                  className="text-xs text-muted-foreground font-mono"
                >
                  Click a row to load that combination in the Pareto front,
                  Merging and Animation sections above.
                </p>
                <div className="flex items-center gap-2">
                  <span className="text-xs font-mono tracking-widest uppercase text-muted-foreground">
                    Export
                  </span>
                  <Button
                    size="sm"
                    variant="outline"
                    onClick={exportCombinationsCsv}
                    disabled={(filteredGrid ?? grid).scenarios.length === 0}
                    className="h-7 text-xs tracking-widest font-mono"
                  >
                    CSV
                  </Button>
                  <Button
                    size="sm"
                    variant="outline"
                    onClick={exportCombinationsXlsx}
                    disabled={(filteredGrid ?? grid).scenarios.length === 0}
                    className="h-7 text-xs tracking-widest font-mono"
                  >
                    XLSX
                  </Button>
                </div>
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
                        <span className="ml-1 text-muted-foreground normal-case tracking-normal font-normal">
                          (best)
                        </span>
                      </TableHead>
                    ))}
                  </TableRow>
                </TableHeader>
                <TableBody>
                  {(filteredGrid ?? grid).scenarios.map((s) => (
                    <TableRow
                      key={s.scenario}
                      onClick={() => selectCombination(s.scenario)}
                      className={cn(
                        "cursor-pointer hover:bg-primary/5 transition-colors",
                        s.scenario === selName && "bg-primary/10"
                      )}
                      role="button"
                      tabIndex={0}
                      onKeyDown={(e) => {
                        if (e.key === "Enter" || e.key === " ") {
                          e.preventDefault();
                          selectCombination(s.scenario);
                        }
                      }}
                      aria-label={`Load scenario ${s.scenario} in the sections above`}
                      aria-describedby="all-combinations-hint"
                    >
                      <TableCell className="font-mono text-xs tabular-nums">
                        {s.number_of_drones}
                      </TableCell>
                      <TableCell className="font-mono text-xs">
                        {s.comm_range != null ? commLabel(s.comm_range) : "—"}
                      </TableCell>
                      <TableCell className="font-mono text-xs tabular-nums">
                        {s.n_visits ?? "—"}
                      </TableCell>
                      <TableCell className="font-mono text-xs tabular-nums text-chart-1">
                        {s.n_solutions}
                      </TableCell>
                      {grid.objectives.map((obj) => (
                        <TableCell
                          key={obj}
                          className="font-mono text-xs tabular-nums"
                        >
                          {tbvMeaningless(obj, s.n_visits)
                            ? "—"
                            : fmtObj(obj, s.objective_stats[obj]?.best)}
                        </TableCell>
                      ))}
                    </TableRow>
                  ))}
                </TableBody>
              </Table>
              </div>
            </div>
          ),
        },
      ]
    : [];

  return (
    <div className="mx-auto flex max-w-7xl flex-col gap-6 px-4 py-6">
      {/* Back link — a model page is reached from the mission browser, so return there. */}
      <Link
        href="/missions"
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
          {/* Header — sticky so the model and its objectives stay on screen
              while the reader scrolls the parameter-effect charts. Measured,
              because the control panel below pins under it and has to start
              where this ends — and this grows a row when the objective badges
              wrap. */}
          <div
            ref={headerRef}
            className="sticky top-14 z-30 flex flex-col gap-2 rounded-xl border border-border bg-background px-4 py-3"
          >
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
          </div>

          <SectionPanelLayout sections={sections} stickyOffset={headerHeight} />
        </>
      ) : null}
    </div>
  );
}
