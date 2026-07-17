"use client";

/**
 * ObjectivesView — shared cross-model objectives comparison view.
 *
 * Renders the Bar | Line | Table chart-type views driven by a
 * ComparisonResponse (POST /api/comparison for seeded scenarios, or
 * POST /api/playground/comparison for uploaded result files). Used by both
 * `/compare/seeded-results` and `/compare/playground` so the objective-rendering
 * logic (stat extraction, per-objective series building, chart-type dispatch)
 * lives in exactly one place.
 *
 * Chart type + sweep param default to internal state (self-contained — this is
 * how the playground page uses it). The seeded page instead lifts them to the
 * page level (so its ModelScenarioPicker can react to line mode) and passes
 * them down as controlled props, along with `hideControls` so its own
 * always-visible switch (shown even while a comparison is loading) isn't
 * duplicated, and `models` so stacked-bar color/legend order stays anchored to
 * the picker's selection instead of first-seen order in the response.
 */

import { useMemo, useState } from "react";
import dynamic from "next/dynamic";
import type { ComparisonResponse, ComparisonScenario } from "@/lib/types";
import MetricComparisonView, {
  type CompareMetric,
  type CompareEntity,
} from "@/components/compare/MetricComparisonView";
import {
  buildModelComboSeries,
  type ModelSeriesRow,
  type SweepParam,
} from "@/components/compare/buildModelSeries";
import {
  buildStackedBars,
  type StackedScenarioRow,
  type StackedBarData,
} from "@/components/compare/buildStackedBars";
import type { CompareStackedBarChartProps } from "@/components/viz/CompareStackedBarChart";
import type {
  ParameterEffectChartProps,
  EffectSeries,
} from "@/components/viz/ParameterEffectChart";
import { Skeleton } from "@/components/ui/skeleton";
import { ToggleGroup, ToggleGroupItem } from "@/components/ui/toggle-group";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";

// ─── Dynamic (SSR-off) chart imports — mirrors the model/compare pages ────────

export const ParameterEffectChart = dynamic<ParameterEffectChartProps>(
  () => import("@/components/viz/ParameterEffectChart"),
  { ssr: false, loading: () => <ChartSkeleton /> }
);

export const CompareStackedBarChart = dynamic<CompareStackedBarChartProps>(
  () => import("@/components/viz/CompareStackedBarChart"),
  { ssr: false, loading: () => <ChartSkeleton /> }
);

// ─── Types + constants ──────────────────────────────────────────────────────

export type ChartType = "bar" | "line" | "table";
type StatKey = "best" | "mean" | "min" | "max";

export const SWEEP_LABELS: Record<SweepParam, string> = {
  drones: "Number of Drones",
  comm_range: "Comm Range",
  n_visits: "n_visits",
};

// Bars on a stacked-bar chart = parameter combinations on the x-axis (models are
// stacked within each bar). Mirrors the breakpoint used by the seeded compare
// page's Time Metrics tab.
const BARS_PER_ROW_BREAKPOINT = 12;

export function barGridClass(comboCount: number): string {
  return comboCount > BARS_PER_ROW_BREAKPOINT
    ? "grid gap-6 grid-cols-1"
    : "grid gap-6 grid-cols-1 md:grid-cols-2";
}

export function ChartSkeleton() {
  return <Skeleton className="h-48 w-full rounded" />;
}

// Compact entity label: `${model_key} · ${drones}d · r${comm} · v${nvisits}`.
export function entityLabel(s: {
  model_key: string;
  number_of_drones: number | null;
  comm_range: string | null;
  n_visits: number | null;
}): string {
  const parts = [s.model_key];
  if (s.number_of_drones != null) parts.push(`${s.number_of_drones}d`);
  if (s.comm_range != null) parts.push(`r${s.comm_range}`);
  if (s.n_visits != null) parts.push(`v${s.n_visits}`);
  return parts.join(" · ");
}

export function ChartTypeSwitch({
  value,
  onChange,
}: {
  value: ChartType;
  onChange: (v: ChartType) => void;
}) {
  return (
    <ToggleGroup
      type="single"
      value={value}
      onValueChange={(v) => {
        if (v === "bar" || v === "line" || v === "table") onChange(v);
      }}
      className="justify-start gap-2"
    >
      <ToggleGroupItem value="bar" className="h-7 px-3 text-xs">
        Bar
      </ToggleGroupItem>
      <ToggleGroupItem value="line" className="h-7 px-3 text-xs">
        Line
      </ToggleGroupItem>
      <ToggleGroupItem value="table" className="h-7 px-3 text-xs">
        Table
      </ToggleGroupItem>
    </ToggleGroup>
  );
}

export function SweepParamSelect({
  value,
  onChange,
}: {
  value: SweepParam;
  onChange: (v: SweepParam) => void;
}) {
  return (
    <div className="flex items-center gap-2">
      <span className="text-xs text-muted-foreground">Sweep parameter</span>
      <Select value={value} onValueChange={(v) => onChange(v as SweepParam)}>
        <SelectTrigger className="h-7 w-40 text-xs">
          <SelectValue />
        </SelectTrigger>
        <SelectContent>
          <SelectItem value="drones" className="text-xs">
            Drones
          </SelectItem>
          <SelectItem value="comm_range" className="text-xs">
            Comm Range
          </SelectItem>
          <SelectItem value="n_visits" className="text-xs">
            n_visits
          </SelectItem>
        </SelectContent>
      </Select>
    </div>
  );
}

// Max Mean TBV is undefined at n_visits = 1 (no interval between visits); mirror
// the model page and show this where the TBV chart would otherwise be.
const TBV_NA_MESSAGE =
  "Max Mean TBV is undefined at n_visits = 1 — there is no interval between " +
  "visits. Increase n_visits to compare it.";

// A titled placeholder shown in a chart slot (no data, or metric not applicable).
export function ChartEmptyNote({ title, message }: { title: string; message: string }) {
  return (
    <div className="flex flex-col gap-1">
      <p className="text-xs font-medium text-foreground">{title}</p>
      <p className="text-xs text-muted-foreground border border-dashed border-border rounded px-3 py-6 text-center">
        {message}
      </p>
    </div>
  );
}

function isTbvName(name: string): boolean {
  return name.includes("TBV");
}

// True when every scenario in the response has n_visits = 1 (so Max Mean TBV
// has no data to show for any of them).
function onlyNVisits1(scenarios: ComparisonScenario[]): boolean {
  return scenarios.length > 0 && scenarios.every((s) => s.n_visits === 1);
}

// ─── Component ────────────────────────────────────────────────────────────────

export interface ObjectivesViewProps {
  data: ComparisonResponse;
  /** Chart type + sweep param default to internal state; pass these (with their
   *  setters) to control them from a parent. */
  chartType?: ChartType;
  onChartTypeChange?: (v: ChartType) => void;
  sweep?: SweepParam;
  onSweepChange?: (v: SweepParam) => void;
  /** Stack/legend order for the bar view. Defaults to the model keys present in
   *  `data`, in first-seen order. Pass the full picker selection to keep
   *  colors/order anchored to it rather than to response order. */
  models?: string[];
  /** Hide the internal Bar|Line|Table + sweep-param controls — pass this
   *  when the caller renders its own copy elsewhere (e.g. to keep the switch
   *  visible above a loading skeleton, before `data` exists). */
  hideControls?: boolean;
}

export function ObjectivesView({
  data,
  chartType: chartTypeProp,
  onChartTypeChange,
  sweep: sweepProp,
  onSweepChange,
  models: modelsProp,
  hideControls = false,
}: ObjectivesViewProps) {
  const [internalChartType, setInternalChartType] = useState<ChartType>("bar");
  const [internalSweep, setInternalSweep] = useState<SweepParam>("drones");
  const chartType = chartTypeProp ?? internalChartType;
  const setChartType = onChartTypeChange ?? setInternalChartType;
  const sweep = sweepProp ?? internalSweep;
  const setSweep = onSweepChange ?? setInternalSweep;

  // Stat is always "best" (the Stat selector was removed on the seeded page).
  const statKey: StatKey = "best";

  const nVisitsNA = useMemo(() => onlyNVisits1(data.scenarios), [data]);

  const models = useMemo(
    () => modelsProp ?? Array.from(new Set(data.scenarios.map((s) => s.model_key))),
    [modelsProp, data]
  );

  const metrics = useMemo<CompareMetric[]>(
    () =>
      data.objectives.map((o) => ({
        name: o,
        polarity: data.polarities[o] ?? 1,
      })),
    [data]
  );

  const entities = useMemo<CompareEntity[]>(
    () =>
      data.scenarios.map((s) => ({
        key: s.scenario,
        label: entityLabel(s),
        values: Object.fromEntries(
          data.objectives.map((o) => [o, s.objective_stats[o]?.[statKey] ?? null])
        ),
      })),
    [data]
  );

  const metricOptimizedBy = useMemo<Record<string, Set<string>>>(() => {
    const out: Record<string, Set<string>> = {};
    for (const o of data.objectives) out[o] = new Set();
    for (const s of data.scenarios) {
      for (const o of s.optimized_objectives) {
        (out[o] ??= new Set()).add(s.scenario);
      }
    }
    return out;
  }, [data]);

  // Line view: one chart per objective; one line per (model × non-swept combo).
  const lineSeriesByObjective = useMemo<Record<string, EffectSeries[]>>(() => {
    const out: Record<string, EffectSeries[]> = {};
    for (const o of data.objectives) {
      const polarity = data.polarities[o] ?? 1;
      const rows: ModelSeriesRow[] = data.scenarios.map((s) => ({
        model_key: s.model_key,
        number_of_drones: s.number_of_drones,
        comm_range: s.comm_range,
        comm_range_value: s.comm_range_value,
        n_visits: s.n_visits,
        value: s.objective_stats[o]?.[statKey] ?? null,
      }));
      out[o] = buildModelComboSeries(rows, sweep, polarity);
    }
    return out;
  }, [data, sweep]);

  // Stacked-bar view: one chart per objective, x = parameter combination,
  // stacked by model.
  const stackedByObjective = useMemo<Record<string, StackedBarData>>(() => {
    const out: Record<string, StackedBarData> = {};
    for (const o of data.objectives) {
      const rows: StackedScenarioRow[] = data.scenarios.map((s) => ({
        model_key: s.model_key,
        number_of_drones: s.number_of_drones,
        comm_range: s.comm_range,
        comm_range_value: s.comm_range_value,
        n_visits: s.n_visits,
        value: s.objective_stats[o]?.[statKey] ?? null,
      }));
      out[o] = buildStackedBars(rows, models);
    }
    return out;
  }, [data, models]);

  // Combo count = bars per chart (uniform across objectives — same scenarios).
  const comboCount = useMemo(
    () => Math.max(0, ...Object.values(stackedByObjective).map((sb) => sb.rows.length)),
    [stackedByObjective]
  );

  return (
    <div className="flex flex-col gap-5">
      {!hideControls && (
        <div className="flex flex-wrap items-center justify-between gap-4">
          <ChartTypeSwitch value={chartType} onChange={setChartType} />
          {chartType === "line" && (
            <SweepParamSelect value={sweep} onChange={setSweep} />
          )}
        </div>
      )}

      {chartType === "line" ? (
        <div className="grid gap-6 grid-cols-1 md:grid-cols-2 xl:grid-cols-3">
          {data.objectives.map((o, idx) => {
            if (isTbvName(o) && nVisitsNA) {
              return <ChartEmptyNote key={o} title={o} message={TBV_NA_MESSAGE} />;
            }
            const series = (lineSeriesByObjective[o] ?? []).filter(
              (s) => s.points.length > 0
            );
            if (series.length === 0) {
              return (
                <ChartEmptyNote
                  key={o}
                  title={o}
                  message="No data for the current selection."
                />
              );
            }
            return (
              <ParameterEffectChart
                key={o}
                objective={o}
                polarity={data.polarities[o] ?? 1}
                sweepLabel={SWEEP_LABELS[sweep]}
                series={series}
                colorIndex={idx}
                heightClass="h-60"
              />
            );
          })}
        </div>
      ) : chartType === "bar" ? (
        <div className={barGridClass(comboCount)}>
          {data.objectives.map((o) => {
            if (isTbvName(o) && nVisitsNA) {
              return <ChartEmptyNote key={o} title={o} message={TBV_NA_MESSAGE} />;
            }
            const sb = stackedByObjective[o];
            if (!sb || sb.models.length === 0 || sb.rows.length === 0) {
              return (
                <ChartEmptyNote
                  key={o}
                  title={o}
                  message="No data for the current selection."
                />
              );
            }
            return (
              <CompareStackedBarChart
                key={o}
                metric={o}
                polarity={data.polarities[o] ?? 1}
                rows={sb.rows}
                models={sb.models}
              />
            );
          })}
        </div>
      ) : (
        <MetricComparisonView
          metrics={metrics}
          entities={entities}
          chartType={chartType}
          metricOptimizedBy={metricOptimizedBy}
        />
      )}

      {data.skipped.length > 0 && (
        <p className="text-xs text-muted-foreground">
          Skipped {data.skipped.length} scenario
          {data.skipped.length !== 1 ? "s" : ""} (no loadable data).
        </p>
      )}
    </div>
  );
}
