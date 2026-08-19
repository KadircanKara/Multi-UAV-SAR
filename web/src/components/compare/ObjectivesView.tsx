"use client";

/**
 * ObjectivesView — shared cross-model objectives comparison view.
 *
 * Renders the Bar | Line | Table chart-type views driven by a
 * ComparisonResponse (POST /api/comparison). Used by /compare so the
 * objective-rendering pieces stay in one place.
 *
 * Chart type + sweep param default to internal state (self-contained), but
 * the seeded page instead lifts them to the page level (so its
 * ModelScenarioPicker can react to line mode) and passes them down as
 * controlled props, along with `hideControls` so its own always-visible
 * switch (shown even while a comparison is loading) isn't duplicated, and
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
import type {
  ParameterEffectChartProps,
  EffectSeries,
} from "@/components/viz/ParameterEffectChart";
import EffectLegend from "@/components/viz/EffectLegend";
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


// ─── Types + constants ──────────────────────────────────────────────────────

export type ChartType = "bar" | "line" | "table";
type StatKey = "best" | "mean" | "min" | "max";

export const SWEEP_LABELS: Record<SweepParam, string> = {
  drones: "Number of Drones",
  comm_range: "Comm Range",
  n_visits: "n_visits",
};


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

// The bar view fixes every parameter but the model, so that combination is
// stated once above the grid instead of repeated in each bar's label. Returns
// null when the selection is not a single combination (nothing to state).
export function comboCaption(
  scenarios: {
    number_of_drones: number | null;
    comm_range: string | null;
    n_visits: number | null;
  }[],
  /** Dimension to leave out because the chart already varies it on its x-axis
   *  (the line view's sweep parameter). Omit for the bar view, where every
   *  dimension is fixed. */
  skip?: SweepParam
): string | null {
  if (scenarios.length === 0) return null;
  const first = scenarios[0]!;
  const same = scenarios.every(
    (s) =>
      (skip === "drones" || s.number_of_drones === first.number_of_drones) &&
      (skip === "comm_range" || s.comm_range === first.comm_range) &&
      (skip === "n_visits" || s.n_visits === first.n_visits)
  );
  if (!same) return null;
  const parts: string[] = [];
  if (first.number_of_drones != null && skip !== "drones")
    parts.push(`${first.number_of_drones} drones`);
  if (first.comm_range != null && skip !== "comm_range")
    parts.push(`comm range ${first.comm_range}`);
  if (first.n_visits != null && skip !== "n_visits")
    parts.push(`n_visits ${first.n_visits}`);
  return parts.length ? parts.join(" · ") : null;
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
/**
 * One caption + one legend above a grid of line charts.
 *
 * ParameterEffectChart colours its series by ARRAY INDEX, so this is only
 * truthful while every chart in the grid is handed the same series list in the
 * same order — which is why the line branch stopped filtering empty series.
 */
export function LineGridHeader({
  caption,
  series,
}: {
  caption: string | null;
  series: { key: string; label: string }[];
}) {
  // NOT wrapped in a positioned box of its own: a sticky element pins within
  // its nearest scroll container but is CLIPPED to its parent, so a wrapper
  // around these two would let the legend travel the wrapper's own 40-odd px
  // and no further. Both are returned as siblings of the chart grid instead —
  // the caption scrolls away with the intro, the legend pins.
  return (
    <>
      {caption && <p className="text-xs text-muted-foreground">{caption}</p>}
      <EffectLegend series={series} sticky />
    </>
  );
}

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
        // Bar mode pins one parameter combination, so the model name alone
        // identifies a bar; the combination is stated once in the caption.
        label: chartType === "bar" ? s.model_key : entityLabel(s),
        values: Object.fromEntries(
          data.objectives.map((o) => [o, s.objective_stats[o]?.[statKey] ?? null])
        ),
      })),
    // chartType matters: it decides whether a bar is labelled by model alone.
    [data, chartType]
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
    // When every series shares the same non-swept parameters, the caption above
    // the grid states them, so repeating them in each label is noise. When they
    // differ there is no caption and the suffix is what tells series apart.
    const captioned = comboCaption(data.scenarios, sweep) !== null;
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
      out[o] = buildModelComboSeries(rows, sweep, polarity, captioned);
    }
    return out;
  }, [data, sweep]);

  // buildModelComboSeries groups the same rows the same way for every objective
  // and label-sorts the result, so each objective's series list is identical in
  // content AND order. That is what lets one legend stand for the whole grid.
  const lineLegendSeries = useMemo(
    () => Object.values(lineSeriesByObjective)[0] ?? [],
    [lineSeriesByObjective]
  );

  // Stacked-bar view: one chart per objective, x = parameter combination,
  // stacked by model.

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
        <div className="flex flex-col gap-4">
          <LineGridHeader
            caption={comboCaption(data.scenarios, sweep)}
            series={lineLegendSeries}
          />
          <div className="grid gap-6 grid-cols-1 md:grid-cols-2 xl:grid-cols-3">
          {data.objectives.map((o, idx) => {
            if (isTbvName(o) && nVisitsNA) {
              return <ChartEmptyNote key={o} title={o} message={TBV_NA_MESSAGE} />;
            }
            // NOT filtered to non-empty series: the palette is index-based, so
            // dropping a model here would shift every later model's colour in
            // THIS chart only — and the single shared legend above would then be
            // wrong for it. Empty series simply draw no line.
            const series = lineSeriesByObjective[o] ?? [];
            if (series.every((s) => s.points.length === 0)) {
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
                showLegend={false}
              />
            );
          })}
          </div>
        </div>
      ) : (
        <MetricComparisonView
          metrics={metrics}
          entities={entities}
          chartType={chartType}
          metricOptimizedBy={metricOptimizedBy}
          {...(chartType === "bar"
            ? { caption: comboCaption(data.scenarios) ?? undefined }
            : {})}
        />
      )}

      {data.skipped.length > 0 && (
        <p className="text-xs text-muted-foreground">
          {/* `skipped` covers three cases, not just bad data: unloadable
              scenarios, the server's per-request scenario cap, and its
              wall-clock budget. Do not claim a cause we cannot tell apart. */}
          Skipped {data.skipped.length} scenario
          {data.skipped.length !== 1 ? "s" : ""} (not included in this
          comparison).
        </p>
      )}
    </div>
  );
}
