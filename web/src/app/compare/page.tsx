"use client";

/**
 * /compare — Model Comparison page.
 *
 * A shared model/parameter picker (ModelScenarioPicker) drives two tabs:
 *   • Objectives  — cross-model objective stats from POST /api/comparison.
 *   • Time Metrics — sensing-replay time metrics from POST /api/comparison/time
 *                    (run on demand against a shared sensing config + strategy).
 *
 * Each tab offers a Bar | Line | Radar | Table chart-type switcher. Bar renders
 * one CompareStackedBarChart per objective/metric (x = parameter combination,
 * stacked by model); Line renders one ParameterEffectChart per objective/metric
 * with one line per (model × non-swept-param combo), built by buildModelComboSeries;
 * Radar/Table go through the shared MetricComparisonView. Stat is always "best".
 *
 * Mirrors the model page's dynamic chart import + skeleton/offline patterns and
 * the MergingTab sensing-config layout. Uses the clean Geist-Sans styling of the
 * landing/optimize pages (sentence-case labels, hud-rise entrance); colors come
 * only from the chart components / theme tokens.
 */

import { useCallback, useEffect, useMemo, useState } from "react";
import dynamic from "next/dynamic";
import Link from "next/link";
import { toast } from "sonner";
import { getLibrary, compareObjectives, compareTimeMetrics } from "@/lib/api";
import type {
  ScenarioSummary,
  ComparisonResponse,
  TimeComparisonResponse,
  SensingConfig,
} from "@/lib/types";
import ModelScenarioPicker, {
  type PickerSelection,
} from "@/components/compare/ModelScenarioPicker";
import CompareRunDetails from "@/components/compare/CompareRunDetails";
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
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Skeleton } from "@/components/ui/skeleton";
import { Tabs, TabsList, TabsTrigger, TabsContent } from "@/components/ui/tabs";
import { ToggleGroup, ToggleGroupItem } from "@/components/ui/toggle-group";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Slider } from "@/components/ui/slider";
import { Separator } from "@/components/ui/separator";

// ─── Dynamic (SSR-off) chart import — mirrors the model page ──────────────────

const ParameterEffectChart = dynamic<ParameterEffectChartProps>(
  () => import("@/components/viz/ParameterEffectChart"),
  { ssr: false, loading: () => <ChartSkeleton /> }
);

const CompareStackedBarChart = dynamic<CompareStackedBarChartProps>(
  () => import("@/components/viz/CompareStackedBarChart"),
  { ssr: false, loading: () => <ChartSkeleton /> }
);

// ─── Constants ────────────────────────────────────────────────────────────────

type ChartType = "bar" | "line" | "radar" | "table";
type StatKey = "best" | "mean" | "min" | "max";

const SWEEP_LABELS: Record<SweepParam, string> = {
  drones: "Number of Drones",
  comm_range: "Comm Range",
  n_visits: "n_visits",
};

// Max model-scenario results per comparison (each is one bar segment). Objectives
// read cached fronts, so a larger batch is cheap; the time tab runs a sensing
// replay per scenario, so it stays smaller. Both match their backend max_length.
const MAX_OBJ_SCENARIOS = 48;
const MAX_TIME_SCENARIOS = 24;

// Parameter-combination key (drones · comm · n_visits) parsed from a scenario name
// "..._n_{drones}_v_{speed}_r_{comm}_nvisits_{nv}" — one bar on the x-axis.
function comboKeyFromName(scenario: string): string {
  const m = /_n_(\d+)_v_[\d.]+_r_(.+?)_nvisits_(\d+)$/.exec(scenario);
  return m ? `${m[1]}|${m[2]}|${m[3]}` : scenario;
}

// Cap the batch by WHOLE parameter combos (all their models), so every shown bar
// is a complete stack rather than a partial one cut off by a flat slice.
function capByCombos(scenarios: string[], maxScenarios: number): string[] {
  const groups = new Map<string, string[]>();
  for (const s of scenarios) {
    const k = comboKeyFromName(s);
    const g = groups.get(k);
    if (g) g.push(s);
    else groups.set(k, [s]);
  }
  const out: string[] = [];
  for (const g of Array.from(groups.values())) {
    if (out.length > 0 && out.length + g.length > maxScenarios) break;
    out.push(...g);
  }
  return out;
}

function OverflowNote({ shown, total }: { shown: number; total: number }) {
  return (
    <p className="rounded-md border border-border bg-muted/40 px-3 py-2 text-xs text-muted-foreground">
      Showing {shown} of {total} selected model-scenario results — narrow the
      parameter selection (fewer values) to include them all.
    </p>
  );
}

// ─── Small utility components (module scope) ──────────────────────────────────

function ChartSkeleton() {
  return <Skeleton className="h-48 w-full rounded" />;
}

function PageSkeleton() {
  return (
    <div className="flex flex-col gap-4">
      <Skeleton className="h-8 w-64" />
      <Skeleton className="h-4 w-40" />
      <Skeleton className="h-32 w-full" />
      <Skeleton className="h-48 w-full" />
    </div>
  );
}

function OfflinePanel({ message }: { message: string }) {
  return (
    <div className="rounded border border-destructive bg-destructive/10 px-4 py-4">
      <p className="text-sm font-semibold text-destructive">
        Backend offline
      </p>
      <p className="text-sm text-muted-foreground mt-1">
        Start the API on :8000 then reload.
      </p>
      {message && (
        <p className="mt-2 text-xs text-muted-foreground/70 break-all">
          {message}
        </p>
      )}
    </div>
  );
}

// Compact entity label: `${model_key} · ${drones}d · r${comm} · v${nvisits}`.
function entityLabel(s: {
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

// ─── Chart-type switcher (module scope) ───────────────────────────────────────

function ChartTypeSwitch({
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
        if (v === "bar" || v === "line" || v === "radar" || v === "table")
          onChange(v);
      }}
      className="justify-start gap-2"
    >
      <ToggleGroupItem value="bar" className="h-7 px-3 text-xs">
        Bar
      </ToggleGroupItem>
      <ToggleGroupItem value="line" className="h-7 px-3 text-xs">
        Line
      </ToggleGroupItem>
      <ToggleGroupItem value="radar" className="h-7 px-3 text-xs">
        Radar
      </ToggleGroupItem>
      <ToggleGroupItem value="table" className="h-7 px-3 text-xs">
        Table
      </ToggleGroupItem>
    </ToggleGroup>
  );
}

// Sweep-parameter Select for the line view (module scope).
function SweepParamSelect({
  value,
  onChange,
}: {
  value: SweepParam;
  onChange: (v: SweepParam) => void;
}) {
  return (
    <div className="flex items-center gap-2">
      <span className="text-xs text-muted-foreground">
        Sweep parameter
      </span>
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
function ChartEmptyNote({ title, message }: { title: string; message: string }) {
  return (
    <div className="flex flex-col gap-1">
      <p className="text-xs font-medium text-foreground">{title}</p>
      <p className="text-xs text-muted-foreground border border-dashed border-border rounded px-3 py-6 text-center">
        {message}
      </p>
    </div>
  );
}

// True when the only selected n_visits value is 1 (so Max Mean TBV has no data).
function onlyNVisits1(nVisits: string[]): boolean {
  return nVisits.length > 0 && nVisits.every((v) => v === "1");
}

function isTbvName(name: string): boolean {
  return name.includes("TBV");
}

function SliderField({
  label,
  value,
  onChange,
  min,
  max,
  step,
}: {
  label: string;
  value: number;
  onChange: (v: number) => void;
  min: number;
  max: number;
  step: number;
}) {
  return (
    <div className="flex flex-col gap-2">
      <div className="flex justify-between">
        <Label className="text-xs text-muted-foreground">
          {label}
        </Label>
        <span className="text-xs tabular-nums text-primary">
          {value.toFixed(2)}
        </span>
      </div>
      <Slider
        min={min}
        max={max}
        step={step}
        value={[value]}
        onValueChange={([v]) => onChange(v)}
      />
    </div>
  );
}

// ─── Objectives tab (module scope) ────────────────────────────────────────────

interface ObjectivesTabProps {
  selection: PickerSelection;
}

function ObjectivesTab({ selection }: ObjectivesTabProps) {
  const { scenarios: allScenarios, models } = selection;
  const nVisitsSel = selection.sweepable.n_visits;
  // Cap by whole combos so bars stay complete; objectives read cached fronts.
  const scenarios = useMemo(
    () => capByCombos(allScenarios, MAX_OBJ_SCENARIOS),
    [allScenarios]
  );
  const overflow = allScenarios.length > scenarios.length;

  const [data, setData] = useState<ComparisonResponse | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const [chartType, setChartType] = useState<ChartType>("bar");
  // Stat is always "best" (the Stat selector was removed).
  const statKey: StatKey = "best";
  const [sweep, setSweep] = useState<SweepParam>("drones");

  // Fetch the comparison whenever the resolved scenario set changes (debounced).
  const scenarioKey = useMemo(() => [...scenarios].sort().join("|"), [scenarios]);
  useEffect(() => {
    if (scenarios.length === 0) {
      setData(null);
      setError(null);
      setLoading(false);
      return;
    }
    let cancelled = false;
    const handle = setTimeout(() => {
      setLoading(true);
      setError(null);
      compareObjectives(scenarios)
        .then((res) => {
          if (!cancelled) {
            setData(res);
            setLoading(false);
          }
        })
        .catch((err: unknown) => {
          if (!cancelled) {
            setError(err instanceof Error ? err.message : String(err));
            setLoading(false);
          }
        });
    }, 250);
    return () => {
      cancelled = true;
      clearTimeout(handle);
    };
    // scenarioKey captures the scenario set; scenarios is its source.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [scenarioKey]);

  const metrics = useMemo<CompareMetric[]>(() => {
    if (!data) return [];
    return data.objectives.map((o) => ({
      name: o,
      polarity: data.polarities[o] ?? 1,
    }));
  }, [data]);

  const entities = useMemo<CompareEntity[]>(() => {
    if (!data) return [];
    return data.scenarios.map((s) => ({
      key: s.scenario,
      label: entityLabel(s),
      values: Object.fromEntries(
        data.objectives.map((o) => [o, s.objective_stats[o]?.[statKey] ?? null])
      ),
    }));
  }, [data, statKey]);

  const metricOptimizedBy = useMemo<Record<string, Set<string>>>(() => {
    if (!data) return {};
    const out: Record<string, Set<string>> = {};
    for (const o of data.objectives) out[o] = new Set();
    for (const s of data.scenarios) {
      for (const o of s.optimized_objectives) {
        (out[o] ??= new Set()).add(s.scenario);
      }
    }
    return out;
  }, [data]);

  // Line view: one chart per objective; one line per (model × non-swept combo),
  // mirroring the model page's overlays across the selected models.
  const lineSeriesByObjective = useMemo<Record<string, EffectSeries[]>>(() => {
    if (!data) return {};
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
  }, [data, statKey, sweep]);

  // Stacked-bar view: one chart per objective, x = parameter combination,
  // stacked by model.
  const stackedByObjective = useMemo<Record<string, StackedBarData>>(() => {
    if (!data) return {};
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
  }, [data, statKey, models]);

  if (scenarios.length === 0) {
    return (
      <p className="text-xs text-muted-foreground border border-dashed border-border rounded px-4 py-3">
        No scenarios selected. Pick models and parameters above.
      </p>
    );
  }

  return (
    <div className="flex flex-col gap-5">
      {/* Controls */}
      <div className="flex flex-wrap items-center justify-between gap-4">
        <ChartTypeSwitch value={chartType} onChange={setChartType} />
        {chartType === "line" && (
          <SweepParamSelect value={sweep} onChange={setSweep} />
        )}
      </div>

      {overflow && <OverflowNote shown={scenarios.length} total={allScenarios.length} />}
      {error && <OfflinePanel message={error} />}
      {loading && !error && <Skeleton className="h-64 w-full rounded" />}

      {!loading && !error && data && (
        <>
          {chartType === "line" ? (
            <div className="grid gap-6 grid-cols-1 md:grid-cols-2 xl:grid-cols-3">
              {data.objectives.map((o, idx) => {
                if (isTbvName(o) && onlyNVisits1(nVisitsSel)) {
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
            <div className="grid gap-6 grid-cols-1 md:grid-cols-2">
              {data.objectives.map((o) => {
                if (isTbvName(o) && onlyNVisits1(nVisitsSel)) {
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
        </>
      )}
    </div>
  );
}

// ─── Time Metrics tab (module scope) ──────────────────────────────────────────

const STRATEGIES = ["balanced", "knee", "best"] as const;
type Strategy = (typeof STRATEGIES)[number];

// The five objectives available for the "best" strategy objective select.
const OBJECTIVE_NAMES = [
  "Mission Time",
  "Percentage Connectivity",
  "Max Disconnected Time",
  "Mean Disconnected Time",
  "Max Mean TBV",
];

interface TimeMetricsTabProps {
  selection: PickerSelection;
}

function TimeMetricsTab({ selection }: TimeMetricsTabProps) {
  const { scenarios: allScenarios, models } = selection;
  // Cap by whole combos; the time tab runs one sensing replay per scenario.
  const scenarios = useMemo(
    () => capByCombos(allScenarios, MAX_TIME_SCENARIOS),
    [allScenarios]
  );
  const overflow = allScenarios.length > scenarios.length;

  // Sensing config (mirrors MergingTab).
  const [mergeTopology, setMergeTopology] = useState<
    "none" | "onboard" | "gcs"
  >("onboard");
  const [timeModel, setTimeModel] = useState<"discrete" | "realtime">(
    "discrete"
  );
  const [detProb, setDetProb] = useState(0.8);
  const [faProb, setFaProb] = useState(0.1);
  const [beliefThresh, setBeliefThresh] = useState(0.9);
  const [targetsInput, setTargetsInput] = useState("12");

  // Strategy.
  const [strategy, setStrategy] = useState<Strategy>("balanced");
  const [objectiveName, setObjectiveName] = useState<string>(OBJECTIVE_NAMES[0]!);

  // Results + view state.
  const [data, setData] = useState<TimeComparisonResponse | null>(null);
  const [running, setRunning] = useState(false);
  const [chartType, setChartType] = useState<ChartType>("bar");
  const [sweep, setSweep] = useState<SweepParam>("drones");

  const pqInvalid = detProb <= faProb;
  const targetList = useMemo(
    () =>
      targetsInput
        .split(",")
        .map((s) => parseInt(s.trim(), 10))
        .filter((n) => !isNaN(n)),
    [targetsInput]
  );

  const canRun =
    !pqInvalid && targetList.length > 0 && scenarios.length > 0 && !running;

  const runComparison = useCallback(async () => {
    if (!canRun) return;
    setRunning(true);
    setData(null);

    const config: SensingConfig = {
      merge_topology: mergeTopology,
      time_model: timeModel,
      detection_prob: detProb,
      false_alarm_prob: faProb,
      belief_threshold: beliefThresh,
      target_locations: targetList,
    };

    try {
      const res = await compareTimeMetrics({
        scenarios,
        config,
        strategy,
        objective_name: strategy === "best" ? objectiveName : null,
      });
      setData(res);
    } catch (err: unknown) {
      const msg = err instanceof Error ? err.message : String(err);
      toast.error("TIME COMPARISON FAILED", { description: msg });
    } finally {
      setRunning(false);
    }
  }, [
    canRun,
    mergeTopology,
    timeModel,
    detProb,
    faProb,
    beliefThresh,
    targetList,
    scenarios,
    strategy,
    objectiveName,
  ]);

  // All four time metrics are lower-is-better → polarity +1.
  const metrics = useMemo<CompareMetric[]>(() => {
    if (!data) return [];
    return data.metrics.map((m) => ({ name: m, polarity: 1 }));
  }, [data]);

  const entities = useMemo<CompareEntity[]>(() => {
    if (!data) return [];
    return data.scenarios.map((s) => ({
      key: s.scenario,
      label: entityLabel(s),
      values: Object.fromEntries(
        data.metrics.map((m) => [m, s.metric_values[m] ?? null])
      ),
    }));
  }, [data]);

  const lineSeriesByMetric = useMemo<Record<string, EffectSeries[]>>(() => {
    if (!data) return {};
    const out: Record<string, EffectSeries[]> = {};
    for (const m of data.metrics) {
      const rows: ModelSeriesRow[] = data.scenarios.map((s) => ({
        model_key: s.model_key,
        number_of_drones: s.number_of_drones,
        comm_range: s.comm_range,
        comm_range_value: s.comm_range_value,
        n_visits: s.n_visits,
        value: s.metric_values[m] ?? null,
      }));
      // Time metrics are all lower-is-better (polarity 1).
      out[m] = buildModelComboSeries(rows, sweep, 1);
    }
    return out;
  }, [data, sweep]);

  // Stacked-bar view: one chart per metric, x = parameter combination,
  // stacked by model.
  const stackedByMetric = useMemo<Record<string, StackedBarData>>(() => {
    if (!data) return {};
    const out: Record<string, StackedBarData> = {};
    for (const m of data.metrics) {
      const rows: StackedScenarioRow[] = data.scenarios.map((s) => ({
        model_key: s.model_key,
        number_of_drones: s.number_of_drones,
        comm_range: s.comm_range,
        comm_range_value: s.comm_range_value,
        n_visits: s.n_visits,
        value: s.metric_values[m] ?? null,
      }));
      out[m] = buildStackedBars(rows, models);
    }
    return out;
  }, [data, models]);

  return (
    <div className="flex flex-col gap-6">
      {/* Sensing config + strategy card */}
      <Card>
        <CardHeader>
          <CardTitle>Sensing config</CardTitle>
        </CardHeader>
        <CardContent className="grid grid-cols-1 gap-5 md:grid-cols-2">
          {/* Left column: topology + time model + strategy */}
          <div className="flex flex-col gap-5">
            <div className="flex flex-col gap-2">
              <Label className="text-xs text-muted-foreground">
                Merge topology
              </Label>
              <ToggleGroup
                type="single"
                value={mergeTopology}
                onValueChange={(v) => {
                  if (v === "none" || v === "onboard" || v === "gcs")
                    setMergeTopology(v);
                }}
                className="justify-start gap-2"
              >
                {(["none", "onboard", "gcs"] as const).map((t) => (
                  <ToggleGroupItem
                    key={t}
                    value={t}
                    className="h-7 text-xs capitalize"
                  >
                    {t}
                  </ToggleGroupItem>
                ))}
              </ToggleGroup>
            </div>

            <div className="flex flex-col gap-2">
              <Label className="text-xs text-muted-foreground">
                Time model
              </Label>
              <ToggleGroup
                type="single"
                value={timeModel}
                onValueChange={(v) => {
                  if (v === "discrete" || v === "realtime") setTimeModel(v);
                }}
                className="justify-start gap-2"
              >
                <ToggleGroupItem value="discrete" className="h-7 text-xs">
                  Discrete
                </ToggleGroupItem>
                <ToggleGroupItem value="realtime" className="h-7 text-xs">
                  Realtime
                </ToggleGroupItem>
              </ToggleGroup>
            </div>

            <div className="flex flex-col gap-2">
              <Label className="text-xs text-muted-foreground">
                Strategy
              </Label>
              <Select
                value={strategy}
                onValueChange={(v) => setStrategy(v as Strategy)}
              >
                <SelectTrigger className="h-7 w-40 text-xs capitalize">
                  <SelectValue />
                </SelectTrigger>
                <SelectContent>
                  {STRATEGIES.map((s) => (
                    <SelectItem key={s} value={s} className="text-xs capitalize">
                      {s}
                    </SelectItem>
                  ))}
                </SelectContent>
              </Select>
              {strategy === "best" && (
                <Select value={objectiveName} onValueChange={setObjectiveName}>
                  <SelectTrigger className="h-7 w-full text-xs">
                    <SelectValue />
                  </SelectTrigger>
                  <SelectContent>
                    {OBJECTIVE_NAMES.map((o) => (
                      <SelectItem key={o} value={o} className="text-xs">
                        {o}
                      </SelectItem>
                    ))}
                  </SelectContent>
                </Select>
              )}
            </div>
          </div>

          {/* Right column: sliders + targets */}
          <div className="flex flex-col gap-5">
            <SliderField
              label="Detection prob (p)"
              value={detProb}
              onChange={setDetProb}
              min={0.01}
              max={0.99}
              step={0.01}
            />
            <SliderField
              label="False alarm prob (q)"
              value={faProb}
              onChange={setFaProb}
              min={0.01}
              max={0.99}
              step={0.01}
            />
            {pqInvalid && (
              <p className="text-xs text-destructive">
                ⚠ Requires p &gt; q — adjust sliders
              </p>
            )}
            <SliderField
              label="Belief threshold (B)"
              value={beliefThresh}
              onChange={setBeliefThresh}
              min={0.01}
              max={0.99}
              step={0.01}
            />
            <div className="flex flex-col gap-2">
              <Label className="text-xs text-muted-foreground">
                Target cells (comma-separated)
              </Label>
              <Input
                value={targetsInput}
                onChange={(e) => setTargetsInput(e.target.value)}
                placeholder="e.g. 12,34,56"
                className="h-7 text-xs"
              />
              {targetList.length === 0 && (
                <p className="text-xs text-destructive">
                  Enter at least one valid cell index
                </p>
              )}
            </div>
          </div>

          <div className="md:col-span-2 flex flex-col gap-3">
            <Separator />
            <Button
              onClick={runComparison}
              disabled={!canRun}
              size="sm"
              className="w-full text-sm font-semibold"
            >
              {running ? "Running replays…" : "Run comparison"}
            </Button>
            <p className="text-xs text-muted-foreground">
              Runs one replay per scenario ({scenarios.length} selected) using the{" "}
              {strategy} solution.
            </p>
            {overflow && <OverflowNote shown={scenarios.length} total={allScenarios.length} />}
          </div>
        </CardContent>
      </Card>

      {/* Results */}
      {running && (
        <div className="flex flex-col gap-3">
          <Skeleton className="h-64 w-full" />
        </div>
      )}

      {!running && !data && (
        <p className="text-xs text-muted-foreground border border-dashed border-border rounded px-4 py-6 text-center">
          Configure the sensing parameters and run the comparison to see time
          metrics.
        </p>
      )}

      {!running && data && (
        <div className="flex flex-col gap-5">
          <div className="flex flex-wrap items-center justify-between gap-4">
            <ChartTypeSwitch value={chartType} onChange={setChartType} />
            {chartType === "line" && (
              <SweepParamSelect value={sweep} onChange={setSweep} />
            )}
          </div>

          {chartType === "line" ? (
            <div className="grid gap-6 grid-cols-1 md:grid-cols-2">
              {data.metrics.map((m, idx) => {
                const series = (lineSeriesByMetric[m] ?? []).filter(
                  (s) => s.points.length > 0
                );
                if (series.length === 0) {
                  return (
                    <ChartEmptyNote
                      key={m}
                      title={m}
                      message="No data for the current selection."
                    />
                  );
                }
                return (
                  <ParameterEffectChart
                    key={m}
                    objective={m}
                    polarity={1}
                    sweepLabel={SWEEP_LABELS[sweep]}
                    series={series}
                    colorIndex={idx}
                    heightClass="h-60"
                  />
                );
              })}
            </div>
          ) : chartType === "bar" ? (
            <div className="grid gap-6 grid-cols-1 md:grid-cols-2">
              {data.metrics.map((m) => {
                const sb = stackedByMetric[m];
                if (!sb || sb.models.length === 0 || sb.rows.length === 0) {
                  return (
                    <ChartEmptyNote
                      key={m}
                      title={m}
                      message="No data for the current selection."
                    />
                  );
                }
                return (
                  <CompareStackedBarChart
                    key={m}
                    metric={m}
                    polarity={1}
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
            />
          )}

          {data.skipped.length > 0 && (
            <p className="text-xs text-muted-foreground">
              Skipped {data.skipped.length} scenario
              {data.skipped.length !== 1 ? "s" : ""} (no loadable data).
            </p>
          )}
        </div>
      )}
    </div>
  );
}

// ─── Page ─────────────────────────────────────────────────────────────────────

export default function ComparePage() {
  const [library, setLibrary] = useState<ScenarioSummary[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  // Picker selection (resolved scenarios + selected models).
  const [selection, setSelection] = useState<PickerSelection>({
    scenarios: [],
    models: [],
    sweepable: { drones: [], comm_range: [], n_visits: [] },
  });

  // Keep a stable onChange so the picker effect doesn't re-fire spuriously.
  const onPickerChange = useCallback((sel: PickerSelection) => {
    setSelection(sel);
  }, []);

  useEffect(() => {
    let cancelled = false;
    setLoading(true);
    setError(null);
    getLibrary()
      .then((data) => {
        if (!cancelled) {
          setLibrary(data);
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

  return (
    <div className="mx-auto flex max-w-7xl flex-col gap-6 px-4 py-6">
      {/* Back link */}
      <Link
        href="/missions"
        className="animate-hud-rise inline-flex items-center gap-1 text-sm text-muted-foreground transition-colors hover:text-foreground"
        style={{ animationDelay: "0ms" }}
      >
        ← Missions
      </Link>

      {/* Header */}
      <div className="flex flex-col gap-1.5">
        <h1
          className="animate-hud-rise text-2xl font-bold tracking-tight text-foreground"
          style={{ animationDelay: "60ms" }}
        >
          Model Comparison
        </h1>
        <p
          className="animate-hud-rise text-[15px] leading-relaxed text-muted-foreground"
          style={{ animationDelay: "120ms" }}
        >
          Compare optimiser models across objectives and sensing time-metrics.
        </p>
      </div>

      {loading ? (
        <PageSkeleton />
      ) : error ? (
        <OfflinePanel message={error} />
      ) : (
        <>
          <ModelScenarioPicker library={library} onChange={onPickerChange} />

          <Tabs defaultValue="objectives" className="w-full">
            <TabsList className="mb-4">
              <TabsTrigger value="objectives" className="text-sm">
                Objectives
              </TabsTrigger>
              <TabsTrigger value="time" className="text-sm">
                Time metrics
              </TabsTrigger>
            </TabsList>

            <TabsContent value="objectives">
              <ObjectivesTab selection={selection} />
            </TabsContent>

            <TabsContent value="time">
              <TimeMetricsTab selection={selection} />
            </TabsContent>
          </Tabs>

          <CompareRunDetails scenarios={selection.scenarios} />
        </>
      )}
    </div>
  );
}
