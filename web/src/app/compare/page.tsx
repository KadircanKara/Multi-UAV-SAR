"use client";

/**
 * /compare — Model Comparison page (seeded library).
 *
 * A shared model/parameter picker (ModelScenarioPicker) drives two tabs:
 *   • Objectives  — cross-model objective stats from POST /api/comparison,
 *                    rendered through the shared ObjectivesView.
 *   • Time Metrics — sensing-replay time metrics from POST /api/comparison/time
 *                    (run on demand against a shared sensing config + strategy).
 *
 * Each tab offers a Bar | Line | Table chart-type switcher. Bar renders
 * one CompareStackedBarChart per objective/metric (x = parameter combination,
 * stacked by model); Line renders one ParameterEffectChart per objective/metric
 * with one line per (model × non-swept-param combo), built by buildModelComboSeries;
 * Table goes through the shared MetricComparisonView. Stat is always "best".
 *
 * Layout: everything that filters/drives the charts — the tab switch, the
 * Bar|Line|Table switch, the model/parameter picker, and (for Time Metrics)
 * the sensing config + Run button — lives in a single SectionPanelLayout
 * section, so it renders once in the sticky left panel (desktop) or the
 * narrow-viewport Sheet drawer; only the resulting charts scroll in the
 * content column. `tab` is lifted to page state (rather than left inside an
 * uncontrolled Tabs) since both the panel's section label and the
 * content-column branch need to read it. Time Metrics' sensing-config state
 * stays lifted (via useTimeMetricsComparison, called unconditionally in
 * ComparePage) even though its inputs and its results now sit together in the
 * content column: being outside the tab branch is what lets a config and a
 * completed comparison survive a visit to Objectives and back. Objectives has
 * no such state to keep, so its fetch stays local to ObjectivesResults,
 * unmounting and refetching on every tab visit exactly as before.
 *
 * Mirrors the model page's dynamic chart import + skeleton/offline patterns and
 * the MergingTab sensing-config layout. Uses the clean Geist-Sans styling of the
 * landing/optimize pages (sentence-case labels, hud-rise entrance); colors come
 * only from the chart components / theme tokens.
 */

import { useCallback, useEffect, useMemo, useState } from "react";
import { toast } from "sonner";
import {
  ApiError,
  getLibrary,
  compareObjectives,
  compareTimeMetrics,
} from "@/lib/api";
import type {
  ScenarioSummary,
  ComparisonResponse,
  TimeComparisonResponse,
  SensingConfig,
} from "@/lib/types";
import ModelScenarioPicker, {
  type PickerSelection,
} from "@/components/compare/ModelScenarioPicker";
import {
  ObjectivesView,
  ParameterEffectChart,
  CompareStackedBarChart,
  SWEEP_LABELS,
  barGridClass,
  entityLabel,
  ChartTypeSwitch,
  SweepParamSelect,
  ChartEmptyNote,
  type ChartType,
} from "@/components/compare/ObjectivesView";
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
import type { EffectSeries } from "@/components/viz/ParameterEffectChart";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Skeleton } from "@/components/ui/skeleton";
import { Tabs, TabsList, TabsTrigger } from "@/components/ui/tabs";
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
import type { PanelSection } from "@/components/layout/PanelSection";
import SectionPanelLayout from "@/components/layout/SectionPanelLayout";
import { useElementHeight } from "@/hooks/useElementHeight";
import {
  useInitialSearchParams,
  useUrlSync,
  readEnum,
  readList,
  listParam,
  scalarParam,
} from "@/hooks/useUrlState";

// ─── Constants ────────────────────────────────────────────────────────────────

// Comparison batches are capped by BARS (parameter combinations on the x-axis),
// not raw scenarios — models stack within a bar, so the bar count is what the user
// reads. Both tabs cap at 36 bars. A scenario ceiling still bounds the total
// model×combo payload sent to the backend (and matches its max_length): objectives
// read cached fronts (cheap, generous ceiling); the time tab runs one sensing
// replay per scenario, so its ceiling is tighter.
const MAX_COMPARE_BARS = 36;
const MAX_OBJ_SCENARIOS = 360; // 36 bars × up to 10 stacked models
const MAX_TIME_SCENARIOS = 144; // 36 bars × up to 4 models (one replay per scenario)

// Parameter-combination key (drones · comm · n_visits) parsed from a scenario name
// "..._n_{drones}_v_{speed}_r_{comm}_nvisits_{nv}" — one bar on the x-axis.
function comboKeyFromName(scenario: string): string {
  const m = /_n_(\d+)_v_[\d.]+_r_(.+?)_nvisits_(\d+)$/.exec(scenario);
  return m ? `${m[1]}|${m[2]}|${m[3]}` : scenario;
}

// Distinct parameter combos (= bars) among a scenario set.
function countCombos(scenarios: string[]): number {
  const seen = new Set<string>();
  for (const s of scenarios) seen.add(comboKeyFromName(s));
  return seen.size;
}

// Cap the batch by WHOLE parameter combos (all their models = a complete stack):
// keep at most `maxBars` combos, and never exceed `maxScenarios` total model×combo
// rows (the backend's max_length). The first combo is always kept even if it alone
// would exceed the scenario ceiling.
function capToCombos(
  scenarios: string[],
  maxBars: number,
  maxScenarios: number
): string[] {
  const groups = new Map<string, string[]>();
  for (const s of scenarios) {
    const k = comboKeyFromName(s);
    const g = groups.get(k);
    if (g) g.push(s);
    else groups.set(k, [s]);
  }
  const out: string[] = [];
  let bars = 0;
  for (const g of Array.from(groups.values())) {
    if (bars >= maxBars) break;
    if (out.length > 0 && out.length + g.length > maxScenarios) break;
    out.push(...g);
    bars++;
  }
  return out;
}

/**
 * How long the Objectives tab waits after the picker settles before fetching.
 *
 * Every model, drones, comm and n_visits toggle changes the scenario set, and
 * a reader working through the picker clicks several in a row a few hundred ms
 * apart. At 250ms each of those clicks was its own POST /api/comparison, so a
 * normal pass through the picker burned the endpoint's 10/minute budget and
 * the page reported a rate-limit error. At 600ms a burst of clicks coalesces
 * into one request while a single deliberate change still feels immediate.
 *
 * This is NOT a rate-limit control — it runs in the browser, and a client that
 * does not want to wait simply does not. The server's own guards (the limiter,
 * the 360/144 payload caps, the comparison concurrency slot) are what bound
 * the cost; this only stops our own UI spending that budget on intermediate
 * states nobody asked to see.
 */
const OBJECTIVES_DEBOUNCE_MS = 600;

function OverflowNote({ shown, total }: { shown: number; total: number }) {
  return (
    <p className="rounded-md border border-border bg-muted/40 px-3 py-2 text-xs text-muted-foreground">
      Showing {shown} of {total} parameter combinations (bars) — narrow the
      parameter selection (fewer values) to include them all.
    </p>
  );
}

// ─── Small utility components (module scope) ──────────────────────────────────

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

/** A failed request, with the status that caused it — see ErrorPanel for why
 *  the status has to travel with the message. */
type PanelError = { message: string; status?: number };

function toPanelError(err: unknown): PanelError {
  if (err instanceof ApiError) return { message: err.message, status: err.status };
  return { message: err instanceof Error ? err.message : String(err) };
}

/**
 * Only a status of 0 means no response arrived at all. Anything else came FROM
 * the backend, so heading it "Backend offline" is not just wrong, it sends the
 * reader off to restart a server that is already running — which is exactly
 * what a rate-limited comparison (429) used to look like here: the offline
 * heading above the real message, "You're going a bit fast".
 */
function ErrorPanel({ message, status }: PanelError) {
  const unreachable = status === undefined || status === 0;
  const title = unreachable
    ? "Backend offline"
    : status === 429
      ? "Too many requests"
      : status === 503
        ? "Server busy"
        : "Couldn't load the comparison";
  return (
    <div className="rounded border border-destructive bg-destructive/10 px-4 py-4">
      <p className="text-sm font-semibold text-destructive">{title}</p>
      {unreachable && (
        <p className="text-sm text-muted-foreground mt-1">
          Start the API on :8000 then reload.
        </p>
      )}
      {message && (
        <p className="mt-2 text-sm text-muted-foreground break-all">
          {message}
        </p>
      )}
    </div>
  );
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

// ─── Objectives results (module scope) ─────────────────────────────────────────
//
// Owns scenario capping + the debounced fetch against /api/comparison; the
// bar/line/table rendering itself is the shared ObjectivesView component.
// Chart type + sweep param are lifted to the page (shared across both tabs,
// and consumed by the picker's line mode) and passed down as controlled
// props. The Bar | Line | Table switch itself now lives in the page's
// section-panel controls (always visible, not tied to this component's own
// mount lifecycle), so ObjectivesView is told to hide its own copy via
// `hideControls`. This component has no state that `controls` needs, so —
// unlike Time Metrics — it stays self-contained and simply mounts/unmounts
// with the active tab.

interface ObjectivesResultsProps {
  selection: PickerSelection;
  chartType: ChartType;
  onChartTypeChange: (v: ChartType) => void;
  sweep: SweepParam;
  onSweepChange: (v: SweepParam) => void;
}

function ObjectivesResults({
  selection,
  chartType,
  onChartTypeChange,
  sweep,
  onSweepChange,
}: ObjectivesResultsProps) {
  const { scenarios: allScenarios, models } = selection;
  // Cap by whole combos so bars stay complete; objectives read cached fronts.
  const scenarios = useMemo(
    () => capToCombos(allScenarios, MAX_COMPARE_BARS, MAX_OBJ_SCENARIOS),
    [allScenarios]
  );
  const shownCombos = countCombos(scenarios);
  const totalCombos = countCombos(allScenarios);
  const overflow = totalCombos > shownCombos;

  const [data, setData] = useState<ComparisonResponse | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<PanelError | null>(null);

  // Fetch the comparison whenever the resolved scenario set changes (debounced).
  // See OBJECTIVES_DEBOUNCE_MS for why the wait is as long as it is.
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
            setError(toPanelError(err));
            setLoading(false);
          }
        });
    }, OBJECTIVES_DEBOUNCE_MS);
    return () => {
      cancelled = true;
      clearTimeout(handle);
    };
    // scenarioKey captures the scenario set; scenarios is its source.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [scenarioKey]);

  if (scenarios.length === 0) {
    return (
      <p className="text-xs text-muted-foreground border border-dashed border-border rounded px-4 py-3">
        No scenarios selected. Pick models and parameters above.
      </p>
    );
  }

  return (
    <div className="flex flex-col gap-5">
      {overflow && <OverflowNote shown={shownCombos} total={totalCombos} />}
      {error && <ErrorPanel {...error} />}
      {loading && !error && <Skeleton className="h-64 w-full rounded" />}

      {!loading && !error && data && (
        <ObjectivesView
          data={data}
          chartType={chartType}
          onChartTypeChange={onChartTypeChange}
          sweep={sweep}
          onSweepChange={onSweepChange}
          models={models}
          hideControls
        />
      )}
    </div>
  );
}

// ─── Time Metrics (module scope) ───────────────────────────────────────────────
//
// Split into a state hook plus two presentational halves: TimeMetricsControls
// (the sensing config + Run button) and TimeMetricsResults (the replay
// results). Both now render in the content column, one above the other, but
// the state stays in useTimeMetricsComparison — called once, unconditionally,
// in ComparePage — so that a config and a finished comparison outlive a switch
// to the Objectives tab and back.

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

interface TimeMetricsState {
  scenarios: string[];
  shownCombos: number;
  totalCombos: number;
  overflow: boolean;

  mergeTopology: "none" | "onboard" | "gcs";
  setMergeTopology: (v: "none" | "onboard" | "gcs") => void;
  timeModel: "discrete" | "realtime";
  setTimeModel: (v: "discrete" | "realtime") => void;
  detProb: number;
  setDetProb: (v: number) => void;
  faProb: number;
  setFaProb: (v: number) => void;
  beliefThresh: number;
  setBeliefThresh: (v: number) => void;
  targetsInput: string;
  setTargetsInput: (v: string) => void;
  pqInvalid: boolean;
  targetList: number[];

  strategy: Strategy;
  setStrategy: (v: Strategy) => void;
  objectiveName: string;
  setObjectiveName: (v: string) => void;

  data: TimeComparisonResponse | null;
  running: boolean;
  canRun: boolean;
  runComparison: () => void;
}

function useTimeMetricsComparison(selection: PickerSelection): TimeMetricsState {
  const { scenarios: allScenarios } = selection;
  // Cap by whole combos; the time tab runs one sensing replay per scenario.
  const scenarios = useMemo(
    () => capToCombos(allScenarios, MAX_COMPARE_BARS, MAX_TIME_SCENARIOS),
    [allScenarios]
  );
  const shownCombos = countCombos(scenarios);
  const totalCombos = countCombos(allScenarios);
  const overflow = totalCombos > shownCombos;

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
      toast.error("Couldn't compare time metrics", { description: msg });
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

  return {
    scenarios,
    shownCombos,
    totalCombos,
    overflow,
    mergeTopology,
    setMergeTopology,
    timeModel,
    setTimeModel,
    detProb,
    setDetProb,
    faProb,
    setFaProb,
    beliefThresh,
    setBeliefThresh,
    targetsInput,
    setTargetsInput,
    pqInvalid,
    targetList,
    strategy,
    setStrategy,
    objectiveName,
    setObjectiveName,
    data,
    running,
    canRun,
    runComparison,
  };
}

function TimeMetricsControls({ tm }: { tm: TimeMetricsState }) {
  return (
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
              value={tm.mergeTopology}
              onValueChange={(v) => {
                if (v === "none" || v === "onboard" || v === "gcs")
                  tm.setMergeTopology(v);
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
              value={tm.timeModel}
              onValueChange={(v) => {
                if (v === "discrete" || v === "realtime") tm.setTimeModel(v);
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
              value={tm.strategy}
              onValueChange={(v) => tm.setStrategy(v as Strategy)}
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
            {tm.strategy === "best" && (
              <Select value={tm.objectiveName} onValueChange={tm.setObjectiveName}>
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
            value={tm.detProb}
            onChange={tm.setDetProb}
            min={0.01}
            max={0.99}
            step={0.01}
          />
          <SliderField
            label="False alarm prob (q)"
            value={tm.faProb}
            onChange={tm.setFaProb}
            min={0.01}
            max={0.99}
            step={0.01}
          />
          {tm.pqInvalid && (
            <p className="text-xs text-destructive">
              ⚠ Requires p &gt; q — adjust sliders
            </p>
          )}
          <SliderField
            label="Belief threshold (B)"
            value={tm.beliefThresh}
            onChange={tm.setBeliefThresh}
            min={0.01}
            max={0.99}
            step={0.01}
          />
          <div className="flex flex-col gap-2">
            <Label className="text-xs text-muted-foreground">
              Target cells (comma-separated)
            </Label>
            <Input
              value={tm.targetsInput}
              onChange={(e) => tm.setTargetsInput(e.target.value)}
              placeholder="e.g. 12,34,56"
              className="h-7 text-xs"
            />
            {tm.targetList.length === 0 && (
              <p className="text-xs text-destructive">
                Enter at least one valid cell index
              </p>
            )}
          </div>
        </div>

        <div className="md:col-span-2 flex flex-col gap-3">
          <Separator />
          <Button
            onClick={tm.runComparison}
            disabled={!tm.canRun}
            size="sm"
            className="w-full text-sm font-semibold"
          >
            {tm.running ? "Running replays…" : "Run comparison"}
          </Button>
          <p className="text-xs text-muted-foreground">
            Runs one replay per scenario ({tm.scenarios.length} selected) using the{" "}
            {tm.strategy} solution.
          </p>
          {tm.overflow && (
            <OverflowNote shown={tm.shownCombos} total={tm.totalCombos} />
          )}
        </div>
      </CardContent>
    </Card>
  );
}

interface TimeMetricsResultsProps {
  data: TimeComparisonResponse | null;
  running: boolean;
  models: string[];
  chartType: ChartType;
  sweep: SweepParam;
}

function TimeMetricsResults({
  data,
  running,
  models,
  chartType,
  sweep,
}: TimeMetricsResultsProps) {
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

  // Combo count = bars per chart (uniform across metrics — same scenarios).
  const comboCount = useMemo(
    () => Math.max(0, ...Object.values(stackedByMetric).map((sb) => sb.rows.length)),
    [stackedByMetric]
  );

  return (
    <div className="flex flex-col gap-6">
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
            <div className={barGridClass(comboCount)}>
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
              {/* `skipped` covers three cases, not just bad data: unloadable
                  scenarios, the server's per-request scenario cap, and its
                  wall-clock budget. Do not claim a cause we cannot tell apart. */}
              Skipped {data.skipped.length} scenario
              {data.skipped.length !== 1 ? "s" : ""} (not included in this
              comparison).
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
  const [error, setError] = useState<PanelError | null>(null);

  // The page's own pinned header. The control panel pins directly below it, so
  // the panel needs its live height — see SectionPanelLayout's `stickyOffset`.
  const { ref: headerRef, height: headerHeight } = useElementHeight();

  // Filter state round-trips through the query string, so a comparison can be
  // shared or reloaded. Read once at mount (see useUrlState); the URL is an
  // output from then on.
  const initialParams = useInitialSearchParams();

  // Picker selection (resolved scenarios + selected models).
  const [selection, setSelection] = useState<PickerSelection>({
    scenarios: [],
    models: [],
    params: {
      drones: [], comm_range: [], n_visits: [], speed: [], grid: [], cell: [],
    },
  });

  // Keep a stable onChange so the picker effect doesn't re-fire spuriously.
  const onPickerChange = useCallback((sel: PickerSelection) => {
    setSelection(sel);
  }, []);

  // Chart type + sweep param live here (shared across both tabs) so the parameter
  // picker can react to line mode: in line mode the non-sweep params go single-
  // select to keep the line plot legible.
  const [chartType, setChartType] = useState<ChartType>(() =>
    readEnum(initialParams, "chart", ["bar", "line", "table"] as const, "bar")
  );
  const [sweep, setSweep] = useState<SweepParam>(() =>
    readEnum(
      initialParams,
      "sweep",
      ["drones", "comm_range", "n_visits"] as const,
      "drones"
    )
  );

  // Active tab, lifted rather than left inside an uncontrolled Tabs: the
  // section's panel label and the content-column branch below both need to
  // read it outside of the Tabs subtree itself.
  const [tab, setTab] = useState<"objectives" | "time">(() =>
    readEnum(initialParams, "tab", ["objectives", "time"] as const, "objectives")
  );

  // Read once, then handed to the picker as its starting point.
  const restoredSelection = useMemo(
    () => ({
      models: readList(initialParams, "models", []),
      drones: readList(initialParams, "drones", []),
      comm_range: readList(initialParams, "comm", []),
      n_visits: readList(initialParams, "nvisits", []),
      speed: readList(initialParams, "speed", []),
      grid: readList(initialParams, "grid", []),
      cell: readList(initialParams, "cell", []),
    }),
    [initialParams]
  );

  // The picker's own selections are the source of truth once it has seeded, so
  // they are written straight back out. They carry no static default to
  // compare against — what counts as "default" is derived from the library —
  // so they appear in full whenever they are non-empty, which is also what
  // makes a shared link reproduce the exact comparison.
  const p = selection.params;
  useUrlSync({
    tab: scalarParam(tab, "objectives"),
    chart: scalarParam(chartType, "bar"),
    sweep: scalarParam(sweep, "drones"),
    models: listParam(selection.models, []),
    drones: listParam(p.drones, []),
    comm: listParam(p.comm_range, []),
    nvisits: listParam(p.n_visits, []),
    speed: listParam(p.speed, []),
    grid: listParam(p.grid, []),
    cell: listParam(p.cell, []),
  });

  // Time Metrics' sensing config + run state, called unconditionally so both
  // the controls half (below) and the content half share one live instance.
  const timeMetrics = useTimeMetricsComparison(selection);

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
          setError(toPanelError(err));
          setLoading(false);
        }
      });
    return () => {
      cancelled = true;
    };
  }, []);

  const compareSections: PanelSection[] = [
    {
      id: "compare-charts",
      label: tab === "objectives" ? "OBJECTIVES" : "TIME METRICS",
      estimatedHeight: 900,
      controls: (
        <>
          <div className="flex flex-col gap-2">
            <Tabs
              value={tab}
              onValueChange={(v) => {
                if (v === "objectives" || v === "time") setTab(v);
              }}
              className="w-full"
            >
              <TabsList className="mb-4">
                <TabsTrigger value="objectives" className="text-sm">
                  Objectives
                </TabsTrigger>
                <TabsTrigger value="time" className="text-sm">
                  Time metrics
                </TabsTrigger>
              </TabsList>
            </Tabs>
            <div className="flex flex-wrap items-center justify-between gap-4">
              <ChartTypeSwitch value={chartType} onChange={setChartType} />
              {chartType === "line" && (
                <SweepParamSelect value={sweep} onChange={setSweep} />
              )}
            </div>
          </div>
          <Separator />
          <ModelScenarioPicker
            library={library}
            onChange={onPickerChange}
            lineMode={chartType === "line"}
            sweepParam={sweep}
            initial={restoredSelection}
          />
        </>
      ),
      content:
        tab === "objectives" ? (
          <ObjectivesResults
            selection={selection}
            chartType={chartType}
            onChartTypeChange={setChartType}
            sweep={sweep}
            onSweepChange={setSweep}
          />
        ) : (
          // The sensing config sits above the charts it produces, in the same
          // column, matching Sensing and Animation on the model page. It is a
          // two-column card of sliders and would have to be rebuilt to fit a
          // 360px panel; the panel keeps what selects WHICH runs to compare.
          <div className="flex flex-col gap-6">
            <TimeMetricsControls tm={timeMetrics} />
            <TimeMetricsResults
              data={timeMetrics.data}
              running={timeMetrics.running}
              models={selection.models}
              chartType={chartType}
              sweep={sweep}
            />
          </div>
        ),
    },
  ];

  return (
    <div className="mx-auto flex max-w-7xl flex-col gap-6 px-4 py-6">
      {/* Header — sticky so the page identity survives the long scroll through
          the picker and the objective/time-metric chart grids. */}
      <div
        ref={headerRef}
        className="sticky top-14 z-30 lg:top-0 flex flex-col gap-1.5 rounded-xl border border-border bg-background px-4 py-3"
      >
        <h1
          className="animate-hud-rise text-2xl font-bold tracking-tight text-foreground"
          style={{ animationDelay: "60ms" }}
        >
          Compare Models
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
        <ErrorPanel {...error} />
      ) : (
        <SectionPanelLayout sections={compareSections} stickyOffset={headerHeight} />
      )}
    </div>
  );
}
