"use client";

/**
 * ScenarioExplorer — the Pareto / Merging / Animation deep-dive for ONE
 * precomputed scenario. Extracted from the former /explore/[scenario] page so
 * it can be embedded inline on the model page (driven by the parameter-
 * combination dropdowns) as well as rendered standalone at /explore/[scenario].
 *
 * Fetches the Pareto front for `scenario` and renders three tabs. When embedded
 * on the model page the parent passes a `key={scenario}` so switching the
 * combination fully remounts this subtree (resetting tab + per-tab state).
 */

import { useEffect, useState, useCallback, useMemo, type ReactNode } from "react";
import dynamic from "next/dynamic";
import { toast } from "sonner";
import { sourceFront, sourceCompare } from "@/lib/source";
import type { ExplorerSource } from "@/lib/source";
import type { ParetoFront, SensingConfig } from "@/lib/types";
import GridPlayback from "@/components/viz/GridPlayback/GridPlayback";
import { Tabs, TabsList, TabsTrigger, TabsContent } from "@/components/ui/tabs";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Skeleton } from "@/components/ui/skeleton";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Slider } from "@/components/ui/slider";
import { ToggleGroup, ToggleGroupItem } from "@/components/ui/toggle-group";
import { Separator } from "@/components/ui/separator";
import SolutionSelectorPanel from "@/components/SolutionSelectorPanel";
import MergingMetricsTable, {
  type CompareTableRow,
} from "@/components/viz/MergingMetricsTable";
import type { BeliefRow } from "@/components/viz/BeliefEvolutionChart";
import type { TargetsKnownRow } from "@/components/viz/TargetsKnownChart";
import type {
  ParameterEffectChartProps,
  EffectSeries,
} from "@/components/viz/ParameterEffectChart";
import type {
  CompareMetric,
  CompareEntity,
} from "@/components/compare/MetricComparisonView";
import { cn } from "@/lib/utils";

// ─── Dynamic (SSR-off) chart imports ─────────────────────────────────────────

const ParetoScatter = dynamic(
  () => import("@/components/viz/ParetoScatter"),
  { ssr: false, loading: () => <ChartSkeleton height="h-72" /> }
);

const BeliefEvolutionChart = dynamic(
  () => import("@/components/viz/BeliefEvolutionChart"),
  { ssr: false, loading: () => <ChartSkeleton height="h-64" /> }
);

const TargetsKnownChart = dynamic(
  () => import("@/components/viz/TargetsKnownChart"),
  { ssr: false, loading: () => <ChartSkeleton height="h-64" /> }
);

// Time-metric plots (bar/radar) + per-metric line chart for the merge compare.
const MetricComparisonView = dynamic(
  () => import("@/components/compare/MetricComparisonView"),
  { ssr: false, loading: () => <ChartSkeleton height="h-64" /> }
);

const ParameterEffectChart = dynamic<ParameterEffectChartProps>(
  () => import("@/components/viz/ParameterEffectChart"),
  { ssr: false, loading: () => <ChartSkeleton height="h-56" /> }
);

// ─── Small utility components ─────────────────────────────────────────────────

function ChartSkeleton({ height }: { height: string }) {
  return <Skeleton className={cn("w-full rounded", height)} />;
}

function ExplorerSkeleton() {
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

function OfflinePanel({ message }: { message: string }) {
  return (
    <div className="rounded border border-destructive bg-destructive/10 px-4 py-4 font-mono">
      <p className="text-sm font-semibold tracking-widest text-destructive uppercase">
        BACKEND OFFLINE
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

// ─── MERGING TAB ─────────────────────────────────────────────────────────────

const METRIC_NAMES = [
  "Effective Mission Time",
  "Detection Time",
  "Inform Time",
  "Time At Least One Drone Knows All Targets",
];

interface MergingTabProps {
  source: ExplorerSource;
  selectedIndex: number;
}

function MergingTab({ source, selectedIndex }: MergingTabProps) {
  // Config state
  const [timeModel, setTimeModel] = useState<"discrete" | "realtime">("discrete");
  const [detProb, setDetProb] = useState(0.8);
  const [faProb, setFaProb] = useState(0.1);
  const [beliefThresh, setBeliefThresh] = useState(0.9);
  const [targetsInput, setTargetsInput] = useState("12");

  // Which view to show for the time-metric results (table is the default).
  const [timeChartType, setTimeChartType] =
    useState<"table" | "bar" | "radar" | "line">("table");

  // Results state
  const [comparing, setComparing] = useState(false);
  const [tableRows, setTableRows] = useState<CompareTableRow[] | null>(null);
  const [beliefRows, setBeliefRows] = useState<BeliefRow[] | null>(null);
  const [knownRows, setKnownRows] = useState<TargetsKnownRow[] | null>(null);

  // Validation
  const pqInvalid = detProb <= faProb;
  const targetList = targetsInput
    .split(",")
    .map((s) => parseInt(s.trim(), 10))
    .filter((n) => !isNaN(n));

  const canCompare = !pqInvalid && targetList.length > 0 && !comparing;

  async function runCompare() {
    if (!canCompare) return;
    setComparing(true);
    setTableRows(null);
    setBeliefRows(null);
    setKnownRows(null);

    const baseConfig = {
      time_model: timeModel,
      detection_prob: detProb,
      false_alarm_prob: faProb,
      belief_threshold: beliefThresh,
      target_locations: targetList,
    };

    const configs: SensingConfig[] = [
      { ...baseConfig, merge_topology: "none" },
      { ...baseConfig, merge_topology: "onboard" },
      { ...baseConfig, merge_topology: "gcs" },
    ];
    const labels = ["none", "onboard", "gcs"];

    try {
      const res = await sourceCompare(source, {
        index: selectedIndex,
        configs,
        labels,
        model_key: null,
      });

      // Parse compare response
      const rawTable = res.table as Record<string, string | number | null>[] | undefined;
      if (rawTable) {
        setTableRows(rawTable as CompareTableRow[]);
      }

      const rawRows = res.rows as Record<string, unknown>[] | undefined;
      if (rawRows) {
        const bRows: BeliefRow[] = rawRows.map((r, i) => ({
          label: labels[i] ?? `config-${i}`,
          cell_occupancy_probabilities: r.cell_occupancy_probabilities as number[][],
          target_locations: r.target_locations as number[],
          belief_threshold: r.belief_threshold as number,
        }));
        setBeliefRows(bRows);
        setKnownRows(bRows as TargetsKnownRow[]);
      }
    } catch (err: unknown) {
      const msg = err instanceof Error ? err.message : String(err);
      toast.error("COMPARE FAILED", { description: msg });
    } finally {
      setComparing(false);
    }
  }

  // ── Time-metric plot data (entities = merge topologies, 4 metrics) ─────────
  const timeMetrics = useMemo<CompareMetric[]>(
    () => METRIC_NAMES.map((name) => ({ name, polarity: 1 })),
    []
  );
  const timeEntities = useMemo<CompareEntity[]>(() => {
    if (!tableRows) return [];
    return tableRows.map((row) => ({
      key: String(row.label),
      label: String(row.label).toUpperCase(),
      values: Object.fromEntries(
        METRIC_NAMES.map((m) => {
          const v = row[m];
          return [m, typeof v === "number" ? v : null];
        })
      ),
    }));
  }, [tableRows]);
  // One single-series line per metric, x-axis = the three merge topologies.
  const timeLineSeries = useMemo<Record<string, EffectSeries[]>>(() => {
    const out: Record<string, EffectSeries[]> = {};
    for (const m of METRIC_NAMES) {
      out[m] = [
        {
          key: m,
          label: m,
          points: timeEntities.map((e, i) => ({
            xLabel: e.label,
            xNum: i,
            best: e.values[m] ?? null,
            min: null,
            max: null,
          })),
        },
      ];
    }
    return out;
  }, [timeEntities]);

  return (
    <div className="flex flex-col gap-6">
      {/* Config builder card */}
      <Card>
        <CardHeader>
          <CardTitle
            className="text-xs font-semibold tracking-widest uppercase text-primary font-display"
            style={{ fontFamily: "var(--font-display)" }}
          >
            SENSING CONFIG
          </CardTitle>
        </CardHeader>
        <CardContent className="flex flex-col gap-5">
          {/* Time model */}
          <div className="flex flex-col gap-2">
            <Label className="text-xs text-muted-foreground tracking-widest uppercase font-mono">
              TIME MODEL
            </Label>
            <ToggleGroup
              type="single"
              value={timeModel}
              onValueChange={(v) => {
                if (v === "discrete" || v === "realtime") setTimeModel(v);
              }}
              className="justify-start gap-2"
            >
              <ToggleGroupItem
                value="discrete"
                className="h-7 text-xs font-mono tracking-widest uppercase"
              >
                DISCRETE
              </ToggleGroupItem>
              <ToggleGroupItem
                value="realtime"
                className="h-7 text-xs font-mono tracking-widest uppercase"
              >
                REALTIME
              </ToggleGroupItem>
            </ToggleGroup>
          </div>

          {/* Detection prob */}
          <SliderField
            label="DETECTION PROB (p)"
            value={detProb}
            onChange={setDetProb}
            min={0.01}
            max={0.99}
            step={0.01}
          />

          {/* False alarm prob */}
          <SliderField
            label="FALSE ALARM PROB (q)"
            value={faProb}
            onChange={setFaProb}
            min={0.01}
            max={0.99}
            step={0.01}
          />
          {pqInvalid && (
            <p className="text-xs text-destructive font-mono">
              ⚠ REQUIRES p &gt; q — adjust sliders
            </p>
          )}

          {/* Belief threshold */}
          <SliderField
            label="BELIEF THRESHOLD (B)"
            value={beliefThresh}
            onChange={setBeliefThresh}
            min={0.01}
            max={0.99}
            step={0.01}
          />

          {/* Target cells */}
          <div className="flex flex-col gap-2">
            <Label className="text-xs text-muted-foreground tracking-widest uppercase font-mono">
              TARGET CELLS (COMMA-SEPARATED)
            </Label>
            <Input
              value={targetsInput}
              onChange={(e) => setTargetsInput(e.target.value)}
              placeholder="e.g. 12,34,56"
              className="h-7 text-xs font-mono"
            />
            {targetList.length === 0 && (
              <p className="text-xs text-destructive font-mono">
                ENTER AT LEAST ONE VALID CELL INDEX
              </p>
            )}
          </div>

          <Separator />

          <Button
            onClick={runCompare}
            disabled={!canCompare}
            size="sm"
            className="w-full text-xs tracking-widest font-mono font-semibold"
          >
            {comparing ? "RUNNING COMPARE…" : "COMPARE MERGING STRATEGIES"}
          </Button>

          <p className="text-xs text-muted-foreground font-mono">
            COMPARING: NONE vs ONBOARD vs GCS — SOLUTION INDEX {selectedIndex}
          </p>
        </CardContent>
      </Card>

      {/* Results */}
      {comparing && (
        <div className="flex flex-col gap-3">
          <Skeleton className="h-32 w-full" />
          <div className="grid grid-cols-1 gap-4 lg:grid-cols-2">
            <Skeleton className="h-64 w-full" />
            <Skeleton className="h-64 w-full" />
          </div>
        </div>
      )}

      {!comparing && tableRows && (
        <>
          {/* Time-metric results — table / bar / radar / line switcher */}
          <div className="flex flex-col gap-3">
            <div className="flex flex-wrap items-center justify-between gap-2">
              <span
                className="text-xs font-semibold tracking-widest uppercase text-primary font-display"
                style={{ fontFamily: "var(--font-display)" }}
              >
                TIME METRICS
              </span>
              <div className="flex items-center gap-2">
                <span className="text-xs text-muted-foreground font-mono">VIEW</span>
                <ToggleGroup
                  type="single"
                  value={timeChartType}
                  onValueChange={(v) => {
                    if (v === "table" || v === "bar" || v === "radar" || v === "line")
                      setTimeChartType(v);
                  }}
                  className="gap-1"
                >
                  {(["table", "bar", "radar", "line"] as const).map((t) => (
                    <ToggleGroupItem
                      key={t}
                      value={t}
                      className="h-7 px-2.5 text-xs font-mono uppercase"
                    >
                      {t}
                    </ToggleGroupItem>
                  ))}
                </ToggleGroup>
              </div>
            </div>

            {timeChartType === "table" && (
              <MergingMetricsTable tableRows={tableRows} metricNames={METRIC_NAMES} />
            )}

            {(timeChartType === "bar" || timeChartType === "radar") && (
              <Card>
                <CardContent className="pt-4">
                  <MetricComparisonView
                    metrics={timeMetrics}
                    entities={timeEntities}
                    chartType={timeChartType}
                  />
                </CardContent>
              </Card>
            )}

            {timeChartType === "line" && (
              <Card>
                <CardContent className="pt-4">
                  <div className="grid gap-6 grid-cols-1 md:grid-cols-2">
                    {METRIC_NAMES.map((m, idx) => (
                      <ParameterEffectChart
                        key={m}
                        objective={m}
                        polarity={1}
                        sweepLabel="Merge topology"
                        series={timeLineSeries[m] ?? []}
                        colorIndex={idx}
                        heightClass="h-56"
                      />
                    ))}
                  </div>
                </CardContent>
              </Card>
            )}
          </div>

          {beliefRows && knownRows && (
            <div className="grid grid-cols-1 gap-4 lg:grid-cols-2">
              <Card>
                <CardContent className="pt-4">
                  <BeliefEvolutionChart rows={beliefRows} />
                </CardContent>
              </Card>
              <Card>
                <CardContent className="pt-4">
                  <TargetsKnownChart rows={knownRows} />
                </CardContent>
              </Card>
            </div>
          )}
        </>
      )}
    </div>
  );
}

// ─── Slider field helper (used in MergingTab, defined at module scope) ────────

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
        <Label className="text-xs text-muted-foreground tracking-widest uppercase font-mono">
          {label}
        </Label>
        <span className="text-xs font-mono tabular-nums text-primary">
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

// ─── ScenarioExplorer ─────────────────────────────────────────────────────────

interface Props {
  source: ExplorerSource;
  /** Show the big scenario title heading (standalone route). Off when embedded. */
  showTitle?: boolean;
  /** Notified once the front loads — lets a parent build a back-link, etc. */
  onFrontLoaded?: (front: ParetoFront) => void;
  /** Optional content rendered inside the Pareto-front card, below the scatter
   *  (e.g. the model route's read-only run-details). Omitted ⇒ nothing extra. */
  paretoFooter?: ReactNode;
}

export default function ScenarioExplorer({
  source,
  showTitle = true,
  onFrontLoaded,
  paretoFooter,
}: Props) {
  const [front, setFront] = useState<ParetoFront | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  // Shared across tabs
  const [selectedIndex, setSelectedIndex] = useState(0);

  // Display label — was `scenario` before; derived for both source modes.
  const displayLabel =
    source.mode === "seeded"
      ? source.scenario
      : source.result.model.model_key ?? "uploaded run";

  // Stable primitive key for the effect below (avoid re-fetch loops caused by
  // a freshly-created `source` object identity on every render).
  const sourceKey = source.mode === "seeded" ? source.scenario : "playground";

  useEffect(() => {
    if (source.mode === "seeded" && !source.scenario) return;
    let cancelled = false;
    setLoading(true);
    setError(null);

    sourceFront(source)
      .then((data) => {
        if (!cancelled) {
          setFront(data);
          // Default to first solution index from the data
          setSelectedIndex(data.solutions[0]?.index ?? 0);
          setLoading(false);
          onFrontLoaded?.(data);
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
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [sourceKey, onFrontLoaded]);

  const handleSelectIndex = useCallback((idx: number) => {
    setSelectedIndex(idx);
  }, []);

  if (source.mode === "seeded" && !source.scenario) return null;
  if (loading) return <ExplorerSkeleton />;
  if (error) {
    return (
      <div className="flex flex-col gap-3">
        {showTitle && (
          <h1
            className="text-sm font-semibold tracking-widest uppercase text-primary font-display"
            style={{ fontFamily: "var(--font-display)" }}
          >
            {displayLabel}
          </h1>
        )}
        <OfflinePanel message={error} />
      </div>
    );
  }
  if (!front) return null;

  return (
    <div className="flex flex-col gap-6">
      {/* Scenario info header */}
      <div className="flex flex-col gap-2">
        {showTitle && (
          <h1
            className="text-sm font-semibold tracking-widest uppercase text-primary font-display"
            style={{ fontFamily: "var(--font-display)" }}
          >
            {displayLabel}
          </h1>
        )}
        <div className="flex flex-wrap items-center gap-2">
          <Badge variant="outline" className="text-xs font-mono tracking-widest">
            {front.model_key}
          </Badge>
          <Badge variant="outline" className="text-xs font-mono tracking-widest">
            {front.result_kind.toUpperCase()}
          </Badge>
          <Badge variant="outline" className="text-xs font-mono tracking-widest">
            {front.n_solutions} SOLUTION{front.n_solutions !== 1 ? "S" : ""}
          </Badge>
          {front.objectives.map((obj) => (
            <Badge
              key={obj}
              className="text-xs font-mono tracking-wide bg-secondary text-secondary-foreground"
            >
              {obj}
              {front.polarities[obj] === -1 && (
                <span className="ml-1 text-muted-foreground">(max)</span>
              )}
            </Badge>
          ))}
        </div>
      </div>

      {/* Main tabs */}
      <Tabs defaultValue="pareto" className="w-full">
        <TabsList className="mb-4 font-mono text-xs tracking-widest">
          <TabsTrigger
            value="pareto"
            className="text-xs font-mono tracking-widest uppercase"
          >
            PARETO
          </TabsTrigger>
          <TabsTrigger
            value="merging"
            className="text-xs font-mono tracking-widest uppercase"
          >
            MERGING
          </TabsTrigger>
          <TabsTrigger
            value="animation"
            className="text-xs font-mono tracking-widest uppercase"
          >
            ANIMATION
          </TabsTrigger>
        </TabsList>

        {/* ── PARETO TAB ─────────────────────────────────────────────── */}
        <TabsContent value="pareto">
          <div className="grid grid-cols-1 gap-4 lg:grid-cols-[1fr_280px]">
            {/* Scatter chart card */}
            <Card>
              <CardHeader>
                <CardTitle
                  className="text-xs font-semibold tracking-widest uppercase text-primary font-display"
                  style={{ fontFamily: "var(--font-display)" }}
                >
                  PARETO FRONT
                </CardTitle>
              </CardHeader>
              <CardContent>
                <ParetoScatter
                  front={front}
                  selectedIndex={selectedIndex}
                  onSelectIndex={handleSelectIndex}
                />
                {paretoFooter}
              </CardContent>
            </Card>

            {/* Selector panel card */}
            <Card>
              <CardHeader>
                <CardTitle
                  className="text-xs font-semibold tracking-widest uppercase text-primary font-display"
                  style={{ fontFamily: "var(--font-display)" }}
                >
                  SOLUTION SELECT
                </CardTitle>
              </CardHeader>
              <CardContent>
                <SolutionSelectorPanel
                  source={source}
                  front={front}
                  selectedIndex={selectedIndex}
                  onSelectIndex={handleSelectIndex}
                />
              </CardContent>
            </Card>
          </div>
        </TabsContent>

        {/* ── MERGING TAB ────────────────────────────────────────────── */}
        <TabsContent value="merging">
          <MergingTab source={source} selectedIndex={selectedIndex} />
        </TabsContent>

        {/* ── ANIMATION TAB ──────────────────────────────────────────── */}
        <TabsContent value="animation">
          <GridPlayback
            source={source}
            front={front}
            selectedIndex={selectedIndex}
          />
        </TabsContent>
      </Tabs>
    </div>
  );
}
