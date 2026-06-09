"use client";

/**
 * /explore/[scenario] — Pareto front + Merging analysis for a precomputed scenario.
 * Task 1.5: EXPLORE screen.
 */

import { useEffect, useState, useCallback } from "react";
import dynamic from "next/dynamic";
import Link from "next/link";
import { useParams } from "next/navigation";
import { toast } from "sonner";
import { getFront, compare } from "@/lib/api";
import type { ParetoFront, SensingConfig } from "@/lib/types";
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

// ─── Small utility components ─────────────────────────────────────────────────

function ChartSkeleton({ height }: { height: string }) {
  return <Skeleton className={cn("w-full rounded", height)} />;
}

function PageSkeleton() {
  return (
    <div className="flex flex-col gap-4">
      <Skeleton className="h-8 w-64" />
      <Skeleton className="h-4 w-40" />
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
  scenario: string;
  selectedIndex: number;
}

function MergingTab({ scenario, selectedIndex }: MergingTabProps) {
  // Config state
  const [timeModel, setTimeModel] = useState<"discrete" | "realtime">("discrete");
  const [detProb, setDetProb] = useState(0.8);
  const [faProb, setFaProb] = useState(0.1);
  const [beliefThresh, setBeliefThresh] = useState(0.9);
  const [targetsInput, setTargetsInput] = useState("12");

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
      const res = await compare(scenario, {
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
          <MergingMetricsTable
            tableRows={tableRows}
            metricNames={METRIC_NAMES}
          />

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

// ─── Page ─────────────────────────────────────────────────────────────────────

export default function ExplorePage() {
  const params = useParams();
  const rawScenario = params?.scenario;
  const scenario = decodeURIComponent(
    Array.isArray(rawScenario) ? rawScenario[0] ?? "" : rawScenario ?? ""
  );

  const [front, setFront] = useState<ParetoFront | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  // Shared across tabs
  const [selectedIndex, setSelectedIndex] = useState(0);

  useEffect(() => {
    if (!scenario) return;
    let cancelled = false;
    setLoading(true);
    setError(null);

    getFront(scenario)
      .then((data) => {
        if (!cancelled) {
          setFront(data);
          // Default to first solution index from the data
          setSelectedIndex(data.solutions[0]?.index ?? 0);
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
  }, [scenario]);

  const handleSelectIndex = useCallback((idx: number) => {
    setSelectedIndex(idx);
  }, []);

  return (
    <div className="mx-auto flex max-w-7xl flex-col gap-6 px-4 py-6">
      {/* Back link */}
      <Link
        href="/"
        className="inline-flex items-center gap-1 text-xs font-mono tracking-widest text-muted-foreground hover:text-primary transition-colors uppercase"
      >
        ← MISSIONS
      </Link>

      {/* Header */}
      {loading ? (
        <PageSkeleton />
      ) : error ? (
        <>
          <div className="flex flex-col gap-1">
            <h1
              className="text-sm font-semibold tracking-widest uppercase text-primary font-display"
              style={{ fontFamily: "var(--font-display)" }}
            >
              {scenario}
            </h1>
          </div>
          <OfflinePanel message={error} />
        </>
      ) : front ? (
        <>
          {/* Scenario info header */}
          <div className="flex flex-col gap-2">
            <h1
              className="text-sm font-semibold tracking-widest uppercase text-primary font-display"
              style={{ fontFamily: "var(--font-display)" }}
            >
              {scenario}
            </h1>
            <div className="flex flex-wrap items-center gap-2">
              <Badge
                variant="outline"
                className="text-xs font-mono tracking-widest"
              >
                {front.model_key}
              </Badge>
              <Badge
                variant="outline"
                className="text-xs font-mono tracking-widest"
              >
                {front.result_kind.toUpperCase()}
              </Badge>
              <Badge
                variant="outline"
                className="text-xs font-mono tracking-widest"
              >
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
                      scenario={scenario}
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
              <MergingTab
                scenario={scenario}
                selectedIndex={selectedIndex}
              />
            </TabsContent>
          </Tabs>
        </>
      ) : null}
    </div>
  );
}
