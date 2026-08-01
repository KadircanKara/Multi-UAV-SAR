"use client";

/**
 * MergingContent — the MERGING section's entire scrolling-column half: the
 * sensing-config builder (time model, p/q/B sliders, target cells, Compare
 * button) ABOVE the comparison result it produces (time-metric table, belief
 * evolution, targets-known).
 *
 * Owns the sensing config and the compare result as its own local state. Both
 * used to live in useScenarioSections, because the config builder and the
 * result were split across two different places — MergingControls (panel)
 * and this file (column) — and each half read part of the same state. The
 * owner has since moved the whole builder back into this column (see
 * task-7-report.md's "Amendment round 2"), so nothing outside this component
 * reads any of it any more — MergingControls is deleted, and the panel now
 * renders only `SelectedSolutionReadout` (front + selectedIndex, unrelated to
 * this state). `selectedIndex` itself stays up in useScenarioSections and
 * arrives here as a prop, because it is still genuinely shared with Pareto
 * and Animation.
 *
 * Every result branch below reserves at least MERGING_HEIGHT of vertical
 * space. Before the config card existed here, the pre-Compare state rendered
 * nothing at all, and a mounted section with zero height can never become
 * useScrollSpy's active section — which made this whole section unreachable
 * by scroll until fixed. The config card alone now guarantees real height
 * unconditionally, but the floor stays as cheap insurance against that
 * regression (see MERGING_HEIGHT's own comment).
 */

import { useState } from "react";
import dynamic from "next/dynamic";
import { toast } from "sonner";
import { sourceCompare, type ExplorerSource } from "@/lib/source";
import type { SensingConfig } from "@/lib/types";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Separator } from "@/components/ui/separator";
import { Skeleton } from "@/components/ui/skeleton";
import { Slider } from "@/components/ui/slider";
import { ToggleGroup, ToggleGroupItem } from "@/components/ui/toggle-group";
import MergingMetricsTable, {
  type CompareTableRow,
} from "@/components/viz/MergingMetricsTable";
import type { BeliefRow } from "@/components/viz/BeliefEvolutionChart";
import type { TargetsKnownRow } from "@/components/viz/TargetsKnownChart";
import ChartSkeleton from "./ChartSkeleton";

// ─── Dynamic (SSR-off) chart imports ─────────────────────────────────────────

const BeliefEvolutionChart = dynamic(
  () => import("@/components/viz/BeliefEvolutionChart"),
  { ssr: false, loading: () => <ChartSkeleton height="h-64" /> }
);

const TargetsKnownChart = dynamic(
  () => import("@/components/viz/TargetsKnownChart"),
  { ssr: false, loading: () => <ChartSkeleton height="h-64" /> }
);

const METRIC_NAMES = [
  "Effective Mission Time",
  "Detection Time",
  "Inform Time",
  "Time At Least One Drone Knows All Targets",
];

/**
 * Reserved height, px, for the result area BELOW the sensing-config card —
 * used as a `minHeight` floor on both the in-flight skeleton and the
 * pre-Compare placeholder, and imported by useScenarioSections as the whole
 * Merging section's `estimatedHeight` for its pre-mount skeleton. Those are
 * two different things (this file's own result-area floor vs. the whole
 * section, config card included) that happen to share one number: the config
 * card was in the panel — uncounted here — when this constant was chosen, so
 * treat the pre-mount estimate as a bit conservative now that the card is
 * real, unconditional content in this same column. Ballparked from a
 * populated result (the TIME METRICS label and table with its footnote, plus
 * the two belief/known charts at their loaded height) — not measured in a
 * browser either way.
 */
export const MERGING_HEIGHT = 620;

// ─── Slider field helper ───────────────────────────────────────────────────────

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

// ─── Component ────────────────────────────────────────────────────────────────

interface Props {
  source: ExplorerSource;
  selectedIndex: number;
}

export default function MergingContent({ source, selectedIndex }: Props) {
  // Sensing config
  const [timeModel, setTimeModel] = useState<"discrete" | "realtime">("discrete");
  const [detProb, setDetProb] = useState(0.8);
  const [faProb, setFaProb] = useState(0.1);
  const [beliefThresh, setBeliefThresh] = useState(0.9);
  const [targetsInput, setTargetsInput] = useState("12");

  // Compare result
  const [comparing, setComparing] = useState(false);
  const [tableRows, setTableRows] = useState<CompareTableRow[] | null>(null);
  const [beliefRows, setBeliefRows] = useState<BeliefRow[] | null>(null);
  const [knownRows, setKnownRows] = useState<TargetsKnownRow[] | null>(null);

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
      toast.error("Comparison failed", { description: msg });
    } finally {
      setComparing(false);
    }
  }

  return (
    <div className="flex flex-col gap-6">
      {/* Sensing-config card */}
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

      {/* Result */}
      {comparing ? (
        <div style={{ minHeight: MERGING_HEIGHT }} className="flex flex-col gap-3">
          <Skeleton className="h-32 w-full" />
          <div className="grid grid-cols-1 gap-4 lg:grid-cols-2">
            <Skeleton className="h-64 w-full" />
            <Skeleton className="h-64 w-full" />
          </div>
        </div>
      ) : !tableRows ? (
        // Reachable before the first Compare, and again after a failed one:
        // the request rejects before any of tableRows/beliefRows/knownRows
        // are set, and `comparing` is already back to false in the `finally`
        // above — there is no separate error flag, so a failed compare shows
        // the same placeholder a never-run one does, rather than collapsing
        // to nothing.
        <div
          style={{ minHeight: MERGING_HEIGHT }}
          className="flex flex-col items-center justify-center rounded border border-dashed border-border px-4 py-6 text-center"
        >
          <p className="text-xs font-mono text-muted-foreground">
            Configure the sensing parameters above, then run the comparison to
            see time metrics, belief evolution, and targets known for the
            selected solution.
          </p>
        </div>
      ) : (
        <div className="flex flex-col gap-6">
          {/* Time-metric results — table */}
          <div className="flex flex-col gap-3">
            <span
              className="text-xs font-semibold tracking-widest uppercase text-primary font-display"
              style={{ fontFamily: "var(--font-display)" }}
            >
              TIME METRICS
            </span>
            <MergingMetricsTable tableRows={tableRows} metricNames={METRIC_NAMES} />
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
        </div>
      )}
    </div>
  );
}
