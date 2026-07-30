"use client";

/**
 * MergingTab — the MERGING section's body: a sensing-config builder plus the
 * none / onboard / gcs comparison it produces (time-metric table, belief
 * evolution, targets-known).
 *
 * Moved here verbatim from ScenarioExplorer so that file could shrink to a
 * wrapper and useScenarioSections could own the section list without importing
 * back into its own consumer. Still one component: the split into
 * MergingControls (panel) + MergingContent (column) is the next task's job,
 * and until then this whole card is the section's `content` and the section
 * has no `controls`.
 */

import { useState } from "react";
import dynamic from "next/dynamic";
import { toast } from "sonner";
import { sourceCompare, type ExplorerSource } from "@/lib/source";
import type { SensingConfig } from "@/lib/types";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Skeleton } from "@/components/ui/skeleton";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Slider } from "@/components/ui/slider";
import { ToggleGroup, ToggleGroupItem } from "@/components/ui/toggle-group";
import { Separator } from "@/components/ui/separator";
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

export default function MergingTab({ source, selectedIndex }: MergingTabProps) {
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
