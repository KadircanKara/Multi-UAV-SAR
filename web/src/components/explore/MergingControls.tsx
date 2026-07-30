"use client";

/**
 * MergingControls — the MERGING section's sticky-panel half: pick which
 * solution is under analysis (SolutionMiniFront), then build the sensing
 * config the Compare button runs.
 *
 * Owns no state itself. The sensing config and the compare result it
 * produces are both read by MergingContent too (the config to know what
 * produced the current result, the result to gate this panel's own button
 * label/disabled state), so both live one level up in useScenarioSections —
 * see that file's doc comment.
 */

import { Button } from "@/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Separator } from "@/components/ui/separator";
import { Slider } from "@/components/ui/slider";
import { ToggleGroup, ToggleGroupItem } from "@/components/ui/toggle-group";
import type { ParetoFront } from "@/lib/types";
import SolutionMiniFront from "./SolutionMiniFront";

interface Props {
  // Selection + axis state — shared with the Pareto section; see
  // useScenarioSections. Forwarded straight through to SolutionMiniFront.
  front: ParetoFront;
  selectedIndex: number;
  onSelectIndex: (idx: number) => void;
  xObj: string;
  yObj: string;
  onXChange: (objective: string) => void;
  onYChange: (objective: string) => void;
  x3DObj: string;
  y3DObj: string;
  z3DObj: string;
  onX3DChange: (objective: string) => void;
  onY3DChange: (objective: string) => void;
  onZ3DChange: (objective: string) => void;

  // Sensing config + its validation, and the compare it drives — shared with
  // MergingContent's result half; see useScenarioSections.
  timeModel: "discrete" | "realtime";
  onTimeModelChange: (model: "discrete" | "realtime") => void;
  detProb: number;
  onDetProbChange: (v: number) => void;
  faProb: number;
  onFaProbChange: (v: number) => void;
  beliefThresh: number;
  onBeliefThreshChange: (v: number) => void;
  targetsInput: string;
  onTargetsInputChange: (v: string) => void;
  pqInvalid: boolean;
  targetList: number[];
  canCompare: boolean;
  comparing: boolean;
  onCompare: () => void;
}

export default function MergingControls({
  front,
  selectedIndex,
  onSelectIndex,
  xObj,
  yObj,
  onXChange,
  onYChange,
  x3DObj,
  y3DObj,
  z3DObj,
  onX3DChange,
  onY3DChange,
  onZ3DChange,
  timeModel,
  onTimeModelChange,
  detProb,
  onDetProbChange,
  faProb,
  onFaProbChange,
  beliefThresh,
  onBeliefThreshChange,
  targetsInput,
  onTargetsInputChange,
  pqInvalid,
  targetList,
  canCompare,
  comparing,
  onCompare,
}: Props) {
  return (
    <div className="flex flex-col gap-4">
      <SolutionMiniFront
        front={front}
        selectedIndex={selectedIndex}
        onSelectIndex={onSelectIndex}
        xObj={xObj}
        yObj={yObj}
        onXChange={onXChange}
        onYChange={onYChange}
        x3DObj={x3DObj}
        y3DObj={y3DObj}
        z3DObj={z3DObj}
        onX3DChange={onX3DChange}
        onY3DChange={onY3DChange}
        onZ3DChange={onZ3DChange}
      />

      <Separator />

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
                if (v === "discrete" || v === "realtime") onTimeModelChange(v);
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
            onChange={onDetProbChange}
            min={0.01}
            max={0.99}
            step={0.01}
          />

          {/* False alarm prob */}
          <SliderField
            label="FALSE ALARM PROB (q)"
            value={faProb}
            onChange={onFaProbChange}
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
            onChange={onBeliefThreshChange}
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
              onChange={(e) => onTargetsInputChange(e.target.value)}
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
            onClick={onCompare}
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
    </div>
  );
}

// ─── Slider field helper (used above, defined at module scope) ────────────────

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
