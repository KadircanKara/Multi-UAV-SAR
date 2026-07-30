"use client";

/**
 * SolutionMiniFront — the Pareto front, shrunk to a 320px panel.
 *
 * Merging and Animation both analyse exactly ONE solution; before this
 * component existed, picking that solution was only possible from the Pareto
 * section, so a reader on Merging could see no indication of which solution
 * was under analysis and had no way to change it. This renders one of the two
 * Pareto charts at a time — a `2D | 3D` toggle stands in for the Pareto
 * card's two stacked plots, which a 320px column has no room for — both with
 * `hideAxisSelectors` on, since a compact picker has even less room for five
 * axis dropdowns on top of that. The axis values themselves are not owned
 * here: they come from the same Pareto axis state in useScenarioSections, so
 * flipping to 3D here shows whatever X/Y/Z the Pareto section last set, and
 * changing axes there is reflected here too.
 *
 * The click target is the shared `onSelectIndex`, so a pick made in this mini
 * front moves the highlighted point on the Pareto section's charts as well —
 * one selection, everywhere.
 */

import { useState } from "react";
import dynamic from "next/dynamic";
import { ToggleGroup, ToggleGroupItem } from "@/components/ui/toggle-group";
import type { ParetoFront } from "@/lib/types";
import { isPercentObjective, percentString } from "@/lib/objective-format";
import { has3DView } from "./ParetoFrontsCard";
import ChartSkeleton from "./ChartSkeleton";

// ─── Dynamic (SSR-off) chart imports ─────────────────────────────────────────

const ParetoScatter = dynamic(() => import("@/components/viz/ParetoScatter"), {
  ssr: false,
  loading: () => <ChartSkeleton height="h-72" />,
});

const ParetoScatter3D = dynamic(
  () => import("@/components/viz/ParetoScatter3D"),
  { ssr: false, loading: () => <ChartSkeleton height="h-96" /> }
);

// ─── Readout formatting ───────────────────────────────────────────────────────

function formatObjective(objective: string, value: number | undefined): string {
  if (value == null) return "—";
  return isPercentObjective(objective) ? percentString(value) : value.toFixed(4);
}

// ─── Component ────────────────────────────────────────────────────────────────

interface Props {
  front: ParetoFront;
  selectedIndex: number;
  onSelectIndex: (idx: number) => void;
  /** 2D axis choice — owned by useScenarioSections, shared with the Pareto
   *  section's own 2D plot. */
  xObj: string;
  yObj: string;
  onXChange: (objective: string) => void;
  onYChange: (objective: string) => void;
  /** 3D axis choice — independent of the 2D pair above, likewise shared with
   *  the Pareto section's own 3D plot. */
  x3DObj: string;
  y3DObj: string;
  z3DObj: string;
  onX3DChange: (objective: string) => void;
  onY3DChange: (objective: string) => void;
  onZ3DChange: (objective: string) => void;
}

export default function SolutionMiniFront({
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
}: Props) {
  const show3D = has3DView(front);
  const [view, setView] = useState<"2d" | "3d">("2d");
  const selected = front.solutions.find((s) => s.index === selectedIndex);

  return (
    <div className="flex flex-col gap-3">
      {show3D && (
        <ToggleGroup
          type="single"
          value={view}
          onValueChange={(v) => {
            if (v === "2d" || v === "3d") setView(v);
          }}
          className="justify-start gap-2"
        >
          <ToggleGroupItem
            value="2d"
            className="h-7 text-xs font-mono tracking-widest uppercase"
          >
            2D
          </ToggleGroupItem>
          <ToggleGroupItem
            value="3d"
            className="h-7 text-xs font-mono tracking-widest uppercase"
          >
            3D
          </ToggleGroupItem>
        </ToggleGroup>
      )}

      {show3D && view === "3d" ? (
        <ParetoScatter3D
          objectives={front.objectives}
          points={front.solutions.map((s) => ({
            index: s.index,
            values: s.objectives_abs,
          }))}
          polarities={front.polarities}
          selectedIndex={selectedIndex}
          onSelectIndex={onSelectIndex}
          xObj={x3DObj}
          yObj={y3DObj}
          zObj={z3DObj}
          onXChange={onX3DChange}
          onYChange={onY3DChange}
          onZChange={onZ3DChange}
          hideAxisSelectors
        />
      ) : (
        <ParetoScatter
          front={front}
          selectedIndex={selectedIndex}
          onSelectIndex={onSelectIndex}
          xObj={xObj}
          yObj={yObj}
          onXChange={onXChange}
          onYChange={onYChange}
          hideAxisSelectors
        />
      )}

      <div className="rounded-md border border-border bg-muted/40 px-3 py-2 font-mono text-[11px]">
        <p className="text-foreground">#{selectedIndex}</p>
        {front.objectives.map((obj) => (
          <p key={obj} className="flex justify-between gap-2">
            <span className="text-muted-foreground">{obj}</span>
            <span className="tabular-nums">
              {formatObjective(obj, selected?.objectives_abs[obj])}
            </span>
          </p>
        ))}
      </div>
    </div>
  );
}
