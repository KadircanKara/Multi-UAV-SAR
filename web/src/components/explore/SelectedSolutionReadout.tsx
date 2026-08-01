"use client";

/**
 * SelectedSolutionReadout — reports which solution the Merging (and, once
 * Task 8 wires it in, Animation) section is analysing: the shared
 * `selectedIndex` and its objective values, formatted the same way every
 * other readout in the app does.
 *
 * This used to be `SolutionMiniFront`, and also carried a compact `2D | 3D`
 * Pareto chart so a reader could pick a solution without leaving the section.
 * At 320px the chart was too cramped to pick from usefully, so picking now
 * happens only in the Pareto section (both fronts render at full width
 * there). This component no longer shows a front at all — only reports the
 * pick that was already made elsewhere — hence the rename: a component
 * called "MiniFront" that contains no front is a lie.
 */

import type { ParetoFront } from "@/lib/types";
import { isPercentObjective, percentString } from "@/lib/objective-format";

// ─── Readout formatting ───────────────────────────────────────────────────────

function formatObjective(objective: string, value: number | undefined): string {
  if (value == null) return "—";
  return isPercentObjective(objective) ? percentString(value) : value.toFixed(4);
}

// ─── Component ────────────────────────────────────────────────────────────────

interface Props {
  front: ParetoFront;
  selectedIndex: number;
}

export default function SelectedSolutionReadout({ front, selectedIndex }: Props) {
  const selected = front.solutions.find((s) => s.index === selectedIndex);

  return (
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
  );
}
