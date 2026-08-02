"use client";

/**
 * ParetoFrontsCard — one view of the front at a time, chosen beside the title.
 *
 * The two plots used to be stacked, which made the card two screens tall and
 * pushed the 3D cube below the fold on every visit. They answer the same
 * question about the same solutions, so only one is worth showing; the other is
 * a dropdown away, and both write the same selection.
 *
 * The charts arrive as nodes, not as data: this file owns the layout of the
 * card and nothing else, so it never re-imports the dynamic chart modules (the
 * section builder already holds those imports, and duplicating them would
 * split the loading fallbacks in two).
 */

import { useState, type ReactNode } from "react";
import { Card, CardContent, CardHeader } from "@/components/ui/card";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import type { ParetoFront } from "@/lib/types";

/** True when a 2D scatter of this front means anything. A single-solution
 *  result or a one-objective front has no trade-off to plot — ParetoScatter
 *  falls back to a value readout — so the axis pickers and the click-to-select
 *  caption are hidden for it, exactly as they were before the card existed. */
export function has2DView(front: ParetoFront): boolean {
  return front.result_kind !== "single" && front.objectives.length >= 2;
}

/** True when the 3D view says something the 2D one cannot. Below three
 *  objectives (and for single-solution results) it is the same plot twice. */
export function has3DView(front: ParetoFront): boolean {
  return front.result_kind !== "single" && front.objectives.length >= 3;
}

interface Props {
  front: ParetoFront;
  scatter2D: ReactNode;
  /** Built by the caller only when `has3DView(front)`; ignored otherwise. */
  scatter3D: ReactNode;
  /** Extra content below the plots, inside the card (the model route's
   *  read-only run details). Omitted ⇒ nothing extra. */
  footer?: ReactNode;
}

export default function ParetoFrontsCard({
  front,
  scatter2D,
  scatter3D,
  footer,
}: Props) {
  const show3D = has3DView(front);
  const [view, setView] = useState<"2d" | "3d">("2d");
  // A front with fewer than three objectives has no 3D view to switch to, and
  // this component is not remounted when the front changes, so derive rather
  // than trust the stored value.
  const active = show3D ? view : "2d";

  return (
    <Card>
      <CardHeader>
        <div className="flex flex-wrap items-center gap-3">
          <h3 className="text-xs font-semibold tracking-widest uppercase text-primary font-display">
            PARETO FRONT
          </h3>
          {show3D && (
            <Select
              value={active}
              onValueChange={(v) => setView(v as "2d" | "3d")}
            >
              <SelectTrigger
                aria-label="Pareto front view"
                className="h-7 w-24 text-xs font-mono"
              >
                <SelectValue />
              </SelectTrigger>
              <SelectContent>
                <SelectItem value="2d" className="text-xs font-mono">
                  2D
                </SelectItem>
                <SelectItem value="3d" className="text-xs font-mono">
                  3D
                </SelectItem>
              </SelectContent>
            </Select>
          )}
        </div>
      </CardHeader>
      <CardContent>
        {active === "3d" ? scatter3D : scatter2D}
        {/* The caption belongs to the card, not to either plot: clicking a
            point selects on both. ParetoScatter used to print it and no longer
            does, so it still appears exactly once — and, as before, not at all
            for a result with no scatter to click. */}
        {has2DView(front) && (
          <p className="mt-3 text-xs text-muted-foreground font-mono">
            {front.n_solutions} SOLUTIONS — CLICK POINT TO SELECT
          </p>
        )}
        {footer}
      </CardContent>
    </Card>
  );
}
