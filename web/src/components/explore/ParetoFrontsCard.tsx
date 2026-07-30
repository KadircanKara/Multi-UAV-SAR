"use client";

/**
 * ParetoFrontsCard — both views of the same front in ONE box.
 *
 * The charts arrive as nodes, not as data: this file owns the layout of the
 * card and nothing else, so it never re-imports the dynamic chart modules (the
 * section builder already holds those imports, and duplicating them would
 * split the loading fallbacks in two).
 */

import type { ReactNode } from "react";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
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
  return (
    <Card>
      <CardHeader>
        <CardTitle className="text-xs font-semibold tracking-widest uppercase text-primary font-display">
          PARETO FRONT
        </CardTitle>
      </CardHeader>
      <CardContent>
        {/* Stacked, not side by side: each plot gets the full width of the
            card, so the 3D cube and the 2D axis labels are both legible at
            the width the content column actually has. */}
        <div className="flex flex-col gap-6">
          {scatter2D}
          {show3D && scatter3D}
        </div>
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
