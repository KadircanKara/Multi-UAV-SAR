"use client";

/**
 * BestValues — the extreme Pareto point per objective, as a row of cards.
 *
 * Shared by /optimize (the run that just finished) and /analysis (an uploaded
 * or handed-over run), which is why it lives here rather than inside either
 * page. It takes the narrowest shape both fronts satisfy, so neither page has
 * to convert.
 */

import {
  isPercentObjective,
  objectiveUnit,
  percentString,
} from "@/lib/objective-format";

/** Minimal front shape this needs. Both the live-run `OptimizeFront` and the
 *  explorer's `ParetoFront` satisfy it. */
export interface BestValuesFront {
  objectives: string[];
  n_solutions: number;
  solutions: {
    index: number;
    objectives_signed: Record<string, number | null>;
    objectives_abs: Record<string, number | null>;
  }[];
}

function fmtObjValue(obj: string, n: number | null | undefined): string {
  if (n == null || Number.isNaN(n)) return "—";
  if (isPercentObjective(obj)) return percentString(n);
  return n.toFixed(2);
}

/** The unit suffix to show after a formatted value. None for a percentage
 *  objective — the "%" is already baked into the formatted value. */
function unitFor(obj: string): string | null {
  return isPercentObjective(obj) ? null : objectiveUnit(obj);
}

/** Best value per objective across the returned front — the extreme Pareto
 *  point for each objective. For a single solution these are just its values;
 *  for a front each objective's optimum is taken independently (minimum SIGNED
 *  value — direction already encoded), so the values may come from different
 *  solutions, hence the per-card solution index. */
export default function BestValues({ front }: { front: BestValuesFront }) {
  if (front.solutions.length === 0) {
    return (
      <p className="text-sm text-muted-foreground">No solution returned.</p>
    );
  }
  return (
    <div className="flex flex-col gap-2">
      {front.solutions.length > 1 && (
        <p className="text-xs text-muted-foreground">
          Best per objective, across {front.n_solutions} solutions — values may
          come from different solutions.
        </p>
      )}
      <div className="grid grid-cols-1 gap-3 sm:grid-cols-2 lg:grid-cols-3">
        {front.objectives.map((o) => {
          let best: BestValuesFront["solutions"][number] | null = null;
          for (const sol of front.solutions) {
            const v = sol.objectives_signed[o];
            if (v == null) continue;
            const b = best?.objectives_signed[o];
            if (b == null || v < b) best = sol;
          }
          const unit = unitFor(o);
          return (
            <div
              key={o}
              className="flex flex-col gap-1 rounded-xl border border-border bg-card p-4"
            >
              <p className="text-xs text-muted-foreground">{o}</p>
              <div className="flex items-baseline justify-between gap-2">
                <p className="text-2xl font-semibold tabular-nums text-foreground">
                  {fmtObjValue(o, best?.objectives_abs[o])}
                  {unit && (
                    <span className="ml-1 text-sm font-normal text-muted-foreground">
                      {unit}
                    </span>
                  )}
                </p>
                {best && front.solutions.length > 1 && (
                  <p className="text-xs tabular-nums text-muted-foreground">
                    #{best.index}
                  </p>
                )}
              </div>
            </div>
          );
        })}
      </div>
    </div>
  );
}
