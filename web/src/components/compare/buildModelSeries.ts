/**
 * buildModelSeries — pure helper for the compare-page Line view.
 *
 * Given a flat list of scenario "rows" (each carrying its model + parameter
 * coordinates and ONE numeric value for the chosen metric/objective) plus the
 * chosen sweep parameter, it groups the rows by `model_key` into one
 * EffectSeries per model. Points are placed on the sweep axis (xNum) and, when
 * several rows of the same model land on the same x (because the non-swept
 * params have multiple selected values), they collapse to the polarity-aware
 * best (min if polarity 1 = lower-is-better, max if polarity -1 = higher).
 *
 * Generic over the source: the objectives tab feeds `objective_stats[obj][stat]`
 * and the time-metrics tab feeds `metric_values[metric]` — both become a single
 * `value` per row here.
 */

import type { EffectSeries, EffectPoint } from "@/components/viz/ParameterEffectChart";

export type SweepParam = "drones" | "comm_range" | "n_visits";

/** A flattened scenario coordinate + its single value for the active metric. */
export interface ModelSeriesRow {
  model_key: string;
  number_of_drones: number | null;
  comm_range: string | null;
  comm_range_value: number | null;
  n_visits: number | null;
  /** the value for the active objective/metric (already reduced to one number) */
  value: number | null;
}

// X-axis coordinate (numeric, for sorting) for a row along the sweep dimension.
function sweepNum(row: ModelSeriesRow, sweep: SweepParam): number {
  if (sweep === "drones") return row.number_of_drones ?? 0;
  if (sweep === "comm_range") return row.comm_range_value ?? 0;
  return row.n_visits ?? 0;
}

// Display label for a row along the sweep dimension (matches the param value).
function sweepLabel(row: ModelSeriesRow, sweep: SweepParam): string {
  if (sweep === "drones")
    return row.number_of_drones != null ? String(row.number_of_drones) : "—";
  if (sweep === "comm_range") return row.comm_range ?? "—";
  return row.n_visits != null ? String(row.n_visits) : "—";
}

// True when the row actually has a value on the sweep axis.
function sweepHasValue(row: ModelSeriesRow, sweep: SweepParam): boolean {
  if (sweep === "drones") return row.number_of_drones != null;
  if (sweep === "comm_range") return row.comm_range_value != null;
  return row.n_visits != null;
}

/**
 * Build one EffectSeries per distinct model_key.
 *
 * @param rows     flattened scenario rows with a single `value` each
 * @param sweep    which parameter forms the x-axis
 * @param polarity 1 = lower is better, -1 = higher is better (collapse direction)
 */
export function buildModelSeries(
  rows: ModelSeriesRow[],
  sweep: SweepParam,
  polarity: number
): EffectSeries[] {
  // Group by model, then by sweep-x label, collapsing duplicates polarity-aware.
  const byModel = new Map<
    string,
    Map<string, { xNum: number; best: number | null }>
  >();

  for (const row of rows) {
    if (!sweepHasValue(row, sweep)) continue;
    let xMap = byModel.get(row.model_key);
    if (!xMap) {
      xMap = new Map();
      byModel.set(row.model_key, xMap);
    }
    const xLabel = sweepLabel(row, sweep);
    const xNum = sweepNum(row, sweep);
    const prev = xMap.get(xLabel);
    const value = row.value;
    if (!prev) {
      xMap.set(xLabel, { xNum, best: value ?? null });
      continue;
    }
    if (value == null) continue;
    if (prev.best == null) {
      prev.best = value;
      continue;
    }
    // Polarity-aware collapse: keep the better of the two.
    prev.best = polarity === -1 ? Math.max(prev.best, value) : Math.min(prev.best, value);
  }

  const series: EffectSeries[] = [];
  for (const [model_key, xMap] of Array.from(byModel.entries())) {
    const points: EffectPoint[] = Array.from(xMap.entries())
      .map(([xLabel, agg]): EffectPoint => ({
        xLabel,
        xNum: agg.xNum,
        best: agg.best,
        min: null,
        max: null,
      }))
      .sort((a, b) => a.xNum - b.xNum);
    series.push({ key: model_key, label: model_key, points });
  }

  // Stable ordering by model name so colors are consistent across renders.
  series.sort((a, b) => a.key.localeCompare(b.key));
  return series;
}
