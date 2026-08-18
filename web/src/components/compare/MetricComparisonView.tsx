"use client";

/**
 * MetricComparisonView — the shared dispatcher for the "snapshot" compare views
 * (bar grid / table). The Line view is handled separately by
 * ParameterEffectChart. Reused by both the mission route (topologies) and the
 * comparison page (scenarios). Also the home of the shared CompareMetric /
 * CompareEntity data shapes.
 */

import CompareBarChart, { buildPalette } from "@/components/viz/CompareBarChart";
import { useChartColors } from "@/hooks/useChartColors";
import CompareMetricTable from "@/components/compare/CompareMetricTable";

// ─── Shared data shapes ───────────────────────────────────────────────────────

export interface CompareMetric {
  name: string;
  /** polarity 1 = lower is better; -1 = higher is better */
  polarity: number;
}

export interface CompareEntity {
  key: string;
  label: string;
  /** values keyed by metric name */
  values: Record<string, number | null>;
}

export type CompareChartType = "bar" | "table";

export interface MetricComparisonViewProps {
  metrics: CompareMetric[];
  entities: CompareEntity[];
  chartType: CompareChartType;
  /** per-metric-name set of entity keys that actually optimized that metric */
  metricOptimizedBy?: Record<string, Set<string>>;
  /** tailwind height class forwarded to bar charts */
  heightClass?: string;
  /** Bar view only: a line describing the fixed scenario every bar shares
   *  (e.g. "8 drones · comm √8 · n_visits 3"). Rendered once above the grid
   *  instead of repeated in every bar's x-label. */
  caption?: string;
}

// ─── Component ────────────────────────────────────────────────────────────────

export default function MetricComparisonView({
  metrics,
  entities,
  chartType,
  metricOptimizedBy,
  heightClass,
  caption,
}: MetricComparisonViewProps) {
  const colors = useChartColors();
  if (entities.length === 0) {
    return (
      <p className="text-xs font-mono text-muted-foreground border border-dashed border-border rounded px-4 py-3">
        No data for this selection.
      </p>
    );
  }

  if (chartType === "table") {
    const merged = metrics.map((m) => ({
      ...m,
      optimizedBy: metricOptimizedBy?.[m.name],
    }));
    return <CompareMetricTable metrics={merged} entities={entities} />;
  }

  const legendPalette = buildPalette(colors.series, entities.length);

  // chartType === "bar" → one chart per metric in a responsive grid.
  // The bar charts hide their x-axis, so identity lives here: one legend and one
  // caption above the whole grid, rather than the same six rotated labels
  // repeated under every chart.
  return (
    <div className="flex flex-col gap-4">
      <div className="flex flex-col gap-2">
        {caption && (
          <p className="text-xs text-muted-foreground">{caption}</p>
        )}
        <div className="flex flex-wrap items-center gap-x-4 gap-y-1">
          {entities.map((e, i) => (
            <span
              key={e.key}
              className="flex items-center gap-1.5 text-xs text-foreground"
            >
              <span
                className="inline-block size-2.5 rounded-[2px]"
                style={{ backgroundColor: legendPalette[i] ?? colors.series[0] }}
                aria-hidden="true"
              />
              {e.label}
            </span>
          ))}
        </div>
      </div>
      <div className="grid gap-6 grid-cols-1 md:grid-cols-2">
      {metrics.map((m) => {
        const points = entities.map((e) => ({
          key: e.key,
          label: e.label,
          value: e.values[m.name] ?? null,
        }));
        return (
          <CompareBarChart
            key={m.name}
            metric={m.name}
            polarity={m.polarity}
            points={points}
            {...(heightClass ? { heightClass } : {})}
          />
        );
      })}
      </div>
    </div>
  );
}
