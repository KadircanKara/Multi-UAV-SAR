"use client";

/**
 * MetricComparisonView — the shared dispatcher for the "snapshot" compare views
 * (bar grid / table). The Line view is handled separately by
 * ParameterEffectChart. Reused by both the mission route (topologies) and the
 * comparison page (scenarios). Also the home of the shared CompareMetric /
 * CompareEntity data shapes.
 */

import CompareBarChart from "@/components/viz/CompareBarChart";
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
}

// ─── Component ────────────────────────────────────────────────────────────────

export default function MetricComparisonView({
  metrics,
  entities,
  chartType,
  metricOptimizedBy,
  heightClass,
}: MetricComparisonViewProps) {
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

  // chartType === "bar" → one chart per metric in a responsive grid
  return (
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
  );
}
