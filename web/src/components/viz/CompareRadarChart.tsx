"use client";

/**
 * CompareRadarChart — one radar polygon per entity, axes = metrics. Each metric
 * is min–max normalized across entities and polarity-flipped so that BETTER is
 * always nearer the outer edge (radius 1). Raw values are kept available in the
 * tooltip so the user sees real numbers, not the 0–1 normalized ones.
 * Loaded via next/dynamic({ ssr: false }) where used.
 */

import {
  RadarChart,
  Radar,
  PolarGrid,
  PolarAngleAxis,
  PolarRadiusAxis,
  Legend,
  Tooltip as RechartsTooltip,
  ResponsiveContainer,
} from "recharts";
import { useChartColors } from "@/hooks/useChartColors";
import { cn } from "@/lib/utils";
import type {
  CompareMetric,
  CompareEntity,
} from "@/components/compare/MetricComparisonView";

// ─── Palette: theme tokens first, then generated distinct hues ────────────────
// (copied from ParameterEffectChart's buildPalette golden-angle helper)

function buildPalette(base: string[], n: number): string[] {
  if (n <= base.length) return base.slice(0, Math.max(n, 1));
  const out = [...base];
  for (let i = base.length; i < n; i++) {
    // golden-angle hue spacing → maximally distinct categorical colors
    const hue = Math.round((i * 137.508) % 360);
    out.push(`hsl(${hue} 70% 58%)`);
  }
  return out;
}

// ─── Types ────────────────────────────────────────────────────────────────────

export interface CompareRadarChartProps {
  metrics: CompareMetric[];
  entities: CompareEntity[];
  /** tailwind height class for the plot area */
  heightClass?: string;
}

// ─── Custom tooltip (shows raw, not normalized, values) ───────────────────────

interface TooltipEntry {
  dataKey?: string | number;
  name?: string;
  color?: string;
  payload?: Record<string, number | string | null>;
}

function RadarTooltip({
  active,
  payload,
}: {
  active?: boolean;
  payload?: TooltipEntry[];
}) {
  if (!active || !payload?.length) return null;
  const row = payload[0]?.payload;
  if (!row) return null;
  const metric = row.metric as string;

  return (
    <div className="rounded border border-border bg-popover px-3 py-2 font-mono text-xs shadow-lg">
      <p className="text-primary font-semibold mb-1">{metric}</p>
      {payload.map((entry) => {
        const raw = row[`${String(entry.dataKey)}__raw`];
        return (
          <p
            key={String(entry.dataKey)}
            className="tabular-nums"
            style={{ color: entry.color }}
          >
            {entry.name}:{" "}
            {raw != null ? Number(raw).toFixed(4) : "—"}
          </p>
        );
      })}
    </div>
  );
}

// ─── Main chart component ─────────────────────────────────────────────────────

export default function CompareRadarChart({
  metrics,
  entities,
  heightClass = "h-80",
}: CompareRadarChartProps) {
  const colors = useChartColors();
  const palette = buildPalette(colors.series, entities.length);

  // One row per metric; per metric compute min/max across entities' finite
  // values, then normalize so better → outer edge (radius 1).
  const data = metrics.map((metric) => {
    const finite = entities
      .map((e) => e.values[metric.name])
      .filter((v): v is number => v != null && Number.isFinite(v));
    const min = finite.length ? Math.min(...finite) : 0;
    const max = finite.length ? Math.max(...finite) : 0;
    const span = max - min;

    const row: Record<string, number | string | null> = {
      metric: metric.name,
    };
    for (const e of entities) {
      const v = e.values[metric.name];
      let normalized: number;
      if (v == null || !Number.isFinite(v)) {
        normalized = 0;
      } else if (span === 0) {
        normalized = 0.5;
      } else {
        normalized =
          metric.polarity === 1 ? (max - v) / span : (v - min) / span;
      }
      row[e.key] = normalized;
      row[`${e.key}__raw`] = v ?? null;
    }
    return row;
  });

  return (
    <div className={cn("w-full", heightClass)}>
      <ResponsiveContainer width="100%" height="100%">
        <RadarChart data={data} margin={{ top: 8, right: 8, bottom: 8, left: 8 }}>
          <PolarGrid stroke={colors.grid} />
          <PolarAngleAxis
            dataKey="metric"
            tick={{
              fontFamily: "var(--font-mono)",
              fontSize: 10,
              fill: colors.axis,
            }}
          />
          <PolarRadiusAxis domain={[0, 1]} tick={false} axisLine={false} />
          <RechartsTooltip content={<RadarTooltip />} />
          <Legend
            wrapperStyle={{
              fontFamily: "var(--font-mono)",
              fontSize: 10,
              paddingTop: 6,
            }}
            iconType="plainline"
          />
          {entities.map((e, i) => {
            const color = palette[i] ?? colors.series[0]!;
            return (
              <Radar
                key={e.key}
                name={e.label}
                dataKey={e.key}
                stroke={color}
                fill={color}
                fillOpacity={0.15}
                strokeWidth={2}
                isAnimationActive={false}
              />
            );
          })}
        </RadarChart>
      </ResponsiveContainer>
    </div>
  );
}
