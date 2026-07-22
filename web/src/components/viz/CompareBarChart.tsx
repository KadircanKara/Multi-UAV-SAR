"use client";

/**
 * CompareBarChart — a single-metric bar chart comparing entities. Each bar is an
 * entity (e.g. a scenario or topology), colored individually from the chart-color
 * palette. The y-axis is fit to the value range (with ~12% padding) so trends are
 * readable, mirroring ParameterEffectChart. Null values are simply absent bars.
 * Loaded via next/dynamic({ ssr: false }) where used.
 */

import {
  BarChart,
  Bar,
  Cell,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip as RechartsTooltip,
  ResponsiveContainer,
} from "recharts";
import { alpha, axisStyles, useChartColors } from "@/hooks/useChartColors";
import { cn } from "@/lib/utils";

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

export interface CompareBarPoint {
  key: string;
  label: string;
  value: number | null;
}

export interface CompareBarChartProps {
  metric: string;
  /** polarity 1 = lower is better; -1 = higher is better */
  polarity: number;
  points: CompareBarPoint[];
  /** tailwind height class for the plot area */
  heightClass?: string;
}

// ─── Custom tooltip ───────────────────────────────────────────────────────────

interface TooltipEntry {
  value?: number | string | null;
  color?: string;
  payload?: Record<string, number | string | null>;
}

function BarTooltip({
  active,
  payload,
  metric,
}: {
  active?: boolean;
  payload?: TooltipEntry[];
  metric: string;
}) {
  if (!active || !payload?.length) return null;
  const row = payload[0]?.payload;
  if (!row) return null;
  const label = row.label as string;
  const value = payload[0]?.value;

  return (
    <div className="rounded border border-border bg-popover px-3 py-2 font-mono text-xs shadow-lg">
      <p className="text-primary font-semibold mb-1">{label}</p>
      <p className="tabular-nums text-foreground">
        {metric}: {value != null ? Number(value).toFixed(4) : "—"}
      </p>
    </div>
  );
}

// ─── Main chart component ─────────────────────────────────────────────────────

export default function CompareBarChart({
  metric,
  polarity,
  points,
  heightClass = "h-56",
}: CompareBarChartProps) {
  const colors = useChartColors();
  const ax = axisStyles(colors);
  const palette = buildPalette(colors.series, points.length);

  // Fit the y-axis to the value range (with padding) so differences read clearly.
  const values = points
    .map((p) => p.value)
    .filter((v): v is number => v != null && Number.isFinite(v));
  let yDomain: [number, number] | undefined;
  if (values.length > 0) {
    const lo = Math.min(...values);
    const hi = Math.max(...values);
    const span = hi - lo;
    const pad = span > 0 ? span * 0.12 : Math.max(Math.abs(hi) * 0.1, 1);
    yDomain = [lo - pad, hi + pad];
  }

  const betterHint = polarity === -1 ? "higher is better" : "lower is better";

  return (
    <div className="flex flex-col gap-1">
      <p className="text-xs font-mono tracking-widest uppercase text-foreground">
        {metric}
        <span className="ml-2 text-muted-foreground normal-case tracking-normal">
          ({betterHint})
        </span>
      </p>
      <div className={cn("w-full", heightClass)}>
        <ResponsiveContainer width="100%" height="100%">
          <BarChart
            data={points}
            margin={{ top: 8, right: 12, bottom: 24, left: 8 }}
          >
            <CartesianGrid strokeDasharray="3 3" stroke={colors.grid} />
            <XAxis
              dataKey="label"
              type="category"
              tick={ax.tick}
              tickLine={false}
              axisLine={ax.axisLine}
            />
            <YAxis
              domain={yDomain ?? ["auto", "auto"]}
              tickFormatter={(v: number) =>
                Math.abs(v) >= 100 ? v.toFixed(0) : v.toFixed(1)
              }
              tick={ax.tick}
              tickLine={false}
              axisLine={ax.axisLine}
              width={60}
            />
            <RechartsTooltip
              content={<BarTooltip metric={metric} />}
              cursor={{ fill: alpha(colors.reference, 0.1) }}
            />
            <Bar dataKey="value" isAnimationActive={false}>
              {points.map((p, i) => (
                <Cell
                  key={p.key}
                  fill={palette[i] ?? colors.series[0]!}
                />
              ))}
            </Bar>
          </BarChart>
        </ResponsiveContainer>
      </div>
    </div>
  );
}
