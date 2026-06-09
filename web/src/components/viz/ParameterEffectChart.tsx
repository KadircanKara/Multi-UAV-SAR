"use client";

/**
 * ParameterEffectChart — one line chart showing how a single objective's best
 * value changes as a sweep parameter varies. The y-axis is fit to the trend
 * range (not anchored at 0) so the trend is clearly visible; the per-point
 * Pareto-front spread is available in the tooltip.
 * Loaded via next/dynamic({ ssr: false }) from the model page.
 */

import {
  ComposedChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip as RechartsTooltip,
  ResponsiveContainer,
} from "recharts";
import { useChartColors } from "@/hooks/useChartColors";

// ─── Types ────────────────────────────────────────────────────────────────────

export interface EffectPoint {
  /** x-axis label / value (may be string for comm_range) */
  xLabel: string;
  xNum: number;
  best: number | null;
  min: number | null;
  max: number | null;
}

export interface ParameterEffectChartProps {
  objective: string;
  /** polarity 1 = lower is better; -1 = higher is better */
  polarity: number;
  sweepLabel: string;
  points: EffectPoint[];
  /** index into colors.series to use (0..4) */
  colorIndex?: number;
}

// ─── Custom tooltip ───────────────────────────────────────────────────────────

interface TooltipPayload {
  payload?: {
    xLabel: string;
    best: number | null;
    min: number | null;
    max: number | null;
  };
}

function EffectTooltip({
  active,
  payload,
  objective,
}: {
  active?: boolean;
  payload?: TooltipPayload[];
  objective: string;
}) {
  if (!active || !payload?.length) return null;
  const d = payload[0]?.payload;
  if (!d) return null;

  return (
    <div className="rounded border border-border bg-popover px-3 py-2 font-mono text-xs shadow-lg">
      <p className="text-primary font-semibold mb-1">{d.xLabel}</p>
      <p className="text-foreground tabular-nums">
        {objective}: {d.best != null ? d.best.toFixed(4) : "—"}
      </p>
      {d.min != null && d.max != null && (
        <p className="text-muted-foreground tabular-nums">
          range: {d.min.toFixed(4)} – {d.max.toFixed(4)}
        </p>
      )}
    </div>
  );
}

// ─── Main chart component ─────────────────────────────────────────────────────

export default function ParameterEffectChart({
  objective,
  polarity,
  sweepLabel,
  points,
  colorIndex = 0,
}: ParameterEffectChartProps) {
  const colors = useChartColors();

  const mainColor = colors.series[colorIndex % colors.series.length] ?? colors.series[0]!;
  const data = points.map((p) => ({
    xLabel: p.xLabel,
    xNum: p.xNum,
    best: p.best,
    min: p.min,
    max: p.max,
  }));

  // Fit the y-axis to the best-value trend (with padding) so the trend is
  // clearly visible. The per-point Pareto-front spread can be far wider than
  // the trend itself (e.g. a ~4-unit trend inside a ~500-unit front), so
  // including it would flatten the line — the spread stays in the tooltip.
  const bests = points
    .map((p) => p.best)
    .filter((v): v is number => v != null && Number.isFinite(v));
  let yDomain: [number, number] | undefined;
  if (bests.length > 0) {
    const lo = Math.min(...bests);
    const hi = Math.max(...bests);
    const span = hi - lo;
    const pad = span > 0 ? span * 0.12 : Math.max(Math.abs(hi) * 0.1, 1);
    yDomain = [lo - pad, hi + pad];
  }

  const betterHint = polarity === -1 ? "higher is better" : "lower is better";

  return (
    <div className="flex flex-col gap-1">
      <p className="text-xs font-mono tracking-widest uppercase text-foreground">
        {objective}
        <span className="ml-2 text-muted-foreground normal-case tracking-normal">
          ({betterHint})
        </span>
      </p>
      <div className="h-48 w-full">
        <ResponsiveContainer width="100%" height="100%">
          <ComposedChart
            data={data}
            margin={{ top: 8, right: 12, bottom: 24, left: 8 }}
          >
            <CartesianGrid strokeDasharray="3 3" stroke={colors.grid} />
            <XAxis
              dataKey="xLabel"
              type="category"
              label={{
                value: sweepLabel,
                position: "insideBottom",
                offset: -10,
                style: {
                  fontFamily: "var(--font-mono)",
                  fontSize: 10,
                  fill: colors.axis,
                },
              }}
              tick={{
                fontFamily: "var(--font-mono)",
                fontSize: 10,
                fill: colors.axis,
              }}
              tickLine={false}
              axisLine={{ stroke: colors.grid }}
            />
            <YAxis
              domain={yDomain ?? ["auto", "auto"]}
              tickFormatter={(v: number) =>
                Math.abs(v) >= 100 ? v.toFixed(0) : v.toFixed(1)
              }
              tick={{
                fontFamily: "var(--font-mono)",
                fontSize: 10,
                fill: colors.axis,
              }}
              tickLine={false}
              axisLine={{ stroke: colors.grid }}
              width={60}
            />
            <RechartsTooltip
              content={
                <EffectTooltip objective={objective} />
              }
              cursor={{ stroke: `${colors.reference}66` }}
            />
            {/* Best-value line */}
            <Line
              type="monotone"
              dataKey="best"
              stroke={mainColor}
              strokeWidth={2}
              dot={{ fill: mainColor, r: 4, strokeWidth: 0 }}
              activeDot={{ fill: mainColor, r: 5, strokeWidth: 0 }}
              connectNulls={false}
              isAnimationActive={false}
            />
          </ComposedChart>
        </ResponsiveContainer>
      </div>
    </div>
  );
}
