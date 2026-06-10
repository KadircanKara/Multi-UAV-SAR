"use client";

/**
 * ParameterEffectChart — a line chart showing how one objective's best value
 * changes across a sweep parameter. Supports MULTIPLE overlaid series (one line
 * per combination of the non-swept parameters); a legend is shown only when
 * there is more than one line. The y-axis is fit to the combined trend range of
 * all series (not anchored at 0) so trends stay readable; the per-point Pareto
 * spread is available in the tooltip (single-series mode).
 * Loaded via next/dynamic({ ssr: false }) from the model page.
 */

import {
  ComposedChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Legend,
  Tooltip as RechartsTooltip,
  ResponsiveContainer,
} from "recharts";
import { useChartColors } from "@/hooks/useChartColors";
import { cn } from "@/lib/utils";

// ─── Types ────────────────────────────────────────────────────────────────────

export interface EffectPoint {
  /** x-axis label / value (may be string for comm_range) */
  xLabel: string;
  xNum: number;
  best: number | null;
  min: number | null;
  max: number | null;
}

export interface EffectSeries {
  /** stable unique key for this line */
  key: string;
  /** legend label (empty string ⇒ single-series, no legend) */
  label: string;
  points: EffectPoint[];
}

export interface ParameterEffectChartProps {
  objective: string;
  /** polarity 1 = lower is better; -1 = higher is better */
  polarity: number;
  sweepLabel: string;
  series: EffectSeries[];
  /** base color index for single-series mode (so objective charts differ) */
  colorIndex?: number;
  /** tailwind height class for the plot area (grows with series count) */
  heightClass?: string;
}

// ─── Palette: theme tokens first, then generated distinct hues ────────────────

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

// ─── Custom tooltip ───────────────────────────────────────────────────────────

interface TooltipEntry {
  dataKey?: string | number;
  name?: string;
  value?: number | string | null;
  color?: string;
  payload?: Record<string, number | string | null>;
}

function EffectTooltip({
  active,
  payload,
  objective,
  singleSeriesKey,
}: {
  active?: boolean;
  payload?: TooltipEntry[];
  objective: string;
  /** when set, the chart has one series — show the Pareto range too */
  singleSeriesKey?: string;
}) {
  if (!active || !payload?.length) return null;
  const row = payload[0]?.payload;
  if (!row) return null;
  const xLabel = row.xLabel as string;

  return (
    <div className="rounded border border-border bg-popover px-3 py-2 font-mono text-xs shadow-lg">
      <p className="text-primary font-semibold mb-1">{xLabel}</p>
      {payload.map((entry) => (
        <p
          key={String(entry.dataKey)}
          className="tabular-nums"
          style={{ color: entry.color }}
        >
          {(singleSeriesKey ? objective : entry.name) ?? objective}:{" "}
          {entry.value != null ? Number(entry.value).toFixed(4) : "—"}
        </p>
      ))}
      {singleSeriesKey &&
        row[`min_${singleSeriesKey}`] != null &&
        row[`max_${singleSeriesKey}`] != null && (
          <p className="text-muted-foreground tabular-nums">
            range: {Number(row[`min_${singleSeriesKey}`]).toFixed(4)} –{" "}
            {Number(row[`max_${singleSeriesKey}`]).toFixed(4)}
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
  series,
  colorIndex = 0,
  heightClass = "h-48",
}: ParameterEffectChartProps) {
  const colors = useChartColors();

  const multi = series.length > 1;
  const palette = buildPalette(colors.series, series.length);
  // Single-series keeps a per-objective color; multi uses the palette in order.
  const colorFor = (i: number): string =>
    multi
      ? (palette[i] ?? colors.series[0]!)
      : (colors.series[colorIndex % colors.series.length] ?? colors.series[0]!);

  // Pivot all series onto a shared, xNum-sorted x axis.
  const xByLabel = new Map<string, number>();
  for (const s of series) {
    for (const p of s.points) {
      if (!xByLabel.has(p.xLabel)) xByLabel.set(p.xLabel, p.xNum);
    }
  }
  const xs = Array.from(xByLabel.entries())
    .sort((a, b) => a[1] - b[1])
    .map(([xLabel]) => xLabel);

  const data = xs.map((xLabel) => {
    const row: Record<string, number | string | null> = { xLabel };
    for (const s of series) {
      const pt = s.points.find((p) => p.xLabel === xLabel);
      row[`best_${s.key}`] = pt?.best ?? null;
      row[`min_${s.key}`] = pt?.min ?? null;
      row[`max_${s.key}`] = pt?.max ?? null;
    }
    return row;
  });

  // Fit the y-axis to the combined best-value trend (with padding) across all
  // series so trends are clearly visible (the Pareto spread can dwarf the trend).
  const bests = series
    .flatMap((s) => s.points.map((p) => p.best))
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
  const singleSeriesKey = multi ? undefined : series[0]?.key;

  return (
    <div className="flex flex-col gap-1">
      <p className="text-xs font-mono tracking-widest uppercase text-foreground">
        {objective}
        <span className="ml-2 text-muted-foreground normal-case tracking-normal">
          ({betterHint})
        </span>
      </p>
      <div className={cn("w-full", heightClass)}>
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
                <EffectTooltip
                  objective={objective}
                  singleSeriesKey={singleSeriesKey}
                />
              }
              cursor={{ stroke: `${colors.reference}66` }}
            />
            {multi && (
              <Legend
                wrapperStyle={{
                  fontFamily: "var(--font-mono)",
                  fontSize: 10,
                  paddingTop: 6,
                }}
                iconType="plainline"
              />
            )}
            {series.map((s, i) => (
              <Line
                key={s.key}
                type="monotone"
                dataKey={`best_${s.key}`}
                name={s.label || objective}
                stroke={colorFor(i)}
                strokeWidth={2}
                dot={{ fill: colorFor(i), r: 3, strokeWidth: 0 }}
                activeDot={{ fill: colorFor(i), r: 5, strokeWidth: 0 }}
                connectNulls={false}
                isAnimationActive={false}
              />
            ))}
          </ComposedChart>
        </ResponsiveContainer>
      </div>
    </div>
  );
}
