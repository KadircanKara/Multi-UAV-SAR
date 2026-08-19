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
  Tooltip as RechartsTooltip,
  ResponsiveContainer,
} from "recharts";
import { alpha, axisStyles, useChartColors } from "@/hooks/useChartColors";
import EffectLegend, { buildPalette } from "@/components/viz/EffectLegend";
import { cn } from "@/lib/utils";
import {
  clampObjectiveDomain,
  isPercentObjective,
  objectiveUnit,
  percentString,
  percentTick,
} from "@/lib/objective-format";

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
  /** Draw this chart's own legend. Set false when a grid of these charts shares
   *  ONE legend above it — repeating an identical six-model legend over every
   *  chart is noise. Colours stay index-based, so a shared legend is only
   *  truthful while every chart receives the same series list in the same
   *  order. */
  showLegend?: boolean;
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
          {entry.value == null
            ? "—"
            : isPercentObjective(objective)
              ? percentString(Number(entry.value))
              : Number(entry.value).toFixed(4)}
        </p>
      ))}
      {singleSeriesKey &&
        row[`min_${singleSeriesKey}`] != null &&
        row[`max_${singleSeriesKey}`] != null && (
          <p className="text-muted-foreground tabular-nums">
            range:{" "}
            {isPercentObjective(objective)
              ? percentString(Number(row[`min_${singleSeriesKey}`]))
              : Number(row[`min_${singleSeriesKey}`]).toFixed(4)}{" "}
            –{" "}
            {isPercentObjective(objective)
              ? percentString(Number(row[`max_${singleSeriesKey}`]))
              : Number(row[`max_${singleSeriesKey}`]).toFixed(4)}
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
  showLegend = true,
  heightClass = "h-48",
}: ParameterEffectChartProps) {
  const colors = useChartColors();
  const ax = axisStyles(colors);

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
    // Percentage Connectivity caps at 100% — never let padding overshoot it.
    yDomain = clampObjectiveDomain(objective, [lo - pad, hi + pad]);
  }

  const betterHint = polarity === -1 ? "higher is better" : "lower is better";
  const unit = objectiveUnit(objective);
  const singleSeriesKey = multi ? undefined : series[0]?.key;

  return (
    <div className="flex flex-col gap-1">
      <p className="text-xs font-mono tracking-widest uppercase text-foreground">
        {objective}
        {/* Unit and polarity share one bracket — two would read as two
            separate asides about the same axis. */}
        <span className="ml-2 text-muted-foreground normal-case tracking-normal">
          ({unit ? `${unit}, ` : ""}
          {betterHint})
        </span>
      </p>
      {showLegend && <EffectLegend series={series} fallbackLabel={objective} />}
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
                style: ax.label,
              }}
              tick={ax.tick}
              tickLine={false}
              axisLine={ax.axisLine}
            />
            <YAxis
              domain={yDomain ?? ["auto", "auto"]}
              tickFormatter={(v: number) =>
                isPercentObjective(objective)
                  ? percentTick(v)
                  : Math.abs(v) >= 100
                    ? v.toFixed(0)
                    : v.toFixed(1)
              }
              tick={ax.tick}
              tickLine={false}
              axisLine={ax.axisLine}
              width={60}
            />
            <RechartsTooltip
              content={
                <EffectTooltip
                  objective={objective}
                  singleSeriesKey={singleSeriesKey}
                />
              }
              cursor={{ stroke: alpha(colors.reference, 0.4) }}
            />
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
