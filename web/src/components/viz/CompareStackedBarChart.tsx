"use client";

/**
 * CompareStackedBarChart — one metric, x-axis = parameter combination
 * (drones · comm · n_visits), with one STACKED bar segment per model. Used by the
 * compare page so models can be read against each other at each combo. Legend is
 * plain HTML above the plot (centered) so it wraps freely without overlapping the
 * chart — mirrors ParameterEffectChart. Loaded via next/dynamic({ ssr: false }).
 */

import {
  BarChart,
  Bar,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip as RechartsTooltip,
  ResponsiveContainer,
} from "recharts";
import { alpha, axisStyles, useChartColors } from "@/hooks/useChartColors";
import { cn } from "@/lib/utils";
import type { StackedRow } from "@/components/compare/buildStackedBars";

// Palette: theme tokens first, then golden-angle hues (mirrors ParameterEffectChart).
function buildPalette(base: string[], n: number): string[] {
  if (n <= base.length) return base.slice(0, Math.max(n, 1));
  const out = [...base];
  for (let i = base.length; i < n; i++) {
    const hue = Math.round((i * 137.508) % 360);
    out.push(`hsl(${hue} 70% 58%)`);
  }
  return out;
}

export interface CompareStackedBarChartProps {
  metric: string;
  /** polarity 1 = lower is better; -1 = higher is better */
  polarity: number;
  rows: StackedRow[];
  /** ordered model keys = the stack series */
  models: string[];
  heightClass?: string;
}

interface TooltipEntry {
  name?: string;
  value?: number | string | null;
  color?: string;
}

function StackedTooltip({
  active,
  payload,
  label,
}: {
  active?: boolean;
  payload?: TooltipEntry[];
  label?: string;
}) {
  if (!active || !payload?.length) return null;
  return (
    <div className="rounded border border-border bg-popover px-3 py-2 text-xs shadow-lg">
      <p className="mb-1 font-semibold text-primary">{label}</p>
      {payload.map((e) => (
        <p key={String(e.name)} className="tabular-nums text-foreground">
          <span
            className="mr-1.5 inline-block size-2 rounded-[2px] align-middle"
            style={{ backgroundColor: e.color }}
            aria-hidden="true"
          />
          {e.name}: {e.value != null ? Number(e.value).toFixed(4) : "—"}
        </p>
      ))}
    </div>
  );
}

export default function CompareStackedBarChart({
  metric,
  polarity,
  rows,
  models,
  heightClass = "h-60",
}: CompareStackedBarChartProps) {
  const colors = useChartColors();
  const ax = axisStyles(colors);
  const palette = buildPalette(colors.series, models.length);
  const colorFor = (i: number) => palette[i] ?? colors.series[0]!;
  const betterHint = polarity === -1 ? "higher is better" : "lower is better";

  // X-axis labels steepen and shrink as bars pack in, so the full-width
  // (one-per-row) layout stays legible up to ~36 combos. At ~1168px wide that's
  // a ~32px pitch per bar; a -60° label footprint (~24px) clears it, where the
  // shallow -25° default (~54px) would overlap. Below ~16 bars the chart may be
  // two-per-row (~530px), where the shallow angle reads best.
  const n = rows.length;
  const xAngle = n > 24 ? -60 : n > 16 ? -45 : -25;
  const xFontSize = n > 24 ? 8 : 9;
  const xHeight = n > 24 ? 62 : n > 16 ? 54 : 44;

  return (
    <div className="flex flex-col gap-1">
      <p className="text-xs font-medium text-foreground">
        {metric}
        <span className="ml-2 font-normal text-muted-foreground">
          ({betterHint})
        </span>
      </p>

      {/* HTML legend above the plot (models = stack series) */}
      {models.length > 1 && (
        <div className="flex flex-wrap items-center justify-center gap-x-3 gap-y-1">
          {models.map((m, i) => (
            <span
              key={m}
              className="flex items-center gap-1.5 text-[10px] text-muted-foreground"
            >
              <span
                className="inline-block size-2 rounded-[2px]"
                style={{ backgroundColor: colorFor(i) }}
                aria-hidden="true"
              />
              {m}
            </span>
          ))}
        </div>
      )}

      <div className={cn("w-full", heightClass)}>
        <ResponsiveContainer width="100%" height="100%">
          <BarChart data={rows} margin={{ top: 12, right: 16, bottom: 16, left: 8 }}>
            <CartesianGrid strokeDasharray="3 3" stroke={colors.grid} />
            <XAxis
              dataKey="comboLabel"
              type="category"
              height={xHeight}
              tickMargin={8}
              interval={0}
              angle={xAngle}
              textAnchor="end"
              tick={{ ...ax.tick, fontSize: xFontSize }}
              tickLine={false}
              axisLine={ax.axisLine}
            />
            <YAxis
              tickFormatter={(v: number) =>
                Math.abs(v) >= 100 ? v.toFixed(0) : v.toFixed(1)
              }
              tick={ax.tick}
              tickLine={false}
              axisLine={ax.axisLine}
              width={56}
            />
            <RechartsTooltip
              content={<StackedTooltip />}
              cursor={{ fill: alpha(colors.reference, 0.1) }}
            />
            {models.map((m, i) => (
              <Bar
                key={m}
                dataKey={m}
                name={m}
                stackId="stack"
                fill={colorFor(i)}
                maxBarSize={48}
                isAnimationActive={false}
              />
            ))}
          </BarChart>
        </ResponsiveContainer>
      </div>
    </div>
  );
}
