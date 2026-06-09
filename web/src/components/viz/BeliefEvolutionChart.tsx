"use client";

/**
 * BeliefEvolutionChart — belief of the first target cell over simulation steps.
 * One line per config (none/onboard/gcs), plus a dashed reference line at B.
 * Loaded via next/dynamic({ ssr: false }).
 */

import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip as RechartsTooltip,
  Legend,
  ReferenceLine,
  ResponsiveContainer,
} from "recharts";
import { useChartColors } from "@/hooks/useChartColors";

// ─── Types ────────────────────────────────────────────────────────────────────

export interface BeliefRow {
  label: string;
  cell_occupancy_probabilities: number[][];
  target_locations: number[];
  belief_threshold: number;
}

interface Props {
  rows: BeliefRow[];
}

// ─── Component ────────────────────────────────────────────────────────────────

export default function BeliefEvolutionChart({ rows }: Props) {
  const colors = useChartColors();

  if (!rows.length) return null;

  // Build step-indexed data from the first row's step count
  const firstRow = rows[0]!;
  const firstTarget = firstRow.target_locations[0] ?? 0;
  const targetSeries = firstRow.cell_occupancy_probabilities[firstTarget];
  const numSteps = targetSeries?.length ?? 0;
  const beliefThreshold = firstRow.belief_threshold;

  if (!numSteps) {
    return (
      <p className="text-xs text-muted-foreground font-mono">
        NO STEP DATA AVAILABLE
      </p>
    );
  }

  // Build data array: [{step: 0, none: 0.1, onboard: 0.12, gcs: 0.15}, ...]
  const chartData = Array.from({ length: numSteps }, (_, step) => {
    const point: Record<string, number> = { step };
    for (const row of rows) {
      const t0 = row.target_locations[0] ?? 0;
      const series = row.cell_occupancy_probabilities[t0];
      point[row.label] = series?.[step] ?? 0;
    }
    return point;
  });

  return (
    <div className="flex flex-col gap-2">
      <p className="text-xs text-muted-foreground tracking-widest font-mono uppercase">
        BELIEF EVOLUTION — FIRST TARGET CELL
      </p>
      <div className="h-64 w-full">
        <ResponsiveContainer width="100%" height="100%">
          <LineChart data={chartData} margin={{ top: 8, right: 16, bottom: 8, left: 0 }}>
            <CartesianGrid strokeDasharray="3 3" stroke={colors.grid} />
            <XAxis
              dataKey="step"
              tick={{ fontFamily: "var(--font-mono)", fontSize: 10, fill: colors.axis }}
              tickLine={false}
              axisLine={{ stroke: colors.grid }}
              label={{
                value: "STEP",
                position: "insideBottom",
                offset: -4,
                style: { fontFamily: "var(--font-mono)", fontSize: 10, fill: colors.axis },
              }}
            />
            <YAxis
              domain={[0, 1]}
              tick={{ fontFamily: "var(--font-mono)", fontSize: 10, fill: colors.axis }}
              tickLine={false}
              axisLine={{ stroke: colors.grid }}
              tickFormatter={(v: number) => v.toFixed(1)}
            />
            <RechartsTooltip
              contentStyle={{
                background: colors.tooltipBg,
                border: `1px solid ${colors.tooltipBorder}`,
                fontFamily: "var(--font-mono)",
                fontSize: 11,
              }}
              labelStyle={{ color: colors.axis }}
            />
            <Legend
              wrapperStyle={{
                fontFamily: "var(--font-mono)",
                fontSize: 10,
                paddingTop: 8,
              }}
            />
            {/* Belief threshold reference line */}
            <ReferenceLine
              y={beliefThreshold}
              stroke={colors.reference}
              strokeDasharray="6 3"
              strokeWidth={1.5}
              label={{
                value: `B=${beliefThreshold.toFixed(2)}`,
                position: "insideTopRight",
                style: {
                  fontFamily: "var(--font-mono)",
                  fontSize: 10,
                  fill: colors.reference,
                },
              }}
            />
            {rows.map((row, i) => (
              <Line
                key={row.label}
                type="monotone"
                dataKey={row.label}
                stroke={colors.series[i] ?? colors.series[0]}
                strokeWidth={2}
                dot={false}
                activeDot={{ r: 4, stroke: colors.series[i] ?? colors.series[0] }}
              />
            ))}
          </LineChart>
        </ResponsiveContainer>
      </div>
    </div>
  );
}
