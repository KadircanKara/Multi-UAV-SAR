"use client";

/**
 * TargetsKnownChart — count of targets with belief > B at each step.
 * One step-line per config. Loaded via next/dynamic({ ssr: false }).
 */

import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip as RechartsTooltip,
  Legend,
  ResponsiveContainer,
} from "recharts";
import { useChartColors } from "@/hooks/useChartColors";

// ─── Types ────────────────────────────────────────────────────────────────────

export interface TargetsKnownRow {
  label: string;
  cell_occupancy_probabilities: number[][];
  target_locations: number[];
  belief_threshold: number;
}

interface Props {
  rows: TargetsKnownRow[];
}

// ─── Component ────────────────────────────────────────────────────────────────

export default function TargetsKnownChart({ rows }: Props) {
  const colors = useChartColors();

  if (!rows.length) return null;

  const firstRow = rows[0]!;
  const firstTarget = firstRow.target_locations[0] ?? 0;
  const referenceSeries = firstRow.cell_occupancy_probabilities[firstTarget];
  const numSteps = referenceSeries?.length ?? 0;

  if (!numSteps) {
    return (
      <p className="text-xs text-muted-foreground font-mono">
        NO STEP DATA AVAILABLE
      </p>
    );
  }

  // Derive targets-known count per step per config
  const chartData = Array.from({ length: numSteps }, (_, step) => {
    const point: Record<string, number> = { step };
    for (const row of rows) {
      const B = row.belief_threshold;
      let count = 0;
      for (const t of row.target_locations) {
        const series = row.cell_occupancy_probabilities[t];
        if (series && (series[step] ?? 0) > B) count++;
      }
      point[row.label] = count;
    }
    return point;
  });

  const totalTargets = firstRow.target_locations.length;

  return (
    <div className="flex flex-col gap-2">
      <p className="text-xs text-muted-foreground tracking-widest font-mono uppercase">
        TARGETS KNOWN (BELIEF &gt; B) — {totalTargets} TARGET
        {totalTargets !== 1 ? "S" : ""}
      </p>
      <div className="h-64 w-full">
        <ResponsiveContainer width="100%" height="100%">
          <LineChart data={chartData} margin={{ top: 8, right: 16, bottom: 8, left: 0 }}>
            <CartesianGrid strokeDasharray="3 3" stroke="hsl(205 30% 13%)" />
            <XAxis
              dataKey="step"
              tick={{ fontFamily: "var(--font-mono)", fontSize: 10, fill: "hsl(169 8% 45%)" }}
              tickLine={false}
              axisLine={{ stroke: "hsl(205 30% 13%)" }}
              label={{
                value: "STEP",
                position: "insideBottom",
                offset: -4,
                style: { fontFamily: "var(--font-mono)", fontSize: 10, fill: "hsl(169 8% 45%)" },
              }}
            />
            <YAxis
              allowDecimals={false}
              domain={[0, totalTargets]}
              tick={{ fontFamily: "var(--font-mono)", fontSize: 10, fill: "hsl(169 8% 45%)" }}
              tickLine={false}
              axisLine={{ stroke: "hsl(205 30% 13%)" }}
            />
            <RechartsTooltip
              contentStyle={{
                background: "hsl(207 32% 6%)",
                border: "1px solid hsl(205 30% 13%)",
                fontFamily: "var(--font-mono)",
                fontSize: 11,
              }}
              labelStyle={{ color: "hsl(169 8% 45%)" }}
            />
            <Legend
              wrapperStyle={{
                fontFamily: "var(--font-mono)",
                fontSize: 10,
                paddingTop: 8,
              }}
            />
            {rows.map((row, i) => (
              <Line
                key={row.label}
                type="stepAfter"
                dataKey={row.label}
                stroke={colors[i] ?? colors[0]}
                strokeWidth={2}
                dot={false}
                activeDot={{ r: 4, stroke: colors[i] ?? colors[0] }}
              />
            ))}
          </LineChart>
        </ResponsiveContainer>
      </div>
    </div>
  );
}
