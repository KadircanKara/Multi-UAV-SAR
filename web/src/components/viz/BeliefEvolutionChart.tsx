"use client";

/**
 * BeliefEvolutionChart — belief of one target cell over simulation steps.
 * One line per config (none/onboard/gcs), plus a dashed reference line at B.
 * The target cell is chosen from a dropdown in the header; every config is
 * plotted for the SAME cell, so the comparison stays like-for-like.
 * Loaded via next/dynamic({ ssr: false }).
 */

import { useState } from "react";
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
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";

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
  // The user's pick, NOT the resolved cell: holding the pick means a later
  // compare run with a different target set falls back to its own first target
  // instead of plotting a cell that is no longer a target.
  const [pickedCell, setPickedCell] = useState<number | null>(null);

  if (!rows.length) return null;

  const firstRow = rows[0]!;
  // dedupe: targets are parsed from a free-text field, so "12,12" is possible
  // and would give the Select two items with the same value.
  const targets = Array.from(new Set(firstRow.target_locations));
  const selectedCell =
    pickedCell !== null && targets.includes(pickedCell)
      ? pickedCell
      : targets[0] ?? 0;
  const targetIndex = targets.indexOf(selectedCell);
  const targetLabel = targetIndex >= 0 ? `T${targetIndex + 1}` : "TARGET";

  const numSteps = firstRow.cell_occupancy_probabilities[selectedCell]?.length ?? 0;
  const beliefThreshold = firstRow.belief_threshold;

  // Build data array: [{step: 0, none: 0.1, onboard: 0.12, gcs: 0.15}, ...]
  // Every row is read at selectedCell, not at its own first target: the whole
  // point is to compare configs on ONE cell. Missing steps stay null rather
  // than 0 -- a config whose mission ended earlier has a shorter series, and
  // 0 would draw it as "certainly no target" instead of ending the line.
  const chartData = Array.from({ length: numSteps }, (_, step) => {
    const point: Record<string, number | null> = { step };
    for (const row of rows) {
      const series = row.cell_occupancy_probabilities[selectedCell];
      point[row.label] = series?.[step] ?? null;
    }
    return point;
  });

  return (
    <div className="flex flex-col gap-2">
      {/* Header row: the Select lives OUT of the plot area, so it cannot collide
          with the threshold line's insideTopRight "B=..." label. */}
      <div className="flex items-center justify-between gap-2">
        <p className="text-xs text-muted-foreground tracking-widest font-mono uppercase">
          BELIEF EVOLUTION — {targetLabel} · CELL {selectedCell}
        </p>
        {targets.length > 0 && (
          <Select
            value={String(selectedCell)}
            onValueChange={(v) => setPickedCell(Number(v))}
          >
            <SelectTrigger className="h-7 w-40 text-xs font-mono">
              <SelectValue />
            </SelectTrigger>
            <SelectContent>
              {targets.map((cell, i) => (
                <SelectItem key={cell} value={String(cell)} className="text-xs font-mono">
                  T{i + 1} — Cell {cell}
                </SelectItem>
              ))}
            </SelectContent>
          </Select>
        )}
      </div>
      {!numSteps ? (
        <p className="text-xs text-muted-foreground font-mono">
          NO STEP DATA AVAILABLE
        </p>
      ) : (
      <div className="h-64 w-full">
        <ResponsiveContainer width="100%" height="100%">
          <LineChart data={chartData} margin={{ top: 8, right: 16, bottom: 16, left: 0 }}>
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
              verticalAlign="top"
              align="left"
              wrapperStyle={{
                fontFamily: "var(--font-mono)",
                fontSize: 10,
                paddingBottom: 8,
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
                connectNulls={false}
                activeDot={{ r: 4, stroke: colors.series[i] ?? colors.series[0] }}
              />
            ))}
          </LineChart>
        </ResponsiveContainer>
      </div>
      )}
    </div>
  );
}
