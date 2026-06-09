"use client";

/**
 * ParetoScatter — dynamic Recharts ScatterChart of the Pareto front.
 * Loaded via next/dynamic({ ssr: false }) from the explore page.
 */

import { useState } from "react";
import {
  ScatterChart,
  Scatter,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip as RechartsTooltip,
  ResponsiveContainer,
  Cell,
} from "recharts";
import type { ParetoFront } from "@/lib/types";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import { useChartColors } from "@/hooks/useChartColors";

// ─── Types ────────────────────────────────────────────────────────────────────

interface Props {
  front: ParetoFront;
  selectedIndex: number;
  onSelectIndex: (idx: number) => void;
}

// ─── Single-objective readout ─────────────────────────────────────────────────

function SingleObjectiveReadout({ front }: { front: ParetoFront }) {
  const sol = front.solutions[0];
  if (!sol) return null;
  const objName = front.objectives[0] ?? "objective";
  const val = sol.objectives_abs[objName];
  const isMax = front.polarities[objName] === -1;

  return (
    <div className="flex flex-col gap-2 rounded border border-border bg-card p-4">
      <p className="text-xs text-muted-foreground tracking-widest uppercase">
        SINGLE-OBJECTIVE RESULT
      </p>
      <p className="font-display text-sm tracking-widest uppercase text-primary">
        {objName}
        {isMax && (
          <span className="ml-1 text-xs text-muted-foreground">(MAX)</span>
        )}
      </p>
      <p className="font-mono text-2xl tabular-nums text-foreground">
        {val != null ? val.toFixed(4) : "—"}
      </p>
      <p className="text-xs text-muted-foreground">
        Solution index {sol.index}
      </p>
    </div>
  );
}

// ─── Custom tooltip ───────────────────────────────────────────────────────────

interface TooltipPayload {
  payload?: {
    index: number;
    objectives_abs: Record<string, number>;
    xVal: number;
    yVal: number;
  };
}

function ScatterTooltip({ active, payload }: { active?: boolean; payload?: TooltipPayload[] }) {
  if (!active || !payload?.length) return null;
  const d = payload[0]?.payload;
  if (!d) return null;

  return (
    <div className="rounded border border-border bg-popover px-3 py-2 font-mono text-xs shadow-lg">
      <p className="text-primary font-semibold mb-1">SOL #{d.index}</p>
      {Object.entries(d.objectives_abs).map(([k, v]) => (
        <p key={k} className="text-foreground tabular-nums">
          {k}: {typeof v === "number" ? v.toFixed(4) : "—"}
        </p>
      ))}
    </div>
  );
}

// ─── Main component ───────────────────────────────────────────────────────────

export default function ParetoScatter({ front, selectedIndex, onSelectIndex }: Props) {
  const colors = useChartColors();
  const objectives = front.objectives;

  const [xObj, setXObj] = useState<string>(objectives[0] ?? "");
  const [yObj, setYObj] = useState<string>(objectives[1] ?? objectives[0] ?? "");

  // Single-objective / single-solution — show readout instead
  if (front.result_kind === "single" || objectives.length < 2) {
    return <SingleObjectiveReadout front={front} />;
  }

  const pointData = front.solutions.map((sol) => ({
    index: sol.index,
    objectives_abs: sol.objectives_abs,
    xVal: sol.objectives_abs[xObj] ?? 0,
    yVal: sol.objectives_abs[yObj] ?? 0,
  }));

  const xIsMax = front.polarities[xObj] === -1;
  const yIsMax = front.polarities[yObj] === -1;

  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  function handleClick(data: any) {
    const idx = data?.index ?? data?.payload?.index;
    if (idx != null) {
      onSelectIndex(idx as number);
    }
  }

  return (
    <div className="flex flex-col gap-3">
      {/* Axis selectors */}
      <div className="flex flex-wrap gap-3">
        <div className="flex items-center gap-2">
          <span className="text-xs text-muted-foreground tracking-widest font-mono">X:</span>
          <Select value={xObj} onValueChange={setXObj}>
            <SelectTrigger className="h-7 w-48 text-xs font-mono">
              <SelectValue />
            </SelectTrigger>
            <SelectContent>
              {objectives.map((obj) => (
                <SelectItem key={obj} value={obj} className="text-xs font-mono">
                  {obj}
                  {front.polarities[obj] === -1 && (
                    <span className="ml-1 text-muted-foreground">(max)</span>
                  )}
                </SelectItem>
              ))}
            </SelectContent>
          </Select>
        </div>

        <div className="flex items-center gap-2">
          <span className="text-xs text-muted-foreground tracking-widest font-mono">Y:</span>
          <Select value={yObj} onValueChange={setYObj}>
            <SelectTrigger className="h-7 w-48 text-xs font-mono">
              <SelectValue />
            </SelectTrigger>
            <SelectContent>
              {objectives.map((obj) => (
                <SelectItem key={obj} value={obj} className="text-xs font-mono">
                  {obj}
                  {front.polarities[obj] === -1 && (
                    <span className="ml-1 text-muted-foreground">(max)</span>
                  )}
                </SelectItem>
              ))}
            </SelectContent>
          </Select>
        </div>
      </div>

      {/* Chart */}
      <div className="h-72 w-full">
        <ResponsiveContainer width="100%" height="100%">
          <ScatterChart margin={{ top: 8, right: 16, bottom: 24, left: 16 }}>
            <CartesianGrid strokeDasharray="3 3" stroke="hsl(205 30% 13%)" />
            <XAxis
              dataKey="xVal"
              type="number"
              name={xObj}
              label={{
                value: xObj + (xIsMax ? " (max)" : ""),
                position: "insideBottom",
                offset: -10,
                style: { fontFamily: "var(--font-mono)", fontSize: 10, fill: "hsl(169 8% 45%)" },
              }}
              tick={{ fontFamily: "var(--font-mono)", fontSize: 10, fill: "hsl(169 8% 45%)" }}
              tickLine={false}
              axisLine={{ stroke: "hsl(205 30% 13%)" }}
            />
            <YAxis
              dataKey="yVal"
              type="number"
              name={yObj}
              label={{
                value: yObj + (yIsMax ? " (max)" : ""),
                angle: -90,
                position: "insideLeft",
                offset: 10,
                style: { fontFamily: "var(--font-mono)", fontSize: 10, fill: "hsl(169 8% 45%)" },
              }}
              tick={{ fontFamily: "var(--font-mono)", fontSize: 10, fill: "hsl(169 8% 45%)" }}
              tickLine={false}
              axisLine={{ stroke: "hsl(205 30% 13%)" }}
            />
            <RechartsTooltip
              content={<ScatterTooltip />}
              cursor={{ stroke: "hsl(42 100% 47% / 0.4)" }}
            />
            <Scatter
              data={pointData}
              onClick={handleClick}
              style={{ cursor: "pointer" }}
            >
              {pointData.map((point) => (
                <Cell
                  key={point.index}
                  fill={
                    point.index === selectedIndex
                      ? colors[0]   // --chart-1 amber (selected)
                      : colors[4]   // --chart-5 muted grey
                  }
                  stroke={
                    point.index === selectedIndex
                      ? colors[0]
                      : "transparent"
                  }
                  strokeWidth={point.index === selectedIndex ? 2 : 0}
                  opacity={point.index === selectedIndex ? 1 : 0.55}
                  r={point.index === selectedIndex ? 7 : 5}
                />
              ))}
            </Scatter>
          </ScatterChart>
        </ResponsiveContainer>
      </div>

      <p className="text-xs text-muted-foreground font-mono">
        {front.n_solutions} SOLUTIONS — CLICK POINT TO SELECT
      </p>
    </div>
  );
}
