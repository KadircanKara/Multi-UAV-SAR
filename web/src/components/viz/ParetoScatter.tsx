"use client";

/**
 * ParetoScatter — dynamic Recharts ScatterChart of the Pareto front.
 * Loaded via next/dynamic({ ssr: false }) from the explore page.
 *
 * The "N SOLUTIONS — CLICK POINT TO SELECT" caption is NOT printed here: it
 * describes the whole Pareto card, whose 3D plot answers the same click, so
 * ParetoFrontsCard prints it once below both plots.
 */

import {
  ScatterChart,
  Scatter,
  XAxis,
  YAxis,
  ZAxis,
  CartesianGrid,
  Tooltip as RechartsTooltip,
  ResponsiveContainer,
  Cell,
} from "recharts";
import type { ParetoFront } from "@/lib/types";
import ObjectiveAxisSelect from "@/components/viz/ObjectiveAxisSelect";
import { alpha, axisStyles, useChartColors } from "@/hooks/useChartColors";
import { isPercentObjective, percentString, percentTick } from "@/lib/objective-format";

// ─── Types ────────────────────────────────────────────────────────────────────

interface Props {
  front: ParetoFront;
  selectedIndex: number;
  onSelectIndex: (idx: number) => void;
  /** Axis choice is owned by the parent so the left panel can drive it. */
  xObj: string;
  yObj: string;
  onXChange: (objective: string) => void;
  onYChange: (objective: string) => void;
  /** True when the parent renders the pickers itself (panel layout). */
  hideAxisSelectors?: boolean;
}

// Dot-size constants for ZAxis (Recharts v3 uses area, not radius)
const SIZE_SELECTED = 196; // ~r=7 equivalent
const SIZE_DEFAULT  = 64;  // ~r=5 equivalent

// ─── Single-objective readout ─────────────────────────────────────────────────

function SingleObjectiveReadout({ front }: { front: ParetoFront }) {
  const sol = front.solutions[0];
  if (!sol) return null;
  const objName = front.objectives[0] ?? "objective";
  const rawVal = sol.objectives_abs[objName];
  const val =
    rawVal != null && isPercentObjective(objName)
      ? percentString(rawVal)
      : rawVal != null
        ? rawVal.toFixed(4)
        : "—";
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
        {val}
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

// Percentage Connectivity is stored as a 0–1 fraction; the readout shows it as
// a percentage (matching the optimizer cards, live progress, and the 3D hover).
function fmtTooltipVal(k: string, v: unknown): string {
  if (typeof v !== "number") return "—";
  if (isPercentObjective(k)) return percentString(v);
  return v.toFixed(4);
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
          {k}: {fmtTooltipVal(k, v)}
        </p>
      ))}
    </div>
  );
}

// ─── Main component ───────────────────────────────────────────────────────────

export default function ParetoScatter({
  front,
  selectedIndex,
  onSelectIndex,
  xObj,
  yObj,
  onXChange,
  onYChange,
  hideAxisSelectors,
}: Props) {
  const colors = useChartColors();
  const ax = axisStyles(colors);
  const objectives = front.objectives;

  // Single-objective / single-solution — show readout instead
  if (front.result_kind === "single" || objectives.length < 2) {
    return <SingleObjectiveReadout front={front} />;
  }

  const pointData = front.solutions.map((sol) => ({
    index: sol.index,
    objectives_abs: sol.objectives_abs,
    xVal: sol.objectives_abs[xObj] ?? 0,
    yVal: sol.objectives_abs[yObj] ?? 0,
    size: sol.index === selectedIndex ? SIZE_SELECTED : SIZE_DEFAULT,
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
      {!hideAxisSelectors && (
        <div className="flex flex-wrap gap-3">
          <ObjectiveAxisSelect
            label="X:" value={xObj} onChange={onXChange}
            objectives={objectives} polarities={front.polarities}
          />
          <ObjectiveAxisSelect
            label="Y:" value={yObj} onChange={onYChange}
            objectives={objectives} polarities={front.polarities}
          />
        </div>
      )}

      {/* Chart */}
      <div className="h-80 w-full sm:h-96">
        <ResponsiveContainer width="100%" height="100%">
          <ScatterChart margin={{ top: 12, right: 24, bottom: 28, left: 12 }}>
            <CartesianGrid strokeDasharray="3 3" stroke={colors.grid} />
            <XAxis
              dataKey="xVal"
              type="number"
              name={xObj}
              height={52}
              tickMargin={8}
              tickFormatter={isPercentObjective(xObj) ? percentTick : undefined}
              label={{
                value: xObj + (xIsMax ? " (max)" : ""),
                position: "insideBottom",
                offset: -2,
                style: { ...ax.label, textAnchor: "middle" },
              }}
              tick={ax.tick}
              tickLine={false}
              axisLine={ax.axisLine}
            />
            <YAxis
              dataKey="yVal"
              type="number"
              name={yObj}
              width={72}
              tickMargin={8}
              tickFormatter={isPercentObjective(yObj) ? percentTick : undefined}
              label={{
                value: yObj + (yIsMax ? " (max)" : ""),
                angle: -90,
                position: "insideLeft",
                offset: 12,
                style: { ...ax.label, textAnchor: "middle" },
              }}
              tick={ax.tick}
              tickLine={false}
              axisLine={ax.axisLine}
            />
            <ZAxis type="number" dataKey="size" range={[SIZE_DEFAULT, SIZE_SELECTED]} />
            <RechartsTooltip
              content={<ScatterTooltip />}
              cursor={{ stroke: alpha(colors.reference, 0.4) }}
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
                      ? colors.series[0]   // --chart-1 amber (selected)
                      : colors.series[4]   // --chart-5 muted grey
                  }
                  stroke={
                    point.index === selectedIndex
                      ? colors.series[0]
                      : "transparent"
                  }
                  strokeWidth={point.index === selectedIndex ? 2 : 0}
                  opacity={point.index === selectedIndex ? 1 : 0.55}
                />
              ))}
            </Scatter>
          </ScatterChart>
        </ResponsiveContainer>
      </div>
    </div>
  );
}
