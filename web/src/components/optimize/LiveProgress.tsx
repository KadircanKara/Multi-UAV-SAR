"use client";

import { useMemo, useState } from "react";
import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  ResponsiveContainer,
  ScatterChart,
  Scatter,
} from "recharts";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import { axisStyles, useChartColors } from "@/hooks/useChartColors";
import { isPercentObjective, percentString, percentTick } from "@/lib/objective-format";
import ParetoScatter3D from "@/components/viz/ParetoScatter3D";

/** One sampled generation: the optimal (absolute) value of each objective. */
export interface ProgressPoint {
  gen: number;
  best: Record<string, number>;
}

interface LiveProgressProps {
  gen: number;
  nGen: number;
  objectives: string[];
  history: ProgressPoint[];
  isMOO: boolean;
}

// Optimisation direction per objective (−1 ⇒ maximise / higher is better).
const POLARITY: Record<string, number> = {
  "Mission Time": 1,
  "Percentage Connectivity": -1,
  "Max Disconnected Time": 1,
  "Mean Disconnected Time": 1,
  "Max Mean TBV": 1,
};

function fmtVal(obj: string, v: number | undefined): string {
  if (v == null || Number.isNaN(v)) return "—";
  if (isPercentObjective(obj)) return percentString(v);
  if (Math.abs(v) >= 100) return Math.round(v).toLocaleString();
  return v.toFixed(2);
}

export function LiveProgress({
  gen,
  nGen,
  objectives,
  history,
  isMOO,
}: LiveProgressProps) {
  const colors = useChartColors();
  const latest = history.length ? history[history.length - 1].best : {};

  return (
    <div className="flex flex-col gap-6">
      <div className="flex flex-wrap items-end justify-between gap-2">
        <div className="flex flex-col gap-1">
          <h2 className="text-lg font-semibold tracking-tight text-foreground">
            Optimization in progress
          </h2>
          <p className="text-sm text-muted-foreground">
            Live best objective values{isMOO ? " and Pareto front" : ""} —
            updating each generation.
          </p>
        </div>
        <span className="text-sm tabular-nums text-muted-foreground">
          Generation {gen} / {nGen}
        </span>
      </div>

      {/* Per-objective best-value trajectories */}
      <div className="grid grid-cols-1 gap-4 sm:grid-cols-2">
        {objectives.map((obj, i) => {
          const data = history
            .map((h) => ({ gen: h.gen, v: h.best[obj] }))
            .filter((d) => d.v != null && !Number.isNaN(d.v));
          const color =
            colors.series[i % colors.series.length] ?? colors.series[0];
          // Weighted-sum runs report a single composite score column (pymoo only
          // has the scalar live), so give it a clean title rather than the long
          // "A & B & C Weighted Sum" name.
          const isWS = obj.includes("Weighted Sum");
          const title = isWS ? "Weighted-sum score" : obj;
          const dir = isWS
            ? "↓ minimise"
            : (POLARITY[obj] ?? 1) < 0
              ? "↑ maximise"
              : "↓ minimise";
          return (
            <div
              key={obj}
              className="flex flex-col gap-2 rounded-xl border border-border bg-card p-4"
            >
              <div className="flex items-baseline justify-between gap-2">
                <span className="truncate text-sm font-medium text-foreground">
                  {title}
                </span>
                <span className="shrink-0 text-[11px] text-muted-foreground">
                  {dir}
                </span>
              </div>
              <span className="text-2xl font-semibold tabular-nums text-foreground">
                {fmtVal(obj, latest[obj])}
              </span>
              <div className="h-16">
                <ResponsiveContainer width="100%" height="100%">
                  <LineChart
                    data={data}
                    margin={{ top: 4, right: 4, bottom: 0, left: 0 }}
                  >
                    <XAxis dataKey="gen" hide />
                    <YAxis hide domain={["auto", "auto"]} />
                    <Tooltip
                      contentStyle={{
                        background: colors.tooltipBg,
                        border: `1px solid ${colors.tooltipBorder}`,
                        borderRadius: 8,
                        fontSize: 12,
                      }}
                      labelFormatter={(g) => `Gen ${g}`}
                      formatter={(val) => [fmtVal(obj, Number(val)), title]}
                    />
                    <Line
                      type="monotone"
                      dataKey="v"
                      stroke={color}
                      strokeWidth={2}
                      dot={false}
                      isAnimationActive={false}
                    />
                  </LineChart>
                </ResponsiveContainer>
              </div>
            </div>
          );
        })}
      </div>

    </div>
  );
}

/**
 * The live Pareto front (2D + 3D). Split out of LiveProgress so the optimizer
 * page can give it the full container width below the run parameters and the
 * per-objective cards. MOO only — a scalar run has no front to plot.
 */
export function LiveParetoFront({
  objectives,
  liveFront,
  isMOO,
}: {
  objectives: string[];
  liveFront: Record<string, number>[] | null;
  isMOO: boolean;
}) {
  const colors = useChartColors();
  if (!isMOO || objectives.length < 2) return null;
  return (
    <div className="flex flex-col gap-4">
      <ParetoLive objectives={objectives} liveFront={liveFront} colors={colors} />
      {objectives.length >= 3 && (
        <LivePareto3D objectives={objectives} liveFront={liveFront} />
      )}
    </div>
  );
}

/** 3D companion to ParetoLive. The live payload carries no solution indices,
 *  so points are view-only (hover readout, no click-to-select). */
function LivePareto3D({
  objectives,
  liveFront,
}: {
  objectives: string[];
  liveFront: Record<string, number>[] | null;
}) {
  // This standalone live view has no left panel to lift into, so it keeps its
  // own axis state — seeded the same way ParetoScatter3D used to seed it
  // internally, so the default view is unchanged.
  const [xObj, setXObj] = useState(objectives[0] ?? "");
  const [yObj, setYObj] = useState(objectives[1] ?? objectives[0] ?? "");
  const [zObj, setZObj] = useState(objectives[2] ?? objectives[0] ?? "");

  const points = useMemo(
    () =>
      (liveFront ?? []).map((values) => ({
        index: null as number | null,
        values,
      })),
    [liveFront]
  );

  return (
    <div className="flex flex-col gap-3 rounded-xl border border-border bg-card p-4">
      <span className="text-sm font-medium text-foreground">
        Pareto front · 3D{" "}
        <span className="text-muted-foreground">
          · {points.length} solution{points.length !== 1 ? "s" : ""}
        </span>
      </span>
      <ParetoScatter3D
        objectives={objectives}
        points={points}
        polarities={POLARITY}
        height={420}
        xObj={xObj}
        yObj={yObj}
        zObj={zObj}
        onXChange={setXObj}
        onYChange={setYObj}
        onZChange={setZObj}
      />
    </div>
  );
}

function ParetoLive({
  objectives,
  liveFront,
  colors,
}: {
  objectives: string[];
  liveFront: Record<string, number>[] | null;
  colors: ReturnType<typeof useChartColors>;
}) {
  const ax = axisStyles(colors);
  const [xSel, setXSel] = useState<string | null>(null);
  const [ySel, setYSel] = useState<string | null>(null);
  const xObj = xSel ?? objectives[0];
  const yObj = ySel ?? objectives[1] ?? objectives[0];

  const data = useMemo(
    () =>
      (liveFront ?? [])
        .map((p) => ({ x: p[xObj], y: p[yObj] }))
        .filter((d) => d.x != null && d.y != null && !Number.isNaN(d.x) && !Number.isNaN(d.y)),
    [liveFront, xObj, yObj]
  );

  return (
    <div className="flex flex-col gap-3 rounded-xl border border-border bg-card p-4">
      <div className="flex flex-wrap items-center justify-between gap-3">
        <span className="text-sm font-medium text-foreground">
          Pareto front{" "}
          <span className="text-muted-foreground">
            · {data.length} solution{data.length !== 1 ? "s" : ""}
          </span>
        </span>
        <div className="flex items-center gap-2">
          <AxisSelect label="X" value={xObj} options={objectives} onChange={setXSel} />
          <AxisSelect label="Y" value={yObj} options={objectives} onChange={setYSel} />
        </div>
      </div>
      <div className="h-72">
        <ResponsiveContainer width="100%" height="100%">
          <ScatterChart margin={{ top: 8, right: 16, bottom: 28, left: 8 }}>
            <CartesianGrid strokeDasharray="3 3" stroke={colors.grid} />
            <XAxis
              type="number"
              dataKey="x"
              name={xObj}
              domain={["auto", "auto"]}
              tick={ax.tick}
              tickFormatter={isPercentObjective(xObj) ? percentTick : undefined}
              axisLine={ax.axisLine}
              tickLine={ax.axisLine}
              label={{
                value: xObj,
                position: "insideBottom",
                offset: -14,
                ...ax.label,
              }}
            />
            <YAxis
              type="number"
              dataKey="y"
              name={yObj}
              domain={["auto", "auto"]}
              width={72}
              tick={ax.tick}
              tickFormatter={isPercentObjective(yObj) ? percentTick : undefined}
              axisLine={ax.axisLine}
              tickLine={ax.axisLine}
              label={{
                value: yObj,
                angle: -90,
                position: "insideLeft",
                ...ax.label,
                style: { textAnchor: "middle" },
              }}
            />
            <Tooltip
              cursor={{ strokeDasharray: "3 3" }}
              contentStyle={{
                background: colors.tooltipBg,
                border: `1px solid ${colors.tooltipBorder}`,
                borderRadius: 8,
                fontSize: 12,
              }}
              formatter={(val, name) => [
                fmtVal(String(name), Number(val)),
                name,
              ]}
            />
            <Scatter data={data} fill={colors.series[0]} isAnimationActive={false} />
          </ScatterChart>
        </ResponsiveContainer>
      </div>
    </div>
  );
}

function AxisSelect({
  label,
  value,
  options,
  onChange,
}: {
  label: string;
  value: string;
  options: string[];
  onChange: (v: string) => void;
}) {
  return (
    <div className="flex items-center gap-1.5">
      <span className="text-xs text-muted-foreground">{label}</span>
      <Select value={value} onValueChange={onChange}>
        <SelectTrigger className="h-8 w-[160px] text-xs">
          <SelectValue />
        </SelectTrigger>
        <SelectContent>
          {options.map((o) => (
            <SelectItem key={o} value={o} className="text-xs">
              {o}
            </SelectItem>
          ))}
        </SelectContent>
      </Select>
    </div>
  );
}
