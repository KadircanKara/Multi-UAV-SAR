"use client";

/**
 * ParetoScatter — dynamic Recharts ScatterChart of the Pareto front.
 * Loaded via next/dynamic({ ssr: false }) from the explore page.
 *
 * The "N SOLUTIONS — CLICK POINT TO SELECT" caption is NOT printed here: it
 * describes the whole Pareto card, whose 3D plot answers the same click, so
 * ParetoFrontsCard prints it once below both plots.
 *
 * Zoom and pan are hand-rolled because Recharts has no viewport of its own:
 * scroll and drag write an explicit axis domain, and RESET VIEW drops back to
 * `null`, which is Recharts' own auto domain — so the untouched chart renders
 * byte-identically to before this was added. A drag that moves more than a few
 * pixels suppresses the click that follows it, or panning across the cloud
 * would re-select whatever point happened to be under the pointer at the end.
 */

import { useCallback, useEffect, useMemo, useRef, useState } from "react";
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

/** Explicit axis window, in data units. `null` ⇒ Recharts' auto domain. */
type Viewport = { x: [number, number]; y: [number, number] };

// Dot-size constants for ZAxis (Recharts v3 uses area, not radius)
const SIZE_SELECTED = 196; // ~r=7 equivalent
const SIZE_DEFAULT  = 64;  // ~r=5 equivalent

// Plot geometry. Recharts gives no way to ask where the plot rectangle is, so
// pointer positions are converted to data coordinates from these — which means
// they MUST stay in step with the margin/width/height props below. They are
// consts precisely so the two cannot drift apart.
const CHART_MARGIN = { top: 12, right: 24, bottom: 28, left: 12 };
const Y_AXIS_WIDTH = 72;
const X_AXIS_HEIGHT = 52;
const PLOT_LEFT = CHART_MARGIN.left + Y_AXIS_WIDTH;
const PLOT_RIGHT = CHART_MARGIN.right;
const PLOT_TOP = CHART_MARGIN.top;
const PLOT_BOTTOM = CHART_MARGIN.bottom + X_AXIS_HEIGHT;

/** Zoom per wheel notch. */
const ZOOM_STEP = 1.15;
/** Furthest out a wheel may go, as a multiple of the data's own extent — past
 *  this the cloud is a dot in an empty field and only RESET is any use. */
const MAX_SPAN_FACTOR = 8;
/** Closest in, likewise. Guards against a span collapsing to zero. */
const MIN_SPAN_FACTOR = 1e-3;
/** Pointer travel that turns a click into a drag. */
const DRAG_SLOP_PX = 4;

function clamp(v: number, lo: number, hi: number): number {
  return v < lo ? lo : v > hi ? hi : v;
}

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

  // ── Zoom / pan state ───────────────────────────────────────────────────────
  // Declared before the single-objective early return below, so the hook order
  // is the same on every render whatever the front turns out to be.

  const [viewport, setViewport] = useState<Viewport | null>(null);
  const wrapRef = useRef<HTMLDivElement>(null);
  // Set on pointer-down, raised once travel passes the slop. Read (and reset)
  // by the point-click handler, which fires after the pointer-up.
  const draggedRef = useRef(false);
  const dragRef = useRef<{ px: number; py: number; from: Viewport } | null>(null);

  const pointData = useMemo(
    () =>
      front.solutions.map((sol) => ({
        index: sol.index,
        objectives_abs: sol.objectives_abs,
        xVal: sol.objectives_abs[xObj] ?? 0,
        yVal: sol.objectives_abs[yObj] ?? 0,
        size: sol.index === selectedIndex ? SIZE_SELECTED : SIZE_DEFAULT,
      })),
    [front, xObj, yObj, selectedIndex]
  );

  // What the first zoom starts from, and the widest a zoom-out may reach.
  // Anchored at 0 like Recharts' own default for a numeric axis, so the first
  // wheel notch does not jump to a different framing of the same data.
  const base = useMemo<Viewport>(() => {
    const xs = pointData.map((p) => p.xVal);
    const ys = pointData.map((p) => p.yVal);
    const span = (lo: number, hi: number): [number, number] =>
      hi > lo ? [lo, hi] : [lo, lo + 1]; // a degenerate axis still needs width
    return {
      x: span(Math.min(0, ...xs), Math.max(0, ...xs)),
      y: span(Math.min(0, ...ys), Math.max(0, ...ys)),
    };
  }, [pointData]);

  // One <Cell> per solution, rebuilt only when the colouring can actually
  // differ. A gesture re-renders this component continuously; without the memo
  // each of those renders also allocated a few hundred elements for React to
  // diff, none of which had changed.
  const cells = useMemo(
    () =>
      pointData.map((point) => {
        const isSelected = point.index === selectedIndex;
        return (
          <Cell
            key={point.index}
            fill={
              isSelected
                ? colors.series[0]   // --chart-1 amber (selected)
                : colors.series[4]   // --chart-5 muted grey
            }
            stroke={isSelected ? colors.series[0] : "transparent"}
            strokeWidth={isSelected ? 2 : 0}
            opacity={isSelected ? 1 : 0.55}
          />
        );
      }),
    [pointData, selectedIndex, colors]
  );

  // A zoom is a statement about two specific objectives. Changing either axis
  // — or loading another front — makes it meaningless, so drop it.
  useEffect(() => {
    setViewport(null);
  }, [xObj, yObj, front]);

  /** Pointer position → data coordinates, or null if outside the plot rect. */
  const toData = useCallback(
    (clientX: number, clientY: number, from: Viewport) => {
      const el = wrapRef.current;
      if (!el) return null;
      const r = el.getBoundingClientRect();
      const left = PLOT_LEFT;
      const right = r.width - PLOT_RIGHT;
      const top = PLOT_TOP;
      const bottom = r.height - PLOT_BOTTOM;
      if (right <= left || bottom <= top) return null;
      const fx = (clamp(clientX - r.left, left, right) - left) / (right - left);
      const fy = (clamp(clientY - r.top, top, bottom) - top) / (bottom - top);
      return {
        x: from.x[0] + fx * (from.x[1] - from.x[0]),
        // Screen y grows downward; the axis does not.
        y: from.y[1] - fy * (from.y[1] - from.y[0]),
      };
    },
    []
  );

  // Wheel zoom about the cursor. Registered by hand rather than via onWheel
  // because it has to preventDefault, and React attaches wheel listeners as
  // passive — where preventDefault is a no-op and the page scrolls instead.
  useEffect(() => {
    const el = wrapRef.current;
    if (!el) return;

    function onWheel(e: WheelEvent) {
      const from = viewport ?? base;
      const at = toData(e.clientX, e.clientY, from);
      if (!at) return;
      e.preventDefault();

      const k = e.deltaY > 0 ? ZOOM_STEP : 1 / ZOOM_STEP;
      const next: Viewport = {
        x: [
          at.x - (at.x - from.x[0]) * k,
          at.x + (from.x[1] - at.x) * k,
        ],
        y: [
          at.y - (at.y - from.y[0]) * k,
          at.y + (from.y[1] - at.y) * k,
        ],
      };

      const baseX = base.x[1] - base.x[0];
      const baseY = base.y[1] - base.y[0];
      const spanX = next.x[1] - next.x[0];
      const spanY = next.y[1] - next.y[0];
      if (
        spanX > baseX * MAX_SPAN_FACTOR ||
        spanY > baseY * MAX_SPAN_FACTOR ||
        spanX < baseX * MIN_SPAN_FACTOR ||
        spanY < baseY * MIN_SPAN_FACTOR
      ) {
        return;
      }
      setViewport(next);
    }

    el.addEventListener("wheel", onWheel, { passive: false });
    return () => el.removeEventListener("wheel", onWheel);
  }, [viewport, base, toData]);

  function handlePointerDown(e: React.PointerEvent<HTMLDivElement>) {
    draggedRef.current = false;
    dragRef.current = { px: e.clientX, py: e.clientY, from: viewport ?? base };
  }

  function handlePointerMove(e: React.PointerEvent<HTMLDivElement>) {
    const drag = dragRef.current;
    if (!drag) return;
    const dx = e.clientX - drag.px;
    const dy = e.clientY - drag.py;
    if (!draggedRef.current) {
      if (Math.abs(dx) < DRAG_SLOP_PX && Math.abs(dy) < DRAG_SLOP_PX) return;
      draggedRef.current = true;
    }
    const el = wrapRef.current;
    if (!el) return;
    const r = el.getBoundingClientRect();
    const w = r.width - PLOT_LEFT - PLOT_RIGHT;
    const h = r.height - PLOT_TOP - PLOT_BOTTOM;
    if (w <= 0 || h <= 0) return;

    const shiftX = (-dx / w) * (drag.from.x[1] - drag.from.x[0]);
    const shiftY = (dy / h) * (drag.from.y[1] - drag.from.y[0]);
    setViewport({
      x: [drag.from.x[0] + shiftX, drag.from.x[1] + shiftX],
      y: [drag.from.y[0] + shiftY, drag.from.y[1] + shiftY],
    });
  }

  function endDrag() {
    dragRef.current = null;
  }

  // Single-objective / single-solution — show readout instead
  if (front.result_kind === "single" || objectives.length < 2) {
    return <SingleObjectiveReadout front={front} />;
  }

  const xIsMax = front.polarities[xObj] === -1;
  const yIsMax = front.polarities[yObj] === -1;

  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  function handleClick(data: any) {
    // The pointer-up that ended a pan is followed by a click on whatever point
    // it landed over. Selecting that would be a side effect of navigating.
    if (draggedRef.current) {
      draggedRef.current = false;
      return;
    }
    const idx = data?.index ?? data?.payload?.index;
    if (idx != null) {
      onSelectIndex(idx as number);
    }
  }

  return (
    <div className="flex flex-col gap-3">
      <div className="flex flex-wrap items-center gap-3">
        {!hideAxisSelectors && (
          <>
            <ObjectiveAxisSelect
              label="X:" value={xObj} onChange={onXChange}
              objectives={objectives} polarities={front.polarities}
            />
            <ObjectiveAxisSelect
              label="Y:" value={yObj} onChange={onYChange}
              objectives={objectives} polarities={front.polarities}
            />
          </>
        )}
        <button
          type="button"
          onClick={() => setViewport(null)}
          className="ml-auto h-7 rounded border border-border bg-secondary px-3 text-xs font-mono tracking-widest text-foreground transition-colors hover:bg-accent"
        >
          RESET VIEW
        </button>
      </div>

      {/* Chart */}
      <div
        ref={wrapRef}
        onPointerDown={handlePointerDown}
        onPointerMove={handlePointerMove}
        onPointerUp={endDrag}
        onPointerLeave={endDrag}
        className="h-80 w-full touch-none cursor-grab active:cursor-grabbing sm:h-96"
      >
        <ResponsiveContainer width="100%" height="100%">
          <ScatterChart margin={CHART_MARGIN}>
            <CartesianGrid strokeDasharray="3 3" stroke={colors.grid} />
            <XAxis
              dataKey="xVal"
              type="number"
              name={xObj}
              height={X_AXIS_HEIGHT}
              // Undefined ⇒ Recharts' own auto domain, i.e. the pre-zoom view.
              domain={viewport ? viewport.x : undefined}
              allowDataOverflow={Boolean(viewport)}
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
              width={Y_AXIS_WIDTH}
              domain={viewport ? viewport.y : undefined}
              allowDataOverflow={Boolean(viewport)}
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
              // Recharts eases points to their new positions on any data or
              // DOMAIN change. Under zoom and pan that is a ~400ms animation
              // restarted on every update, so the cloud visibly trails the
              // cursor and never quite arrives. The gesture is the animation.
              isAnimationActive={false}
            >
              {cells}
            </Scatter>
          </ScatterChart>
        </ResponsiveContainer>
      </div>
    </div>
  );
}
