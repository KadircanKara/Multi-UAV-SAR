"use client";

/**
 * ParetoScatter3D — interactive 3D scatter of a Pareto front on a <canvas>.
 *
 * Deliberately dependency-free: the projection is a plain yaw/pitch rotation
 * plus a perspective divide, so the plot inherits the app's theme tokens and
 * costs nothing in bundle size (see ParetoScatter for the 2D companion).
 *
 * Interaction: drag to orbit, wheel to zoom, hover for a readout, click a point
 * to select it (when the caller passes onSelectIndex).
 *
 * Coordinate spaces:
 *   data      — raw objective values
 *   unit cube — each axis min→max mapped to −0.5…+0.5 (so the box is centred)
 *   camera    — unit cube rotated by (yaw, pitch), then translated `dist` away
 *   screen    — perspective divide, Y flipped (canvas Y grows downward)
 */

import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import { useChartColors } from "@/hooks/useChartColors";
import { useCanvasDPR } from "@/hooks/useCanvasDPR";

// ─── Types ────────────────────────────────────────────────────────────────────

export interface Point3D {
  /** Solution index, or null when the source has no stable indices (live front). */
  index: number | null;
  values: Record<string, number>;
}

interface Props {
  objectives: string[];
  points: Point3D[];
  /** −1 ⇒ the objective is maximised; only used for the "(max)" axis hints. */
  polarities?: Record<string, number>;
  selectedIndex?: number | null;
  onSelectIndex?: (index: number) => void;
  /** Canvas height in px. */
  height?: number;
}

interface Camera {
  yaw: number;
  pitch: number;
  dist: number;
}

const INITIAL_CAMERA: Camera = { yaw: -0.7, pitch: 0.45, dist: 3.2 };
// Sized so the unit cube plus its tick labels and axis names clear the canvas
// edges at the initial camera distance.
const FOCAL = 1.7;
const PITCH_LIMIT = Math.PI / 2 - 0.05;
const DIST_MIN = 1.8;
const DIST_MAX = 8;
const HIT_RADIUS = 12; // px
const DRAG_SLOP = 4; // px of movement that still counts as a click
const TICKS = 4;
// Canvas font strings can't contain CSS vars — they must be a concrete stack.
const MONO = "ui-monospace, SFMono-Regular, Menlo, Consolas, monospace";

// ─── Value formatting ─────────────────────────────────────────────────────────

function fmt(value: number): string {
  if (!Number.isFinite(value)) return "—";
  const abs = Math.abs(value);
  if (abs >= 1000) return Math.round(value).toLocaleString();
  if (abs >= 10) return value.toFixed(1);
  if (abs >= 1) return value.toFixed(2);
  return value.toFixed(3);
}

// Percentage Connectivity is stored as a 0–1 fraction; per-solution readouts
// show it as a percentage (matching the optimizer cards and live progress).
function fmtReadout(obj: string, value: number): string {
  if (!Number.isFinite(value)) return "—";
  if (obj === "Percentage Connectivity") return `${(value * 100).toFixed(1)}%`;
  return fmt(value);
}

// ─── Projection ───────────────────────────────────────────────────────────────

interface Projected {
  sx: number;
  sy: number;
  /** Distance from the camera; larger = further away. */
  depth: number;
}

function project(
  x: number,
  y: number,
  z: number,
  cam: Camera,
  w: number,
  h: number
): Projected {
  const cy = Math.cos(cam.yaw);
  const sy = Math.sin(cam.yaw);
  const x1 = x * cy + z * sy;
  const z1 = -x * sy + z * cy;

  const cp = Math.cos(cam.pitch);
  const sp = Math.sin(cam.pitch);
  const y2 = y * cp - z1 * sp;
  const z2 = y * sp + z1 * cp;

  const depth = Math.max(cam.dist - z2, 0.1);
  const scale = (Math.min(w, h) * FOCAL) / depth;
  return { sx: w / 2 + x1 * scale, sy: h / 2 - y2 * scale, depth };
}

// ─── Axis extents ─────────────────────────────────────────────────────────────

interface Extent {
  min: number;
  max: number;
}

function extentOf(points: Point3D[], obj: string): Extent {
  let min = Infinity;
  let max = -Infinity;
  for (const p of points) {
    const v = p.values[obj];
    if (v == null || !Number.isFinite(v)) continue;
    if (v < min) min = v;
    if (v > max) max = v;
  }
  if (!Number.isFinite(min) || !Number.isFinite(max)) return { min: 0, max: 1 };
  // A degenerate axis (one solution, or every solution identical) would divide
  // by zero. Pad it relative to the value so the ticks stay in a sane range —
  // an absolute pad would print negative ticks on a 0–1 objective and identical
  // ones on a four-digit objective.
  if (min === max) {
    const pad = Math.abs(min) > 0 ? Math.abs(min) * 0.05 : 0.5;
    return { min: min - pad, max: max + pad };
  }
  return { min, max };
}

function norm(v: number, e: Extent): number {
  return (v - e.min) / (e.max - e.min) - 0.5;
}

// ─── Cube geometry ────────────────────────────────────────────────────────────

const H = 0.5;
// 8 corners, indexed by bit flags: 1 = +x, 2 = +y, 4 = +z
const CORNERS: [number, number, number][] = [
  [-H, -H, -H], [+H, -H, -H], [-H, +H, -H], [+H, +H, -H],
  [-H, -H, +H], [+H, -H, +H], [-H, +H, +H], [+H, +H, +H],
];
/** The four cube edges parallel to each axis, as [fromCorner, toCorner]. */
const AXIS_EDGES: Record<"x" | "y" | "z", [number, number][]> = {
  x: [[0, 1], [2, 3], [4, 5], [6, 7]],
  y: [[0, 2], [1, 3], [4, 6], [5, 7]],
  z: [[0, 4], [1, 5], [2, 6], [3, 7]],
};
// All 12 wireframe edges — derived so the two lists can't drift apart.
const EDGES: [number, number][] = [...AXIS_EDGES.x, ...AXIS_EDGES.y, ...AXIS_EDGES.z];

// ─── Component ────────────────────────────────────────────────────────────────

export default function ParetoScatter3D({
  objectives,
  points,
  polarities,
  selectedIndex = null,
  onSelectIndex,
  height = 384,
}: Props) {
  const colors = useChartColors();
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const cameraRef = useRef<Camera>({ ...INITIAL_CAMERA });
  /** Screen position of every drawn point, for hover/click hit-testing. */
  const hitsRef = useRef<{ sx: number; sy: number; point: Point3D }[]>([]);
  const dragRef = useRef<{
    active: boolean;
    lastX: number;
    lastY: number;
    moved: number;
  }>({ active: false, lastX: 0, lastY: 0, moved: 0 });

  const [xObj, setXObj] = useState(objectives[0] ?? "");
  const [yObj, setYObj] = useState(objectives[1] ?? objectives[0] ?? "");
  const [zObj, setZObj] = useState(objectives[2] ?? objectives[0] ?? "");
  const [hover, setHover] = useState<{
    point: Point3D;
    sx: number;
    sy: number;
  } | null>(null);

  const enoughAxes = objectives.length >= 3;

  const extents = useMemo(
    () => ({
      x: extentOf(points, xObj),
      y: extentOf(points, yObj),
      z: extentOf(points, zObj),
    }),
    [points, xObj, yObj, zObj]
  );

  // ── Draw ────────────────────────────────────────────────────────────────────
  const draw = useCallback(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    const dpr = window.devicePixelRatio || 1;
    const w = canvas.width / dpr;
    const h = canvas.height / dpr;
    const cam = cameraRef.current;
    ctx.clearRect(0, 0, w, h);

    const corners = CORNERS.map(([cx, cy, cz]) => project(cx, cy, cz, cam, w, h));

    // ── Cube wireframe ───────────────────────────────────────────────────────
    // --border is too faint to read as a 3D box; --muted-foreground holds up in
    // both themes (47% lightness on light, 65% on dark).
    ctx.strokeStyle = colors.axis;
    ctx.lineWidth = 1.25;
    ctx.globalAlpha = 0.85;
    for (const [a, b] of EDGES) {
      ctx.beginPath();
      ctx.moveTo(corners[a]!.sx, corners[a]!.sy);
      ctx.lineTo(corners[b]!.sx, corners[b]!.sy);
      ctx.stroke();
    }
    ctx.globalAlpha = 1;

    // ── Axis ticks + labels ──────────────────────────────────────────────────
    // Pick which of the four parallel cube edges carries each axis's ticks:
    // the vertical (Y) axis goes on the leftmost edge, the two horizontal axes
    // on the bottom-most ones, so the three label runs never pile up on the
    // same corner however the box is rotated.
    ctx.font = `11px ${MONO}`;
    ctx.fillStyle = colors.foreground;
    const axisSpec: {
      key: "x" | "y" | "z";
      obj: string;
      extent: Extent;
    }[] = [
      { key: "x", obj: xObj, extent: extents.x },
      { key: "y", obj: yObj, extent: extents.y },
      { key: "z", obj: zObj, extent: extents.z },
    ];

    // Tick runs of two axes meet at a shared cube corner, so their end labels
    // would print on top of each other; keep what has been placed and skip any
    // label that would land on an existing one.
    const placed: { x: number; y: number }[] = [];
    const collides = (x: number, y: number) =>
      placed.some((p) => Math.abs(p.x - x) < 34 && Math.abs(p.y - y) < 9);

    for (const { key, obj, extent } of axisSpec) {
      let best: [number, number] | null = null;
      let bestScore = -Infinity;
      for (const [a, b] of AXIS_EDGES[key]) {
        const mx = (corners[a]!.sx + corners[b]!.sx) / 2;
        const my = (corners[a]!.sy + corners[b]!.sy) / 2;
        const depth = (corners[a]!.depth + corners[b]!.depth) / 2;
        // Y is the vertical axis — label it down the leftmost edge; X and Z lie
        // flat, so they get the bottom-most edge. Nearer edges break ties.
        const score = key === "y" ? -mx - depth : my - depth;
        if (score > bestScore) {
          bestScore = score;
          best = [a, b];
        }
      }
      if (!best) continue;
      const [a, b] = best;
      const from = CORNERS[a]!;
      const to = CORNERS[b]!;
      // Outward direction: away from the cube centre, used to offset the text.
      const midProj = {
        sx: (corners[a]!.sx + corners[b]!.sx) / 2,
        sy: (corners[a]!.sy + corners[b]!.sy) / 2,
      };
      const centre = project(0, 0, 0, cam, w, h);
      const ox = midProj.sx - centre.sx;
      const oy = midProj.sy - centre.sy;
      const olen = Math.hypot(ox, oy) || 1;
      const ux = (ox / olen) * 14;
      const uy = (oy / olen) * 14;

      ctx.textAlign = "center";
      ctx.textBaseline = "middle";

      for (let i = 0; i <= TICKS; i++) {
        const t = i / TICKS;
        const p = project(
          from[0] + (to[0] - from[0]) * t,
          from[1] + (to[1] - from[1]) * t,
          from[2] + (to[2] - from[2]) * t,
          cam,
          w,
          h
        );
        const value = extent.min + (extent.max - extent.min) * t;
        const tx = p.sx + ux;
        const ty = p.sy + uy;
        if (collides(tx, ty)) continue;
        placed.push({ x: tx, y: ty });
        ctx.fillText(fmt(value), tx, ty);
      }

      // Axis name, further out along the same outward normal. Anchor it away
      // from the box (so a long name grows outward, not back over the ticks)
      // and clamp it inside the canvas so it can never be half-cropped.
      ctx.save();
      ctx.font = `bold 12px ${MONO}`;
      ctx.fillStyle = colors.foreground;
      ctx.textAlign = ux < -4 ? "right" : ux > 4 ? "left" : "center";
      ctx.textBaseline = uy > 4 ? "bottom" : uy < -4 ? "top" : "middle";
      const isMax = polarities?.[obj] === -1;
      const nx = Math.max(6, Math.min(w - 6, midProj.sx + ux * 2.6));
      const ny = Math.max(10, Math.min(h - 6, midProj.sy + uy * 2.6));
      ctx.fillText(`${obj}${isMax ? " (max)" : ""}`, nx, ny);
      ctx.restore();
    }

    // ── Points, painted back to front so near ones win ───────────────────────
    const drawn: { sx: number; sy: number; point: Point3D; depth: number }[] = [];
    for (const point of points) {
      const xv = point.values[xObj];
      const yv = point.values[yObj];
      const zv = point.values[zObj];
      if (
        xv == null || yv == null || zv == null ||
        !Number.isFinite(xv) || !Number.isFinite(yv) || !Number.isFinite(zv)
      ) {
        continue;
      }
      const p = project(
        norm(xv, extents.x),
        norm(yv, extents.y),
        norm(zv, extents.z),
        cam,
        w,
        h
      );
      drawn.push({ sx: p.sx, sy: p.sy, point, depth: p.depth });
    }
    drawn.sort((a, b) => b.depth - a.depth);

    let selected: (typeof drawn)[number] | null = null;
    for (const d of drawn) {
      if (d.point.index != null && d.point.index === selectedIndex) {
        selected = d;
        continue; // drawn last, on top of everything
      }
      // Nearer points read larger and more solid — the only depth cue a
      // wireframe box doesn't already give.
      const near = Math.max(0, Math.min(1, (cameraRef.current.dist + 0.6 - d.depth) / 1.6));
      const r = 2.5 + near * 2;
      ctx.beginPath();
      ctx.arc(d.sx, d.sy, r, 0, Math.PI * 2);
      ctx.fillStyle = colors.series[4]!;
      ctx.globalAlpha = 0.35 + near * 0.4;
      ctx.fill();
      ctx.globalAlpha = 1;
    }

    // Selected point: a halo ring in the card colour separates it from the
    // cloud whatever the two series hues are (they can sit ~20° apart).
    if (selected) {
      ctx.beginPath();
      ctx.arc(selected.sx, selected.sy, 6.5, 0, Math.PI * 2);
      ctx.fillStyle = colors.series[0]!;
      ctx.fill();
      ctx.beginPath();
      ctx.arc(selected.sx, selected.sy, 9.5, 0, Math.PI * 2);
      ctx.strokeStyle = colors.tooltipBg;
      ctx.lineWidth = 3;
      ctx.stroke();
      ctx.beginPath();
      ctx.arc(selected.sx, selected.sy, 9.5, 0, Math.PI * 2);
      ctx.strokeStyle = colors.series[0]!;
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }

    hitsRef.current = drawn.map(({ sx, sy, point }) => ({ sx, sy, point }));
  }, [colors, points, xObj, yObj, zObj, extents, selectedIndex, polarities]);

  // ── Canvas sizing (DPR-aware). The wrapper is stable and reads the latest
  // draw through a ref, so the ResizeObserver survives re-renders instead of
  // being torn down and recreated whenever a draw parameter changes. ──────────
  const drawRef = useRef(draw);
  useEffect(() => {
    drawRef.current = draw;
  }, [draw]);
  const drawCurrent = useCallback(() => drawRef.current(), []);
  useCanvasDPR(canvasRef, drawCurrent);

  // Repaint when any draw input changes (points, axes, selection, colors).
  useEffect(() => {
    draw();
  }, [draw]);

  // Live polling replaces `points` while the cursor sits still — drop the
  // tooltip rather than pin a superseded solution at a stale position.
  useEffect(() => {
    setHover(null);
  }, [points]);

  // ── Wheel zoom (non-passive so the page doesn't scroll under the cursor) ────
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    function onWheel(e: WheelEvent) {
      e.preventDefault();
      const cam = cameraRef.current;
      cam.dist = Math.max(
        DIST_MIN,
        Math.min(DIST_MAX, cam.dist * (1 + Math.sign(e.deltaY) * 0.12))
      );
      draw();
    }
    canvas.addEventListener("wheel", onWheel, { passive: false });
    return () => canvas.removeEventListener("wheel", onWheel);
  }, [draw]);

  // ── Pointer: orbit / hover / click-select ───────────────────────────────────
  const pickAt = useCallback((sx: number, sy: number) => {
    let best: { sx: number; sy: number; point: Point3D } | null = null;
    let bestDist = HIT_RADIUS;
    for (const hit of hitsRef.current) {
      const d = Math.hypot(hit.sx - sx, hit.sy - sy);
      if (d <= bestDist) {
        bestDist = d;
        best = hit;
      }
    }
    return best;
  }, []);

  function handlePointerDown(e: React.PointerEvent<HTMLCanvasElement>) {
    e.currentTarget.setPointerCapture(e.pointerId);
    dragRef.current = {
      active: true,
      lastX: e.clientX,
      lastY: e.clientY,
      moved: 0,
    };
  }

  function handlePointerMove(e: React.PointerEvent<HTMLCanvasElement>) {
    const drag = dragRef.current;
    const rect = e.currentTarget.getBoundingClientRect();

    if (drag.active) {
      const dx = e.clientX - drag.lastX;
      const dy = e.clientY - drag.lastY;
      drag.lastX = e.clientX;
      drag.lastY = e.clientY;
      drag.moved += Math.abs(dx) + Math.abs(dy);
      const cam = cameraRef.current;
      cam.yaw += dx * 0.01;
      cam.pitch = Math.max(
        -PITCH_LIMIT,
        Math.min(PITCH_LIMIT, cam.pitch + dy * 0.01)
      );
      if (hover) setHover(null);
      draw();
      return;
    }

    const hit = pickAt(e.clientX - rect.left, e.clientY - rect.top);
    if (!hit) {
      if (hover) setHover(null);
      return;
    }
    if (hover?.point !== hit.point) {
      setHover({ point: hit.point, sx: hit.sx, sy: hit.sy });
    }
  }

  function handlePointerUp(e: React.PointerEvent<HTMLCanvasElement>) {
    const drag = dragRef.current;
    drag.active = false;
    if (drag.moved > DRAG_SLOP) return; // it was an orbit, not a click
    const rect = e.currentTarget.getBoundingClientRect();
    const hit = pickAt(e.clientX - rect.left, e.clientY - rect.top);
    if (hit?.point.index != null) onSelectIndex?.(hit.point.index);
  }

  function resetView() {
    cameraRef.current = { ...INITIAL_CAMERA };
    draw();
  }

  // ── Fewer than three objectives: nothing meaningful to plot ─────────────────
  if (!enoughAxes) {
    return (
      <p className="rounded border border-border bg-card px-3 py-2 text-xs text-muted-foreground font-mono">
        3D VIEW NEEDS AT LEAST 3 OBJECTIVES — THIS RUN HAS{" "}
        {objectives.length}
      </p>
    );
  }

  return (
    <div className="flex flex-col gap-3">
      {/* Axis selectors + reset */}
      <div className="flex flex-wrap items-center gap-3">
        {(
          [
            ["X", xObj, setXObj],
            ["Y", yObj, setYObj],
            ["Z", zObj, setZObj],
          ] as const
        ).map(([label, value, setter]) => (
          <div key={label} className="flex items-center gap-2">
            <span className="text-xs text-muted-foreground tracking-widest font-mono">
              {label}:
            </span>
            <Select value={value} onValueChange={setter}>
              <SelectTrigger className="h-7 w-48 text-xs font-mono">
                <SelectValue />
              </SelectTrigger>
              <SelectContent>
                {objectives.map((obj) => (
                  <SelectItem key={obj} value={obj} className="text-xs font-mono">
                    {obj}
                    {polarities?.[obj] === -1 && (
                      <span className="ml-1 text-muted-foreground">(max)</span>
                    )}
                  </SelectItem>
                ))}
              </SelectContent>
            </Select>
          </div>
        ))}
        <button
          type="button"
          onClick={resetView}
          className="ml-auto h-7 rounded border border-border bg-secondary px-3 text-xs font-mono tracking-widest text-foreground transition-colors hover:bg-accent"
        >
          RESET VIEW
        </button>
      </div>

      {/* Canvas + hover readout */}
      <div className="relative w-full" style={{ height }}>
        <canvas
          ref={canvasRef}
          className="h-full w-full cursor-grab touch-none active:cursor-grabbing"
          onPointerDown={handlePointerDown}
          onPointerMove={handlePointerMove}
          onPointerUp={handlePointerUp}
          onPointerLeave={() => {
            dragRef.current.active = false;
            setHover(null);
          }}
        />
        {hover && (
          <div
            className="pointer-events-none absolute z-10 rounded border border-border bg-popover px-3 py-2 font-mono text-xs shadow-lg"
            style={{
              left: Math.round(hover.sx) + 12,
              top: Math.round(hover.sy) + 12,
            }}
          >
            {hover.point.index != null && (
              <p className="mb-1 font-semibold text-primary">
                SOL #{hover.point.index}
              </p>
            )}
            {objectives.map((obj) => (
              <p key={obj} className="tabular-nums text-foreground">
                {obj}: {fmtReadout(obj, hover.point.values[obj] ?? NaN)}
              </p>
            ))}
          </div>
        )}
      </div>

      <p className="text-xs text-muted-foreground font-mono">
        {points.length} SOLUTION{points.length !== 1 ? "S" : ""} — DRAG TO
        ROTATE · SCROLL TO ZOOM
        {onSelectIndex ? " · CLICK POINT TO SELECT" : ""}
      </p>
    </div>
  );
}
