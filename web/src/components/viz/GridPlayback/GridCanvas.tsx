"use client";

/**
 * GridCanvas — HTML5 canvas renderer for playback animation.
 *
 * Coordinate system (matches PathAnimation.py):
 *   Cell c center: x = (c % grid_size + 0.5) * cell_side_length
 *                  y = (c // grid_size + 0.5) * cell_side_length
 *   Y-axis: matplotlib Y increases UPWARD → flip Y when mapping world→canvas.
 *   Base/GCS (node 0) position already included in trajectories.x[0]/y[0].
 *
 * SMOOTH motion: the timeline position is a CONTINUOUS float `pos`. Drone/base
 * positions, connectivity edge endpoints, trails, and belief fade are linearly
 * interpolated between the surrounding discrete waypoints (floor(pos) and
 * floor(pos)+1). The rAF loop redraws EVERY frame (not just at step
 * boundaries), advancing `pos` by elapsed/msPerStep — so the drones glide
 * between cells rather than teleporting.
 *
 * rAF loop design (no per-frame setState for main React tree):
 *   - frameRef (useRef<number>) holds the current continuous position.
 *   - rafRef holds the rAF handle.
 *   - onFrameChange callback is throttled for slider/readout sync (rounded step).
 *   - Slider scrub writes directly to frameRef + requests a one-shot redraw.
 */

import {
  useRef,
  useEffect,
  useCallback,
  forwardRef,
  useImperativeHandle,
} from "react";
import type { PlaybackPayload } from "@/lib/types";
import type { PlaybackColors } from "./usePlaybackColors";
import { useCanvasDPR } from "@/hooks/useCanvasDPR";

// ─── Types ────────────────────────────────────────────────────────────────────

export interface GridCanvasHandle {
  /** Seek to a specific step and redraw immediately (from slider). */
  seekTo(step: number): void;
  /** Start playback from current frame. */
  play(): void;
  /** Pause playback. */
  pause(): void;
  /** Returns true if currently playing. */
  isPlaying(): boolean;
}

interface Props {
  payload: PlaybackPayload;
  colors: PlaybackColors;
  showAllBeliefLabels: boolean;
  speedMultiplier: number;
  /** Called (throttled) with current step so controls can stay in sync. */
  onFrameChange: (step: number, playing: boolean) => void;
}

// ─── World→canvas transform helpers ──────────────────────────────────────────

interface Transform {
  /** Convert world x (meters) to canvas pixel x */
  wx: (worldX: number) => number;
  /** Convert world y (meters) to canvas pixel y — NOTE: Y is flipped */
  wy: (worldY: number) => number;
  /** World length → pixel length */
  wl: (worldLen: number) => number;
}

function buildTransform(
  canvasW: number,
  canvasH: number,
  payload: PlaybackPayload
): Transform {
  const { grid_size, cell_side_length, trajectories } = payload;

  // World bounds: cells go from 0 to grid_size*cell_side_length,
  // plus we give 1 cell of margin on each side (for base station at -0.5*csl)
  const margin = cell_side_length;
  let xMin = -margin;
  let xMax = grid_size * cell_side_length + margin;
  let yMin = -margin;
  let yMax = grid_size * cell_side_length + margin;

  // Also account for any trajectory points outside the expected bounds
  for (const row of trajectories.x) {
    for (const v of row) {
      if (v < xMin) xMin = v;
      if (v > xMax) xMax = v;
    }
  }
  for (const row of trajectories.y) {
    for (const v of row) {
      if (v < yMin) yMin = v;
      if (v > yMax) yMax = v;
    }
  }

  // Fit world bbox to canvas with padding
  const padding = 20; // px
  const viewW = canvasW - padding * 2;
  const viewH = canvasH - padding * 2;

  const worldW = xMax - xMin;
  const worldH = yMax - yMin;
  const scale = Math.min(viewW / worldW, viewH / worldH);

  // Center the world in the canvas
  const offsetX = padding + (viewW - worldW * scale) / 2;
  const offsetY = padding + (viewH - worldH * scale) / 2;

  return {
    wx: (wx) => offsetX + (wx - xMin) * scale,
    // Flip Y: large worldY → small canvas Y
    wy: (wy) => canvasH - (offsetY + (wy - yMin) * scale),
    wl: (wl) => wl * scale,
  };
}

// ─── BFS to find nodes connected to base (node 0) ───────────────────────────

function getBaseConnectedSet(edges: number[][]): Set<number> {
  const adj = new Map<number, number[]>();
  for (const [i, j] of edges) {
    if (!adj.has(i)) adj.set(i, []);
    if (!adj.has(j)) adj.set(j, []);
    adj.get(i)!.push(j);
    adj.get(j)!.push(i);
  }
  const visited = new Set<number>();
  const queue = [0];
  visited.add(0);
  while (queue.length > 0) {
    const node = queue.shift()!;
    for (const neighbor of adj.get(node) ?? []) {
      if (!visited.has(neighbor)) {
        visited.add(neighbor);
        queue.push(neighbor);
      }
    }
  }
  return visited;
}

// ─── Belief→color interpolation (token-based, no hardcoded hex) ─────────────

/**
 * Parses `hsl(H S% L%)` string to [H, S, L] numbers.
 * Handles both space-separated values from CSS vars and full hsl() strings.
 */
function parseHSL(hslStr: string): [number, number, number] {
  // Match "hsl(42 100% 47%)" or "hsl(42, 100%, 47%)"
  const m = hslStr.match(
    /hsl\(\s*([\d.]+)\s*[,\s]\s*([\d.]+)%?\s*[,\s]\s*([\d.]+)%?\s*\)/
  );
  if (m) return [parseFloat(m[1]!), parseFloat(m[2]!), parseFloat(m[3]!)];
  return [0, 0, 10]; // dark fallback
}

function lerpHSL(
  from: [number, number, number],
  to: [number, number, number],
  t: number
): string {
  // Belief heat: keep the "hot" (high-belief) hue throughout and ramp only
  // saturation + lightness from the cool/low end to the hot/high end. A linear
  // HUE interpolation from a cool colour (~blue 207°) to amber (42°) passes
  // through green (~120°) at the midpoint, which misreads as "every cell is
  // active". Fixing the hue makes belief 0→1 read as dark→bright/hot.
  const h = to[0];
  const s = from[1] + (to[1] - from[1]) * t;
  const l = from[2] + (to[2] - from[2]) * t;
  return `hsl(${h.toFixed(1)} ${s.toFixed(1)}% ${l.toFixed(1)}%)`;
}

// ─── Main drawing function ────────────────────────────────────────────────────

function drawFrame(
  ctx: CanvasRenderingContext2D,
  payload: PlaybackPayload,
  pos: number,
  colors: PlaybackColors,
  showAllLabels: boolean,
  t: Transform
) {
  const {
    grid_size,
    cell_side_length,
    number_of_nodes,
    targets,
    belief_threshold,
    trajectories,
    connectivity,
    belief,
    targets_known,
  } = payload;

  const canvasW = ctx.canvas.width;
  const canvasH = ctx.canvas.height;
  ctx.clearRect(0, 0, canvasW, canvasH);

  // Continuous position → surrounding discrete steps + fraction for tweening.
  const maxStep = payload.steps - 1;
  const p = Math.max(0, Math.min(pos, maxStep));
  const i0 = Math.floor(p);
  const i1 = Math.min(i0 + 1, maxStep);
  const frac = p - i0;

  // Interpolated world position of a node at the current continuous position.
  const nodeX = (n: number): number => {
    const a = trajectories.x[n]?.[i0];
    const b = trajectories.x[n]?.[i1];
    if (a == null) return 0;
    return b == null ? a : a + (b - a) * frac;
  };
  const nodeY = (n: number): number => {
    const a = trajectories.y[n]?.[i0];
    const b = trajectories.y[n]?.[i1];
    if (a == null) return 0;
    return b == null ? a : a + (b - a) * frac;
  };

  // Belief at the current position (interpolated so the heat fades smoothly).
  const beliefAt = (c: number): number => {
    const a = belief[c]?.[i0];
    if (a == null) return 0;
    const av = Math.max(0, Math.min(1, a));
    const b = belief[c]?.[i1];
    if (b == null) return av;
    const bv = Math.max(0, Math.min(1, b));
    return av + (bv - av) * frac;
  };

  const cellPx = t.wl(cell_side_length);
  const lowHSL = parseHSL(colors.beliefLow);
  const highHSL = parseHSL(colors.beliefHigh);
  const targetSet = new Set(targets);

  // ── Layer 1: Grid cells with belief heat ───────────────────────────────────
  const numCells = grid_size * grid_size;
  for (let c = 0; c < numCells; c++) {
    const cx = (c % grid_size + 0.5) * cell_side_length;
    const cy = (Math.floor(c / grid_size) + 0.5) * cell_side_length;
    const px = t.wx(cx - cell_side_length / 2);
    const py = t.wy(cy + cell_side_length / 2); // top in canvas coords (y flipped)

    const beliefVal = beliefAt(c);
    ctx.fillStyle = lerpHSL(lowHSL, highHSL, beliefVal);
    ctx.fillRect(px, py, cellPx, cellPx);

    ctx.strokeStyle = colors.gridLine;
    ctx.lineWidth = 0.5;
    ctx.strokeRect(px, py, cellPx, cellPx);
  }

  // ── Layer 2: Target cells + "found" state ─────────────────────────────────
  for (const cell of targets) {
    const cx = (cell % grid_size + 0.5) * cell_side_length;
    const cy = (Math.floor(cell / grid_size) + 0.5) * cell_side_length;
    const px = t.wx(cx - cell_side_length / 2);
    const py = t.wy(cy + cell_side_length / 2);

    const beliefVal = beliefAt(cell);
    const found = beliefVal >= belief_threshold;

    if (found) {
      ctx.fillStyle = colors.foundFill;
      ctx.globalAlpha = 0.75;
      ctx.fillRect(px, py, cellPx, cellPx);
      ctx.globalAlpha = 1.0;
    }

    ctx.strokeStyle = colors.targetOutline;
    ctx.lineWidth = 2;
    ctx.strokeRect(px + 1, py + 1, cellPx - 2, cellPx - 2);

    ctx.fillStyle = found ? colors.foundFill : colors.labelColor;
    ctx.font = `bold ${Math.max(9, cellPx * 0.22).toFixed(0)}px monospace`;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(beliefVal.toFixed(2), px + cellPx / 2, py + cellPx / 2);
  }

  // ── Layer 2b: Show all cell belief labels if toggled ──────────────────────
  if (showAllLabels) {
    for (let c = 0; c < numCells; c++) {
      if (targetSet.has(c)) continue; // already labeled above
      if (belief[c]?.[i0] == null) continue;
      const cx = (c % grid_size + 0.5) * cell_side_length;
      const cy = (Math.floor(c / grid_size) + 0.5) * cell_side_length;
      const px = t.wx(cx - cell_side_length / 2);
      const py = t.wy(cy + cell_side_length / 2);
      ctx.fillStyle = colors.labelColor;
      ctx.globalAlpha = 0.55;
      ctx.font = `${Math.max(7, cellPx * 0.18).toFixed(0)}px monospace`;
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(beliefAt(c).toFixed(2), px + cellPx / 2, py + cellPx / 2);
      ctx.globalAlpha = 1.0;
    }
  }

  // ── Layer 3: Drone path trails (through waypoints 0..i0, then tween) ───────
  for (let n = 1; n < number_of_nodes; n++) {
    const droneColor = colors.droneColors[(n - 1) % colors.droneColors.length]!;
    const xArr = trajectories.x[n];
    const yArr = trajectories.y[n];
    if (!xArr || !yArr) continue;
    ctx.beginPath();
    ctx.strokeStyle = droneColor;
    ctx.lineWidth = 1.2;
    ctx.globalAlpha = 0.35;
    ctx.moveTo(t.wx(xArr[0]!), t.wy(yArr[0]!));
    for (let s = 1; s <= i0; s++) {
      ctx.lineTo(t.wx(xArr[s]!), t.wy(yArr[s]!));
    }
    // Partial segment from the last waypoint to the current interpolated pos
    ctx.lineTo(t.wx(nodeX(n)), t.wy(nodeY(n)));
    ctx.stroke();
    ctx.globalAlpha = 1.0;
  }

  // ── Layer 4: Connectivity edges (set at i0; endpoints tweened) ────────────
  const stepEdges = connectivity[i0] ?? [];
  const baseConnected = getBaseConnectedSet(stepEdges);

  for (const [i, j] of stepEdges) {
    const connected = baseConnected.has(i) && baseConnected.has(j);
    ctx.beginPath();
    ctx.strokeStyle = connected ? colors.edgeConnected : colors.edgeMuted;
    ctx.lineWidth = connected ? 1.8 : 0.9;
    ctx.globalAlpha = connected ? 0.7 : 0.3;
    ctx.moveTo(t.wx(nodeX(i)), t.wy(nodeY(i)));
    ctx.lineTo(t.wx(nodeX(j)), t.wy(nodeY(j)));
    ctx.stroke();
    ctx.globalAlpha = 1.0;
  }

  // ── Layer 5: Base station (node 0) — distinct square glyph ────────────────
  {
    const bx = t.wx(nodeX(0));
    const by = t.wy(nodeY(0));
    const sz = Math.max(8, cellPx * 0.2);
    ctx.fillStyle = colors.baseColor;
    ctx.fillRect(bx - sz / 2, by - sz / 2, sz, sz);
    ctx.globalAlpha = 0.4;
    ctx.fillStyle = colors.foundFill;
    ctx.fillRect(bx - sz / 4, by - sz / 4, sz / 2, sz / 2);
    ctx.globalAlpha = 1.0;
  }

  // Drones (nodes 1..number_of_nodes-1) — filled circles at tweened positions
  const droneR = Math.max(4, cellPx * 0.15);
  for (let n = 1; n < number_of_nodes; n++) {
    const droneColor = colors.droneColors[(n - 1) % colors.droneColors.length]!;
    const dx = t.wx(nodeX(n));
    const dy = t.wy(nodeY(n));

    ctx.beginPath();
    ctx.arc(dx, dy, droneR, 0, Math.PI * 2);
    ctx.fillStyle = droneColor;
    ctx.fill();
    ctx.strokeStyle = colors.foundFill;
    ctx.lineWidth = 1.2;
    ctx.globalAlpha = 0.6;
    ctx.stroke();
    ctx.globalAlpha = 1.0;

    ctx.fillStyle = colors.labelColor;
    ctx.font = `bold ${Math.max(8, droneR * 0.9).toFixed(0)}px monospace`;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(String(n), dx, dy);
  }

  // ── HUD: step counter + targets known ─────────────────────────────────────
  const dispStep = Math.round(p);
  const tk = targets_known[Math.min(dispStep, maxStep)] ?? 0;
  ctx.fillStyle = colors.labelColor;
  ctx.globalAlpha = 0.7;
  ctx.font = "11px monospace";
  ctx.textAlign = "left";
  ctx.textBaseline = "top";
  ctx.fillText(`STEP ${dispStep} / ${maxStep}`, 8, 8);
  ctx.fillText(`TARGETS FOUND: ${tk} / ${targets.length}`, 8, 22);
  ctx.globalAlpha = 1.0;
}

// ─── GridCanvas component ─────────────────────────────────────────────────────

const GridCanvas = forwardRef<GridCanvasHandle, Props>(function GridCanvas(
  { payload, colors, showAllBeliefLabels, speedMultiplier, onFrameChange },
  ref
) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const frameRef = useRef<number>(0); // continuous position (float)
  const playingRef = useRef<boolean>(false);
  const rafRef = useRef<number | null>(null);
  const lastRafTimeRef = useRef<number>(0);
  const lastNotifyRef = useRef<number>(0);
  const speedRef = useRef<number>(speedMultiplier);
  // ms per DISCRETE step at 1× speed. Smaller for fine-grained realtime (many
  // steps), larger for discrete so the tweened motion is clearly visible.
  const BASE_MS_PER_STEP = payload.steps > 200 ? 45 : 150;

  const getCtx = useCallback((): CanvasRenderingContext2D | null => {
    const canvas = canvasRef.current;
    if (!canvas) return null;
    return canvas.getContext("2d");
  }, []);

  const buildT = useCallback((): Transform | null => {
    const canvas = canvasRef.current;
    if (!canvas) return null;
    // Pass CSS logical size (not physical pixel size) so transform coordinates
    // stay in the space that ctx.scale(dpr, dpr) maps to.
    const dpr = window.devicePixelRatio || 1;
    return buildTransform(canvas.width / dpr, canvas.height / dpr, payload);
  }, [payload]);

  const redraw = useCallback(
    (pos: number) => {
      const ctx = getCtx();
      const t = buildT();
      if (!ctx || !t) return;
      drawFrame(ctx, payload, pos, colors, showAllBeliefLabels, t);
    },
    [getCtx, buildT, payload, colors, showAllBeliefLabels]
  );

  // `redraw` changes identity whenever a DRAW parameter changes (labels toggle,
  // theme colors). Everything that owns the rAF loop reads it through this ref
  // instead of depending on it, so a draw-parameter change can never tear the
  // running animation down (that used to cancel the rAF while playingRef stayed
  // true — animation frozen, but the PLAY/PAUSE button still read "PAUSE").
  const redrawRef = useRef(redraw);
  useEffect(() => {
    redrawRef.current = redraw;
  }, [redraw]);

  // Keep speedRef in sync so rafLoop always reads the latest speed
  useEffect(() => {
    speedRef.current = speedMultiplier;
  }, [speedMultiplier]);

  // rAF loop — advances the continuous position and redraws EVERY frame so the
  // drones glide between waypoints. speedMultiplier is read via speedRef.
  const rafLoop = useCallback(
    (ts: number) => {
      if (!playingRef.current) return;
      const maxStep = payload.steps - 1;
      const dt = ts - lastRafTimeRef.current;
      lastRafTimeRef.current = ts;
      const msPerStep = BASE_MS_PER_STEP / speedRef.current;
      const next = Math.min(frameRef.current + dt / msPerStep, maxStep);
      frameRef.current = next;
      redrawRef.current(next);

      // Throttled React state update for slider/readout (rounded to a step)
      if (ts - lastNotifyRef.current > 80) {
        lastNotifyRef.current = ts;
        onFrameChange(Math.round(next), true);
      }

      if (next >= maxStep) {
        playingRef.current = false;
        onFrameChange(maxStep, false);
        return; // stop loop
      }
      rafRef.current = requestAnimationFrame(rafLoop);
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [onFrameChange, payload.steps, BASE_MS_PER_STEP]
  );

  // Expose handle to parent
  useImperativeHandle(
    ref,
    () => ({
      seekTo(step: number) {
        frameRef.current = Math.max(0, Math.min(step, payload.steps - 1));
        redrawRef.current(frameRef.current);
        onFrameChange(Math.round(frameRef.current), playingRef.current);
      },
      play() {
        if (playingRef.current) return;
        // If at (or past) end, restart from the beginning
        if (frameRef.current >= payload.steps - 1) frameRef.current = 0;
        playingRef.current = true;
        lastRafTimeRef.current = performance.now();
        rafRef.current = requestAnimationFrame(rafLoop);
        onFrameChange(Math.round(frameRef.current), true);
      },
      pause() {
        playingRef.current = false;
        if (rafRef.current != null) {
          cancelAnimationFrame(rafRef.current);
          rafRef.current = null;
        }
        onFrameChange(Math.round(frameRef.current), false);
      },
      isPlaying() {
        return playingRef.current;
      },
    }),
    [rafLoop, onFrameChange, payload.steps]
  );

  // Initial draw + resize support. The callback is stable (reads the latest
  // redraw + frame through refs), so the sizing effect inside useCanvasDPR is
  // effectively mount-only and can never tear the rAF loop down mid-playback.
  const drawCurrent = useCallback(() => {
    redrawRef.current(frameRef.current);
  }, []);
  useCanvasDPR(canvasRef, drawCurrent);

  // rAF teardown on unmount.
  useEffect(() => {
    return () => {
      if (rafRef.current != null) {
        cancelAnimationFrame(rafRef.current);
        rafRef.current = null;
      }
    };
  }, []);

  // Draw parameters changed (labels toggle, theme colors) — repaint the current
  // frame. While playing, the rAF loop already repaints every frame with the
  // fresh parameters via redrawRef, so this is only needed when paused.
  useEffect(() => {
    if (!playingRef.current) redraw(frameRef.current);
  }, [redraw]);

  return (
    <canvas
      ref={canvasRef}
      style={{ width: "100%", height: "100%", display: "block" }}
    />
  );
});

export default GridCanvas;
