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
 * rAF loop design (no per-frame setState for main React tree):
 *   - frameRef (useRef<number>) holds current step — only the canvas reads it.
 *   - rafRef holds the rAF handle.
 *   - onFrameChange callback is throttled (every ~100ms) for slider/readout sync.
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
  const worldXMin = -margin;
  const worldXMax = grid_size * cell_side_length + margin;
  const worldYMin = -margin;
  const worldYMax = grid_size * cell_side_length + margin;

  // Also account for any trajectory points outside the expected bounds
  let xMin = worldXMin;
  let xMax = worldXMax;
  let yMin = worldYMin;
  let yMax = worldYMax;

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
  // For Y: canvas Y=0 is top; world Y increases upward → flip
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
  step: number,
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

  // Clear
  ctx.clearRect(0, 0, canvasW, canvasH);

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
    const py = t.wy(cy + cell_side_length / 2); // top in canvas coords (y is flipped)

    const rawBelief = belief[c]?.[step];
    const beliefVal = rawBelief != null ? Math.max(0, Math.min(1, rawBelief)) : 0;

    // Belief heat ramp
    const fillColor = lerpHSL(lowHSL, highHSL, beliefVal);
    ctx.fillStyle = fillColor;
    ctx.fillRect(px, py, cellPx, cellPx);

    // Grid line
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

    const rawBelief = belief[cell]?.[step];
    const beliefVal = rawBelief != null ? Math.max(0, Math.min(1, rawBelief)) : 0;
    const found = beliefVal >= belief_threshold;

    if (found) {
      ctx.fillStyle = colors.foundFill;
      ctx.globalAlpha = 0.75;
      ctx.fillRect(px, py, cellPx, cellPx);
      ctx.globalAlpha = 1.0;
    }

    // Target outline
    ctx.strokeStyle = colors.targetOutline;
    ctx.lineWidth = 2;
    ctx.strokeRect(px + 1, py + 1, cellPx - 2, cellPx - 2);

    // Belief label on target cells
    const labelVal = rawBelief != null ? rawBelief.toFixed(2) : "?";
    ctx.fillStyle = found ? colors.foundFill : colors.labelColor;
    ctx.font = `bold ${Math.max(9, cellPx * 0.22).toFixed(0)}px monospace`;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(labelVal, px + cellPx / 2, py + cellPx / 2);
  }

  // ── Layer 2b: Show all cell belief labels if toggled ──────────────────────
  if (showAllLabels) {
    for (let c = 0; c < numCells; c++) {
      if (targetSet.has(c)) continue; // already labeled above
      const cx = (c % grid_size + 0.5) * cell_side_length;
      const cy = (Math.floor(c / grid_size) + 0.5) * cell_side_length;
      const px = t.wx(cx - cell_side_length / 2);
      const py = t.wy(cy + cell_side_length / 2);

      const rawBelief = belief[c]?.[step];
      if (rawBelief == null) continue;
      ctx.fillStyle = colors.labelColor;
      ctx.globalAlpha = 0.55;
      ctx.font = `${Math.max(7, cellPx * 0.18).toFixed(0)}px monospace`;
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(rawBelief.toFixed(2), px + cellPx / 2, py + cellPx / 2);
      ctx.globalAlpha = 1.0;
    }
  }

  // ── Layer 3: Drone path trails (node 0 = base, skip trail for base) ────────
  for (let n = 1; n < number_of_nodes; n++) {
    const droneColor = colors.droneColors[(n - 1) % colors.droneColors.length]!;
    ctx.beginPath();
    ctx.strokeStyle = droneColor;
    ctx.lineWidth = 1.2;
    ctx.globalAlpha = 0.35;
    const xArr = trajectories.x[n]!;
    const yArr = trajectories.y[n]!;
    let started = false;
    for (let s = 0; s <= step; s++) {
      const px = t.wx(xArr[s]!);
      const py = t.wy(yArr[s]!);
      if (!started) {
        ctx.moveTo(px, py);
        started = true;
      } else {
        ctx.lineTo(px, py);
      }
    }
    ctx.stroke();
    ctx.globalAlpha = 1.0;
  }

  // ── Layer 4: Connectivity edges ────────────────────────────────────────────
  const stepEdges = connectivity[step] ?? [];
  const baseConnected = getBaseConnectedSet(stepEdges);

  for (const [i, j] of stepEdges) {
    const xiArr = trajectories.x[i]!;
    const yiArr = trajectories.y[i]!;
    const xjArr = trajectories.x[j]!;
    const yjArr = trajectories.y[j]!;

    const connected = baseConnected.has(i) && baseConnected.has(j);
    ctx.beginPath();
    ctx.strokeStyle = connected ? colors.edgeConnected : colors.edgeMuted;
    ctx.lineWidth = connected ? 1.8 : 0.9;
    ctx.globalAlpha = connected ? 0.7 : 0.3;
    ctx.moveTo(t.wx(xiArr[step]!), t.wy(yiArr[step]!));
    ctx.lineTo(t.wx(xjArr[step]!), t.wy(yjArr[step]!));
    ctx.stroke();
    ctx.globalAlpha = 1.0;
  }

  // ── Layer 5: Drone markers ─────────────────────────────────────────────────
  // Base station (node 0) — distinct square glyph
  {
    const bx = t.wx(trajectories.x[0]![step]!);
    const by = t.wy(trajectories.y[0]![step]!);
    const sz = Math.max(8, cellPx * 0.2);
    ctx.fillStyle = colors.baseColor;
    ctx.strokeStyle = colors.baseColor;
    ctx.lineWidth = 2;
    ctx.fillRect(bx - sz / 2, by - sz / 2, sz, sz);
    // Inner highlight
    ctx.globalAlpha = 0.4;
    ctx.fillStyle = colors.foundFill;
    ctx.fillRect(bx - sz / 4, by - sz / 4, sz / 2, sz / 2);
    ctx.globalAlpha = 1.0;
  }

  // Drones (nodes 1..number_of_nodes-1) — filled circles
  const droneR = Math.max(4, cellPx * 0.15);
  for (let n = 1; n < number_of_nodes; n++) {
    const droneColor = colors.droneColors[(n - 1) % colors.droneColors.length]!;
    const dx = t.wx(trajectories.x[n]![step]!);
    const dy = t.wy(trajectories.y[n]![step]!);

    ctx.beginPath();
    ctx.arc(dx, dy, droneR, 0, Math.PI * 2);
    ctx.fillStyle = droneColor;
    ctx.fill();
    ctx.strokeStyle = colors.foundFill;
    ctx.lineWidth = 1.2;
    ctx.globalAlpha = 0.6;
    ctx.stroke();
    ctx.globalAlpha = 1.0;

    // Drone index label
    ctx.fillStyle = colors.labelColor;
    ctx.font = `bold ${Math.max(8, droneR * 0.9).toFixed(0)}px monospace`;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(String(n), dx, dy);
  }

  // ── HUD: step counter + targets known ─────────────────────────────────────
  const tk = targets_known[step] ?? 0;
  ctx.fillStyle = colors.labelColor;
  ctx.globalAlpha = 0.7;
  ctx.font = "11px monospace";
  ctx.textAlign = "left";
  ctx.textBaseline = "top";
  ctx.fillText(`STEP ${step} / ${payload.steps - 1}`, 8, 8);
  ctx.fillText(`TARGETS FOUND: ${tk} / ${targets.length}`, 8, 22);
  ctx.globalAlpha = 1.0;
}

// ─── GridCanvas component ─────────────────────────────────────────────────────

const GridCanvas = forwardRef<GridCanvasHandle, Props>(function GridCanvas(
  { payload, colors, showAllBeliefLabels, speedMultiplier, onFrameChange },
  ref
) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const frameRef = useRef<number>(0);
  const playingRef = useRef<boolean>(false);
  const rafRef = useRef<number | null>(null);
  const lastRafTimeRef = useRef<number>(0);
  const lastNotifyRef = useRef<number>(0);
  const speedRef = useRef<number>(speedMultiplier);
  // ms per step at 1× speed — target ~30fps for discrete; ~10fps for large realtime
  const BASE_MS_PER_STEP = payload.steps > 200 ? 50 : 100;

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
    (step: number) => {
      const ctx = getCtx();
      const t = buildT();
      if (!ctx || !t) return;
      drawFrame(ctx, payload, step, colors, showAllBeliefLabels, t);
    },
    [getCtx, buildT, payload, colors, showAllBeliefLabels]
  );

  // Keep speedRef in sync so rafLoop always reads the latest speed
  useEffect(() => {
    speedRef.current = speedMultiplier;
  }, [speedMultiplier]);

  // rAF loop — advances frameRef, does NOT call setState
  // speedMultiplier is intentionally NOT in deps; read via speedRef for stability
  const rafLoop = useCallback(
    (ts: number) => {
      if (!playingRef.current) return;
      const msPerStep = BASE_MS_PER_STEP / speedRef.current;
      const elapsed = ts - lastRafTimeRef.current;
      if (elapsed >= msPerStep) {
        const steps = Math.max(1, Math.floor(elapsed / msPerStep));
        lastRafTimeRef.current = ts;
        const next = Math.min(frameRef.current + steps, payload.steps - 1);
        frameRef.current = next;
        redraw(next);

        // Throttled React state update (≤10/sec) for slider/readout
        if (ts - lastNotifyRef.current > 100) {
          lastNotifyRef.current = ts;
          onFrameChange(next, true);
        }

        if (next >= payload.steps - 1) {
          playingRef.current = false;
          onFrameChange(payload.steps - 1, false);
          return; // stop loop
        }
      }
      rafRef.current = requestAnimationFrame(rafLoop);
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [redraw, onFrameChange, payload.steps, BASE_MS_PER_STEP]
  );

  // Expose handle to parent
  useImperativeHandle(
    ref,
    () => ({
      seekTo(step: number) {
        frameRef.current = Math.max(0, Math.min(step, payload.steps - 1));
        redraw(frameRef.current);
        onFrameChange(frameRef.current, playingRef.current);
      },
      play() {
        if (playingRef.current) return;
        // If at end, restart
        if (frameRef.current >= payload.steps - 1) frameRef.current = 0;
        playingRef.current = true;
        lastRafTimeRef.current = performance.now();
        rafRef.current = requestAnimationFrame(rafLoop);
        onFrameChange(frameRef.current, true);
      },
      pause() {
        playingRef.current = false;
        if (rafRef.current != null) {
          cancelAnimationFrame(rafRef.current);
          rafRef.current = null;
        }
        onFrameChange(frameRef.current, false);
      },
      isPlaying() {
        return playingRef.current;
      },
    }),
    [redraw, rafLoop, onFrameChange, payload.steps]
  );

  // Initial draw + resize support
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    function applyDPR() {
      if (!canvas) return;
      const dpr = window.devicePixelRatio || 1;
      const rect = canvas.getBoundingClientRect();
      canvas.width = rect.width * dpr;
      canvas.height = rect.height * dpr;
      const ctx = canvas.getContext("2d");
      if (ctx) ctx.scale(dpr, dpr);
      // After resize, reset transform to logical sizes
    }

    applyDPR();
    redraw(frameRef.current);

    const ro = new ResizeObserver(() => {
      applyDPR();
      redraw(frameRef.current);
    });
    ro.observe(canvas);

    return () => {
      ro.disconnect();
      if (rafRef.current != null) cancelAnimationFrame(rafRef.current);
    };
  }, [redraw]);

  // Note: colors/labels/payload changes are handled by the effect above via [redraw],
  // since `redraw` itself depends on colors, showAllBeliefLabels, and payload.

  return (
    <canvas
      ref={canvasRef}
      style={{ width: "100%", height: "100%", display: "block" }}
    />
  );
});

export default GridCanvas;
