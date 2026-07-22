"use client";

import { useEffect, type RefObject } from "react";

/**
 * useCanvasDPR — keeps a <canvas> backing store sized to its CSS box ×
 * devicePixelRatio and redraws whenever the element resizes. Shared by the
 * canvas-based charts (GridCanvas, ParetoScatter3D) so DPR handling cannot
 * drift between them. Uses setTransform (not a cumulative scale) so
 * re-applying after a resize is idempotent.
 *
 * `draw` is an effect dependency. Pass a STABLE callback — typically one that
 * reads the real draw function through a ref — when the effect must not tear
 * down and re-observe on every render (or, in GridCanvas's case, must never
 * re-run at all while an animation loop owns the canvas).
 */
export function useCanvasDPR(
  canvasRef: RefObject<HTMLCanvasElement>,
  draw: () => void
) {
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
      if (ctx) ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    }

    applyDPR();
    draw();

    const ro = new ResizeObserver(() => {
      applyDPR();
      draw();
    });
    ro.observe(canvas);
    return () => ro.disconnect();
  }, [canvasRef, draw]);
}
