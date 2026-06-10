"use client";

/**
 * usePlaybackColors — resolves all canvas drawing colors from CSS theme tokens.
 * NO hardcoded hex/rgb/hsl literals drive the live UI; every color comes from
 * getComputedStyle on document.documentElement. The literals below are only
 * SSR / first-paint fallbacks (light theme).
 *
 * Note: the vivid roles (found / connected / belief heat) read --chart-* rather
 * than --accent, because in the clean theme --accent is a subtle slate hover.
 */

import { useEffect, useState } from "react";

export interface PlaybackColors {
  /** Grid cell fill when belief = 0 (background card color) */
  beliefLow: string;
  /** Grid cell fill when belief = 1 (warm heat) */
  beliefHigh: string;
  /** Grid line stroke */
  gridLine: string;
  /** Target cell outline stroke */
  targetOutline: string;
  /** "Found" target fill */
  foundFill: string;
  /** Connectivity edge — base-connected */
  edgeConnected: string;
  /** Connectivity edge — not connected to base (muted) */
  edgeMuted: string;
  /** Drone trail + marker colors cycling chart-1..5 */
  droneColors: string[];
  /** Base station glyph color */
  baseColor: string;
  /** Belief label text color */
  labelColor: string;
}

const FALLBACKS: PlaybackColors = {
  beliefLow:      "hsl(0 0% 100%)",     // --card (white)
  beliefHigh:     "hsl(25 95% 53%)",    // --chart-3 orange (heat)
  gridLine:       "hsl(214 32% 91%)",   // --border
  targetOutline:  "hsl(222 47% 11%)",   // --primary
  foundFill:      "hsl(160 84% 39%)",   // --chart-2 emerald
  edgeConnected:  "hsl(243 75% 59%)",   // --chart-1 indigo
  edgeMuted:      "hsl(215 16% 47%)",   // --muted-foreground
  droneColors: [
    "hsl(243 75% 59%)",   // chart-1 indigo
    "hsl(160 84% 39%)",   // chart-2 emerald
    "hsl(25 95% 53%)",    // chart-3 orange
    "hsl(339 90% 51%)",   // chart-4 rose
    "hsl(262 83% 58%)",   // chart-5 violet
  ],
  baseColor:  "hsl(222 47% 11%)",  // --primary
  labelColor: "hsl(222 47% 11%)",  // --foreground
};

export function usePlaybackColors(): PlaybackColors {
  const [colors, setColors] = useState<PlaybackColors>(FALLBACKS);

  useEffect(() => {
    if (typeof window === "undefined") return;

    function resolve(): PlaybackColors {
      const style = getComputedStyle(document.documentElement);
      function read(token: string, fallback: string): string {
        const raw = style.getPropertyValue(token).trim();
        return raw ? `hsl(${raw})` : fallback;
      }
      const droneColors = [1, 2, 3, 4, 5].map((i) =>
        read(`--chart-${i}`, FALLBACKS.droneColors[i - 1]!)
      );
      return {
        beliefLow:     read("--card",             FALLBACKS.beliefLow),
        beliefHigh:    read("--chart-3",          FALLBACKS.beliefHigh),
        gridLine:      read("--border",           FALLBACKS.gridLine),
        targetOutline: read("--primary",          FALLBACKS.targetOutline),
        foundFill:     read("--chart-2",          FALLBACKS.foundFill),
        edgeConnected: read("--chart-1",          FALLBACKS.edgeConnected),
        edgeMuted:     read("--muted-foreground", FALLBACKS.edgeMuted),
        droneColors,
        baseColor:     read("--primary",          FALLBACKS.baseColor),
        labelColor:    read("--foreground",       FALLBACKS.labelColor),
      };
    }

    setColors(resolve());

    // Re-resolve when the theme (class on <html>) changes.
    const observer = new MutationObserver(() => setColors(resolve()));
    observer.observe(document.documentElement, {
      attributes: true,
      attributeFilter: ["class"],
    });
    return () => observer.disconnect();
  }, []);

  return colors;
}
