"use client";

/**
 * usePlaybackColors — resolves all canvas drawing colors from CSS theme tokens.
 * NO hardcoded hex/rgb/hsl literals in this file; every color comes from
 * getComputedStyle on document.documentElement.
 */

import { useEffect, useState } from "react";

export interface PlaybackColors {
  /** Grid cell fill when belief = 0 (background card color) */
  beliefLow: string;
  /** Grid cell fill when belief = 1 (warm amber) */
  beliefHigh: string;
  /** Grid line stroke */
  gridLine: string;
  /** Target cell outline stroke */
  targetOutline: string;
  /** "Found" target fill (accent green) */
  foundFill: string;
  /** Connectivity edge — base-connected (accent) */
  edgeConnected: string;
  /** Connectivity edge — not connected to base (muted) */
  edgeMuted: string;
  /** Drone trail + marker colors cycling chart-1..5 */
  droneColors: string[];
  /** Base station glyph color (primary amber) */
  baseColor: string;
  /** Belief label text color */
  labelColor: string;
}

const FALLBACKS: PlaybackColors = {
  beliefLow:      "hsl(207 32% 6%)",   // --card
  beliefHigh:     "hsl(42 100% 47%)",  // --chart-1 amber
  gridLine:       "hsl(205 30% 13%)",  // --border
  targetOutline:  "hsl(42 100% 47%)",  // --primary
  foundFill:      "hsl(151 100% 61%)", // --accent
  edgeConnected:  "hsl(151 100% 61%)", // --accent
  edgeMuted:      "hsl(169 8% 45%)",   // --muted-foreground
  droneColors: [
    "hsl(42 100% 47%)",   // chart-1
    "hsl(151 100% 61%)",  // chart-2
    "hsl(190 82% 53%)",   // chart-3
    "hsl(294 94% 70%)",   // chart-4
    "hsl(200 17% 59%)",   // chart-5
  ],
  baseColor:  "hsl(42 100% 47%)",  // --primary
  labelColor: "hsl(150 8% 80%)",   // --foreground
};

export function usePlaybackColors(): PlaybackColors {
  const [colors, setColors] = useState<PlaybackColors>(FALLBACKS);

  useEffect(() => {
    if (typeof window === "undefined") return;
    const style = getComputedStyle(document.documentElement);

    function read(token: string, fallback: string): string {
      const raw = style.getPropertyValue(token).trim();
      return raw ? `hsl(${raw})` : fallback;
    }

    const droneColors = [1, 2, 3, 4, 5].map((i) =>
      read(`--chart-${i}`, FALLBACKS.droneColors[i - 1]!)
    );

    setColors({
      beliefLow:     read("--card",             FALLBACKS.beliefLow),
      beliefHigh:    read("--chart-1",          FALLBACKS.beliefHigh),
      gridLine:      read("--border",           FALLBACKS.gridLine),
      targetOutline: read("--primary",          FALLBACKS.targetOutline),
      foundFill:     read("--accent",           FALLBACKS.foundFill),
      edgeConnected: read("--accent",           FALLBACKS.edgeConnected),
      edgeMuted:     read("--muted-foreground", FALLBACKS.edgeMuted),
      droneColors,
      baseColor:     read("--primary",          FALLBACKS.baseColor),
      labelColor:    read("--foreground",       FALLBACKS.labelColor),
    });
  }, []);

  return colors;
}
