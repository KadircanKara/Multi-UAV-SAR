"use client";

import { useEffect, useRef, useState } from "react";

/**
 * Reads the resolved HSL values of CSS tokens from a DOM element via
 * getComputedStyle, so Recharts SVG attributes get concrete color strings
 * (CSS vars don't reliably resolve inside SVG fill/stroke).
 *
 * Returns:
 *   series[0..4]   — --chart-1 through --chart-5
 *   grid           — --border  (grid lines, axis lines)
 *   axis           — --muted-foreground  (tick labels)
 *   tooltipBg      — --card  (tooltip background)
 *   tooltipBorder  — --border  (tooltip border)
 *   reference      — --chart-1  (reference lines; same as series[0])
 */
export interface ChartColors {
  series: string[];
  grid: string;
  axis: string;
  tooltipBg: string;
  tooltipBorder: string;
  reference: string;
}

// SSR / first-paint literal fallbacks derived from the theme token values.
const FALLBACKS: ChartColors = {
  series: [
    "hsl(42 100% 47%)",   // chart-1 amber / primary
    "hsl(151 100% 61%)",  // chart-2 accent green
    "hsl(190 82% 53%)",   // chart-3 cyan
    "hsl(294 94% 70%)",   // chart-4 magenta
    "hsl(200 17% 59%)",   // chart-5 muted grey
  ],
  grid:          "hsl(205 30% 13%)",  // --border
  axis:          "hsl(169 8% 45%)",   // --muted-foreground
  tooltipBg:     "hsl(207 32% 6%)",   // --card
  tooltipBorder: "hsl(205 30% 13%)",  // --border
  reference:     "hsl(42 100% 47%)",  // --chart-1
};

export function useChartColors(): ChartColors {
  const ref = useRef<HTMLDivElement | null>(null);
  const [colors, setColors] = useState<ChartColors>(FALLBACKS);

  useEffect(() => {
    if (typeof window === "undefined") return;
    const el = ref.current ?? document.documentElement;
    const style = getComputedStyle(el);

    function read(token: string, fallback: string): string {
      const raw = style.getPropertyValue(token).trim();
      return raw ? `hsl(${raw})` : fallback;
    }

    const series = [1, 2, 3, 4, 5].map((i) =>
      read(`--chart-${i}`, FALLBACKS.series[i - 1]!)
    );
    const grid          = read("--border",           FALLBACKS.grid);
    const axis          = read("--muted-foreground",  FALLBACKS.axis);
    const tooltipBg     = read("--card",              FALLBACKS.tooltipBg);
    const tooltipBorder = read("--border",            FALLBACKS.tooltipBorder);
    const reference     = series[0]!;

    setColors({ series, grid, axis, tooltipBg, tooltipBorder, reference });
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  return colors;
}
