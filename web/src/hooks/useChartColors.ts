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

// SSR / first-paint literal fallbacks (light theme — clean indigo + slate).
const FALLBACKS: ChartColors = {
  series: [
    "hsl(243 75% 59%)",   // chart-1 indigo
    "hsl(160 84% 39%)",   // chart-2 emerald
    "hsl(25 95% 53%)",    // chart-3 orange
    "hsl(339 90% 51%)",   // chart-4 rose
    "hsl(262 83% 58%)",   // chart-5 violet
  ],
  grid:          "hsl(214 32% 91%)",  // --border
  axis:          "hsl(215 16% 47%)",  // --muted-foreground
  tooltipBg:     "hsl(0 0% 100%)",    // --card
  tooltipBorder: "hsl(214 32% 91%)",  // --border
  reference:     "hsl(243 75% 59%)",  // --chart-1
};

export function useChartColors(): ChartColors {
  const ref = useRef<HTMLDivElement | null>(null);
  const [colors, setColors] = useState<ChartColors>(FALLBACKS);

  useEffect(() => {
    if (typeof window === "undefined") return;

    function resolve(): ChartColors {
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
      return { series, grid, axis, tooltipBg, tooltipBorder, reference: series[0]! };
    }

    setColors(resolve());

    // Re-resolve when the theme class on <html> changes (light ↔ dark).
    const observer = new MutationObserver(() => setColors(resolve()));
    observer.observe(document.documentElement, {
      attributes: true,
      attributeFilter: ["class"],
    });
    return () => observer.disconnect();
  }, []);

  return colors;
}
