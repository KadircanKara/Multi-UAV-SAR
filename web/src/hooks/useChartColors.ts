"use client";

import { useEffect, useRef, useState } from "react";

/**
 * Reads the resolved HSL values of --chart-1 through --chart-5 CSS tokens
 * from a DOM element via getComputedStyle, so Recharts SVG attributes get
 * concrete color strings (CSS vars don't resolve inside SVG fill/stroke).
 */
export function useChartColors(): string[] {
  const ref = useRef<HTMLDivElement | null>(null);
  const [colors, setColors] = useState<string[]>([
    "hsl(42 100% 47%)",   // chart-1 amber / primary
    "hsl(151 100% 61%)",  // chart-2 accent green
    "hsl(190 82% 53%)",   // chart-3 cyan
    "hsl(294 94% 70%)",   // chart-4 magenta
    "hsl(200 17% 59%)",   // chart-5 muted grey
  ]);

  useEffect(() => {
    if (typeof window === "undefined") return;
    const el = ref.current ?? document.documentElement;
    const style = getComputedStyle(el);
    const resolved = [1, 2, 3, 4, 5].map((i) => {
      const raw = style.getPropertyValue(`--chart-${i}`).trim();
      return raw ? `hsl(${raw})` : colors[i - 1];
    });
    setColors(resolved);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  return colors;
}
