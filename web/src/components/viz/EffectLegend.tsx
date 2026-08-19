"use client";

/**
 * The legend for a set of overlaid ParameterEffectChart lines, plus the palette
 * both it and the charts colour from.
 *
 * It lives in its own module so a page can render ONE legend above a grid of
 * charts (each chart then gets `showLegend={false}`) without importing the
 * chart module itself — the model page loads ParameterEffectChart via
 * next/dynamic({ ssr: false }), and a static import of the same module for the
 * legend would undo that.
 *
 * Colours are assigned BY ARRAY INDEX, so a shared legend is only truthful
 * while every chart under it receives the same series list in the same order.
 */

import { useChartColors } from "@/hooks/useChartColors";
import { cn } from "@/lib/utils";
import { useStickyTop } from "@/components/layout/stickyOffsets";

export function buildPalette(base: string[], n: number): string[] {
  if (n <= base.length) return base.slice(0, Math.max(n, 1));
  const out = [...base];
  for (let i = base.length; i < n; i++) {
    // golden-angle hue spacing → maximally distinct categorical colors
    const hue = Math.round((i * 137.508) % 360);
    out.push(`hsl(${hue} 70% 58%)`);
  }
  return out;
}

/**
 * The row a shared legend sits in. `sticky` pins it under the page's own
 * header while its chart grid scrolls past — a legend that stands for a whole
 * grid is useless once it has scrolled away, which is exactly when the reader
 * reaches the charts furthest down.
 *
 * Exported so the bar-grid legend (square swatches, not line dashes) pins
 * identically without a second copy of the positioning.
 */
export function StickyLegendBar({
  sticky = false,
  surfaceClass = "bg-background",
  className,
  children,
}: {
  sticky?: boolean;
  /** Background of whatever this pins OVER — the bar has to be opaque in the
   *  same colour or it reads as a floating box. Page-level grids sit on the
   *  page background (the default); a grid inside a Card wants `bg-card`. */
  surfaceClass?: string;
  className?: string;
  children: React.ReactNode;
}) {
  const top = useStickyTop();
  return (
    <div
      style={sticky ? { top } : undefined}
      className={cn(
        "flex flex-wrap items-center gap-x-3 gap-y-1",
        // Opaque (not a blur): the marks it pins over are 1px strokes and
        // small squares that stay legible through a translucent panel and read
        // as part of the legend itself.
        sticky && ["sticky z-10 border-b border-border py-2", surfaceClass],
        className
      )}
    >
      {children}
    </div>
  );
}

/**
 * HTML (not in-SVG) so it wraps freely without ever overlapping a plot area —
 * a Recharts <Legend> mis-reserves space once it wraps to a second row.
 */
export default function EffectLegend({
  series,
  fallbackLabel,
  sticky = false,
  surfaceClass,
}: {
  /** one entry per line, in the same order the charts receive them */
  series: { key: string; label: string }[];
  /** label for an unlabelled series (single-dimension sweeps carry no label) */
  fallbackLabel?: string;
  /** pin it while its grid scrolls — see StickyLegendBar */
  sticky?: boolean;
  /** background to pin over, when that is not the page background */
  surfaceClass?: string;
}) {
  const colors = useChartColors();
  const palette = buildPalette(colors.series, series.length);
  if (series.length < 2) return null;
  return (
    <StickyLegendBar
      sticky={sticky}
      surfaceClass={surfaceClass}
      className="justify-center"
    >
      {series.map((s, i) => (
        <span
          key={s.key}
          className="flex items-center gap-1.5 font-mono text-[10px] text-muted-foreground"
        >
          <span
            className="inline-block h-0.5 w-3 rounded-full"
            style={{ backgroundColor: palette[i] ?? colors.series[0] }}
            aria-hidden="true"
          />
          {s.label || fallbackLabel}
        </span>
      ))}
    </StickyLegendBar>
  );
}
