/**
 * Objective value formatting shared across every readout, table, tooltip, and
 * chart axis.
 *
 * Some objectives are stored as a 0–1 fraction but are meaningful to the user as
 * a percentage (Percentage Connectivity → "83.0%"). Centralising WHICH objectives
 * are percentages, and HOW a percentage renders, keeps every view in agreement —
 * so a tooltip, a table cell, and the axis a point is plotted against never
 * disagree on units.
 *
 * These helpers deliberately own only the percentage rule; each call site keeps
 * its own numeric precision for non-percentage objectives, so wiring a site
 * through here never changes how its other values look.
 */

/** Objectives stored as a 0–1 fraction but displayed as a percentage. */
const PERCENT_OBJECTIVES = new Set<string>(["Percentage Connectivity"]);

export function isPercentObjective(objective: string): boolean {
  return PERCENT_OBJECTIVES.has(objective);
}

/** Render a 0–1 fraction as a percentage string, e.g. 0.831 → "83.1%". */
export function percentString(fraction: number, decimals = 1): string {
  return `${(fraction * 100).toFixed(decimals)}%`;
}

/** Percentage for a chart axis tick — whole numbers keep ticks compact
 *  (0.8 → "80%"). */
export function percentTick(fraction: number): string {
  return `${Math.round(fraction * 100)}%`;
}

/** Clamp a padded axis domain to the objective's physical range. A percentage
 *  objective is a 0–1 fraction, so its axis must never run past 100% (or below
 *  0%) however much headroom the trend padding would otherwise add. Non-percent
 *  objectives are returned unchanged. */
export function clampObjectiveDomain(
  objective: string,
  domain: [number, number]
): [number, number] {
  if (!isPercentObjective(objective)) return domain;
  return [Math.max(0, domain[0]), Math.min(1, domain[1])];
}
