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

/**
 * Physical unit of each objective, for axis labels.
 *
 * Read off the computations, not off the backend's `obj_unit_dict` in
 * PathOptimizationModel.py, which disagrees with the code in one place: it
 * gives Mean Disconnected Time as a ratio (t_iso/T), but PathSolution.py
 * computes it as `np.mean(drone_disconnected_times)` where each entry is
 * incremented once per time slot a drone spends disconnected — a mean over
 * drones of a STEP COUNT, which is why the UI shows values well above 1.
 *
 * The times that are genuinely seconds are the ones summed from
 * `time_elapsed_at_steps`, each term being metres / (m/s): Mission Time
 * (PathSolution.py), Max Mean TBV (`get_tbv`), and the sensing metrics
 * derived from the same array in Sensing.py.
 *
 * An objective that is absent here simply gets no unit — which is correct for
 * a genuinely dimensionless one, and safer than guessing for an objective
 * added later.
 */
const OBJECTIVE_UNITS: Record<string, string> = {
  "Mission Time": "s",
  "Percentage Connectivity": "%",
  "Max Mean TBV": "s",
  // Counts of time slots spent disconnected, not seconds.
  "Max Disconnected Time": "steps",
  "Mean Disconnected Time": "steps",
  // Sensing-replay metrics, summed from time_elapsed_at_steps.
  "Effective Mission Time": "s",
  "Detection Time": "s",
  "Inform Time": "s",
  "Time At Least One Drone Knows All Targets": "s",
};

/** Unit symbol for an objective, or null when it has none / is unknown. */
export function objectiveUnit(objective: string): string | null {
  return OBJECTIVE_UNITS[objective] ?? null;
}

/**
 * Axis label for an objective: its name, then its unit and whether it is
 * maximised, in one bracket — "Mission Time (s)", "Percentage Connectivity
 * (%, max)". One bracket rather than two so a maximised objective with a unit
 * does not read "Name (%) (max)".
 */
export function objectiveAxisLabel(
  objective: string,
  polarity?: number
): string {
  const parts: string[] = [];
  const unit = objectiveUnit(objective);
  if (unit) parts.push(unit);
  if (polarity === -1) parts.push("max");
  return parts.length > 0 ? `${objective} (${parts.join(", ")})` : objective;
}

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
