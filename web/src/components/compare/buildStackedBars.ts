/**
 * buildStackedBars — pure helper for the compare-page stacked Bar view.
 *
 * Given flat scenario rows (model + parameter coordinates + ONE numeric value
 * for the active metric) it groups them by parameter COMBINATION (drones · comm ·
 * n_visits) along the x-axis, with one stack series per model. Each combo row
 * carries a value per model key (null where that model has no scenario for the
 * combo), so Recharts can stack the models within each bar.
 */

export interface StackedScenarioRow {
  model_key: string;
  number_of_drones: number | null;
  comm_range: string | null;
  comm_range_value: number | null;
  n_visits: number | null;
  /** value for the active objective/metric (already reduced to one number) */
  value: number | null;
}

/** One x-axis combo row: { comboKey, comboLabel, [modelKey]: value }. */
export type StackedRow = {
  comboKey: string;
  comboLabel: string;
} & Record<string, number | string | null>;

export interface StackedBarData {
  rows: StackedRow[];
  /** ordered model keys = the stack series (also legend order) */
  models: string[];
}

function comboLabel(
  drones: number | null,
  comm: string | null,
  nVisits: number | null
): string {
  const parts: string[] = [];
  if (drones != null) parts.push(`${drones}d`);
  if (comm != null) parts.push(`r${comm}`);
  if (nVisits != null) parts.push(`v${nVisits}`);
  return parts.join(" · ") || "—";
}

/**
 * Build stacked-bar rows (one per parameter combination) for a single metric.
 *
 * @param rows   flattened scenario rows with a single `value` each
 * @param models ordered model keys (stack series + legend order; usually the
 *               selected model set so colors stay stable across metrics)
 */
export function buildStackedBars(
  rows: StackedScenarioRow[],
  models: string[]
): StackedBarData {
  // Group rows by parameter combination, preserving sort coordinates.
  const combos = new Map<
    string,
    {
      label: string;
      dN: number;
      cN: number;
      vN: number;
      values: Record<string, number | null>;
    }
  >();

  for (const row of rows) {
    const key = `${row.number_of_drones}|${row.comm_range}|${row.n_visits}`;
    let combo = combos.get(key);
    if (!combo) {
      combo = {
        label: comboLabel(row.number_of_drones, row.comm_range, row.n_visits),
        dN: row.number_of_drones ?? 0,
        cN: row.comm_range_value ?? 0,
        vN: row.n_visits ?? 0,
        values: {},
      };
      combos.set(key, combo);
    }
    // Keep the (finite) value for this model; if a model somehow has two rows in
    // the same combo, the first non-null wins (combos are unique per model here).
    if (combo.values[row.model_key] == null) {
      combo.values[row.model_key] = row.value;
    }
  }

  const ordered = Array.from(combos.entries()).sort((a, b) => {
    const [, A] = a;
    const [, B] = b;
    return A.dN - B.dN || A.cN - B.cN || A.vN - B.vN;
  });

  const stackRows: StackedRow[] = ordered.map(([key, combo]) => {
    const row: StackedRow = { comboKey: key, comboLabel: combo.label };
    for (const m of models) row[m] = combo.values[m] ?? null;
    return row;
  });

  // Only keep models that actually have at least one value (so empty stack
  // series don't clutter the legend), preserving the requested order.
  const present = models.filter((m) =>
    stackRows.some((r) => typeof r[m] === "number")
  );

  return { rows: stackRows, models: present };
}
