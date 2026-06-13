import type { RunConfig, MissionConfigResponse } from "@/lib/types";

/** Narrow the read-endpoint response to a recorded RunConfig. */
export function isRunConfig(r: MissionConfigResponse): r is RunConfig {
  return (r as RunConfig).schema_version != null;
}

/** "Fixed · 800 generations" or "Max 200 gens · early-stop after 15 stalls (<5%)". */
export function strategyLabel(c: RunConfig): string {
  if (c.gen_strategy === "max") {
    const p = c.early_stop_patience ?? "?";
    const t =
      c.early_stop_threshold != null
        ? `${Math.round(c.early_stop_threshold * 100)}%`
        : "?";
    return `Max ${c.n_gen} gens · early-stop after ${p} stalls (<${t})`;
  }
  return `Fixed · ${c.n_gen} generations`;
}

/** Human list of the active constraints with their thresholds. */
export function constraintList(c: RunConfig): string[] {
  const out: string[] = ["Speed feasibility"];
  const k = c.constraints;
  if (k.max_mission_time != null) out.push(`Max mission time ≤ ${k.max_mission_time}s`);
  if (k.min_connectivity != null) out.push(`Min connectivity ≥ ${k.min_connectivity}`);
  if (k.max_mean_tbv != null) out.push(`Max mean TBV ≤ ${k.max_mean_tbv}s`);
  return out;
}

/** "Mission Time 0.50 · Percentage Connectivity 0.50", or null for non-WS runs. */
export function weightsLabel(c: RunConfig): string | null {
  if (!c.weights) return null;
  return Object.entries(c.weights)
    .map(([k, v]) => `${k} ${v.toFixed(2)}`)
    .join(" · ");
}

/** "Seeded (thesis)" vs "Optimizer run". */
export function sourceLabel(c: RunConfig): string {
  return c.source === "seed" ? "Seeded (thesis)" : "Optimizer run";
}
