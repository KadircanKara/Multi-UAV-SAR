/**
 * RunSummary — one-line identity + parameters for the run being analysed.
 *
 * Lives in the Analysis section's sticky header so the reader never loses track
 * of WHICH run the charts below belong to: the model key and objectives (the
 * badge row ScenarioExplorer would otherwise render further down, hence
 * `showSummary={false}` at that call site) plus the scenario and algorithm
 * parameters, which appear nowhere else for an uploaded file.
 *
 * Everything is read from the uploaded result; `front` only adds the solution
 * count and objective polarities once it has loaded, so the summary renders
 * immediately on upload rather than waiting on the reconstruct round-trip.
 */

import type { ParetoFront, PlaygroundResult } from "@/lib/types";
import { commLabel } from "@/lib/comm";
import { Badge } from "@/components/ui/badge";

// ─── Helpers ──────────────────────────────────────────────────────────────────

/** Scenario/run_config values are loosely typed (they come straight off an
 *  uploaded file), so read numbers defensively rather than casting. */
function num(bag: Record<string, unknown>, key: string): number | null {
  const v = bag[key];
  return typeof v === "number" && Number.isFinite(v) ? v : null;
}

function str(bag: Record<string, unknown>, key: string): string | null {
  const v = bag[key];
  return typeof v === "string" && v ? v : null;
}

/** comm_cell_range arrives as a number of cells; commLabel wants the library's
 *  key form, where a diagonal range is written sqrt(n) (2√2 → "sqrt(8)") so it
 *  can say "2 diagonal cells" instead of "2.83 cells". */
function commKey(cells: number): string {
  if (Number.isInteger(cells)) return String(cells);
  const squared = cells * cells;
  const rounded = Math.round(squared);
  return Math.abs(squared - rounded) < 1e-6 ? `sqrt(${rounded})` : String(cells);
}

function Fact({ label, value }: { label: string; value: string }) {
  return (
    <span className="flex items-baseline gap-1.5 whitespace-nowrap">
      <span className="tracking-widest text-muted-foreground">{label}</span>
      <span className="tabular-nums text-foreground">{value}</span>
    </span>
  );
}

// ─── Component ────────────────────────────────────────────────────────────────

interface Props {
  result: PlaygroundResult;
  front?: ParetoFront | null;
}

export default function RunSummary({ result, front }: Props) {
  const scenario = result.scenario as Record<string, unknown>;
  const runConfig = result.run_config as Record<string, unknown>;

  const modelKey = front?.model_key ?? result.model.model_key ?? "uploaded run";
  const objectives = front?.objectives ?? result.model.F;

  const drones = num(scenario, "number_of_drones");
  const commCells = num(scenario, "comm_cell_range");
  const cellSide = num(scenario, "cell_side_length");
  const visits = num(scenario, "n_visits");
  const grid = num(scenario, "grid_size");
  const speed = num(scenario, "max_drone_speed");

  // Prefer the method actually run over the model key's suffix — a saved run
  // carries both and they can disagree if the file was hand-edited.
  const method = str(runConfig, "method") ?? result.model.Alg;
  const optType = str(runConfig, "optimization_type") ?? result.model.Type;
  const popSize = num(runConfig, "pop_size");
  const nGen = num(runConfig, "n_gen");
  const seed = num(runConfig, "seed");

  return (
    <div className="flex flex-wrap items-center gap-x-4 gap-y-2">
      <div className="flex flex-wrap items-center gap-2">
        <Badge variant="outline" className="text-xs font-mono tracking-widest">
          {modelKey}
        </Badge>
        {optType && method && (
          <Badge variant="outline" className="text-xs font-mono tracking-widest">
            {optType} · {method}
          </Badge>
        )}
        {front && (
          <Badge variant="outline" className="text-xs font-mono tracking-widest">
            {front.n_solutions} SOLUTION{front.n_solutions !== 1 ? "S" : ""}
          </Badge>
        )}
        {objectives.map((obj) => (
          <Badge
            key={obj}
            className="text-xs font-mono tracking-wide bg-secondary text-secondary-foreground"
          >
            {obj}
            {front?.polarities[obj] === -1 && (
              <span className="ml-1 text-muted-foreground">(max)</span>
            )}
          </Badge>
        ))}
      </div>

      <div className="flex flex-wrap items-center gap-x-3 gap-y-1 font-mono text-[11px]">
        {drones != null && <Fact label="DRONES" value={String(drones)} />}
        {commCells != null && (
          <Fact label="COMM" value={commLabel(commKey(commCells), cellSide ?? 50)} />
        )}
        {visits != null && <Fact label="N_VISITS" value={String(visits)} />}
        {grid != null && (
          <Fact label="GRID" value={`${grid} × ${grid}`} />
        )}
        {cellSide != null && <Fact label="CELL" value={`${cellSide} m`} />}
        {speed != null && <Fact label="SPEED" value={`${speed} m/s`} />}
        {popSize != null && <Fact label="POP" value={String(popSize)} />}
        {nGen != null && <Fact label="GEN" value={String(nGen)} />}
        {seed != null && <Fact label="SEED" value={String(seed)} />}
      </div>
    </div>
  );
}
