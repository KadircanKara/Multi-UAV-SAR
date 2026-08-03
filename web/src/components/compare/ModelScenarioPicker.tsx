"use client";

/**
 * ModelScenarioPicker — the selection panel for the /compare page.
 *
 * Lets the user pick a set of MODELS (multi-select chips) plus a set of
 * parameter values (drones / comm-range / n_visits, multi-select chips taken as
 * the UNION across the selected models). The resolved scenario set is every
 * library scenario whose (model, drones, comm, n_visits) all fall inside the
 * current selection. Mirrors the chip/overlay pattern from the model page's
 * SweepControls but here the entities span MULTIPLE models.
 *
 * Emits, on every change, the resolved scenario-name set plus the selected
 * model_keys and the per-dimension value selections (the page needs both: the
 * scenario set for the snapshot views, the model list for the line view's
 * ≥2-models guard).
 */

import { useEffect, useMemo, useRef, useState } from "react";
import type { ScenarioSummary } from "@/lib/types";
import type { SweepParam } from "@/components/compare/buildModelSeries";
import { commCellValue, commLabel } from "@/lib/comm";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { ToggleGroup, ToggleGroupItem } from "@/components/ui/toggle-group";

// ─── Emitted selection shape ──────────────────────────────────────────────────

export interface PickerSelection {
  /** resolved library scenario names matching every active dimension */
  scenarios: string[];
  /** the selected model_keys (line view needs ≥2) */
  models: string[];
  /** Raw per-dimension selections (string keys) — everything the picker holds,
   *  so a caller can put the selection in the URL and hand it back verbatim. */
  params: {
    drones: string[];
    comm_range: string[];
    n_visits: string[];
    speed: string[];
    grid: string[];
    cell: string[];
  };
}

interface Props {
  library: ScenarioSummary[];
  onChange: (sel: PickerSelection) => void;
  /** When true (line chart active), the non-sweep parameter rows become single-
   *  select to keep the line plot legible — only the sweep dimension and the
   *  models stay multi-select. */
  lineMode?: boolean;
  /** The active sweep dimension (line-view x-axis); stays multi-select. */
  sweepParam?: SweepParam;
  /** Selections to start from instead of the library-derived defaults — the
   *  caller's way of restoring a selection from the URL. Applied once, in the
   *  same pass that would otherwise seed defaults, so a key that is absent or
   *  empty still falls back rather than leaving that dimension unselected. */
  initial?: {
    models?: string[];
    drones?: string[];
    comm_range?: string[];
    n_visits?: string[];
    speed?: string[];
    grid?: string[];
    cell?: string[];
  };
}

// ─── Pure helpers (module scope) ──────────────────────────────────────────────

function distinctModels(library: ScenarioSummary[]): string[] {
  return Array.from(new Set(library.map((s) => s.model_key))).sort((a, b) =>
    a.localeCompare(b)
  );
}

function nVisitsOf(s: ScenarioSummary): number | null {
  return s.variant === "nvisits" ? s.variant_value : null;
}

// Distinct drone counts (sorted numerically) across the given scenarios.
function droneOptions(scenarios: ScenarioSummary[]): { value: string; label: string }[] {
  const vals = Array.from(
    new Set(
      scenarios
        .map((s) => s.number_of_drones)
        .filter((d): d is number => d != null)
    )
  ).sort((a, b) => a - b);
  return vals.map((d) => ({ value: String(d), label: String(d) }));
}

// Distinct comm-range options, ordered smallest→largest by cell value and
// labelled in cell form (e.g. "2 cells · 100 m", "2 diagonal cells · 141 m").
function commOptions(scenarios: ScenarioSummary[]): { value: string; label: string }[] {
  const cellSide =
    scenarios.find((s) => s.cell_side_length != null)?.cell_side_length ?? 50;
  const vals = Array.from(
    new Set(
      scenarios.map((s) => s.comm_range).filter((c): c is string => c != null)
    )
  ).sort((a, b) => commCellValue(a) - commCellValue(b));
  return vals.map((c) => ({ value: c, label: commLabel(c, cellSide) }));
}

// Distinct n_visits values (variant === "nvisits"), sorted numerically.
function nVisitsOptions(scenarios: ScenarioSummary[]): { value: string; label: string }[] {
  const vals = Array.from(
    new Set(scenarios.map(nVisitsOf).filter((v): v is number => v != null))
  ).sort((a, b) => a - b);
  return vals.map((v) => ({ value: String(v), label: String(v) }));
}

// Distinct max-drone-speed values (m/s), sorted numerically.
function speedOptions(scenarios: ScenarioSummary[]): { value: string; label: string }[] {
  const vals = Array.from(
    new Set(scenarios.map((s) => s.max_drone_speed).filter((v): v is number => v != null))
  ).sort((a, b) => a - b);
  return vals.map((v) => ({ value: String(v), label: `${v} m/s` }));
}

// Distinct grid sizes (N → an N×N grid), sorted numerically.
function gridOptions(scenarios: ScenarioSummary[]): { value: string; label: string }[] {
  const vals = Array.from(
    new Set(scenarios.map((s) => s.grid_size).filter((v): v is number => v != null))
  ).sort((a, b) => a - b);
  return vals.map((v) => ({ value: String(v), label: `${v} × ${v}` }));
}

// Distinct cell side lengths (m), sorted numerically.
function cellOptions(scenarios: ScenarioSummary[]): { value: string; label: string }[] {
  const vals = Array.from(
    new Set(scenarios.map((s) => s.cell_side_length).filter((v): v is number => v != null))
  ).sort((a, b) => a - b);
  return vals.map((v) => ({ value: String(v), label: `${v} m` }));
}

// Resolve the scenario set for the active selection. The extra params (speed /
// grid / cell) are lenient: a scenario missing the field isn't filtered out.
function resolveScenarios(
  library: ScenarioSummary[],
  models: Set<string>,
  drones: Set<string>,
  comm: Set<string>,
  nvisits: Set<string>,
  speed: Set<string>,
  grid: Set<string>,
  cell: Set<string>
): string[] {
  return library
    .filter((s) => models.has(s.model_key))
    .filter((s) => s.number_of_drones != null && drones.has(String(s.number_of_drones)))
    .filter((s) => s.comm_range != null && comm.has(s.comm_range))
    .filter((s) => {
      const nv = nVisitsOf(s);
      return nv != null && nvisits.has(String(nv));
    })
    .filter((s) => s.max_drone_speed == null || speed.has(String(s.max_drone_speed)))
    .filter((s) => s.grid_size == null || grid.has(String(s.grid_size)))
    .filter((s) => s.cell_side_length == null || cell.has(String(s.cell_side_length)))
    // Order by parameter combo (drones · comm · n_visits) then model so a
    // combo-aware cap downstream keeps whole stacks (and smallest combos first).
    .sort(
      (a, b) =>
        (a.number_of_drones ?? 0) - (b.number_of_drones ?? 0) ||
        commCellValue(a.comm_range ?? "") - commCellValue(b.comm_range ?? "") ||
        (nVisitsOf(a) ?? 0) - (nVisitsOf(b) ?? 0) ||
        a.model_key.localeCompare(b.model_key)
    )
    .map((s) => s.scenario);
}

// ─── Chip row (module scope, not nested in the component) ─────────────────────

interface ChipRowProps {
  label: string;
  options: { value: string; label: string }[];
  value: string[];
  onChange: (values: string[]) => void;
  /** Single-select (one value); used for speed/grid/cell which are scenario-
   *  defining params, not comparison dimensions. */
  single?: boolean;
}

function ChipRow({ label, options, value, onChange, single }: ChipRowProps) {
  if (options.length === 0) return null;
  const items = options.map((o) => (
    <ToggleGroupItem key={o.value} value={o.value} className="h-7 px-2.5 text-xs">
      {o.label}
    </ToggleGroupItem>
  ));
  return (
    <div className="flex flex-wrap items-center gap-2">
      <span className="w-16 shrink-0 text-xs text-muted-foreground">
        {label}
      </span>
      {single ? (
        <ToggleGroup
          type="single"
          value={value[0] ?? ""}
          onValueChange={(v: string) => {
            if (v) onChange([v]); // ignore deselect of the only value
          }}
          className="flex-wrap justify-start gap-1"
        >
          {items}
        </ToggleGroup>
      ) : (
        <ToggleGroup
          type="multiple"
          value={value}
          onValueChange={(v: string[]) => onChange(v)}
          className="flex-wrap justify-start gap-1"
        >
          {items}
        </ToggleGroup>
      )}
    </div>
  );
}

// ─── Component ────────────────────────────────────────────────────────────────

export default function ModelScenarioPicker({
  library,
  onChange,
  lineMode = false,
  sweepParam = "drones",
  initial,
}: Props) {
  const allModels = useMemo(() => distinctModels(library), [library]);

  // Which parameter rows are single-select right now: in line mode every
  // comparison dimension except the sweep one collapses to a single value.
  const dronesSingle = lineMode && sweepParam !== "drones";
  const commSingle = lineMode && sweepParam !== "comm_range";
  const nVisitsSingle = lineMode && sweepParam !== "n_visits";

  // Selection state (string keys throughout).
  const [selModels, setSelModels] = useState<string[]>([]);
  const [selDrones, setSelDrones] = useState<string[]>([]);
  const [selComm, setSelComm] = useState<string[]>([]);
  const [selNVisits, setSelNVisits] = useState<string[]>([]);
  const [selSpeed, setSelSpeed] = useState<string[]>([]);
  const [selGrid, setSelGrid] = useState<string[]>([]);
  const [selCell, setSelCell] = useState<string[]>([]);
  const [seeded, setSeeded] = useState(false);

  // Param chip options are derived from the FULL library (union across every
  // model), not just the selected models, so every value (e.g. Comm 4, which
  // only some models have data for) is always offerable. A combination a given
  // model lacks simply resolves to no scenario for it.
  const droneOpts = useMemo(() => droneOptions(library), [library]);
  const commOpts = useMemo(() => commOptions(library), [library]);
  const nVisitsOpts = useMemo(() => nVisitsOptions(library), [library]);
  const speedOpts = useMemo(() => speedOptions(library), [library]);
  const gridOpts = useMemo(() => gridOptions(library), [library]);
  const cellOpts = useMemo(() => cellOptions(library), [library]);

  // Seed a sensible non-empty default once the library is available: first two
  // models, smallest drones, first comm, n_visits=2 if present else first.
  useEffect(() => {
    if (seeded || allModels.length === 0) return;
    const defaultModels = allModels.slice(0, Math.min(2, allModels.length));
    const seedScenarios = library.filter((s) =>
      new Set(defaultModels).has(s.model_key)
    );
    const dOpts = droneOptions(seedScenarios);
    const cOpts = commOptions(seedScenarios);
    const nOpts = nVisitsOptions(seedScenarios);
    const nDefault =
      nOpts.find((o) => o.value === "2")?.value ?? nOpts[0]?.value;

    // Speed / grid / cell are scenario-defining params (single-select); default
    // to the first present value each.
    const spOpts = speedOptions(library);
    const grOpts = gridOptions(library);
    const ceOpts = cellOptions(library);

    // A restored selection wins over the derived default, but only for values
    // this library actually offers — a stale or hand-edited URL must not be
    // able to select a scenario that does not exist.
    const restore = (
      wanted: string[] | undefined,
      options: { value: string }[],
      fallback: string[]
    ): string[] => {
      if (!wanted || wanted.length === 0) return fallback;
      const offered = new Set(options.map((o) => o.value));
      const kept = wanted.filter((v) => offered.has(v));
      return kept.length > 0 ? kept : fallback;
    };

    setSelModels(
      restore(
        initial?.models,
        allModels.map((m) => ({ value: m })),
        defaultModels
      )
    );
    setSelDrones(restore(initial?.drones, dOpts, dOpts[0] ? [dOpts[0].value] : []));
    setSelComm(restore(initial?.comm_range, cOpts, cOpts[0] ? [cOpts[0].value] : []));
    setSelNVisits(restore(initial?.n_visits, nOpts, nDefault ? [nDefault] : []));
    setSelSpeed(restore(initial?.speed, spOpts, spOpts[0] ? [spOpts[0].value] : []));
    setSelGrid(restore(initial?.grid, grOpts, grOpts[0] ? [grOpts[0].value] : []));
    setSelCell(restore(initial?.cell, ceOpts, ceOpts[0] ? [ceOpts[0].value] : []));
    setSeeded(true);
    // `initial` is read once, in the seeding pass; it is deliberately not a
    // dependency, or a caller passing a fresh object each render would reseed
    // over the reader's own subsequent choices.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [seeded, allModels, library]);

  // Prune param selections that no longer exist after the model set changes,
  // keeping at least one value where options exist.
  useEffect(() => {
    if (!seeded) return;
    const dSet = new Set(droneOpts.map((o) => o.value));
    const cSet = new Set(commOpts.map((o) => o.value));
    const nSet = new Set(nVisitsOpts.map((o) => o.value));

    setSelDrones((prev) => {
      const kept = prev.filter((v) => dSet.has(v));
      if (kept.length > 0) return kept.length === prev.length ? prev : kept;
      return droneOpts[0] ? [droneOpts[0].value] : [];
    });
    setSelComm((prev) => {
      const kept = prev.filter((v) => cSet.has(v));
      if (kept.length > 0) return kept.length === prev.length ? prev : kept;
      return commOpts[0] ? [commOpts[0].value] : [];
    });
    setSelNVisits((prev) => {
      const kept = prev.filter((v) => nSet.has(v));
      if (kept.length > 0) return kept.length === prev.length ? prev : kept;
      return nVisitsOpts[0] ? [nVisitsOpts[0].value] : [];
    });

    const spSet = new Set(speedOpts.map((o) => o.value));
    const grSet = new Set(gridOpts.map((o) => o.value));
    const ceSet = new Set(cellOpts.map((o) => o.value));
    // Single-select filters: keep the one valid value, else fall back to first.
    const keepOne = (
      prev: string[],
      set: Set<string>,
      opts: { value: string }[]
    ): string[] => {
      if (prev[0] && set.has(prev[0])) return prev.length === 1 ? prev : [prev[0]];
      return opts[0] ? [opts[0].value] : [];
    };
    setSelSpeed((prev) => keepOne(prev, spSet, speedOpts));
    setSelGrid((prev) => keepOne(prev, grSet, gridOpts));
    setSelCell((prev) => keepOne(prev, ceSet, cellOpts));
  }, [seeded, droneOpts, commOpts, nVisitsOpts, speedOpts, gridOpts, cellOpts]);

  // When a comparison dimension becomes single-select (entering line mode or
  // switching the sweep dimension), drop any extra values it carried over from
  // bar mode so the single-select row shows a consistent lone value.
  useEffect(() => {
    if (dronesSingle) setSelDrones((p) => (p.length > 1 ? [p[0]!] : p));
    if (commSingle) setSelComm((p) => (p.length > 1 ? [p[0]!] : p));
    if (nVisitsSingle) setSelNVisits((p) => (p.length > 1 ? [p[0]!] : p));
  }, [dronesSingle, commSingle, nVisitsSingle]);

  // Whenever the sweep dimension (re)activates — entering line mode OR switching
  // the sweep parameter — default it to ALL available values, so the line plot's
  // x-axis always sweeps the full range (matching the model page). The ref resets
  // on leaving line mode, so re-entering re-selects all.
  const prevSweepRef = useRef<SweepParam | null>(null);
  useEffect(() => {
    if (!lineMode) {
      prevSweepRef.current = null;
      return;
    }
    if (prevSweepRef.current === sweepParam) return;
    prevSweepRef.current = sweepParam;
    if (sweepParam === "drones") setSelDrones(droneOpts.map((o) => o.value));
    else if (sweepParam === "comm_range") setSelComm(commOpts.map((o) => o.value));
    else setSelNVisits(nVisitsOpts.map((o) => o.value));
  }, [lineMode, sweepParam, droneOpts, commOpts, nVisitsOpts]);

  // Resolve + emit on any change.
  const resolved = useMemo(
    () =>
      resolveScenarios(
        library,
        new Set(selModels),
        new Set(selDrones),
        new Set(selComm),
        new Set(selNVisits),
        new Set(selSpeed),
        new Set(selGrid),
        new Set(selCell)
      ),
    [library, selModels, selDrones, selComm, selNVisits, selSpeed, selGrid, selCell]
  );

  useEffect(() => {
    onChange({
      scenarios: resolved,
      models: selModels,
      params: {
        drones: selDrones,
        comm_range: selComm,
        n_visits: selNVisits,
        speed: selSpeed,
        grid: selGrid,
        cell: selCell,
      },
    });
  }, [
    onChange, resolved, selModels, selDrones, selComm, selNVisits,
    selSpeed, selGrid, selCell,
  ]);

  // Toggle guards: never allow an empty model set; keep ≥1 value per dimension.
  function onToggleModels(values: string[]) {
    if (values.length === 0) return;
    setSelModels(values);
  }
  function guardDim(setter: (v: string[]) => void) {
    return (values: string[]) => {
      if (values.length === 0) return;
      setter(values);
    };
  }

  return (
    <Card>
      <CardHeader>
        <CardTitle>Select models &amp; parameters</CardTitle>
      </CardHeader>
      <CardContent className="flex flex-col gap-4">
        <ChipRow
          label="Models"
          options={allModels.map((m) => ({ value: m, label: m }))}
          value={selModels}
          onChange={onToggleModels}
        />
        <ChipRow
          label="Drones"
          options={droneOpts}
          value={selDrones}
          onChange={guardDim(setSelDrones)}
          single={dronesSingle}
        />
        <ChipRow
          label="Comm"
          options={commOpts}
          value={selComm}
          onChange={guardDim(setSelComm)}
          single={commSingle}
        />
        <ChipRow
          label="n_visits"
          options={nVisitsOpts}
          value={selNVisits}
          onChange={guardDim(setSelNVisits)}
          single={nVisitsSingle}
        />
        <ChipRow
          single
          label="Speed"
          options={speedOpts}
          value={selSpeed}
          onChange={setSelSpeed}
        />
        <ChipRow
          single
          label="Grid"
          options={gridOpts}
          value={selGrid}
          onChange={setSelGrid}
        />
        <ChipRow
          single
          label="Cell"
          options={cellOpts}
          value={selCell}
          onChange={setSelCell}
        />

        {resolved.length === 0 ? (
          <p className="rounded-lg border border-amber-500/40 bg-amber-500/10 px-4 py-2.5 text-xs text-amber-700 dark:text-amber-400">
            ⚠ No saved mission matches the selected parameters. This combination
            may not have been run yet — adjust the models or parameter values.
          </p>
        ) : (
          <p className="text-xs text-muted-foreground">
            <span className="text-foreground tabular-nums">{resolved.length}</span>{" "}
            scenario{resolved.length !== 1 ? "s" : ""} resolved across{" "}
            <span className="text-foreground tabular-nums">{selModels.length}</span>{" "}
            model{selModels.length !== 1 ? "s" : ""}.
          </p>
        )}
      </CardContent>
    </Card>
  );
}
