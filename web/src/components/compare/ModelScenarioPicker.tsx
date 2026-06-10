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

import { useEffect, useMemo, useState } from "react";
import type { ScenarioSummary } from "@/lib/types";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { ToggleGroup, ToggleGroupItem } from "@/components/ui/toggle-group";
import { cn } from "@/lib/utils";

// ─── Emitted selection shape ──────────────────────────────────────────────────

export interface PickerSelection {
  /** resolved library scenario names matching every active dimension */
  scenarios: string[];
  /** the selected model_keys (line view needs ≥2) */
  models: string[];
  /** raw per-dimension selections (string keys), for the line-view sweep axis */
  sweepable: {
    drones: string[];
    comm_range: string[];
    n_visits: string[];
  };
}

interface Props {
  library: ScenarioSummary[];
  onChange: (sel: PickerSelection) => void;
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

// Distinct comm-range labels, ordered by their numeric cell value when known
// (falls back to lexical). Comm range is a string key (e.g. "sqrt(8)").
function commOptions(scenarios: ScenarioSummary[]): { value: string; label: string }[] {
  const vals = Array.from(
    new Set(
      scenarios.map((s) => s.comm_range).filter((c): c is string => c != null)
    )
  ).sort((a, b) => a.localeCompare(b));
  return vals.map((c) => ({ value: c, label: c }));
}

// Distinct n_visits values (variant === "nvisits"), sorted numerically.
function nVisitsOptions(scenarios: ScenarioSummary[]): { value: string; label: string }[] {
  const vals = Array.from(
    new Set(scenarios.map(nVisitsOf).filter((v): v is number => v != null))
  ).sort((a, b) => a - b);
  return vals.map((v) => ({ value: String(v), label: String(v) }));
}

// Resolve the scenario set for the active selection.
function resolveScenarios(
  library: ScenarioSummary[],
  models: Set<string>,
  drones: Set<string>,
  comm: Set<string>,
  nvisits: Set<string>
): string[] {
  return library
    .filter((s) => models.has(s.model_key))
    .filter((s) => s.number_of_drones != null && drones.has(String(s.number_of_drones)))
    .filter((s) => s.comm_range != null && comm.has(s.comm_range))
    .filter((s) => {
      const nv = nVisitsOf(s);
      return nv != null && nvisits.has(String(nv));
    })
    .map((s) => s.scenario);
}

// ─── Chip row (module scope, not nested in the component) ─────────────────────

interface ChipRowProps {
  label: string;
  options: { value: string; label: string }[];
  value: string[];
  onChange: (values: string[]) => void;
}

function ChipRow({ label, options, value, onChange }: ChipRowProps) {
  if (options.length === 0) return null;
  return (
    <div className="flex flex-wrap items-center gap-2">
      <span className="w-16 shrink-0 text-xs font-mono tracking-widest text-muted-foreground uppercase">
        {label}
      </span>
      <ToggleGroup
        type="multiple"
        value={value}
        onValueChange={onChange}
        className="flex-wrap justify-start gap-1"
      >
        {options.map((o) => (
          <ToggleGroupItem
            key={o.value}
            value={o.value}
            className="h-7 px-2.5 text-xs font-mono"
          >
            {o.label}
          </ToggleGroupItem>
        ))}
      </ToggleGroup>
    </div>
  );
}

// ─── Component ────────────────────────────────────────────────────────────────

export default function ModelScenarioPicker({ library, onChange }: Props) {
  const allModels = useMemo(() => distinctModels(library), [library]);

  // Selection state (string keys throughout).
  const [selModels, setSelModels] = useState<string[]>([]);
  const [selDrones, setSelDrones] = useState<string[]>([]);
  const [selComm, setSelComm] = useState<string[]>([]);
  const [selNVisits, setSelNVisits] = useState<string[]>([]);
  const [seeded, setSeeded] = useState(false);

  // Scenarios belonging to the currently-selected models (drives the param
  // chip options as the UNION across those models).
  const modelScenarios = useMemo(() => {
    const set = new Set(selModels);
    return library.filter((s) => set.has(s.model_key));
  }, [library, selModels]);

  const droneOpts = useMemo(() => droneOptions(modelScenarios), [modelScenarios]);
  const commOpts = useMemo(() => commOptions(modelScenarios), [modelScenarios]);
  const nVisitsOpts = useMemo(() => nVisitsOptions(modelScenarios), [modelScenarios]);

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

    setSelModels(defaultModels);
    setSelDrones(dOpts[0] ? [dOpts[0].value] : []);
    setSelComm(cOpts[0] ? [cOpts[0].value] : []);
    setSelNVisits(nDefault ? [nDefault] : []);
    setSeeded(true);
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
  }, [seeded, droneOpts, commOpts, nVisitsOpts]);

  // Resolve + emit on any change.
  const resolved = useMemo(
    () =>
      resolveScenarios(
        library,
        new Set(selModels),
        new Set(selDrones),
        new Set(selComm),
        new Set(selNVisits)
      ),
    [library, selModels, selDrones, selComm, selNVisits]
  );

  useEffect(() => {
    onChange({
      scenarios: resolved,
      models: selModels,
      sweepable: {
        drones: selDrones,
        comm_range: selComm,
        n_visits: selNVisits,
      },
    });
  }, [onChange, resolved, selModels, selDrones, selComm, selNVisits]);

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
        <CardTitle
          className="text-xs font-semibold tracking-widest uppercase text-primary font-display"
          style={{ fontFamily: "var(--font-display)" }}
        >
          SELECT MODELS &amp; PARAMETERS
        </CardTitle>
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
        />
        <ChipRow
          label="Comm"
          options={commOpts}
          value={selComm}
          onChange={guardDim(setSelComm)}
        />
        <ChipRow
          label="n_visits"
          options={nVisitsOpts}
          value={selNVisits}
          onChange={guardDim(setSelNVisits)}
        />

        {resolved.length === 0 ? (
          <p className="text-xs font-mono text-muted-foreground border border-dashed border-border rounded px-4 py-3">
            No scenarios match this selection. Try different models or parameter
            values.
          </p>
        ) : (
          <p className="text-xs font-mono text-muted-foreground">
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
