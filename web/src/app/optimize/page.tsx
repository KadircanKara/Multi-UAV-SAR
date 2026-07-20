"use client";

/**
 * /optimize — Optimizer page.
 *
 * Configure and run a bespoke optimisation: choose SOO/MOO, a method, the
 * objectives (with weighted-sum weights where applicable), GA parameters, and a
 * scenario. Live-polls progress and renders the resulting Pareto front (MOO) or
 * the single best solution (SOO / weighted-sum).
 */

import { useEffect, useMemo, useRef, useState } from "react";
import Link from "next/link";
import dynamic from "next/dynamic";
import { toast } from "sonner";

import {
  checkOptimize,
  startOptimize,
  getOptimizeStatus,
  stopOptimize,
  optimizeExportUrl,
  getDefaultScenario,
} from "@/lib/api";
import type {
  OptimizeConfig,
  OptimizeFront,
  OptimizeStatus,
  ParetoFront,
  ScenarioConfig,
} from "@/lib/types";
import { cn } from "@/lib/utils";
import type { ProgressPoint } from "@/components/optimize/LiveProgress";

import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Slider } from "@/components/ui/slider";
import { Progress } from "@/components/ui/progress";
import { Separator } from "@/components/ui/separator";
import {
  Card,
  CardContent,
  CardHeader,
  CardTitle,
  CardDescription,
} from "@/components/ui/card";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import { ToggleGroup, ToggleGroupItem } from "@/components/ui/toggle-group";
import {
  Tooltip,
  TooltipContent,
  TooltipProvider,
  TooltipTrigger,
} from "@/components/ui/tooltip";

// ParetoScatter pulls in Recharts; load client-side only (mirrors explore page).
const ParetoScatter = dynamic(
  () => import("@/components/viz/ParetoScatter"),
  { ssr: false }
);

// LiveProgress (Recharts) — only mounted while a run is in flight.
const LiveProgress = dynamic(
  () => import("@/components/optimize/LiveProgress").then((m) => m.LiveProgress),
  { ssr: false }
);

// ─── Static config ─────────────────────────────────────────────────────────────

interface ObjectiveSpec {
  name: string;
  /** +1 ⇒ lower is better; -1 ⇒ higher is better. */
  polarity: 1 | -1;
  unit: string;
  tbv?: boolean;
}

const OBJECTIVES: ObjectiveSpec[] = [
  { name: "Mission Time", polarity: 1, unit: "s" },
  { name: "Percentage Connectivity", polarity: -1, unit: "%" },
  { name: "Max Disconnected Time", polarity: 1, unit: "steps" },
  { name: "Mean Disconnected Time", polarity: 1, unit: "steps" },
  { name: "Max Mean TBV", polarity: 1, unit: "s", tbv: true },
];

const SOO_METHODS = ["GA", "WS"] as const;
const MOO_METHODS = ["NSGA2", "NSGA3", "MOEAD"] as const;
const DISABLED_METHODS = new Set<string>(["MOEAD"]);

// Selected SOO/MOO toggle uses the same indigo treatment as the Method /
// Objectives chips (the default outline `bg-accent` slate was too faint).
const OPT_TYPE_ITEM_CLS =
  "px-5 data-[state=on]:border-chart-1/40 data-[state=on]:bg-chart-1/10 " +
  "data-[state=on]:text-chart-1 data-[state=on]:hover:bg-chart-1/10 " +
  "data-[state=on]:hover:text-chart-1";

const POP_MIN = 10;
const POP_MAX = 500;
const POP_DEFAULT = 100;
const NGEN_MIN = 5;
const NGEN_MAX = 1000;
const NGEN_DEFAULT = 300;
const SEED_DEFAULT = 1;

type GenStrategy = "fixed" | "max";
// Early-stop tuning (Max Generations) — defaults match the backend.
const ES_PATIENCE_DEFAULT = 10; // generations without meaningful improvement → stop
const ES_THRESH_PCT_DEFAULT = 10; // % improvement that counts as progress

// Constraint defaults (the speed-violation constraint is always applied).
const MMT_DEFAULT = 3600; // max mission time (seconds)
const MIN_CONN_DEFAULT = 0.5; // min percentage connectivity (fraction)

function Toggle({
  on,
  onChange,
  disabled,
}: {
  on: boolean;
  onChange: (v: boolean) => void;
  disabled?: boolean;
}) {
  return (
    <button
      type="button"
      role="switch"
      aria-checked={on}
      disabled={disabled}
      onClick={() => !disabled && onChange(!on)}
      className={cn(
        "inline-flex h-6 w-11 shrink-0 items-center rounded-full transition-colors",
        on ? "bg-foreground" : "bg-muted",
        disabled && "cursor-not-allowed opacity-40"
      )}
    >
      <span
        className={cn(
          "size-5 rounded-full bg-background shadow transition-transform",
          on ? "translate-x-[22px]" : "translate-x-0.5"
        )}
      />
    </button>
  );
}

const DRONE_OPTIONS = [4, 8, 12, 16];
const NVISIT_OPTIONS = [1, 2, 3];

// Comm range — connectivity/disconnectivity use an actual METRE test
// (connected iff D[a,b] ≤ comm_cell_range × cell_side_length), so the range is a
// distance. `cells` is the EXACT comm_cell_range stored: the √8 option must be
// 2√2 precisely so PathInfo renders the seeded `r_sqrt(8)` scenario name
// (`comm_cell_range == 2*sqrt(2)` is an exact float check). The metre equivalent
// shown to the user is `cells × cell_side_length`, recomputed live.
const COMM_PRESETS = [
  { id: "2", cells: 2, label: "2 cells" },
  { id: "sqrt8", cells: 2 * Math.SQRT2, label: "√8 · 2 diagonal" },
  { id: "4", cells: 4, label: "4 cells" },
] as const;
const COMM_CUSTOM = "custom";
const COMM_EPS = 1e-9;

// Persist the in-flight run id so a reload / navigation can re-attach to it
// (the backend recovers a finished run from disk; a lost run is cleared).
const RUN_STORAGE_KEY = "optimize.runId";
const RUN_PARAMS_STORAGE_KEY = "optimize.runParams";

/** Pretty-print a comm_cell_range in cell-length units (symbolic where it matches). */
function formatCommCells(v: number): string {
  if (Math.abs(v - Math.SQRT2) < COMM_EPS) return "√2";
  if (Math.abs(v - 2 * Math.SQRT2) < COMM_EPS) return "√8";
  return Number.isInteger(v) ? String(v) : v.toFixed(2);
}

/** One-line human summary of the parameters that define a scenario's identity
 *  (existence is matched on the full combo, not just the model). */
function scenarioSummary(s: ScenarioConfig): string {
  const metres = Math.round(s.comm_cell_range * s.cell_side_length);
  return `${s.number_of_drones} drones · comm ${formatCommCells(
    s.comm_cell_range
  )} (${metres} m) · n_visits ${s.n_visits}`;
}

type OptType = "SOO" | "MOO";

/** A snapshot of the parameters a run was launched with — shown under the live
 *  progress bar so the user doesn't lose track of what's running (the config
 *  panel is hidden while solving). */
interface RunParams {
  optType: OptType;
  method: string;
  objectives: string[];
  weights: Record<string, number> | null;
  popSize: number;
  nGen: number;
  genStrategy: GenStrategy;
  earlyStopPatience: number;
  earlyStopThresholdPct: number;
  scenario: ScenarioConfig;
  maxMissionTime: number | null;
  minConnectivity: number | null;
}

function constraintsSummary(p: RunParams): string {
  const extra: string[] = [];
  if (p.maxMissionTime != null) extra.push(`max time ${p.maxMissionTime}s`);
  if (p.minConnectivity != null) extra.push(`min conn ${p.minConnectivity}`);
  return extra.length ? `speed · ${extra.join(" · ")}` : "speed only";
}

function ParamRow({ label, value }: { label: string; value: string }) {
  return (
    <div className="flex items-start justify-between gap-3">
      <span className="shrink-0 text-muted-foreground">{label}</span>
      <span className="text-right text-foreground">{value}</span>
    </div>
  );
}

// ─── Pure helpers (module scope) ───────────────────────────────────────────────

function defaultMethodFor(type: OptType): string {
  return type === "SOO" ? "GA" : "NSGA2";
}

function methodsFor(type: OptType): readonly string[] {
  return type === "SOO" ? SOO_METHODS : MOO_METHODS;
}

function isSingleSelect(type: OptType, method: string): boolean {
  return type === "SOO" && method === "GA";
}

function needsWeights(type: OptType, method: string): boolean {
  return type === "SOO" && method === "WS";
}

/** Trim/normalise a selection so it satisfies the rule for {type, method}. */
function fitSelection(
  type: OptType,
  method: string,
  selected: string[]
): string[] {
  const ordered = OBJECTIVES.map((o) => o.name).filter((n) =>
    selected.includes(n)
  );
  if (isSingleSelect(type, method)) {
    return ordered.length > 0 ? [ordered[0]!] : [OBJECTIVES[0]!.name];
  }
  // multi-select rules (WS / NSGA2 / NSGA3): need ≥2
  if (ordered.length >= 2) return ordered;
  // grow to ≥2 by adding objectives in canonical order (no duplicates)
  const grown = new Set(ordered);
  for (const o of OBJECTIVES) {
    if (grown.size >= 2) break;
    grown.add(o.name);
  }
  return OBJECTIVES.map((o) => o.name).filter((n) => grown.has(n));
}

/** Equal-split weights that sum to EXACTLY 1 (the rounding remainder — e.g. the
 *  0.0001 left by 0.3333×3 — is absorbed into the first objective). */
function equalWeights(objs: string[]): Record<string, number> {
  if (objs.length === 0) return {};
  const w = Number((1 / objs.length).toFixed(4));
  const out: Record<string, number> = {};
  for (const o of objs) out[o] = w;
  out[objs[0]] = Number((w + (1 - w * objs.length)).toFixed(4));
  return out;
}

function sumWeights(weights: Record<string, number>, objs: string[]): number {
  return objs.reduce((acc, o) => acc + (Number(weights[o]) || 0), 0);
}

/** Format an objective's absolute value for display. Percentage Connectivity is
 *  a 0–1 fraction in the data — render it as a percentage so the result views
 *  agree with the live-progress view (which already scales ×100). */
function fmtObjValue(obj: string, n: number | null | undefined): string {
  if (n == null || Number.isNaN(n)) return "—";
  if (obj === "Percentage Connectivity") return `${(n * 100).toFixed(1)}%`;
  return n.toFixed(2);
}

/** The unit suffix to show after a formatted value (none for connectivity — the
 *  "%" is already baked into the formatted percentage). */
function unitFor(obj: string): string | null {
  if (obj === "Percentage Connectivity") return null;
  return OBJECTIVES.find((s) => s.name === obj)?.unit ?? null;
}

// ─── Small presentational pieces (module scope) ────────────────────────────────

function SectionLabel({ children }: { children: React.ReactNode }) {
  return (
    <p className="text-sm font-medium text-foreground">{children}</p>
  );
}

function ObjectiveChip({
  spec,
  selected,
  disabled,
  onToggle,
}: {
  spec: ObjectiveSpec;
  selected: boolean;
  disabled: boolean;
  onToggle: () => void;
}) {
  return (
    <button
      type="button"
      disabled={disabled}
      aria-pressed={selected}
      onClick={onToggle}
      className={cn(
        "rounded-full border px-3 py-1 text-sm font-medium transition-colors",
        selected
          ? "border-chart-1/40 bg-chart-1/10 text-chart-1"
          : "border-border bg-card text-muted-foreground hover:border-foreground/20 hover:text-foreground",
        disabled && "cursor-not-allowed opacity-40 hover:border-border hover:text-muted-foreground"
      )}
    >
      {spec.name}
      <span className="ml-1.5 text-xs text-muted-foreground">
        {spec.polarity === -1 ? "↑" : "↓"}
      </span>
    </button>
  );
}

function InlineWarning({ children }: { children: React.ReactNode }) {
  return (
    <p className="rounded-lg border border-amber-500/30 bg-amber-500/5 px-3 py-2 text-xs text-amber-600 dark:text-amber-400">
      {children}
    </p>
  );
}

function ResultTable({
  front,
}: {
  front: OptimizeFront;
}) {
  return (
    <div className="overflow-x-auto rounded-xl border border-border">
      <table className="w-full text-sm">
        <thead>
          <tr className="border-b border-border bg-muted/40">
            <th className="px-3 py-2 text-left font-medium text-muted-foreground">
              #
            </th>
            {front.objectives.map((o) => (
              <th
                key={o}
                className="px-3 py-2 text-right font-medium text-muted-foreground"
              >
                {o}
              </th>
            ))}
          </tr>
        </thead>
        <tbody>
          {front.solutions.map((sol) => (
            <tr
              key={sol.index}
              className="border-b border-border last:border-0"
            >
              <td className="px-3 py-2 text-left tabular-nums text-muted-foreground">
                {sol.index}
              </td>
              {front.objectives.map((o) => (
                <td
                  key={o}
                  className="px-3 py-2 text-right tabular-nums text-foreground"
                >
                  {fmtObjValue(o, sol.objectives_abs[o])}
                </td>
              ))}
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}

function SingleResult({ front }: { front: OptimizeFront }) {
  const sol = front.solutions[0];
  if (!sol) {
    return (
      <p className="text-sm text-muted-foreground">No solution returned.</p>
    );
  }
  return (
    <div className="grid grid-cols-1 gap-3 sm:grid-cols-2 lg:grid-cols-3">
      {front.objectives.map((o) => {
        const unit = unitFor(o);
        return (
          <div
            key={o}
            className="flex flex-col gap-1 rounded-xl border border-border bg-card p-4"
          >
            <p className="text-xs text-muted-foreground">{o}</p>
            <p className="text-2xl font-semibold tabular-nums text-foreground">
              {fmtObjValue(o, sol.objectives_abs[o])}
              {unit && (
                <span className="ml-1 text-sm font-normal text-muted-foreground">
                  {unit}
                </span>
              )}
            </p>
          </div>
        );
      })}
    </div>
  );
}

// ─── Page ──────────────────────────────────────────────────────────────────────

export default function OptimizePage() {
  // Optimisation type / method
  const [optType, setOptType] = useState<OptType>("MOO");
  const [method, setMethod] = useState<string>("NSGA2");

  // Objectives + weights
  const [selected, setSelected] = useState<string[]>([
    "Mission Time",
    "Percentage Connectivity",
  ]);
  const [weights, setWeights] = useState<Record<string, number>>({});

  // GA parameters
  const [popSize, setPopSize] = useState(POP_DEFAULT);
  const [nGen, setNGen] = useState(NGEN_DEFAULT);
  const [genStrategy, setGenStrategy] = useState<GenStrategy>("fixed");
  const [earlyStopPatience, setEarlyStopPatience] = useState(ES_PATIENCE_DEFAULT);
  const [earlyStopThresholdPct, setEarlyStopThresholdPct] =
    useState(ES_THRESH_PCT_DEFAULT);

  // Constraints (speed-violation is always applied; these two are configurable)
  const [mmtEnabled, setMmtEnabled] = useState(false);
  const [mmtValue, setMmtValue] = useState(MMT_DEFAULT);
  const [minConnEnabled, setMinConnEnabled] = useState(false);
  const [minConnValue, setMinConnValue] = useState(MIN_CONN_DEFAULT);
  const [seed, setSeed] = useState(SEED_DEFAULT);

  // Scenario
  const [scenario, setScenario] = useState<ScenarioConfig | null>(null);
  // Comm range: when true the user is entering a raw metre distance (not a preset).
  const [commCustom, setCommCustom] = useState(false);
  const [commMetresRaw, setCommMetresRaw] = useState("");

  // Duplicate pre-check
  const [duplicate, setDuplicate] = useState<{
    scenario_name: string;
    seeded: boolean;
  } | null>(null);

  // Run / poll state
  const [running, setRunning] = useState(false);
  const [progress, setProgress] = useState<{ gen: number; nGen: number } | null>(
    null
  );
  const [result, setResult] = useState<OptimizeFront | null>(null);
  const [selectedSolution, setSelectedSolution] = useState(0);
  const [stopping, setStopping] = useState(false);
  // Live progress (while running): sampled best-per-objective history + front.
  const [progressHistory, setProgressHistory] = useState<ProgressPoint[]>([]);
  const [liveFront, setLiveFront] = useState<Record<string, number>[] | null>(
    null
  );
  // Snapshot of the params the current run was launched with (shown while solving).
  const [runParams, setRunParams] = useState<RunParams | null>(null);

  // Run identity (used for the post-run Download JSON action — runs are not
  // persisted server-side; this is memoryless by design).
  const [runId, setRunId] = useState<string | null>(null);

  const pollRef = useRef<ReturnType<typeof setInterval> | null>(null);
  const mountedRef = useRef(true);

  // Load default scenario once.
  useEffect(() => {
    let cancelled = false;
    getDefaultScenario()
      .then((sc) => {
        if (cancelled) return;
        setScenario(sc);
      })
      .catch((err: unknown) => {
        if (!cancelled) {
          toast.error("Failed to load default scenario", {
            description: err instanceof Error ? err.message : String(err),
          });
        }
      });
    return () => {
      cancelled = true;
    };
  }, []);

  // Track mount state + stop polling on unmount. Set true on every mount: React
  // 18 Strict Mode remounts in dev, and useRef does NOT re-init on remount, so
  // without this mountedRef would stay false and block handleRun's beginPoll.
  useEffect(() => {
    mountedRef.current = true;
    return () => {
      mountedRef.current = false;
      if (pollRef.current) clearInterval(pollRef.current);
    };
  }, []);

  // Re-attach to an in-flight run after a reload / navigation back. The backend
  // recovers a finished run from disk; a run that's truly gone is cleared by the
  // poll loop's 404 handling (so this never wedges into a stuck spinner).
  useEffect(() => {
    let stored: string | null = null;
    try {
      stored = sessionStorage.getItem(RUN_STORAGE_KEY);
    } catch {
      stored = null;
    }
    if (!stored) return;
    try {
      const rp = sessionStorage.getItem(RUN_PARAMS_STORAGE_KEY);
      if (rp) setRunParams(JSON.parse(rp) as RunParams);
    } catch {
      /* ignore — params are a nicety, not required to re-attach */
    }
    setRunId(stored);
    setRunning(true);
    setProgress({ gen: 0, nGen: nGen });
    beginPoll(stored);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // ── Type change: reset method + fit the selection. ──
  function handleTypeChange(next: OptType) {
    if (!next || next === optType) return;
    const nextMethod = defaultMethodFor(next);
    setOptType(next);
    setMethod(nextMethod);
    setSelected((prev) => {
      const fitted = fitSelection(next, nextMethod, prev);
      if (needsWeights(next, nextMethod)) setWeights(equalWeights(fitted));
      return fitted;
    });
  }

  // ── Method change: fit the selection / reset weights. ──
  function handleMethodChange(next: string) {
    if (DISABLED_METHODS.has(next) || next === method) return;
    setMethod(next);
    setSelected((prev) => {
      const fitted = fitSelection(optType, next, prev);
      if (needsWeights(optType, next)) setWeights(equalWeights(fitted));
      return fitted;
    });
  }

  // ── Objective toggle (respects single vs multi select). ──
  function toggleObjective(name: string) {
    const single = isSingleSelect(optType, method);
    if (single) {
      setSelected([name]);
      return;
    }
    setSelected((prev) => {
      const has = prev.includes(name);
      const next = has
        ? prev.filter((n) => n !== name)
        : [...prev, name];
      // keep canonical order
      const ordered = OBJECTIVES.map((o) => o.name).filter((n) =>
        next.includes(n)
      );
      if (needsWeights(optType, method)) {
        setWeights(equalWeights(ordered));
      }
      return ordered;
    });
  }

  function setWeightFor(name: string, value: number) {
    setWeights((prev) => ({ ...prev, [name]: value }));
  }

  // ── Derived validity ──
  const weightSum = useMemo(
    () => sumWeights(weights, selected),
    [weights, selected]
  );
  const weightsOk =
    !needsWeights(optType, method) || Math.abs(weightSum - 1) <= 0.001;

  const selectionOk = isSingleSelect(optType, method)
    ? selected.length === 1
    : selected.length >= 2;

  const tbvSelected = selected.some(
    (n) => OBJECTIVES.find((o) => o.name === n)?.tbv
  );
  const tbvWarning =
    tbvSelected && scenario != null && scenario.n_visits < 2;

  // A cleared number input coerces to 0/NaN; gate the run so we never POST an
  // invalid constraint (max_mission_time must be > 0; min_connectivity ∈ [0,1]).
  const mmtOk = !mmtEnabled || (Number.isFinite(mmtValue) && mmtValue > 0);
  const minConnOk =
    !minConnEnabled ||
    (Number.isFinite(minConnValue) && minConnValue >= 0 && minConnValue <= 1);

  const earlyStopOk =
    genStrategy !== "max" ||
    (Number.isFinite(earlyStopPatience) &&
      earlyStopPatience >= 2 &&
      Number.isFinite(earlyStopThresholdPct) &&
      earlyStopThresholdPct > 0 &&
      earlyStopThresholdPct <= 100);

  const isValid =
    scenario != null &&
    selectionOk &&
    weightsOk &&
    mmtOk &&
    minConnOk &&
    earlyStopOk &&
    !running;

  // ── Build the config object from state. ──
  const config: OptimizeConfig | null = useMemo(() => {
    if (!scenario) return null;
    return {
      optimization_type: optType,
      method,
      objectives: selected,
      weights: needsWeights(optType, method) ? weights : null,
      pop_size: popSize,
      n_gen: nGen,
      seed,
      max_mission_time: mmtEnabled ? mmtValue : null,
      min_connectivity: minConnEnabled ? minConnValue : null,
      gen_strategy: genStrategy,
      // Only send the tuning when Max mode is active; otherwise the backend
      // applies its defaults (and a cleared field can't 422 a Fixed run).
      ...(genStrategy === "max"
        ? {
            early_stop_patience: earlyStopPatience,
            early_stop_threshold: earlyStopThresholdPct / 100,
          }
        : {}),
      // Target cells are irrelevant to optimization (they only affect the
      // sensing time-metrics, computed post-hoc). Send a valid placeholder so
      // PathInfo / ScenarioConfig validation passes for any grid size.
      scenario: { ...scenario, target_positions: [0] },
    };
  }, [
    scenario,
    optType,
    method,
    selected,
    weights,
    popSize,
    nGen,
    seed,
    mmtEnabled,
    mmtValue,
    minConnEnabled,
    minConnValue,
    genStrategy,
    earlyStopPatience,
    earlyStopThresholdPct,
  ]);

  // Existence depends ONLY on the model identity (type · method · objectives) and
  // the scenario params — NOT on pop size, generations, seed, constraints, or the
  // generation strategy. This payload is memoised on just those fields so the
  // pre-check fires only when something that could change the answer changes.
  // (Weights don't affect the model key/scenario name, so equal weights are used
  // for the WS validity gate — tweaking weight values won't re-trigger the check.)
  const checkConfig: OptimizeConfig | null = useMemo(() => {
    if (!scenario) return null;
    return {
      optimization_type: optType,
      method,
      objectives: selected,
      weights: needsWeights(optType, method) ? equalWeights(selected) : null,
      pop_size: POP_DEFAULT,
      n_gen: NGEN_DEFAULT,
      seed: SEED_DEFAULT,
      max_mission_time: null,
      min_connectivity: null,
      gen_strategy: "fixed",
      scenario: { ...scenario, target_positions: [0] },
    };
  }, [scenario, optType, method, selected]);

  // ── Debounced duplicate pre-check (guarded against stale responses). ──
  const checkTokenRef = useRef<symbol | null>(null);
  useEffect(() => {
    setDuplicate(null);
    if (!checkConfig || !selectionOk || !weightsOk) return;
    let cancelled = false;
    const token = Symbol("check");
    checkTokenRef.current = token;
    const handle = setTimeout(() => {
      checkOptimize(checkConfig)
        .then((res) => {
          if (cancelled || checkTokenRef.current !== token) return;
          if (res.exists)
            setDuplicate({ scenario_name: res.scenario_name, seeded: res.seeded });
          else setDuplicate(null);
        })
        .catch(() => {
          /* silent — pre-check is best-effort */
        });
    }, 400);
    return () => {
      cancelled = true;
      clearTimeout(handle);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [checkConfig, selectionOk, weightsOk]);

  // ── Run + poll. ──
  function stopPolling() {
    if (pollRef.current) {
      clearInterval(pollRef.current);
      pollRef.current = null;
    }
  }

  function clearStoredRun() {
    try {
      sessionStorage.removeItem(RUN_STORAGE_KEY);
      sessionStorage.removeItem(RUN_PARAMS_STORAGE_KEY);
    } catch {
      /* sessionStorage unavailable — ignore */
    }
  }

  function beginPoll(runId: string) {
    stopPolling();
    let inFlight = false; // serialize: never overlap requests (avoids reordering)
    let lastGen = -1; // monotonic guard against any stale response
    let failures = 0;
    pollRef.current = setInterval(async () => {
      if (inFlight) return;
      inFlight = true;
      let status: OptimizeStatus;
      try {
        status = await getOptimizeStatus(runId);
        failures = 0;
      } catch (err: unknown) {
        inFlight = false;
        const msg = err instanceof Error ? err.message : String(err);
        // A 404 means the run is gone (e.g. the server restarted and it wasn't
        // a finished run recoverable from disk). Don't spin forever.
        const permanent = msg.includes("404");
        if (permanent || ++failures >= 5) {
          stopPolling();
          clearStoredRun();
          setRunning(false);
          setProgress(null);
          setStopping(false);
          toast.error("Lost contact with the optimization run", {
            description: permanent
              ? "The run is no longer available (the server may have restarted). Configure and run it again."
              : "Repeated errors while polling the run status.",
          });
        }
        return;
      }
      inFlight = false;

      if (status.state === "running") {
        const g = status.gen ?? 0;
        if (g < lastGen) return; // out-of-order / stale tick — ignore
        lastGen = g;
        if (status.n_gen != null) setProgress({ gen: g, nGen: status.n_gen });
        if (status.best) {
          const best = status.best;
          setProgressHistory((h) =>
            h.length && h[h.length - 1].gen === g ? h : [...h, { gen: g, best }]
          );
        }
        if (status.live_front) setLiveFront(status.live_front);
        return;
      }
      // terminal
      stopPolling();
      clearStoredRun();
      setRunning(false);
      setProgress(null);
      setStopping(false);
      if (status.state === "done" && status.front) {
        setResult(status.front);
        setSelectedSolution(status.front.solutions[0]?.index ?? 0);
      } else if (status.state === "failed") {
        toast.error("Optimization failed", {
          description: status.error ?? "Unknown error",
        });
      }
    }, 1000);
  }

  async function handleRun() {
    if (!config || !isValid) return;
    setResult(null);
    setRunId(null);
    setStopping(false);
    setProgressHistory([]);
    setLiveFront(null);
    setProgress({ gen: 0, nGen: nGen });
    setRunning(true);
    // Snapshot the submitted parameters so they stay visible while solving.
    const snapshot: RunParams = {
      optType,
      method,
      objectives: selected,
      weights: needsWeights(optType, method) ? weights : null,
      popSize,
      nGen,
      genStrategy,
      earlyStopPatience,
      earlyStopThresholdPct,
      scenario: config.scenario,
      maxMissionTime: mmtEnabled ? mmtValue : null,
      minConnectivity: minConnEnabled ? minConnValue : null,
    };
    setRunParams(snapshot);
    try {
      const res = await startOptimize(config);
      try {
        sessionStorage.setItem(RUN_STORAGE_KEY, res.run_id);
        sessionStorage.setItem(RUN_PARAMS_STORAGE_KEY, JSON.stringify(snapshot));
      } catch {
        /* sessionStorage unavailable — ignore */
      }
      if (!mountedRef.current) return; // unmounted during the await — don't poll
      setRunId(res.run_id);
      beginPoll(res.run_id);
    } catch (err: unknown) {
      setRunning(false);
      setProgress(null);
      const msg = err instanceof Error ? err.message : String(err);
      if (msg.includes("409")) {
        toast.error("A run is already in progress");
      } else {
        toast.error("Failed to start optimization", { description: msg });
      }
    }
  }

  async function handleStop() {
    if (!runId || stopping) return;
    setStopping(true);
    try {
      const res = await stopOptimize(runId);
      if (res.stopping) {
        toast("Stopping the optimization…", {
          description:
            "Finishing the current generation — the best-so-far result will appear.",
        });
      }
      // If it had already finished (stopping=false), the poll surfaces the result.
    } catch (err: unknown) {
      setStopping(false);
      toast.error("Failed to stop", {
        description: err instanceof Error ? err.message : String(err),
      });
    }
  }

  // ── Scenario field helpers ──
  function updateScenario<K extends keyof ScenarioConfig>(
    key: K,
    value: ScenarioConfig[K]
  ) {
    setScenario((prev) => (prev ? { ...prev, [key]: value } : prev));
  }

  // Download the finished run as JSON. The export lives on the API origin
  // (:8000), so an <a download> filename is cross-origin and ignored by the
  // browser — fetch it into a same-origin blob so we can name the file after
  // the canonical scenario (model + every parameter), not the backend's
  // model_key-only default.
  async function downloadResultJson() {
    if (!runId || !result) return;
    try {
      const res = await fetch(optimizeExportUrl(runId));
      if (!res.ok) throw new Error(`export failed (${res.status})`);
      const blob = await res.blob();
      const url = URL.createObjectURL(blob);
      const a = document.createElement("a");
      a.href = url;
      a.download = `${result.scenario}.json`;
      document.body.appendChild(a);
      a.click();
      a.remove();
      URL.revokeObjectURL(url);
    } catch (e) {
      toast.error("Download failed", {
        description: e instanceof Error ? e.message : String(e),
      });
    }
  }

  const methods = methodsFor(optType);

  // Comm range (derived during render). Match the current comm_cell_range to a
  // preset; if none matches (or the user chose "Custom"), the metre input shows.
  const commCells = scenario?.comm_cell_range ?? 2;
  const cellSide = scenario?.cell_side_length ?? 50;
  const matchedComm = COMM_PRESETS.find((o) => Math.abs(o.cells - commCells) < COMM_EPS);
  const isCommCustom = commCustom || !matchedComm;
  const commSelectValue = isCommCustom ? COMM_CUSTOM : matchedComm.id;
  const commMetres = Math.round(commCells * cellSide);

  // Live objective columns (from the latest snapshot; fall back to the selection).
  const liveObjectives = useMemo(() => {
    const last = progressHistory[progressHistory.length - 1];
    if (last) return Object.keys(last.best);
    if (liveFront && liveFront[0]) return Object.keys(liveFront[0]);
    return selected;
  }, [progressHistory, liveFront, selected]);

  // ─────────────────────────────────────────────────────────────────────────────

  return (
    <TooltipProvider delayDuration={150}>
      <div className="mx-auto flex max-w-7xl flex-col gap-6 px-6 py-8">
        {/* Header */}
        <div className="flex flex-col gap-1.5">
          <h1 className="text-2xl font-bold tracking-tight text-foreground">
            Optimizer
          </h1>
          <p className="text-[15px] text-muted-foreground">
            Configure and run your own optimization — pick the objectives,
            method, and algorithm, then watch it solve.
          </p>
        </div>

        {/* ── Sticky run bar (always visible on scroll) ── */}
        <div className="sticky top-14 z-30 flex flex-col gap-3 rounded-xl border border-border bg-background/85 px-4 py-3 backdrop-blur supports-[backdrop-filter]:bg-background/70">
          <div className="flex flex-wrap items-center gap-3">
            <Button
              onClick={handleRun}
              disabled={!isValid}
              size="lg"
              className="min-w-[220px] flex-1 sm:flex-none"
            >
              {running ? "Running…" : "Run optimization"}
            </Button>
            {running && runId && (
              <Button
                onClick={handleStop}
                disabled={stopping}
                variant="outline"
                size="lg"
              >
                {stopping ? "Stopping…" : "Stop"}
              </Button>
            )}
            {running && progress ? (
              <div className="flex min-w-[220px] flex-1 flex-col gap-1.5">
                <div className="flex items-center justify-between text-xs">
                  <span className="text-muted-foreground">Solving…</span>
                  <span className="tabular-nums text-foreground">
                    Generation {progress.gen} / {progress.nGen}
                  </span>
                </div>
                <Progress
                  value={
                    progress.nGen > 0 ? (progress.gen / progress.nGen) * 100 : 0
                  }
                />
              </div>
            ) : duplicate && !running ? (
              <div className="flex min-w-0 flex-1 items-center gap-2 overflow-hidden rounded-lg border border-chart-1/30 bg-chart-1/5 px-3 py-1.5">
                <span className="shrink truncate text-sm text-foreground">
                  {duplicate.seeded
                    ? "A published (seeded) result for this mission already exists."
                    : "A run for this model and parameter combination already exists."}
                </span>
                <span className="hidden min-w-0 shrink truncate font-mono text-[11px] text-muted-foreground lg:inline">
                  {duplicate.scenario_name}
                </span>
                <Link
                  href={`/explore/${encodeURIComponent(duplicate.scenario_name)}`}
                  className="ml-auto shrink-0 whitespace-nowrap text-sm font-medium text-chart-1 hover:underline"
                >
                  Open it →
                </Link>
              </div>
            ) : null}
          </div>
        </div>

        {/* ── Body: live progress while running, else the config grid ── */}
        {running ? (
          <div className="flex flex-col gap-6 lg:flex-row lg:items-start">
            {runParams && (
              <div className="flex flex-col gap-2 rounded-xl border border-border bg-card p-4 lg:w-72 lg:shrink-0">
                <p className="text-sm font-medium text-foreground">
                  Run parameters
                </p>
                <div className="flex flex-col gap-1.5 text-xs">
                  <ParamRow
                    label="Method"
                    value={`${runParams.optType} · ${runParams.method}`}
                  />
                  <ParamRow
                    label="Objectives"
                    value={runParams.objectives.join(", ")}
                  />
                  {runParams.weights && (
                    <ParamRow
                      label="Weights"
                      value={runParams.objectives
                        .map((o) => `${runParams.weights![o] ?? 0}`)
                        .join(" · ")}
                    />
                  )}
                  <ParamRow label="Population" value={`${runParams.popSize}`} />
                  <ParamRow
                    label="Generations"
                    value={`${runParams.nGen} (${
                      runParams.genStrategy === "max" ? "max · early-stop" : "fixed"
                    })`}
                  />
                  {runParams.genStrategy === "max" && (
                    <ParamRow
                      label="Early-stop"
                      value={`≥${runParams.earlyStopThresholdPct}% · ${runParams.earlyStopPatience} gen`}
                    />
                  )}
                  <ParamRow
                    label="Scenario"
                    value={scenarioSummary(runParams.scenario)}
                  />
                  <ParamRow
                    label="Grid"
                    value={`${runParams.scenario.grid_size} × ${runParams.scenario.grid_size} · cell ${runParams.scenario.cell_side_length} m · speed ${runParams.scenario.max_drone_speed}`}
                  />
                  <ParamRow
                    label="Constraints"
                    value={constraintsSummary(runParams)}
                  />
                </div>
              </div>
            )}
            <div className="min-w-0 flex-1">
              <LiveProgress
                gen={progress?.gen ?? 0}
                nGen={progress?.nGen ?? nGen}
                objectives={liveObjectives}
                history={progressHistory}
                liveFront={liveFront}
                isMOO={optType === "MOO"}
              />
            </div>
          </div>
        ) : (
          <div className="grid grid-cols-1 gap-6 md:grid-cols-2 xl:grid-cols-3">
            {/* Optimization (type · method · objectives) — row 1, col 1 */}
            <Card className="xl:col-start-1 xl:row-start-1">
              <CardContent className="flex flex-col gap-6 pt-6">
            {/* Optimisation type */}
            <div className="flex flex-col gap-2.5">
              <SectionLabel>Optimization type</SectionLabel>
              <ToggleGroup
                type="single"
                value={optType}
                onValueChange={(v) => handleTypeChange(v as OptType)}
                className="justify-start"
              >
                <ToggleGroupItem value="SOO" variant="outline" className={OPT_TYPE_ITEM_CLS}>
                  SOO
                </ToggleGroupItem>
                <ToggleGroupItem value="MOO" variant="outline" className={OPT_TYPE_ITEM_CLS}>
                  MOO
                </ToggleGroupItem>
              </ToggleGroup>
              <p className="text-xs text-muted-foreground">
                {optType === "SOO"
                  ? "Single-objective: optimise one objective, or a weighted sum."
                  : "Multi-objective: find a Pareto front of trade-offs."}
              </p>
            </div>

            {/* Method */}
            <div className="flex flex-col gap-2.5">
              <SectionLabel>Method</SectionLabel>
              <div className="flex flex-wrap gap-2">
                {methods.map((m) => {
                  const disabled = DISABLED_METHODS.has(m);
                  const active = m === method;
                  const chip = (
                    <button
                      key={m}
                      type="button"
                      disabled={disabled}
                      aria-pressed={active}
                      onClick={() => handleMethodChange(m)}
                      className={cn(
                        "rounded-lg border px-4 py-1.5 text-sm font-medium transition-colors",
                        active
                          ? "border-chart-1/40 bg-chart-1/10 text-chart-1"
                          : "border-border bg-card text-muted-foreground hover:border-foreground/20 hover:text-foreground",
                        disabled &&
                          "cursor-not-allowed opacity-40 hover:border-border hover:text-muted-foreground"
                      )}
                    >
                      {m}
                    </button>
                  );
                  return disabled ? (
                    <Tooltip key={m}>
                      <TooltipTrigger asChild>
                        {/* span keeps tooltip working on a disabled button */}
                        <span className="inline-flex">{chip}</span>
                      </TooltipTrigger>
                      <TooltipContent>Coming soon</TooltipContent>
                    </Tooltip>
                  ) : (
                    chip
                  );
                })}
              </div>
            </div>

            {/* Objectives */}
            <div className="flex flex-col gap-2.5">
              <SectionLabel>Objectives</SectionLabel>
              <p className="text-xs text-muted-foreground">
                {isSingleSelect(optType, method)
                  ? "Select exactly one objective."
                  : needsWeights(optType, method)
                    ? "Select two or more, then assign weights that sum to 1.00."
                    : "Select two or more objectives."}
              </p>
              <div className="flex flex-wrap gap-2">
                {OBJECTIVES.map((spec) => {
                  const sel = selected.includes(spec.name);
                  // In single-select, clicking another chip swaps the selection,
                  // so chips stay clickable; never hard-disabled here.
                  return (
                    <ObjectiveChip
                      key={spec.name}
                      spec={spec}
                      selected={sel}
                      disabled={false}
                      onToggle={() => toggleObjective(spec.name)}
                    />
                  );
                })}
              </div>

              {!selectionOk && (
                <p className="text-xs text-destructive">
                  {isSingleSelect(optType, method)
                    ? "Select exactly one objective."
                    : "Select at least two objectives."}
                </p>
              )}

              {tbvWarning && (
                <InlineWarning>
                  Max Mean TBV needs n_visits ≥ 2 (it is 0 at n_visits = 1).
                  Increase n_visits in the scenario below.
                </InlineWarning>
              )}

              {/* Weighted-sum weights */}
              {needsWeights(optType, method) && selected.length >= 2 && (
                <div className="mt-1 flex flex-col gap-3 rounded-xl border border-border bg-card p-4">
                  <div className="flex items-center justify-between">
                    <p className="text-sm font-medium text-foreground">
                      Weights
                    </p>
                    <p
                      className={cn(
                        "text-sm tabular-nums",
                        weightsOk ? "text-chart-1" : "text-destructive"
                      )}
                    >
                      Σ = {weightSum.toFixed(2)}
                    </p>
                  </div>
                  <div className="flex flex-col gap-2">
                    {selected.map((name) => (
                      <div
                        key={name}
                        className="flex items-center justify-between gap-3"
                      >
                        <Label className="text-sm text-muted-foreground">
                          {name}
                        </Label>
                        <Input
                          type="number"
                          min={0}
                          max={1}
                          step={0.05}
                          value={weights[name] ?? 0}
                          onChange={(e) =>
                            setWeightFor(name, Number(e.target.value))
                          }
                          className="h-8 w-24 text-right tabular-nums"
                        />
                      </div>
                    ))}
                  </div>
                  {!weightsOk && (
                    <p className="text-xs text-destructive">
                      Weights must sum to 1.00 (currently{" "}
                      {weightSum.toFixed(2)}).
                    </p>
                  )}
                </div>
              )}
            </div>
              </CardContent>
            </Card>

            {/* Algorithm parameters (pop · gen · seed · generation strategy) — row 2, full width */}
            <Card className="xl:col-start-1 xl:col-span-3 xl:row-start-2">
              <CardHeader>
                <CardTitle>Algorithm parameters</CardTitle>
              </CardHeader>
              <CardContent>
                <div className="grid gap-x-8 gap-y-6 xl:grid-cols-2">
                  {/* Left: population · generations · seed */}
                  <div className="flex flex-col gap-6">
                <div className="flex flex-col gap-2">
                  <div className="flex items-center justify-between gap-3">
                    <Label className="text-sm text-muted-foreground">
                      Population size
                    </Label>
                    <Input
                      type="number"
                      inputMode="numeric"
                      min={POP_MIN}
                      max={POP_MAX}
                      value={popSize}
                      onChange={(e) => {
                        const n = parseInt(e.target.value, 10);
                        if (!Number.isNaN(n)) setPopSize(n);
                      }}
                      onBlur={() =>
                        setPopSize((p) =>
                          Math.min(POP_MAX, Math.max(POP_MIN, p || POP_DEFAULT))
                        )
                      }
                      className="h-8 w-24 text-right tabular-nums"
                    />
                  </div>
                  <Slider
                    min={POP_MIN}
                    max={POP_MAX}
                    step={1}
                    value={[popSize]}
                    onValueChange={(v) => setPopSize(v[0] ?? POP_DEFAULT)}
                  />
                </div>

                <div className="flex flex-col gap-2">
                  <div className="flex items-center justify-between gap-3">
                    <Label className="text-sm text-muted-foreground">
                      {genStrategy === "max" ? "Max generations (cap)" : "Generations"}
                    </Label>
                    <Input
                      type="number"
                      inputMode="numeric"
                      min={NGEN_MIN}
                      max={NGEN_MAX}
                      value={nGen}
                      onChange={(e) => {
                        const n = parseInt(e.target.value, 10);
                        if (!Number.isNaN(n)) setNGen(n);
                      }}
                      onBlur={() =>
                        setNGen((g) =>
                          Math.min(NGEN_MAX, Math.max(NGEN_MIN, g || NGEN_DEFAULT))
                        )
                      }
                      className="h-8 w-24 text-right tabular-nums"
                    />
                  </div>
                  <Slider
                    min={NGEN_MIN}
                    max={NGEN_MAX}
                    step={1}
                    value={[nGen]}
                    onValueChange={(v) => setNGen(v[0] ?? NGEN_DEFAULT)}
                  />
                </div>

                <div className="flex items-center justify-between gap-3 rounded-xl border border-border bg-card p-4">
                  <div className="flex flex-col">
                    <Label className="text-sm text-foreground">Seed</Label>
                    <span className="text-xs text-muted-foreground">
                      Same seed ⇒ reproducible run; change it to sample a different result.
                    </span>
                  </div>
                  <Input
                    type="number"
                    min={0}
                    value={seed}
                    onChange={(e) => {
                      const n = parseInt(e.target.value, 10);
                      if (!Number.isNaN(n)) setSeed(n);
                    }}
                    onBlur={() =>
                      setSeed((s) => (Number.isFinite(s) && s >= 0 ? s : SEED_DEFAULT))
                    }
                    className="h-8 w-24 text-right tabular-nums"
                  />
                </div>
                  </div>

                  {/* Right: generation strategy — merged from the former Optimizer settings card */}
                  <div className="flex flex-col gap-4 xl:border-l xl:border-border xl:pl-8">
                    <div className="flex flex-col gap-1.5">
                      <div className="flex flex-wrap items-center gap-2">
                        <SectionLabel>Generation strategy</SectionLabel>
                        <span className="rounded-full border border-chart-1/40 bg-chart-1/10 px-2 py-0.5 text-[11px] font-medium text-chart-1">
                          Recommended: Fixed Generations
                        </span>
                      </div>
                      <div className="flex flex-col gap-1.5 text-xs text-muted-foreground">
                        <p>
                          <span className="font-medium text-foreground">
                            Fixed Generations:
                          </span>{" "}
                          Runs the full number of generations for the most
                          refined, absolute objective-optimal solutions.
                        </p>
                        <p>
                          <span className="font-medium text-foreground">
                            Max Generations:
                          </span>{" "}
                          Stops early once no objective improves by a factor for
                          several generations — faster, but you may lose some
                          middle-ground solutions.
                        </p>
                        <p>
                          Choose Fixed when the absolute best objective values
                          matter most.
                        </p>
                      </div>
                    </div>
                <ToggleGroup
                  type="single"
                  value={genStrategy}
                  onValueChange={(v) => v && setGenStrategy(v as GenStrategy)}
                  className="grid grid-cols-2 gap-2 sm:max-w-sm"
                >
                  <ToggleGroupItem value="fixed" variant="outline" className="text-xs">
                    Fixed Generations
                  </ToggleGroupItem>
                  <ToggleGroupItem value="max" variant="outline" className="text-xs">
                    Max Generations
                  </ToggleGroupItem>
                </ToggleGroup>
                <p className="text-xs text-muted-foreground">
                  {genStrategy === "max"
                    ? `Stops early once no objective improves ≥${earlyStopThresholdPct}% for ${earlyStopPatience} generations (a cap, not a target).`
                    : "Runs the full number of generations for the most refined solutions."}
                </p>
                {genStrategy === "max" && (
                  <div className="grid gap-4 sm:grid-cols-2">
                    <div className="flex items-center justify-between gap-3 rounded-xl border border-border bg-card p-4">
                      <div className="flex flex-col">
                        <Label className="text-sm text-foreground">Patience</Label>
                        <span className="text-xs text-muted-foreground">
                          Generations with no qualifying improvement before stopping.
                        </span>
                      </div>
                      <Input
                        type="number"
                        min={2}
                        max={500}
                        value={earlyStopPatience}
                        onChange={(e) => {
                          const n = parseInt(e.target.value, 10);
                          if (!Number.isNaN(n)) setEarlyStopPatience(n);
                        }}
                        onBlur={() =>
                          setEarlyStopPatience((p) =>
                            Number.isFinite(p) && p >= 2
                              ? Math.min(500, p)
                              : ES_PATIENCE_DEFAULT
                          )
                        }
                        className="h-9 w-20 text-right tabular-nums"
                      />
                    </div>
                    <div className="flex items-center justify-between gap-3 rounded-xl border border-border bg-card p-4">
                      <div className="flex flex-col">
                        <Label className="text-sm text-foreground">
                          Improvement threshold (%)
                        </Label>
                        <span className="text-xs text-muted-foreground">
                          Minimum objective gain that counts as progress.
                        </span>
                      </div>
                      <Input
                        type="number"
                        min={1}
                        max={100}
                        step={1}
                        value={earlyStopThresholdPct}
                        onChange={(e) => {
                          const n = Number(e.target.value);
                          if (!Number.isNaN(n)) setEarlyStopThresholdPct(n);
                        }}
                        onBlur={() =>
                          setEarlyStopThresholdPct((t) =>
                            Number.isFinite(t) && t > 0 && t <= 100
                              ? t
                              : ES_THRESH_PCT_DEFAULT
                          )
                        }
                        className="h-9 w-20 text-right tabular-nums"
                      />
                    </div>
                  </div>
                )}
                  </div>
                </div>
              </CardContent>
            </Card>

            {/* Constraints — row 1, col 3 */}
            <Card className="flex flex-col xl:col-start-3 xl:row-start-1">
              <CardHeader>
                <CardTitle>Constraints</CardTitle>
                <CardDescription>
                  Feasibility rules the optimizer must satisfy. Tighter
                  constraints usually need more generations to converge.
                </CardDescription>
              </CardHeader>
              <CardContent className="flex flex-1 flex-col gap-4">
                {/* Speed Violation — always enforced (the ever-present base
                    constraint); toggle is locked on, no numeric bound. */}
                <div className="flex items-center justify-between gap-3">
                  <div className="flex flex-col">
                    <span className="text-sm text-foreground">Speed Violation</span>
                    <span className="text-xs text-muted-foreground">
                      Restricts drones from making long jumps.
                    </span>
                  </div>
                  <div className="flex items-center gap-3">
                    <Toggle on={true} onChange={() => {}} disabled />
                  </div>
                </div>
                <div className="flex items-center justify-between gap-3">
                  <div className="flex flex-col">
                    <span className="text-sm text-foreground">Mission Time Ceiling</span>
                    <span className="text-xs text-muted-foreground">
                      Mission time ≤ this (seconds).
                    </span>
                  </div>
                  <div className="flex items-center gap-3">
                    {mmtEnabled && (
                      <Input
                        type="number"
                        min={1}
                        value={mmtValue}
                        onChange={(e) => {
                          const n = Number(e.target.value);
                          if (!Number.isNaN(n)) setMmtValue(n);
                        }}
                        onBlur={() =>
                          setMmtValue((v) =>
                            Number.isFinite(v) && v > 0 ? v : MMT_DEFAULT
                          )
                        }
                        className="h-9 w-20 text-right tabular-nums"
                      />
                    )}
                    <Toggle on={mmtEnabled} onChange={setMmtEnabled} />
                  </div>
                </div>
                <div className="flex items-center justify-between gap-3">
                  <div className="flex flex-col">
                    <span className="text-sm text-foreground">Connectivity Floor</span>
                    <span className="text-xs text-muted-foreground">
                      Connectivity ≥ this (0–1).
                    </span>
                  </div>
                  <div className="flex items-center gap-3">
                    {minConnEnabled && (
                      <Input
                        type="number"
                        min={0}
                        max={1}
                        step={0.05}
                        value={minConnValue}
                        onChange={(e) => {
                          const n = Number(e.target.value);
                          if (!Number.isNaN(n)) setMinConnValue(n);
                        }}
                        onBlur={() =>
                          setMinConnValue((v) =>
                            Number.isFinite(v) && v >= 0 && v <= 1
                              ? v
                              : MIN_CONN_DEFAULT
                          )
                        }
                        className="h-9 w-20 text-right tabular-nums"
                      />
                    )}
                    <Toggle on={minConnEnabled} onChange={setMinConnEnabled} />
                  </div>
                </div>
              </CardContent>
            </Card>

            {/* Scenario — row 1, col 2 */}
            <Card className="flex flex-col xl:col-start-2 xl:row-start-1">
              <CardHeader>
                <CardTitle>Scenario</CardTitle>
                <CardDescription>
                  {scenario
                    ? `${scenario.number_of_drones} drones · n_visits ${scenario.n_visits} · grid ${scenario.grid_size}`
                    : "Loading defaults…"}
                </CardDescription>
              </CardHeader>
              {scenario && (
                <CardContent className="flex flex-1 flex-col gap-4">
                  <div className="grid flex-1 grid-cols-2 content-start gap-4">
                    {/* number_of_drones */}
                    <div className="flex flex-col gap-1.5">
                      <Label className="text-xs text-muted-foreground">
                        Number of drones
                      </Label>
                      <Select
                        value={String(scenario.number_of_drones)}
                        onValueChange={(v) =>
                          updateScenario("number_of_drones", Number(v))
                        }
                      >
                        <SelectTrigger className="h-9">
                          <SelectValue />
                        </SelectTrigger>
                        <SelectContent>
                          {DRONE_OPTIONS.map((d) => (
                            <SelectItem key={d} value={String(d)}>
                              {d}
                            </SelectItem>
                          ))}
                        </SelectContent>
                      </Select>
                    </div>

                    {/* n_visits */}
                    <div className="flex flex-col gap-1.5">
                      <Label className="text-xs text-muted-foreground">
                        n_visits
                      </Label>
                      <Select
                        value={String(scenario.n_visits)}
                        onValueChange={(v) =>
                          updateScenario("n_visits", Number(v))
                        }
                      >
                        <SelectTrigger className="h-9">
                          <SelectValue />
                        </SelectTrigger>
                        <SelectContent>
                          {NVISIT_OPTIONS.map((n) => (
                            <SelectItem key={n} value={String(n)}>
                              {n}
                            </SelectItem>
                          ))}
                        </SelectContent>
                      </Select>
                    </div>

                    {/* comm range — labelled selector (cells + metres) + custom metres */}
                    <div className="flex flex-col gap-1.5">
                      <Label className="text-xs text-muted-foreground">
                        Comm range
                      </Label>
                      <Select
                        value={commSelectValue}
                        onValueChange={(v) => {
                          if (v === COMM_CUSTOM) {
                            setCommCustom(true);
                            setCommMetresRaw(String(commMetres));
                          } else {
                            setCommCustom(false);
                            const preset = COMM_PRESETS.find((o) => o.id === v);
                            if (preset)
                              updateScenario("comm_cell_range", preset.cells);
                          }
                        }}
                      >
                        <SelectTrigger className="h-9">
                          <SelectValue />
                        </SelectTrigger>
                        <SelectContent>
                          {COMM_PRESETS.map((o) => (
                            <SelectItem key={o.id} value={o.id}>
                              {o.label} · {Math.round(o.cells * cellSide)} m
                            </SelectItem>
                          ))}
                          <SelectItem value={COMM_CUSTOM}>
                            Custom (metres)…
                          </SelectItem>
                        </SelectContent>
                      </Select>
                      {isCommCustom ? (
                        <Input
                          type="number"
                          min={1}
                          value={commMetresRaw}
                          onChange={(e) => {
                            setCommMetresRaw(e.target.value);
                            const m = Number(e.target.value);
                            if (Number.isFinite(m) && m > 0)
                              updateScenario(
                                "comm_cell_range",
                                m / scenario.cell_side_length
                              );
                          }}
                          placeholder="metres"
                          className="h-9 w-24 tabular-nums"
                        />
                      ) : null}
                      <p className="text-[11px] tabular-nums text-muted-foreground">
                        ≈ {commMetres} m · {formatCommCells(commCells)} cell-lengths
                      </p>
                    </div>

                    {/* max_drone_speed */}
                    <div className="flex flex-col gap-1.5">
                      <Label className="text-xs text-muted-foreground">
                        Max drone speed (m/s)
                      </Label>
                      <Input
                        type="number"
                        step={0.1}
                        value={scenario.max_drone_speed}
                        onChange={(e) =>
                          updateScenario(
                            "max_drone_speed",
                            Number(e.target.value)
                          )
                        }
                        className="h-9 tabular-nums"
                      />
                    </div>

                    {/* grid_size */}
                    <div className="flex flex-col gap-1.5">
                      <Label className="text-xs text-muted-foreground">
                        Grid size
                      </Label>
                      <Input
                        type="number"
                        value={scenario.grid_size}
                        onChange={(e) =>
                          updateScenario("grid_size", Number(e.target.value))
                        }
                        className="h-9 tabular-nums"
                      />
                    </div>

                    {/* cell_side_length */}
                    <div className="flex flex-col gap-1.5">
                      <Label className="text-xs text-muted-foreground">
                        Cell side length (m)
                      </Label>
                      <Input
                        type="number"
                        value={scenario.cell_side_length}
                        onChange={(e) => {
                          const newCell = Number(e.target.value);
                          updateScenario("cell_side_length", newCell);
                          // In custom mode the metre distance is the user's intent,
                          // so keep it fixed by recomputing comm_cell_range.
                          if (isCommCustom && newCell > 0) {
                            const m = Number(commMetresRaw);
                            if (Number.isFinite(m) && m > 0)
                              updateScenario("comm_cell_range", m / newCell);
                          }
                        }}
                        className="h-9 tabular-nums"
                      />
                    </div>
                  </div>
                </CardContent>
              )}
            </Card>
          </div>
        )}

        {/* ── Result ── */}
        {result && (
          <div className="flex flex-col gap-5">
            <Separator />
            <div className="flex flex-wrap items-start justify-between gap-3">
              <div className="flex flex-col gap-1">
                <h2 className="text-lg font-semibold tracking-tight text-foreground">
                  Result
                </h2>
                <p className="text-sm text-muted-foreground">
                  {result.model_key} · {result.n_solutions} solution
                  {result.n_solutions !== 1 ? "s" : ""} ·{" "}
                  {result.result_kind === "front"
                    ? "Pareto front"
                    : "single solution"}
                </p>
                {result.cancelled && (
                  <p className="text-xs font-medium text-amber-600 dark:text-amber-500">
                    Stopped early
                    {result.stopped_at_gen != null
                      ? ` at generation ${result.stopped_at_gen}`
                      : ""}{" "}
                    — showing the best solutions found so far.
                  </p>
                )}
                {result.early_stopped && !result.cancelled && (
                  <p className="text-xs font-medium text-amber-600 dark:text-amber-500">
                    Converged
                    {result.stopped_at_gen != null
                      ? ` at generation ${result.stopped_at_gen}`
                      : ""}{" "}
                    — no ≥10% objective improvement for several generations.
                    Feasible solutions retained.
                  </p>
                )}
              </div>
              {/* Download result (runs are not saved server-side) */}
              {runId && result.n_solutions > 0 && (
                <div className="flex flex-col items-end gap-1.5">
                  <button
                    type="button"
                    onClick={downloadResultJson}
                    className="inline-flex h-9 items-center gap-1.5 rounded-md border border-border bg-secondary px-4 text-sm font-medium text-foreground transition-colors hover:bg-accent"
                  >
                    Download result (JSON)
                  </button>
                  <p className="max-w-xs text-right text-xs text-muted-foreground">
                    Runs are not saved. Download the JSON and upload it in the
                    Playground to explore or compare it.
                  </p>
                </div>
              )}
            </div>

            {result.n_solutions === 0 ? (
              <div className="rounded-xl border border-amber-500/30 bg-amber-500/10 px-4 py-4">
                <p className="text-sm font-medium text-foreground">
                  No feasible solution found
                </p>
                <p className="mt-1 text-sm text-muted-foreground">
                  The optimizer could not satisfy the constraints within{" "}
                  {nGen} generations. Try increasing the population size and
                  generations, or relaxing the Max mission time / Min
                  connectivity constraints.
                </p>
              </div>
            ) : result.result_kind === "front" && result.n_solutions > 1 ? (
              <div className="flex flex-col gap-5">
                <div className="rounded-xl border border-border bg-card p-4">
                  <ParetoScatter
                    front={
                      { ...result, capabilities: {} } as unknown as ParetoFront
                    }
                    selectedIndex={selectedSolution}
                    onSelectIndex={setSelectedSolution}
                  />
                </div>
                <ResultTable front={result} />
              </div>
            ) : (
              <SingleResult front={result} />
            )}
          </div>
        )}
      </div>
    </TooltipProvider>
  );
}
