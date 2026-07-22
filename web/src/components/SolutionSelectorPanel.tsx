"use client";

/**
 * SolutionSelectorPanel — capability-gated strategy controls for selecting
 * a solution from the Pareto front (or single result).
 */

import React from "react";
import { useState } from "react";
import { toast } from "sonner";
import type { ParetoFront, SolutionDetail } from "@/lib/types";
import { sourceSelect, type ExplorerSource } from "@/lib/source";
import { isPercentObjective, percentString } from "@/lib/objective-format";
import { Button } from "@/components/ui/button";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import { Slider } from "@/components/ui/slider";
import { Separator } from "@/components/ui/separator";

// ─── Helpers ──────────────────────────────────────────────────────────────────

function SectionLabel({ children }: { children: React.ReactNode }) {
  return (
    <p className="text-xs text-muted-foreground tracking-widest uppercase font-mono">
      {children}
    </p>
  );
}

function DetailCard({ detail }: { detail: SolutionDetail }) {
  return (
    <div className="rounded border border-primary/30 bg-primary/5 px-3 py-2 font-mono text-xs">
      <p className="text-primary font-semibold tracking-wide mb-1">
        {detail.label}
      </p>
      <dl className="grid grid-cols-2 gap-x-4 gap-y-0.5 tabular-nums">
        {Object.entries(detail.objectives_abs).map(([k, v]) => (
          <React.Fragment key={k}>
            <dt className="text-muted-foreground truncate">{k}</dt>
            <dd className="text-foreground">
              {typeof v !== "number"
                ? "—"
                : isPercentObjective(k)
                  ? percentString(v)
                  : v.toFixed(4)}
            </dd>
          </React.Fragment>
        ))}
      </dl>
    </div>
  );
}

// ─── Main component ───────────────────────────────────────────────────────────

interface Props {
  source: ExplorerSource;
  front: ParetoFront;
  selectedIndex: number;
  onSelectIndex: (idx: number) => void;
}

export default function SolutionSelectorPanel({
  source,
  front,
  selectedIndex,
  onSelectIndex,
}: Props) {
  const caps = front.capabilities as Record<string, unknown>;
  const objectives = front.objectives;

  // Local state
  const [bestObj, setBestObj] = useState<string>(
    Array.isArray(caps.best) && (caps.best as string[]).length > 0
      ? (caps.best as string[])[0]
      : objectives[0] ?? ""
  );
  const [byIndexVal, setByIndexVal] = useState(0);
  const [weights, setWeights] = useState<Record<string, number>>(
    Object.fromEntries(objectives.map((o) => [o, 1 / (objectives.length || 1)]))
  );
  const [detail, setDetail] = useState<SolutionDetail | null>(null);
  const [loading, setLoading] = useState(false);

  async function runSelect(
    strategy: string,
    extra?: Partial<Parameters<typeof sourceSelect>[1]>
  ) {
    setLoading(true);
    try {
      const res = await sourceSelect(source, {
        strategy,
        model_key: front.model_key,
        ...extra,
      });
      onSelectIndex(res.index);
      setDetail(res.detail);
    } catch (err: unknown) {
      const msg = err instanceof Error ? err.message : String(err);
      toast.error("SELECTION FAILED", { description: msg });
    } finally {
      setLoading(false);
    }
  }

  const maxByIndex =
    typeof caps.by_index === "number" ? (caps.by_index as number) : 0;

  const hasTheSolution = Boolean(caps.the_solution);
  const hasBest =
    Array.isArray(caps.best) && (caps.best as string[]).length > 0;
  const hasBalanced = Boolean(caps.balanced);
  const hasKnee = Boolean(caps.knee);
  const hasByWeights = Boolean(caps.by_weights);
  const hasByIndex = typeof caps.by_index === "number";

  // Solution to display: a strategy result if it matches the current index,
  // otherwise looked up directly from the front by index — so clicking a
  // scatter point immediately shows that solution's objective values without
  // any extra backend round-trip (the front already carries every solution).
  const currentFromFront =
    front.solutions.find((s) => s.index === selectedIndex) ??
    front.solutions[selectedIndex];
  const shownDetail: SolutionDetail | null =
    detail && detail.index === selectedIndex
      ? detail
      : currentFromFront
      ? {
          index: selectedIndex,
          label: `Solution #${selectedIndex}`,
          objectives_abs: currentFromFront.objectives_abs,
        }
      : null;

  return (
    <div className="flex flex-col gap-4">
      {/* THE SOLUTION (single / WS) */}
      {hasTheSolution && (
        <div className="flex flex-col gap-2">
          <SectionLabel>RESULT</SectionLabel>
          <Button
            size="sm"
            variant="default"
            disabled={loading}
            onClick={() => runSelect("the_solution")}
            className="w-full text-xs tracking-widest font-mono"
          >
            THE SOLUTION
          </Button>
        </div>
      )}

      {/* BEST (Pareto front) */}
      {hasBest && (
        <div className="flex flex-col gap-2">
          <SectionLabel>BEST OBJECTIVE</SectionLabel>
          <div className="flex gap-2">
            <Select value={bestObj} onValueChange={setBestObj}>
              <SelectTrigger className="h-7 min-w-0 flex-1 text-xs font-mono">
                <SelectValue />
              </SelectTrigger>
              <SelectContent>
                {(caps.best as string[]).map((obj) => (
                  <SelectItem key={obj} value={obj} className="text-xs font-mono">
                    {obj}
                  </SelectItem>
                ))}
              </SelectContent>
            </Select>
            <Button
              size="sm"
              variant="outline"
              disabled={loading || !bestObj}
              onClick={() => runSelect("best", { objective_name: bestObj })}
              className="h-7 text-xs tracking-widest font-mono shrink-0"
            >
              BEST
            </Button>
          </div>
        </div>
      )}

      {/* BALANCED */}
      {hasBalanced && (
        <div className="flex flex-col gap-2">
          <SectionLabel>BALANCED</SectionLabel>
          <Button
            size="sm"
            variant="outline"
            disabled={loading}
            onClick={() => runSelect("balanced")}
            title="The most even compromise — the front solution closest to the centre of all objectives (each normalised 0–1)."
            className="w-full text-xs tracking-widest font-mono"
          >
            BALANCED
          </Button>
        </div>
      )}

      {/* KNEE */}
      {hasKnee && (
        <div className="flex flex-col gap-2">
          <SectionLabel>KNEE POINT</SectionLabel>
          <Button
            size="sm"
            variant="outline"
            disabled={loading}
            onClick={() => runSelect("knee")}
            title="The knee of the Pareto front — the best bang-for-buck trade-off, where improving any objective further would cost a disproportionate sacrifice in another."
            className="w-full text-xs tracking-widest font-mono"
          >
            KNEE
          </Button>
        </div>
      )}

      {/* BY INDEX */}
      {hasByIndex && (
        <div className="flex flex-col gap-2">
          <SectionLabel>BY INDEX (0–{maxByIndex})</SectionLabel>
          <div className="flex items-center gap-3">
            <Slider
              min={0}
              max={maxByIndex}
              step={1}
              value={[byIndexVal]}
              onValueChange={([v]) => setByIndexVal(v)}
              className="flex-1"
            />
            <span className="w-8 text-right font-mono text-xs tabular-nums text-primary">
              {byIndexVal}
            </span>
          </div>
          <Button
            size="sm"
            variant="outline"
            disabled={loading}
            onClick={() => runSelect("by_index", { index: byIndexVal })}
            className="w-full text-xs tracking-widest font-mono"
          >
            SELECT #{byIndexVal}
          </Button>
        </div>
      )}

      {/* BY WEIGHTS */}
      {hasByWeights && objectives.length > 0 && (
        <div className="flex flex-col gap-2">
          <SectionLabel>BY WEIGHTS</SectionLabel>
          {objectives.map((obj) => (
            <div key={obj} className="flex flex-col gap-1">
              <div className="flex justify-between">
                <span className="text-xs font-mono text-muted-foreground truncate">
                  {obj}
                </span>
                <span className="text-xs font-mono tabular-nums text-primary">
                  {(weights[obj] ?? 0).toFixed(2)}
                </span>
              </div>
              <Slider
                min={0}
                max={1}
                step={0.01}
                value={[weights[obj] ?? 0]}
                onValueChange={([v]) =>
                  setWeights((prev) => ({ ...prev, [obj]: v }))
                }
              />
            </div>
          ))}
          <Button
            size="sm"
            variant="outline"
            disabled={loading}
            onClick={() => runSelect("by_weights", { weights })}
            className="w-full text-xs tracking-widest font-mono"
          >
            APPLY WEIGHTS
          </Button>
        </div>
      )}

      {/* No capabilities */}
      {!hasTheSolution && !hasBest && !hasBalanced && !hasKnee && !hasByIndex && !hasByWeights && (
        <p className="text-xs text-muted-foreground font-mono">
          NO SELECTION STRATEGIES AVAILABLE
        </p>
      )}

      {/* Separator + active-solution readout (always shows the selected point) */}
      <Separator />

      <div className="flex flex-col gap-2">
        <SectionLabel>ACTIVE SOLUTION</SectionLabel>
        {shownDetail ? (
          <DetailCard detail={shownDetail} />
        ) : (
          <p className="font-mono text-xs text-muted-foreground">—</p>
        )}
      </div>
    </div>
  );
}
