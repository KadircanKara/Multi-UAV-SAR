"use client";

/**
 * ScenarioExplorer — the Pareto / Merging / Animation deep-dive for ONE
 * precomputed scenario, as a self-contained block: it loads the front, renders
 * the scenario's identity, and hands the sections to the panel layout.
 *
 * The three sections used to be tabs; they are now scroll sections whose
 * controls share one sticky panel (see SectionPanelLayout), which is why
 * everything below the header is `useScenarioSections` — a page that wants to
 * interleave its OWN sections with these (the model route) calls that hook
 * directly and skips this wrapper.
 *
 * Used standalone at /missions/[modelKey]/scenario/[suffix] and in the Analysis
 * block of /optimize (playground source). The model page does NOT use it — it
 * calls useScenarioSections directly so that its own parameter-effect and
 * combination-table sections share one panel and one scrollspy with these
 * three. Whatever remount that page's combination switch needs is arranged
 * inside the sections themselves (see the `key`s in useScenarioSections), not
 * by a wrapper here.
 */

import type { ReactNode } from "react";
import type { ExplorerSource } from "@/lib/source";
import type { ParetoFront } from "@/lib/types";
import SectionPanelLayout from "@/components/layout/SectionPanelLayout";
import { Badge } from "@/components/ui/badge";
import { ExplorerSkeleton, OfflinePanel } from "./ExplorerStates";
import { useScenarioSections } from "./useScenarioSections";

// ─── Small utility components ─────────────────────────────────────────────────

function ScenarioTitle({ label }: { label: string }) {
  return (
    <h1
      className="text-sm font-semibold tracking-widest uppercase text-primary font-display"
      style={{ fontFamily: "var(--font-display)" }}
    >
      {label}
    </h1>
  );
}

function ScenarioBadges({ front }: { front: ParetoFront }) {
  return (
    <div className="flex flex-wrap items-center gap-2">
      <Badge variant="outline" className="text-xs font-mono tracking-widest">
        {front.model_key}
      </Badge>
      <Badge variant="outline" className="text-xs font-mono tracking-widest">
        {front.result_kind.toUpperCase()}
      </Badge>
      <Badge variant="outline" className="text-xs font-mono tracking-widest">
        {front.n_solutions} SOLUTION{front.n_solutions !== 1 ? "S" : ""}
      </Badge>
      {front.objectives.map((obj) => (
        <Badge
          key={obj}
          className="text-xs font-mono tracking-wide bg-secondary text-secondary-foreground"
        >
          {obj}
          {front.polarities[obj] === -1 && (
            <span className="ml-1 text-muted-foreground">(max)</span>
          )}
        </Badge>
      ))}
    </div>
  );
}

// ─── ScenarioExplorer ─────────────────────────────────────────────────────────

interface Props {
  source: ExplorerSource;
  /** Show the big scenario title heading (standalone route). Off when embedded. */
  showTitle?: boolean;
  /** Show the model/kind/solutions/objectives badge row. Off when the parent
   *  already renders that identity itself (the Analysis section's sticky
   *  header does, via RunSummary) so it isn't stated twice on one screen. */
  showSummary?: boolean;
  /** Notified once the front loads — lets a parent build a back-link, etc. */
  onFrontLoaded?: (front: ParetoFront) => void;
  /** Optional content rendered inside the Pareto-front card, below the plots
   *  (e.g. the model route's read-only run-details). Omitted ⇒ nothing extra. */
  paretoFooter?: ReactNode;
}

export default function ScenarioExplorer({
  source,
  showTitle = true,
  showSummary = true,
  onFrontLoaded,
  paretoFooter,
}: Props) {
  const { front, loading, error, sections } = useScenarioSections({
    source,
    onFrontLoaded,
    paretoFooter,
  });

  // Display label — was `scenario` before; derived for both source modes.
  const displayLabel =
    source.mode === "seeded"
      ? source.scenario
      : source.result.model.model_key ?? "uploaded run";

  if (source.mode === "seeded" && !source.scenario) return null;
  if (loading) return <ExplorerSkeleton />;
  if (error) {
    return (
      <div className="flex flex-col gap-3">
        {showTitle && <ScenarioTitle label={displayLabel} />}
        <OfflinePanel message={error} />
      </div>
    );
  }
  if (!front) return null;

  return (
    <div className="flex flex-col gap-6">
      {/* Scenario info header */}
      {(showTitle || showSummary) && (
        <div className="flex flex-col gap-2">
          {showTitle && <ScenarioTitle label={displayLabel} />}
          {showSummary && <ScenarioBadges front={front} />}
        </div>
      )}

      <SectionPanelLayout sections={sections} />
    </div>
  );
}
