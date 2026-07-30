"use client";

/**
 * MergingContent — the MERGING section's scrolling-column half: the compare
 * result the sensing-config builder in MergingControls produces (time-metric
 * table, belief evolution, targets-known).
 *
 * Owns no state itself. The compare result and its still-running flag are
 * read by MergingControls too (to disable/label its own Compare button and to
 * show these same loading skeletons' sibling state), so both live one level
 * up in useScenarioSections — see that file's doc comment.
 *
 * Every branch below reserves at least MERGING_HEIGHT of vertical space. The
 * pre-Compare state used to render nothing — a mounted section with zero
 * height can never become useScrollSpy's active section, so the panel could
 * never switch to MERGING at all, making the controls this task just built
 * unreachable by scroll. The dashed-border placeholder below is the same
 * "nothing to show yet" convention `compare/page.tsx` and
 * `MetricComparisonView` already use elsewhere in the app.
 */

import dynamic from "next/dynamic";
import { Card, CardContent } from "@/components/ui/card";
import { Skeleton } from "@/components/ui/skeleton";
import MergingMetricsTable, {
  type CompareTableRow,
} from "@/components/viz/MergingMetricsTable";
import type { BeliefRow } from "@/components/viz/BeliefEvolutionChart";
import type { TargetsKnownRow } from "@/components/viz/TargetsKnownChart";
import ChartSkeleton from "./ChartSkeleton";

// ─── Dynamic (SSR-off) chart imports ─────────────────────────────────────────

const BeliefEvolutionChart = dynamic(
  () => import("@/components/viz/BeliefEvolutionChart"),
  { ssr: false, loading: () => <ChartSkeleton height="h-64" /> }
);

const TargetsKnownChart = dynamic(
  () => import("@/components/viz/TargetsKnownChart"),
  { ssr: false, loading: () => <ChartSkeleton height="h-64" /> }
);

const METRIC_NAMES = [
  "Effective Mission Time",
  "Detection Time",
  "Inform Time",
  "Time At Least One Drone Knows All Targets",
];

/**
 * Reserved height, px, for Merging's content column. Declared here (not
 * grouped with PARETO_HEIGHT/ANIMATION_HEIGHT in useScenarioSections, which
 * imports this one back) because this file is what actually has to hit the
 * number: its own pre-Compare and in-flight states use it as a `minHeight`
 * floor, and the pre-mount skeleton in useScenarioSections just needs to
 * roughly agree so neither transition visibly jumps. Ballparked from a
 * populated result (the TIME METRICS label and table with its footnote, plus
 * the two belief/known charts at their loaded height) — not measured in a
 * browser.
 */
export const MERGING_HEIGHT = 620;

// ─── Component ────────────────────────────────────────────────────────────────

interface Props {
  comparing: boolean;
  tableRows: CompareTableRow[] | null;
  beliefRows: BeliefRow[] | null;
  knownRows: TargetsKnownRow[] | null;
}

export default function MergingContent({
  comparing,
  tableRows,
  beliefRows,
  knownRows,
}: Props) {
  if (comparing) {
    return (
      <div style={{ minHeight: MERGING_HEIGHT }} className="flex flex-col gap-3">
        <Skeleton className="h-32 w-full" />
        <div className="grid grid-cols-1 gap-4 lg:grid-cols-2">
          <Skeleton className="h-64 w-full" />
          <Skeleton className="h-64 w-full" />
        </div>
      </div>
    );
  }

  // Reachable before the first Compare, and again after a failed one: the
  // request rejects before any of tableRows/beliefRows/knownRows are set, and
  // `comparing` is already back to false in runCompare's `finally` (see
  // useScenarioSections) — there is no separate error flag, so a failed
  // compare shows the same placeholder a never-run one does, rather than
  // collapsing to nothing.
  if (!tableRows) {
    return (
      <div
        style={{ minHeight: MERGING_HEIGHT }}
        className="flex flex-col items-center justify-center rounded border border-dashed border-border px-4 py-6 text-center"
      >
        <p className="text-xs font-mono text-muted-foreground">
          Configure the sensing parameters in the panel, then run the
          comparison to see time metrics, belief evolution, and targets known
          for the selected solution.
        </p>
      </div>
    );
  }

  return (
    <div className="flex flex-col gap-6">
      {/* Time-metric results — table */}
      <div className="flex flex-col gap-3">
        <span
          className="text-xs font-semibold tracking-widest uppercase text-primary font-display"
          style={{ fontFamily: "var(--font-display)" }}
        >
          TIME METRICS
        </span>
        <MergingMetricsTable tableRows={tableRows} metricNames={METRIC_NAMES} />
      </div>

      {beliefRows && knownRows && (
        <div className="grid grid-cols-1 gap-4 lg:grid-cols-2">
          <Card>
            <CardContent className="pt-4">
              <BeliefEvolutionChart rows={beliefRows} />
            </CardContent>
          </Card>
          <Card>
            <CardContent className="pt-4">
              <TargetsKnownChart rows={knownRows} />
            </CardContent>
          </Card>
        </div>
      )}
    </div>
  );
}
