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
      <div className="flex flex-col gap-3">
        <Skeleton className="h-32 w-full" />
        <div className="grid grid-cols-1 gap-4 lg:grid-cols-2">
          <Skeleton className="h-64 w-full" />
          <Skeleton className="h-64 w-full" />
        </div>
      </div>
    );
  }

  if (!tableRows) return null;

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
