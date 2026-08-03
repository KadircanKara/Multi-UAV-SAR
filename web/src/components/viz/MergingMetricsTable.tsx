"use client";

/**
 * MergingMetricsTable — compare results table with best-per-column highlight.
 * Rows = none/onboard/gcs. Columns = the 4 metric names.
 * null values rendered as "—". Best (minimum) per column highlighted in accent.
 *
 * Inform Time carries a footnote because its rows are not the same quantity:
 * Sensing.py prices onboard/gcs as the detection→BS window (0 when the BS
 * learns in the detection step) but overrides `none` to mission_time, since
 * under that topology there is no comms channel to time. The column is still
 * ranked as-is; the footnote keeps a reader from reading the gap as latency.
 */

import {
  Table,
  TableBody,
  TableCaption,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from "@/components/ui/table";
import { cn } from "@/lib/utils";

// ─── Types ────────────────────────────────────────────────────────────────────

export interface CompareTableRow {
  label: string;
  [metricName: string]: string | number | null;
}

interface Props {
  tableRows: CompareTableRow[];
  metricNames: string[];
}

/** Metric whose rows are not a single quantity — see the file header. */
const FOOTNOTED_METRIC = "Inform Time";

// ─── Component ────────────────────────────────────────────────────────────────

export default function MergingMetricsTable({ tableRows, metricNames }: Props) {
  if (!tableRows.length) return null;

  const showsFootnote = metricNames.includes(FOOTNOTED_METRIC);

  // Find minimum (best) per column — null counts as Infinity
  const bestPerCol: Record<string, number> = {};
  for (const metric of metricNames) {
    let best = Infinity;
    for (const row of tableRows) {
      const v = row[metric];
      if (typeof v === "number" && v < best) best = v;
    }
    bestPerCol[metric] = best;
  }

  return (
    <div className="rounded border border-border bg-card">
      <Table>
        <TableCaption className="font-mono text-xs text-muted-foreground tracking-wide pb-3">
          ALL METRICS IN SIMULATION TIME UNITS — LOWER IS BETTER —{" "}
          <span className="text-emerald-600 dark:text-emerald-400">GREEN</span> = BEST PER COLUMN
          {showsFootnote && (
            <span className="mt-2 block normal-case tracking-normal">
              † Inform Time is not one quantity across rows: for onboard and gcs
              it is the delay from detection until the base station knows (0 when
              it learns in the same step), while none has no comms channel, so
              its inform time is defined as the mission time — the drones carry
              the news home. Read the two as different measurements, not as one
              being faster than the other.
            </span>
          )}
        </TableCaption>
        <TableHeader>
          <TableRow>
            <TableHead className="font-mono text-xs tracking-widest uppercase text-muted-foreground">
              CONFIG
            </TableHead>
            {metricNames.map((m) => (
              <TableHead
                key={m}
                className="font-mono text-xs tracking-wider text-muted-foreground"
              >
                {m}
                {m === FOOTNOTED_METRIC && <sup aria-hidden="true">†</sup>}
              </TableHead>
            ))}
          </TableRow>
        </TableHeader>
        <TableBody>
          {tableRows.map((row) => (
            <TableRow key={row.label}>
              <TableCell className="font-mono text-xs font-semibold tracking-widest uppercase text-primary">
                {row.label}
              </TableCell>
              {metricNames.map((m) => {
                const v = row[m];
                const isBest =
                  typeof v === "number" &&
                  isFinite(bestPerCol[m]) &&
                  v === bestPerCol[m];
                return (
                  <TableCell
                    key={m}
                    className={cn(
                      "font-mono text-xs tabular-nums",
                      isBest
                        ? "text-emerald-600 dark:text-emerald-400 font-semibold"
                        : v == null
                        ? "text-muted-foreground"
                        : "text-foreground"
                    )}
                  >
                    {v == null ? "—" : typeof v === "number" ? v.toFixed(2) : v}
                  </TableCell>
                );
              })}
            </TableRow>
          ))}
        </TableBody>
      </Table>
    </div>
  );
}
