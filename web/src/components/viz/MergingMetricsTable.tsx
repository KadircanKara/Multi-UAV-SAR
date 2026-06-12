"use client";

/**
 * MergingMetricsTable — compare results table with best-per-column highlight.
 * Rows = none/onboard/gcs. Columns = the 4 metric names.
 * null values rendered as "—". Best (minimum) per column highlighted in accent.
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

// ─── Component ────────────────────────────────────────────────────────────────

export default function MergingMetricsTable({ tableRows, metricNames }: Props) {
  if (!tableRows.length) return null;

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
