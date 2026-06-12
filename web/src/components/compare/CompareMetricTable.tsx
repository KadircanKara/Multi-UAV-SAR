"use client";

/**
 * CompareMetricTable — entities × metrics comparison table with best-per-column
 * highlight (best = min if polarity 1, max if polarity -1, over finite values),
 * mirroring MergingMetricsTable. Null values render as "—". When a metric carries
 * an `optimizedBy` set and an entity's key is NOT in it, the cell gets a subtle
 * muted asterisk + title flagging that the value was computed but not optimized
 * by that model.
 */

import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from "@/components/ui/table";
import { cn } from "@/lib/utils";
import type {
  CompareMetric,
  CompareEntity,
} from "@/components/compare/MetricComparisonView";

// ─── Types ────────────────────────────────────────────────────────────────────

export type CompareTableMetric = CompareMetric & {
  /** entity keys that actually optimized this metric */
  optimizedBy?: Set<string>;
};

export interface CompareMetricTableProps {
  metrics: CompareTableMetric[];
  entities: CompareEntity[];
}

// ─── Component ────────────────────────────────────────────────────────────────

export default function CompareMetricTable({
  metrics,
  entities,
}: CompareMetricTableProps) {
  if (!entities.length) return null;

  // Best (highlighted) value per metric column over finite values.
  // polarity 1 → min is best; polarity -1 → max is best.
  const bestPerMetric: Record<string, number | undefined> = {};
  for (const metric of metrics) {
    let best: number | undefined;
    for (const e of entities) {
      const v = e.values[metric.name];
      if (typeof v !== "number" || !Number.isFinite(v)) continue;
      if (best === undefined) {
        best = v;
      } else if (metric.polarity === 1) {
        if (v < best) best = v;
      } else if (v > best) {
        best = v;
      }
    }
    bestPerMetric[metric.name] = best;
  }

  return (
    <div className="rounded border border-border bg-card">
      <Table>
        <TableHeader>
          <TableRow>
            <TableHead className="font-mono text-xs tracking-widest uppercase text-muted-foreground">
              ENTITY
            </TableHead>
            {metrics.map((m) => (
              <TableHead
                key={m.name}
                className="font-mono text-xs tracking-wider text-muted-foreground"
              >
                {m.name}
                <span className="ml-1 normal-case tracking-normal text-muted-foreground/70">
                  ({m.polarity === -1 ? "↑" : "↓"})
                </span>
              </TableHead>
            ))}
          </TableRow>
        </TableHeader>
        <TableBody>
          {entities.map((e) => (
            <TableRow key={e.key}>
              <TableCell className="font-mono text-xs font-semibold tracking-widest uppercase text-primary">
                {e.label}
              </TableCell>
              {metrics.map((m) => {
                const v = e.values[m.name];
                const best = bestPerMetric[m.name];
                const isBest =
                  typeof v === "number" &&
                  Number.isFinite(v) &&
                  best !== undefined &&
                  v === best;
                const notOptimized =
                  m.optimizedBy !== undefined && !m.optimizedBy.has(e.key);
                return (
                  <TableCell
                    key={m.name}
                    className={cn(
                      "font-mono text-xs tabular-nums",
                      isBest
                        ? "text-emerald-600 dark:text-emerald-400 font-semibold"
                        : v == null
                        ? "text-muted-foreground"
                        : "text-foreground"
                    )}
                  >
                    {v == null ? "—" : v.toFixed(2)}
                    {notOptimized && v != null && (
                      <sup
                        className="ml-0.5 text-muted-foreground/60"
                        title="computed, not optimized by this model"
                      >
                        *
                      </sup>
                    )}
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
