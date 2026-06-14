"use client";

import { useEffect, useState } from "react";
import { ChevronDown } from "lucide-react";

import { getMissionConfig } from "@/lib/api";
import type { RunConfig } from "@/lib/types";
import {
  isRunConfig,
  strategyLabel,
  constraintList,
  weightsLabel,
} from "@/components/missions/runConfigFields";
import { cn } from "@/lib/utils";

interface Row {
  scenario: string;
  cfg: RunConfig | null;
}

/** Read-only run-config table — one row per compared mission. Collapsible via the
 *  Hide/Show toggle; run-configs are only fetched while the table is shown. */
export default function CompareRunDetails({ scenarios }: { scenarios: string[] }) {
  const [rows, setRows] = useState<Row[]>([]);
  const [collapsed, setCollapsed] = useState(true); // hidden by default
  const key = [...scenarios].sort().join("|");

  useEffect(() => {
    let cancelled = false;
    if (scenarios.length === 0) {
      setRows([]);
      return;
    }
    if (collapsed) return; // don't fetch run-configs while the table is hidden
    Promise.all(
      scenarios.map((s) =>
        getMissionConfig(s)
          .then((r): Row => ({ scenario: s, cfg: isRunConfig(r) ? r : null }))
          .catch((): Row => ({ scenario: s, cfg: null }))
      )
    ).then((res) => {
      if (!cancelled) setRows(res);
    });
    return () => {
      cancelled = true;
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [key, collapsed]);

  if (scenarios.length === 0) return null;

  return (
    <div className="flex flex-col gap-2">
      <h2 className="text-sm font-semibold text-foreground">
        <button
          type="button"
          onClick={() => setCollapsed((c) => !c)}
          aria-expanded={!collapsed}
          className="-mx-1 flex items-center gap-1.5 rounded px-1 py-0.5 transition-colors hover:text-foreground/80 focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-ring"
        >
          <ChevronDown
            aria-hidden="true"
            className={cn(
              "size-4 text-muted-foreground transition-transform",
              collapsed && "-rotate-90"
            )}
          />
          Run details
        </button>
      </h2>

      {!collapsed && (
        <div className="overflow-x-auto rounded-lg border">
          <table className="w-full text-xs">
            <thead className="bg-muted/40 text-muted-foreground">
              <tr>
                <th className="px-3 py-2 text-left font-medium">Mission</th>
                <th className="px-3 py-2 text-right font-medium">Pop</th>
                <th className="px-3 py-2 text-left font-medium">Generations</th>
                <th className="px-3 py-2 text-right font-medium">Seed</th>
                <th className="px-3 py-2 text-left font-medium">Constraints</th>
                <th className="px-3 py-2 text-left font-medium">Weights</th>
              </tr>
            </thead>
            <tbody>
              {rows.map(({ scenario, cfg }) => (
                <tr key={scenario} className="border-t">
                  <td className="px-3 py-2 font-mono text-[11px]">{scenario}</td>
                  {cfg ? (
                    <>
                      <td className="px-3 py-2 text-right">{cfg.pop_size}</td>
                      <td className="px-3 py-2">{strategyLabel(cfg)}</td>
                      <td className="px-3 py-2 text-right">{cfg.seed}</td>
                      <td className="px-3 py-2">{constraintList(cfg).join(" · ")}</td>
                      <td className="px-3 py-2">{weightsLabel(cfg) ?? "—"}</td>
                    </>
                  ) : (
                    <td className="px-3 py-2 text-muted-foreground" colSpan={5}>
                      Not recorded
                    </td>
                  )}
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      )}
    </div>
  );
}
