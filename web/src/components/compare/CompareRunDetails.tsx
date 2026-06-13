"use client";

import { useEffect, useState } from "react";

import { getMissionConfig } from "@/lib/api";
import type { RunConfig } from "@/lib/types";
import {
  isRunConfig,
  strategyLabel,
  constraintList,
  weightsLabel,
  sourceLabel,
} from "@/components/missions/runConfigFields";

interface Row {
  scenario: string;
  cfg: RunConfig | null;
}

/** Read-only run-config table — one row per compared mission. */
export default function CompareRunDetails({ scenarios }: { scenarios: string[] }) {
  const [rows, setRows] = useState<Row[]>([]);
  const key = [...scenarios].sort().join("|");

  useEffect(() => {
    let cancelled = false;
    if (scenarios.length === 0) {
      setRows([]);
      return;
    }
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
  }, [key]);

  if (scenarios.length === 0) return null;

  return (
    <div className="flex flex-col gap-2">
      <h2 className="text-sm font-semibold text-foreground">Run details</h2>
      <div className="overflow-x-auto rounded-lg border">
        <table className="w-full text-xs">
          <thead className="bg-muted/40 text-muted-foreground">
            <tr>
              <th className="px-3 py-2 text-left font-medium">Mission</th>
              <th className="px-3 py-2 text-left font-medium">Source</th>
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
                    <td className="px-3 py-2">{sourceLabel(cfg)}</td>
                    <td className="px-3 py-2 text-right">{cfg.pop_size}</td>
                    <td className="px-3 py-2">{strategyLabel(cfg)}</td>
                    <td className="px-3 py-2 text-right">{cfg.seed}</td>
                    <td className="px-3 py-2">{constraintList(cfg).join(" · ")}</td>
                    <td className="px-3 py-2">{weightsLabel(cfg) ?? "—"}</td>
                  </>
                ) : (
                  <td className="px-3 py-2 text-muted-foreground" colSpan={6}>
                    Not recorded
                  </td>
                )}
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </div>
  );
}
