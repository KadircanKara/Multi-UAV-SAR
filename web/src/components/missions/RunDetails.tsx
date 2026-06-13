"use client";

import { useEffect, useState, type ReactNode } from "react";

import { getMissionConfig } from "@/lib/api";
import type { MissionConfigResponse } from "@/lib/types";
import { Card, CardContent } from "@/components/ui/card";
import {
  isRunConfig,
  strategyLabel,
  constraintList,
  weightsLabel,
  sourceLabel,
} from "./runConfigFields";

function Row({ label, value }: { label: string; value: ReactNode }) {
  return (
    <div className="flex justify-between gap-4 py-1 text-xs">
      <span className="text-muted-foreground">{label}</span>
      <span className="text-right font-medium text-foreground">{value}</span>
    </div>
  );
}

/** Read-only optimizer run-config for one mission. Renders nothing until loaded;
 *  a graceful note when the mission has no recorded config. */
export default function RunDetails({ scenario }: { scenario: string }) {
  const [state, setState] = useState<MissionConfigResponse | null>(null);
  const [failed, setFailed] = useState(false);

  useEffect(() => {
    let cancelled = false;
    setState(null);
    setFailed(false);
    getMissionConfig(scenario)
      .then((r) => {
        if (!cancelled) setState(r);
      })
      .catch(() => {
        if (!cancelled) setFailed(true);
      });
    return () => {
      cancelled = true;
    };
  }, [scenario]);

  if (failed || state === null) return null;

  if (!isRunConfig(state)) {
    return (
      <Card>
        <CardContent className="pt-4 text-xs text-muted-foreground">
          Run details not recorded for this mission.
        </CardContent>
      </Card>
    );
  }

  const c = state;
  const weights = weightsLabel(c);
  return (
    <Card>
      <CardContent className="pt-4">
        <h3 className="mb-2 text-xs font-semibold uppercase tracking-wider text-muted-foreground">
          Run details
        </h3>
        <Row label="Source" value={sourceLabel(c)} />
        <Row label="Population" value={c.pop_size} />
        <Row label="Generations" value={strategyLabel(c)} />
        <Row label="Seed" value={c.seed} />
        <Row label="Constraints" value={constraintList(c).join(" · ")} />
        {weights ? <Row label="Weights" value={weights} /> : null}
      </CardContent>
    </Card>
  );
}
