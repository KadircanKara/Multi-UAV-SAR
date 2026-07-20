"use client";

/**
 * /missions/[modelKey]/scenario/[suffix] — deep-link one seeded scenario.
 *
 * The suffix is the scenario name with its model prefix stripped (everything
 * from "g_" on: `g_8_a_50_n_4_v_2.5_r_2_nvisits_2`). The full backend name is
 * NOT rebuilt by concatenation — modelKey (`TC_MOO_NSGA2`) and the scenario
 * prefix (`MOO_NSGA2_TC`) order their tokens differently — instead the page
 * loads the model's scenario list and picks the entry ending with the suffix.
 * The Pareto/Merging/Animation UI is the same <ScenarioExplorer> the model
 * page embeds.
 */

import { useEffect, useState } from "react";
import Link from "next/link";
import { useParams } from "next/navigation";
import { getModelGrid } from "@/lib/api";
import ScenarioExplorer from "@/components/explore/ScenarioExplorer";

function param(v: string | string[] | undefined): string {
  return decodeURIComponent(Array.isArray(v) ? v[0] ?? "" : v ?? "");
}

export default function ScenarioDeepLinkPage() {
  const params = useParams();
  const modelKey = param(params?.modelKey);
  const suffix = param(params?.suffix);

  const [scenario, setScenario] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    if (!modelKey || !suffix) return;
    let cancelled = false;
    getModelGrid(modelKey)
      .then((grid) => {
        if (cancelled) return;
        const match = grid.scenarios.find((s) =>
          s.scenario.endsWith("_" + suffix)
        );
        if (match) setScenario(match.scenario);
        else setError(`No "${suffix}" combination in ${modelKey}.`);
      })
      .catch((e: unknown) => {
        if (!cancelled) setError(e instanceof Error ? e.message : String(e));
      });
    return () => {
      cancelled = true;
    };
  }, [modelKey, suffix]);

  return (
    <div className="mx-auto flex max-w-7xl flex-col gap-6 px-4 py-6">
      <Link
        href={"/missions/" + encodeURIComponent(modelKey)}
        className="inline-flex items-center gap-1 text-xs font-mono tracking-widest text-muted-foreground hover:text-primary transition-colors uppercase"
      >
        ← {modelKey || "MISSIONS"}
      </Link>

      {error ? (
        <div className="rounded-xl border border-destructive/30 bg-destructive/5 px-4 py-3">
          <p className="text-sm font-medium text-destructive">Scenario not found</p>
          <p className="text-sm text-muted-foreground">{error}</p>
        </div>
      ) : scenario ? (
        <ScenarioExplorer source={{ mode: "seeded", scenario }} />
      ) : (
        <p className="text-sm text-muted-foreground">Loading…</p>
      )}
    </div>
  );
}
