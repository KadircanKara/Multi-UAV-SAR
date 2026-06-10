"use client";

/**
 * /explore/[scenario] — standalone deep-dive for a precomputed scenario.
 *
 * The actual Pareto / Merging / Animation UI lives in <ScenarioExplorer>, which
 * is also embedded inline on the model page (driven by the parameter-combination
 * dropdowns). This route stays for deep-linking / bookmarking a single scenario.
 */

import { useState, useCallback } from "react";
import Link from "next/link";
import { useParams } from "next/navigation";
import type { ParetoFront } from "@/lib/types";
import ScenarioExplorer from "@/components/explore/ScenarioExplorer";

export default function ExplorePage() {
  const params = useParams();
  const rawScenario = params?.scenario;
  const scenario = decodeURIComponent(
    Array.isArray(rawScenario) ? rawScenario[0] ?? "" : rawScenario ?? ""
  );

  // Captured from the loaded front so the back-link can return to the model page.
  const [modelKey, setModelKey] = useState<string | null>(null);
  const handleFrontLoaded = useCallback((front: ParetoFront) => {
    setModelKey(front.model_key ?? null);
  }, []);

  return (
    <div className="mx-auto flex max-w-7xl flex-col gap-6 px-4 py-6">
      {/* Back link — goes to the model page once the front is loaded, else missions */}
      <Link
        href={modelKey ? "/model/" + encodeURIComponent(modelKey) : "/missions"}
        className="inline-flex items-center gap-1 text-xs font-mono tracking-widest text-muted-foreground hover:text-primary transition-colors uppercase"
      >
        ← {modelKey ?? "MISSIONS"}
      </Link>

      <ScenarioExplorer scenario={scenario} onFrontLoaded={handleFrontLoaded} />
    </div>
  );
}
