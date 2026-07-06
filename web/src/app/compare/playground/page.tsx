"use client";
import { useState } from "react";
import Link from "next/link";
import { UploadResult } from "@/components/playground/UploadResult";
import { playgroundComparison } from "@/lib/api";
import type { PlaygroundResult, ComparisonResponse } from "@/lib/types";
import { ObjectivesView } from "@/components/compare/ObjectivesView";

export default function ComparePlayground() {
  const [results, setResults] = useState<PlaygroundResult[]>([]);
  const [data, setData] = useState<ComparisonResponse | null>(null);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState<string | null>(null);

  async function run() {
    setBusy(true);
    setError(null);
    try {
      setData(await playgroundComparison(results));
    } catch (e) {
      setError(e instanceof Error ? e.message : "Comparison failed.");
    } finally {
      setBusy(false);
    }
  }

  return (
    <div className="mx-auto flex max-w-7xl flex-col gap-6 px-4 py-6">
      <Link href="/compare" className="text-sm text-muted-foreground hover:text-foreground">← Compare</Link>
      <h1 className="text-2xl font-bold tracking-tight">Compare — Playground</h1>
      <p className="text-sm text-muted-foreground">Upload two or more result files to compare their objectives. (Time-metric comparison is available for seeded results only.)</p>
      <UploadResult onLoaded={(r) => setResults((xs) => [...xs, r])} />
      <p className="text-sm text-muted-foreground">{results.length} file(s) loaded.</p>
      <button disabled={results.length < 1 || busy} onClick={run} className="w-fit rounded-md border border-border bg-secondary px-4 py-2 text-sm disabled:opacity-50">{busy ? "Comparing…" : "Compare objectives"}</button>
      {error && <p className="text-sm text-destructive">{error}</p>}
      {data && <ObjectivesView data={data} />}
    </div>
  );
}
