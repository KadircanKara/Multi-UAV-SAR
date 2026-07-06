"use client";
import { useState } from "react";
import Link from "next/link";
import ScenarioExplorer from "@/components/explore/ScenarioExplorer";
import { UploadResult } from "@/components/playground/UploadResult";
import type { PlaygroundResult } from "@/lib/types";

export default function MissionsPlayground() {
  const [result, setResult] = useState<PlaygroundResult | null>(null);
  return (
    <div className="mx-auto flex max-w-7xl flex-col gap-6 px-4 py-6">
      <Link href="/missions" className="text-sm text-muted-foreground hover:text-foreground">← Missions</Link>
      <h1 className="text-2xl font-bold tracking-tight">Playground</h1>
      <p className="text-sm text-muted-foreground">Results are not stored — everything runs from the file you upload.</p>
      <UploadResult onLoaded={setResult} />
      {result && <ScenarioExplorer source={{ mode: "playground", result }} />}
    </div>
  );
}
