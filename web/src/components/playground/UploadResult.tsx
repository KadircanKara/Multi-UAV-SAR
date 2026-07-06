"use client";
import { useState } from "react";
import type { PlaygroundResult } from "@/lib/types";

export function UploadResult({ onLoaded }: { onLoaded: (r: PlaygroundResult) => void }) {
  const [error, setError] = useState<string | null>(null);
  async function handle(file: File | undefined) {
    if (!file) return;
    setError(null);
    try {
      const parsed = JSON.parse(await file.text());
      if (parsed?.schema_version !== 1 || !Array.isArray(parsed?.solutions)) {
        throw new Error("Not a valid result file (expected schema_version 1).");
      }
      onLoaded(parsed as PlaygroundResult);
    } catch (e) {
      setError(e instanceof Error ? e.message : "Could not read file.");
    }
  }
  return (
    <div className="flex flex-col gap-2">
      <input
        type="file" accept="application/json"
        onChange={(e) => handle(e.target.files?.[0])}
        className="text-sm text-muted-foreground file:mr-3 file:rounded-md file:border file:border-border file:bg-secondary file:px-3 file:py-1.5 file:text-sm file:text-foreground"
      />
      {error && <p className="text-sm text-destructive">{error}</p>}
    </div>
  );
}
