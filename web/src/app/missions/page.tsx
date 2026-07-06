"use client";
import Link from "next/link";
import { Card } from "@/components/ui/card";

export default function MissionsLanding() {
  return (
    <div className="mx-auto flex max-w-4xl flex-col gap-8 px-6 py-10">
      <div className="flex flex-col gap-1.5">
        <h1 className="text-2xl font-bold tracking-tight">Missions</h1>
        <p className="text-[15px] text-muted-foreground">Browse seeded results, or explore your own uploaded run.</p>
      </div>
      <div className="grid gap-4 sm:grid-cols-2">
        <Link href="/missions/seeded-results"><Card className="h-full p-6 transition-colors hover:border-primary"><h2 className="text-lg font-semibold">Seeded Results</h2><p className="mt-1 text-sm text-muted-foreground">The precomputed mission library — browse models, parameter grids, and Pareto fronts.</p></Card></Link>
        <Link href="/missions/playground"><Card className="h-full p-6 transition-colors hover:border-primary"><h2 className="text-lg font-semibold">Playground</h2><p className="mt-1 text-sm text-muted-foreground">Upload a result file you generated and explore it with the same tools.</p></Card></Link>
      </div>
    </div>
  );
}
