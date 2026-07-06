"use client";
import Link from "next/link";
import { Card } from "@/components/ui/card";

export default function CompareLanding() {
  return (
    <div className="mx-auto flex max-w-4xl flex-col gap-8 px-6 py-10">
      <div className="flex flex-col gap-1.5">
        <h1 className="text-2xl font-bold tracking-tight">Compare</h1>
        <p className="text-[15px] text-muted-foreground">Compare optimiser models against the seeded library, or compare your own uploaded runs.</p>
      </div>
      <div className="grid gap-4 sm:grid-cols-2">
        <Link href="/compare/seeded-results"><Card className="h-full p-6 transition-colors hover:border-primary"><h2 className="text-lg font-semibold">Seeded Results</h2><p className="mt-1 text-sm text-muted-foreground">Compare models across the precomputed library — objectives and sensing time-metrics.</p></Card></Link>
        <Link href="/compare/playground"><Card className="h-full p-6 transition-colors hover:border-primary"><h2 className="text-lg font-semibold">Playground</h2><p className="mt-1 text-sm text-muted-foreground">Upload two or more result files and compare their objectives.</p></Card></Link>
      </div>
    </div>
  );
}
