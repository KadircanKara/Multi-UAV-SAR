"use client";

/** Placeholder for a next/dynamic chart that is still loading. Shared by the
 *  explore sections (which import their charts dynamically from two different
 *  files now) so every chart reserves space the same way. */

import { Skeleton } from "@/components/ui/skeleton";
import { cn } from "@/lib/utils";

export default function ChartSkeleton({ height }: { height: string }) {
  return <Skeleton className={cn("w-full rounded", height)} />;
}
