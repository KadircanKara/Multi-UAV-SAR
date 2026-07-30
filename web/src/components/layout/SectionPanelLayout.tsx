"use client";

/**
 * Two-column shell: a sticky panel showing the active section's controls, and the
 * sections' content stacked beside it.
 *
 * The cross-fade animates an inner div, never the sticky element — a transform on
 * the sticky box would create a containing block and break the pinning.
 */

import { Menu } from "lucide-react";
import { Button } from "@/components/ui/button";
import {
  Sheet, SheetContent, SheetHeader, SheetTitle, SheetTrigger,
} from "@/components/ui/sheet";
import { Skeleton } from "@/components/ui/skeleton";
import { useScrollSpy } from "@/hooks/useScrollSpy";
import type { PanelSection } from "./PanelSection";

const DEFAULT_ESTIMATED_HEIGHT = 480;

export default function SectionPanelLayout({ sections }: { sections: PanelSection[] }) {
  const ids = sections.map((s) => s.id);
  const { activeId, seenIds, register } = useScrollSpy(ids);
  const active = sections.find((s) => s.id === activeId) ?? sections[0];

  const panelBody = active && (
    <div key={active.id} className="animate-hud-rise flex flex-col gap-4">
      {active.controls}
    </div>
  );

  return (
    <div className="flex flex-col gap-6 lg:grid lg:grid-cols-[320px_1fr] lg:items-start lg:gap-6">
      {/* Desktop panel */}
      <aside className="hidden lg:flex lg:sticky lg:top-14 lg:max-h-[calc(100vh-3.5rem-2rem)] lg:flex-col lg:gap-3 lg:overflow-y-auto rounded-xl border border-border bg-background px-4 py-3">
        <p className="text-xs font-semibold tracking-widest uppercase text-primary font-display">
          {active?.label}
        </p>
        {panelBody}
      </aside>

      {/* Narrow: same controls behind a Sheet */}
      <div className="sticky top-14 z-30 lg:hidden rounded-xl border border-border bg-background px-3 py-2">
        <Sheet>
          <SheetTrigger asChild>
            <Button variant="outline" size="sm" className="w-full justify-start gap-2">
              <Menu className="size-4" aria-hidden="true" />
              <span className="font-mono text-xs tracking-widest uppercase">
                {active?.label}
              </span>
            </Button>
          </SheetTrigger>
          <SheetContent side="left" className="w-[min(88vw,340px)] overflow-y-auto">
            <SheetHeader>
              <SheetTitle className="text-xs font-semibold tracking-widest uppercase text-primary">
                {active?.label}
              </SheetTitle>
            </SheetHeader>
            <div className="mt-4 flex flex-col gap-4">{active?.controls}</div>
          </SheetContent>
        </Sheet>
      </div>

      {/* Content column */}
      <div className="flex min-w-0 flex-col gap-10">
        {sections.map((section) => (
          <section
            key={section.id}
            id={section.id}
            ref={register(section.id)}
            style={{
              contentVisibility: "auto",
              containIntrinsicSize: `${section.estimatedHeight ?? DEFAULT_ESTIMATED_HEIGHT}px`,
            }}
            className="scroll-mt-20"
          >
            {seenIds.has(section.id) ? (
              section.content
            ) : (
              <Skeleton
                style={{ height: section.estimatedHeight ?? DEFAULT_ESTIMATED_HEIGHT }}
                className="w-full"
              />
            )}
          </section>
        ))}
      </div>
    </div>
  );
}
