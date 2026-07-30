"use client";

/**
 * Two-column shell: a sticky panel showing the active section's controls, and the
 * sections' content stacked beside it.
 *
 * The desktop aside and the mobile Sheet render the SAME `controls` tree, so
 * exactly one of the two must ever be mounted — otherwise a section with
 * stateful controls exists as two independent, silently-diverging copies.
 * `hidden lg:flex` / `lg:hidden` (CSS-only visibility) cannot provide that:
 * both stay DOM-mounted at every viewport width, so opening the Sheet on a
 * narrow screen mounted a second, independently-stateful instance alongside
 * the (still-mounted, merely hidden) aside. `isDesktop` — a real
 * media-query read via useMediaQuery — decides which ONE of the two gets
 * rendered at all; the other is not created that render, so it cannot hold
 * state to begin with.
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
import { useMediaQuery } from "@/hooks/useMediaQuery";
import { useScrollSpy } from "@/hooks/useScrollSpy";
import type { PanelSection } from "./PanelSection";

const DEFAULT_ESTIMATED_HEIGHT = 480;

// Matches Tailwind's default `lg` breakpoint, which the outer grid below
// still switches on via CSS — the two must agree so the aside/Sheet choice
// (JS) and the column geometry (CSS) never disagree about "desktop."
const DESKTOP_QUERY = "(min-width: 1024px)";

export default function SectionPanelLayout({ sections }: { sections: PanelSection[] }) {
  const ids = sections.map((s) => s.id);
  const { activeId, seenIds, register } = useScrollSpy(ids);
  const active = sections.find((s) => s.id === activeId) ?? sections[0];
  const isDesktop = useMediaQuery(DESKTOP_QUERY);

  const panelBody = active && (
    <div key={active.id} className="animate-hud-rise flex flex-col gap-4">
      {active.controls}
    </div>
  );

  return (
    <div className="flex flex-col gap-6 lg:grid lg:grid-cols-[320px_1fr] lg:items-start lg:gap-6">
      {isDesktop ? (
        <aside className="flex sticky top-14 max-h-[calc(100vh-3.5rem-2rem)] flex-col gap-3 overflow-y-auto rounded-xl border border-border bg-background px-4 py-3">
          <p className="text-xs font-semibold tracking-widest uppercase text-primary font-display">
            {active?.label}
          </p>
          {panelBody}
        </aside>
      ) : (
        <div className="sticky top-14 z-30 rounded-xl border border-border bg-background px-3 py-2">
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
      )}

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
