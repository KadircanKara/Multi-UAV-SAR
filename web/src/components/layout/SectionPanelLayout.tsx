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
 *
 * A section may legitimately have no `controls` at all. Neither the aside nor
 * the drawer trigger is rendered for one, and the two-column grid collapses to
 * a single column, so the section's content gets the full width instead of
 * sitting beside a tall empty outlined box.
 *
 * A short FINAL section used to be padded out to a viewport height so its top
 * could still scroll under the header and be selected. That is gone: it cost a
 * screen of blank space, and useScrollSpy now handles the case directly by
 * treating the last section as active whenever the document is scrolled to its
 * bottom — which is what the padding was really buying.
 */

import { Menu } from "lucide-react";
import { Button } from "@/components/ui/button";
import {
  Sheet, SheetContent, SheetHeader, SheetTitle, SheetTrigger,
} from "@/components/ui/sheet";
import { Skeleton } from "@/components/ui/skeleton";
import { useMediaQuery } from "@/hooks/useMediaQuery";
import {
  DESKTOP_QUERY, HEADER_GAP_PX, StickyTopProvider, useHeaderOffset,
} from "@/components/layout/stickyOffsets";
import { useScrollSpy } from "@/hooks/useScrollSpy";
import { cn } from "@/lib/utils";
import type { PanelSection } from "./PanelSection";

const DEFAULT_ESTIMATED_HEIGHT = 480;

/** Space left under the panel so it doesn't run to the exact viewport edge. */
const PANEL_BOTTOM_PX = 24;

interface Props {
  sections: PanelSection[];
  /**
   * Height of whatever the page pins directly below the nav — every route
   * using this layout pins its own identity header there, with `z-30` and an
   * opaque background. The panel is `sticky` at the SAME offset, so without
   * this it pins underneath that header and its top is painted over: on a tall
   * panel the reader loses the first controls, and on a short one (a
   * three-line solution readout) almost the whole thing disappears. Measure
   * the header with useElementHeight and pass it; the panel then pins clear of
   * it and sizes itself to the space that is actually left.
   */
  stickyOffset?: number;
}

export default function SectionPanelLayout({
  sections,
  stickyOffset = 0,
}: Props) {
  const ids = sections.map((s) => s.id);
  const isDesktop = useMediaQuery(DESKTOP_QUERY);

  // Where the page's own header ends. Published to the sections below, so
  // anything their content pins (the shared chart legend) lands clear of it
  // without every intervening component passing the number along.
  const headerOffset = useHeaderOffset(stickyOffset);
  // The panel itself floats a gap below that header rather than butting
  // against it — it is a bordered box, not an opaque bar.
  const stickyTop = headerOffset + (stickyOffset > 0 ? HEADER_GAP_PX : 0);

  const { activeId, seenIds, register } = useScrollSpy(
    ids,
    stickyTop + HEADER_GAP_PX
  );
  const active = sections.find((s) => s.id === activeId) ?? sections[0];
  const panelMaxHeight = `calc(100vh - ${stickyTop + PANEL_BOTTOM_PX}px)`;

  // Keyed on the CONTROLS, not the section: sections that share one panel say
  // so with `controlsKey`, and scrolling between them then keeps the same
  // mounted instance (and skips the fade, since nothing changed).
  const panelBody = active && (
    <div
      key={active.controlsKey ?? active.id}
      className="animate-hud-rise flex flex-col gap-4"
    >
      {active.controls}
    </div>
  );

  // Sections are allowed to have nothing to configure — the combinations table
  // is one. The panel is dropped for those, so it does not trail past the last
  // section that had any use for it.
  const hasControls = Boolean(active?.controls);
  // The COLUMN, though, is kept for as long as any section wants one. Letting
  // the grid collapse instead would re-lay-out every section above at a new
  // width the moment the reader scrolled into a section without controls —
  // chart grids reflow, the document changes height, and the scroll position
  // slides out from under them.
  const anyControls = sections.some((s) => Boolean(s.controls));

  return (
    <div
      className={cn(
        "flex flex-col gap-6",
        anyControls && "lg:grid lg:grid-cols-[360px_1fr] lg:items-start lg:gap-6"
      )}
    >
      {/* Holds the column open when the active section has no controls. */}
      {anyControls && !hasControls && <div aria-hidden="true" />}

      {hasControls && (isDesktop ? (
        <aside
          style={{ top: stickyTop, maxHeight: panelMaxHeight }}
          className="flex sticky z-20 flex-col gap-3 overflow-y-auto rounded-xl border border-border bg-background px-4 py-3"
        >
          <p className="text-xs font-semibold tracking-widest uppercase text-primary font-display">
            {active?.label}
          </p>
          {panelBody}
        </aside>
      ) : (
        <div
          style={{ top: stickyTop }}
          className="sticky z-20 rounded-xl border border-border bg-background px-3 py-2"
        >
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
      ))}

      {/* Content column */}
      <StickyTopProvider value={headerOffset}>
      <div className="flex min-w-0 flex-col gap-10">
        {sections.map((section) => (
          <section
            key={section.id}
            id={section.id}
            ref={register(section.id)}
            style={{
              contentVisibility: "auto",
              containIntrinsicSize: `${section.estimatedHeight ?? DEFAULT_ESTIMATED_HEIGHT}px`,
              // Anchor scrolling (the combination table's row clicks) has to
              // clear the page header too, or the section it jumps to lands
              // behind it.
              scrollMarginTop: stickyTop + HEADER_GAP_PX,
            }}
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
      </StickyTopProvider>
    </div>
  );
}
