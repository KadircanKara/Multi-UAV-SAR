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
 * The LAST section of several carries a viewport-height floor, unless the page
 * says it renders something after this layout. A section only becomes active
 * once its top scrolls above the nav, and if nothing follows the last one there
 * is nothing left to scroll it up there: a final section shorter than the
 * viewport keeps its top far down the screen even at maximum scroll, so it can
 * never be selected and its controls are unreachable — the same "a section the
 * panel can't reach" failure as a zero-height section, arriving from the other
 * end. The floor is a no-op for a section already taller than the viewport,
 * which is the normal case, and wrong when the page has its own content below
 * (that content does the scrolling instead, and the floor would only open a gap
 * before it).
 */

import { Menu } from "lucide-react";
import { Button } from "@/components/ui/button";
import {
  Sheet, SheetContent, SheetHeader, SheetTitle, SheetTrigger,
} from "@/components/ui/sheet";
import { Skeleton } from "@/components/ui/skeleton";
import { useMediaQuery } from "@/hooks/useMediaQuery";
import { useScrollSpy } from "@/hooks/useScrollSpy";
import { cn } from "@/lib/utils";
import type { PanelSection } from "./PanelSection";

const DEFAULT_ESTIMATED_HEIGHT = 480;

/** Nav height (h-14). Must match useScrollSpy's NAV_OFFSET_PX. */
const NAV_PX = 56;
/** Breathing room between the page's pinned header and the panel below it. */
const HEADER_GAP_PX = 8;
/** Space left under the panel so it doesn't run to the exact viewport edge. */
const PANEL_BOTTOM_PX = 24;

// Matches Tailwind's default `lg` breakpoint, which the outer grid below
// still switches on via CSS — the two must agree so the aside/Sheet choice
// (JS) and the column geometry (CSS) never disagree about "desktop."
const DESKTOP_QUERY = "(min-width: 1024px)";

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
  /** True when the page renders its own content BELOW this layout. Drops the
   *  last section's viewport-height floor, which exists only to make a short
   *  final section reachable when the layout is the end of the page. */
  hasContentBelow?: boolean;
}

export default function SectionPanelLayout({
  sections,
  stickyOffset = 0,
  hasContentBelow = false,
}: Props) {
  const ids = sections.map((s) => s.id);
  const { activeId, seenIds, register } = useScrollSpy(ids, stickyOffset + HEADER_GAP_PX);
  const active = sections.find((s) => s.id === activeId) ?? sections[0];
  const isDesktop = useMediaQuery(DESKTOP_QUERY);

  const stickyTop = NAV_PX + (stickyOffset > 0 ? stickyOffset + HEADER_GAP_PX : 0);
  const panelMaxHeight = `calc(100vh - ${stickyTop + PANEL_BOTTOM_PX}px)`;

  const panelBody = active && (
    <div key={active.id} className="animate-hud-rise flex flex-col gap-4">
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
      <div className="flex min-w-0 flex-col gap-10">
        {sections.map((section, i) => (
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
            className={cn(
              // See the file comment: without this the last section is
              // unreachable by the scrollspy whenever it is shorter than the
              // viewport, taking its controls with it. A lone section needs no
              // floor — it is the scrollspy's fallback, so it is active from
              // the first paint whatever its height, and padding it out would
              // only leave dead space below a one-section page.
              sections.length > 1 &&
                i === sections.length - 1 &&
                !hasContentBelow &&
                "min-h-[calc(100vh-3.5rem)]"
            )}
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
