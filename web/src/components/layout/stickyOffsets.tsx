"use client";

/**
 * The one place that answers "how far below the viewport top does something
 * pinned on a section-panel page start?".
 *
 * Several things now pin at that offset — SectionPanelLayout's controls panel
 * and the shared legend above every chart grid — and a second copy of this
 * arithmetic would drift the moment the app nav or the header gap changed,
 * leaving one of them tucked behind the page header.
 *
 * Section CONTENT reads the offset from context rather than being handed it:
 * the legend sits several components below the page that measures the header
 * (page → sections → view → grid → legend), and threading a pixel count
 * through those props buys nothing that the layout cannot publish once.
 */

import { createContext, useContext } from "react";
import { useMediaQuery } from "@/hooks/useMediaQuery";

/**
 * Height of the app's own top bar — which exists only BELOW `lg`. At `lg` and
 * above the app nav is a fixed left sidebar that takes no vertical space, so
 * anything pinning to the viewport top starts at 0 there. Must switch at the
 * same width as DESKTOP_QUERY below and as the `lg:top-0` on each route's
 * pinned header.
 */
export const MOBILE_NAV_PX = 56;
/** Breathing room between the page's pinned header and the panel below it. */
export const HEADER_GAP_PX = 8;

// Matches Tailwind's default `lg` breakpoint, which SectionPanelLayout's outer
// grid still switches on via CSS — the two must agree so the aside/Sheet choice
// (JS) and the column geometry (CSS) never disagree about "desktop."
export const DESKTOP_QUERY = "(min-width: 1024px)";

/**
 * Where the page's pinned header ends, in px from the viewport top — the
 * FLUSH offset, with no gap added.
 *
 * @param headerHeight measured height of that header (0 ⇒ the page pins
 *   nothing of its own).
 */
export function useHeaderOffset(headerHeight: number): number {
  const isDesktop = useMediaQuery(DESKTOP_QUERY);
  // No app top bar at lg and above — the nav is the left sidebar there.
  const navPx = isDesktop ? 0 : MOBILE_NAV_PX;
  return navPx + headerHeight;
}

const StickyTopContext = createContext(0);

/** Publishes the flush header offset to everything inside a section's content. */
export const StickyTopProvider = StickyTopContext.Provider;

/**
 * `top` (px) for a pinned element inside section content. Zero outside a
 * SectionPanelLayout, which pins such an element to the viewport top — correct
 * for a page with no header of its own.
 *
 * Flush against the header, with no HEADER_GAP_PX: a pinned legend paints its
 * own opaque background, and a transparent gap above it only lets the chart
 * lines scrolling underneath show through as a moving sliver.
 */
export function useStickyTop(): number {
  return useContext(StickyTopContext);
}
