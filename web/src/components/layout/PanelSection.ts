import type { ReactNode } from "react";

/** One scroll section and the controls that drive it.
 *  Producers return these; SectionPanelLayout decides where they render. */
export type PanelSection = {
  /** Scroll anchor and scrollspy key. Must be unique within a page. */
  id: string;
  /** Shown as the panel header when this section is active. */
  label: string;
  /** Rendered in the sticky left panel (or the Sheet below `lg`). Both may be
   *  mounted concurrently, so this must be fully controlled from a shared
   *  parent — no local `useState` — or the two copies will desync. */
  controls: ReactNode;
  /** Rendered in the right column. Mounts on first intersection. */
  content: ReactNode;
  /** Reserved height for the not-yet-mounted skeleton, px. Default 480. */
  estimatedHeight?: number;
};
