import type { ReactNode } from "react";

/** One scroll section and the controls that drive it.
 *  Producers return these; SectionPanelLayout decides where they render. */
export type PanelSection = {
  /** Scroll anchor and scrollspy key. Must be unique within a page. */
  id: string;
  /** Shown as the panel header when this section is active. */
  label: string;
  /** Rendered in the sticky left panel at `lg` (>=1024px) and above, or in
   *  the drawer Sheet below that. SectionPanelLayout picks one of the two by
   *  media query (not CSS visibility), so exactly one instance of this tree
   *  is ever mounted at a time. */
  controls: ReactNode;
  /** Rendered in the right column. Mounts on first intersection. */
  content: ReactNode;
  /** Reserved height for the not-yet-mounted skeleton, px. Default 480. */
  estimatedHeight?: number;
};
