"use client";

/**
 * ParetoControls — what drives the Pareto section from the sticky panel (or
 * its narrow-viewport drawer): the route's own combination picker, then the
 * solution selector.
 *
 * The axis dropdowns are NOT here. They render on the plots themselves, above
 * each chart, where the axis they name is the one you are looking at — five of
 * them (2D X/Y and 3D X/Y/Z, independent of each other) would otherwise push
 * the solution selector below the fold of a 320px panel. The axis *state*
 * still lives one level up in useScenarioSections and reaches the charts as
 * props, so the panel could drive it again without moving anything.
 *
 * The selection this panel writes is shared: one `selectedIndex` behind both
 * plots, which is what makes BEST / BALANCED / KNEE / by-index / by-weights
 * move the highlighted point on both at once.
 */

import type { ReactNode } from "react";
import { Separator } from "@/components/ui/separator";
import SolutionSelectorPanel from "@/components/SolutionSelectorPanel";
import type { ExplorerSource } from "@/lib/source";
import type { ParetoFront } from "@/lib/types";

interface Props {
  source: ExplorerSource;
  front: ParetoFront;
  selectedIndex: number;
  onSelectIndex: (idx: number) => void;
  /** The route's own picker for which run this section is showing (the model
   *  page's combination select). Routes without one pass nothing. */
  combinationControls?: ReactNode;
}

export default function ParetoControls({
  source,
  front,
  selectedIndex,
  onSelectIndex,
  combinationControls,
}: Props) {
  return (
    <div className="flex flex-col gap-4">
      {combinationControls && (
        <>
          {combinationControls}
          <Separator />
        </>
      )}

      <SolutionSelectorPanel
        source={source}
        front={front}
        selectedIndex={selectedIndex}
        onSelectIndex={onSelectIndex}
      />
    </div>
  );
}
