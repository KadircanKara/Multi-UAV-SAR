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
 *
 * `combinationControls` renders whether or not a front exists, and stays in
 * the same position either way. That is deliberate on both counts: a run whose
 * front failed to load is precisely when the reader needs the picker in order
 * to leave it, and moving the picker between two parents as the front resolves
 * would remount it mid-interaction.
 */

import type { ReactNode } from "react";
import { Separator } from "@/components/ui/separator";
import SolutionSelectorPanel from "@/components/SolutionSelectorPanel";
import { sourceKey, type ExplorerSource } from "@/lib/source";
import type { ParetoFront } from "@/lib/types";

interface Props {
  source: ExplorerSource;
  /** Null while the front is loading or after it failed — the picker below
   *  still renders, the solution selector does not. */
  front: ParetoFront | null;
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

      {front && (
        // Keyed on the run, not the front: this panel seeds weight sliders and
        // an index input from the front it mounted with and has no effect that
        // re-seeds them, so switching combination has to give it a new
        // instance or it keeps offering the previous run's weights.
        <SolutionSelectorPanel
          key={sourceKey(source)}
          source={source}
          front={front}
          selectedIndex={selectedIndex}
          onSelectIndex={onSelectIndex}
        />
      )}
    </div>
  );
}
