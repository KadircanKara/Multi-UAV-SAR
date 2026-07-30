"use client";

/**
 * ParetoControls — everything that drives the Pareto section, rendered in the
 * sticky panel (or its narrow-viewport drawer): the route's own combination
 * picker, the axis pickers for both plots, and the solution selector.
 *
 * The 2D and 3D plots have INDEPENDENT axes — five dropdowns in two labelled
 * blocks, never a shared pair — so a fixed 3D view can be read against a
 * changing 2D slice. The selection they highlight is shared: one
 * `selectedIndex` behind both plots, which is what makes BEST / BALANCED /
 * KNEE / by-index / by-weights below move the point on both at once.
 */

import type { ReactNode } from "react";
import { Separator } from "@/components/ui/separator";
import ObjectiveAxisSelect from "@/components/viz/ObjectiveAxisSelect";
import SolutionSelectorPanel from "@/components/SolutionSelectorPanel";
import type { ExplorerSource } from "@/lib/source";
import type { ParetoFront } from "@/lib/types";
import { has2DView, has3DView } from "./ParetoFrontsCard";

/** The five axis choices behind the two plots, bundled so this component's
 *  signature stays readable. Owned by useScenarioSections. */
export interface ParetoAxes {
  xObj: string;
  yObj: string;
  onXChange: (objective: string) => void;
  onYChange: (objective: string) => void;
  x3DObj: string;
  y3DObj: string;
  z3DObj: string;
  onX3DChange: (objective: string) => void;
  onY3DChange: (objective: string) => void;
  onZ3DChange: (objective: string) => void;
}

interface Props {
  source: ExplorerSource;
  front: ParetoFront;
  selectedIndex: number;
  onSelectIndex: (idx: number) => void;
  axes: ParetoAxes;
  /** The route's own picker for which run this section is showing (the model
   *  page's combination select). Routes without one pass nothing. */
  combinationControls?: ReactNode;
}

function BlockLabel({ children }: { children: ReactNode }) {
  return (
    <p className="text-xs text-muted-foreground tracking-widest uppercase font-mono">
      {children}
    </p>
  );
}

export default function ParetoControls({
  source,
  front,
  selectedIndex,
  onSelectIndex,
  axes,
  combinationControls,
}: Props) {
  const show2D = has2DView(front);
  const show3D = has3DView(front);

  return (
    <div className="flex flex-col gap-4">
      {combinationControls && (
        <>
          {combinationControls}
          <Separator />
        </>
      )}

      {show2D && (
        <div className="flex flex-col gap-2">
          <BlockLabel>2D AXES</BlockLabel>
          <ObjectiveAxisSelect
            label="X:"
            value={axes.xObj}
            onChange={axes.onXChange}
            objectives={front.objectives}
            polarities={front.polarities}
          />
          <ObjectiveAxisSelect
            label="Y:"
            value={axes.yObj}
            onChange={axes.onYChange}
            objectives={front.objectives}
            polarities={front.polarities}
          />
        </div>
      )}

      {show3D && (
        <div className="flex flex-col gap-2 rounded-md border border-border px-3 py-2">
          <BlockLabel>3D AXES</BlockLabel>
          <ObjectiveAxisSelect
            label="X:"
            value={axes.x3DObj}
            onChange={axes.onX3DChange}
            objectives={front.objectives}
            polarities={front.polarities}
          />
          <ObjectiveAxisSelect
            label="Y:"
            value={axes.y3DObj}
            onChange={axes.onY3DChange}
            objectives={front.objectives}
            polarities={front.polarities}
          />
          <ObjectiveAxisSelect
            label="Z:"
            value={axes.z3DObj}
            onChange={axes.onZ3DChange}
            objectives={front.objectives}
            polarities={front.polarities}
          />
        </div>
      )}

      {(show2D || show3D) && <Separator />}

      <SolutionSelectorPanel
        source={source}
        front={front}
        selectedIndex={selectedIndex}
        onSelectIndex={onSelectIndex}
      />
    </div>
  );
}
