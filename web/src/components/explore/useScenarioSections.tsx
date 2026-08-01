"use client";

/**
 * useScenarioSections — turns one ExplorerSource into the PARETO / MERGING /
 * ANIMATION sections that SectionPanelLayout renders.
 *
 * This is where the deep-dive's shared state lives, because the two halves of
 * a section are rendered in two different places: `controls` in the sticky
 * panel, `content` in the scrolling column. Anything both halves read — the
 * selected solution above all — therefore has to sit above both, which is
 * here. The five axis choices live here too: the dropdowns that write them
 * render on the charts, but the state is one level up so the panel can take
 * them over again without moving anything.
 *
 * `selectedIndex` is deliberately ONE value for all three sections: picking
 * BEST / BALANCED / KNEE / by-index / by-weights in the Pareto panel moves the
 * highlighted point on the 2D plot AND the 3D plot, and is the same solution
 * the Merging comparison and the Animation replay run on.
 *
 * Merging's controls/content are split into MergingControls (panel) +
 * MergingContent (column); its sensing config and the compare result that
 * config produces live here for the same reason `selectedIndex` does — both
 * halves read them (see the "Merging section state" block below). Animation
 * still carries its pre-refactor content and no controls of its own;
 * splitting it is the next task.
 */

import { useCallback, useEffect, useState, type ReactNode } from "react";
import dynamic from "next/dynamic";
import { toast } from "sonner";
import { sourceCompare, type ExplorerSource } from "@/lib/source";
import type { ParetoFront, SensingConfig } from "@/lib/types";
import type { PanelSection } from "@/components/layout/PanelSection";
import GridPlayback from "@/components/viz/GridPlayback/GridPlayback";
import type { BeliefRow } from "@/components/viz/BeliefEvolutionChart";
import type { CompareTableRow } from "@/components/viz/MergingMetricsTable";
import type { TargetsKnownRow } from "@/components/viz/TargetsKnownChart";
import ChartSkeleton from "./ChartSkeleton";
import MergingContent, { MERGING_HEIGHT } from "./MergingContent";
import MergingControls from "./MergingControls";
import ParetoControls from "./ParetoControls";
import ParetoFrontsCard, { has3DView } from "./ParetoFrontsCard";
import { useScenarioFront } from "./useScenarioFront";

// ─── Dynamic (SSR-off) chart imports ─────────────────────────────────────────

const ParetoScatter = dynamic(() => import("@/components/viz/ParetoScatter"), {
  ssr: false,
  loading: () => <ChartSkeleton height="h-72" />,
});

const ParetoScatter3D = dynamic(
  () => import("@/components/viz/ParetoScatter3D"),
  { ssr: false, loading: () => <ChartSkeleton height="h-96" /> }
);

// Reserved scroll height for a section's CONTENT COLUMN before it has mounted,
// px. Roughly what each measures once loaded on a desktop viewport, so the
// scrollbar doesn't jump as sections come in: the Pareto card is a header plus
// a ~384px chart row, and Animation is its config card plus the canvas.
// Merging's own number is declared in (and imported from) MergingContent.tsx
// instead of alongside these two — that file's empty/loading states use it as
// a real `minHeight` floor, not just a pre-mount estimate (see its comment).
const PARETO_HEIGHT = 560;
const ANIMATION_HEIGHT = 720;

// ─── Options / result ─────────────────────────────────────────────────────────

export interface ScenarioSectionsOptions {
  source: ExplorerSource;
  /** Notified once per loaded front — lets a parent build a back-link, etc. */
  onFrontLoaded?: (front: ParetoFront) => void;
  /** The route's own picker for which run these sections show; rendered at the
   *  top of the Pareto panel. Routes without one pass nothing. */
  combinationControls?: ReactNode;
  /** Extra content inside the Pareto card, below the plots. */
  paretoFooter?: ReactNode;
}

export interface ScenarioSections {
  front: ParetoFront | null;
  loading: boolean;
  error: string | null;
  /** Empty until the front loads — a section cannot be built without it. */
  sections: PanelSection[];
}

// ─── Hook ─────────────────────────────────────────────────────────────────────

export function useScenarioSections({
  source,
  onFrontLoaded,
  combinationControls,
  paretoFooter,
}: ScenarioSectionsOptions): ScenarioSections {
  const { front, loading, error } = useScenarioFront(source);

  // Shared across all three sections.
  const [selectedIndex, setSelectedIndex] = useState(0);

  // 2D Pareto axis choice. ParetoScatter renders the dropdowns that write it
  // (see its `xObj`/`yObj`/`hideAxisSelectors` props) but does not own it: the
  // seeding effect below has to reach it when the front changes, and a panel
  // could drive it instead without either chart changing.
  const [xObj, setXObj] = useState<string>("");
  const [yObj, setYObj] = useState<string>("");

  // 3D Pareto axis choice — INDEPENDENT of the 2D pair above, so a fixed 3D
  // view can be read against a changing 2D slice. Held here for the same
  // reason (see ParetoScatter3D's `xObj`/`yObj`/`zObj` props) and named with a
  // `3D` infix so no reader can mistake these for the 2D xObj/yObj.
  const [x3DObj, setX3DObj] = useState<string>("");
  const [y3DObj, setY3DObj] = useState<string>("");
  const [z3DObj, setZ3DObj] = useState<string>("");

  // Start each newly loaded front on its own first solution. This ran inside
  // the fetch callback before the fetch moved into its own hook; depending on
  // the front OBJECT reproduces it exactly — `front`'s identity changes once
  // per successful load and never otherwise — without the selection resetting
  // on unrelated re-renders. (One frame can paint the new front with the old
  // index, the same passive-effect lag the axis seeding below has.)
  useEffect(() => {
    if (front) setSelectedIndex(front.solutions[0]?.index ?? 0);
  }, [front]);

  // Notify the parent, likewise once per loaded front. Kept out of the fetch
  // effect so an unmemoised callback can no longer trigger a refetch loop; the
  // worst it can now do is re-notify with the same front.
  useEffect(() => {
    if (front) onFrontLoaded?.(front);
  }, [front, onFrontLoaded]);

  // Default axis values derived from the front's objective list. Plain consts
  // (not memoized) so the effect below can list the primitive values it
  // actually uses instead of `front` itself — exhaustive-deps then verifies
  // the dependency array for us instead of us silencing it.
  const defaultXObj = front?.objectives[0] ?? "";
  const defaultYObj = front?.objectives[1] ?? front?.objectives[0] ?? "";
  // The 3D chart only renders from 3 objectives up (see has3DView), and its
  // axes are independent of the 2D pair above, so each defaults straight off
  // its own index with no fallback chain — "" is a sane, inert default when
  // that index doesn't exist.
  const defaultX3DObj = front?.objectives[0] ?? "";
  const defaultY3DObj = front?.objectives[1] ?? "";
  const defaultZ3DObj = front?.objectives[2] ?? "";
  // Full-list fingerprint. The per-index defaults above alone only notice a
  // change to the entries they read — but a manually-selected axis can point
  // at an objective at any index, which can go stale (renamed/removed) while
  // the entries this effect reads stay the same. Depending on this too forces
  // a reseed on ANY change to the list, so none of the five axis values below
  // ever keep pointing at a key that's absent from the new front (a stale key
  // reads as `objectives_abs[key] ?? 0` and collapses every point to origin).
  const objectivesKey = front?.objectives.join("|") ?? "";

  // Seed the axes once the front arrives; the front can change under us when
  // the parent switches combination, so re-seed whenever the objective list
  // changes in any way (not just its first two entries), and not on every
  // render.
  useEffect(() => {
    setXObj(defaultXObj);
    setYObj(defaultYObj);
    setX3DObj(defaultX3DObj);
    setY3DObj(defaultY3DObj);
    setZ3DObj(defaultZ3DObj);
  }, [
    defaultXObj, defaultYObj,
    defaultX3DObj, defaultY3DObj, defaultZ3DObj,
    objectivesKey,
  ]);

  const handleSelectIndex = useCallback((idx: number) => {
    setSelectedIndex(idx);
  }, []);

  // ─── Merging section state ─────────────────────────────────────────────────
  // Not shared with Pareto or Animation (unlike selectedIndex and the axis
  // state above) — this is local to Merging, but split across the same
  // panel/column boundary as everything else here: MergingControls (panel)
  // writes the sensing config and triggers the compare, MergingContent
  // (column) reads the result, and `comparing` gates both a disabled/labelled
  // button in the former and a loading skeleton in the latter. Neither half
  // alone sees both ends, so the state sits here.
  const [timeModel, setTimeModel] = useState<"discrete" | "realtime">("discrete");
  const [detProb, setDetProb] = useState(0.8);
  const [faProb, setFaProb] = useState(0.1);
  const [beliefThresh, setBeliefThresh] = useState(0.9);
  const [targetsInput, setTargetsInput] = useState("12");

  const [comparing, setComparing] = useState(false);
  const [tableRows, setTableRows] = useState<CompareTableRow[] | null>(null);
  const [beliefRows, setBeliefRows] = useState<BeliefRow[] | null>(null);
  const [knownRows, setKnownRows] = useState<TargetsKnownRow[] | null>(null);

  const pqInvalid = detProb <= faProb;
  const targetList = targetsInput
    .split(",")
    .map((s) => parseInt(s.trim(), 10))
    .filter((n) => !isNaN(n));
  const canCompare = !pqInvalid && targetList.length > 0 && !comparing;

  async function runCompare() {
    if (!canCompare) return;
    setComparing(true);
    setTableRows(null);
    setBeliefRows(null);
    setKnownRows(null);

    const baseConfig = {
      time_model: timeModel,
      detection_prob: detProb,
      false_alarm_prob: faProb,
      belief_threshold: beliefThresh,
      target_locations: targetList,
    };

    const configs: SensingConfig[] = [
      { ...baseConfig, merge_topology: "none" },
      { ...baseConfig, merge_topology: "onboard" },
      { ...baseConfig, merge_topology: "gcs" },
    ];
    const labels = ["none", "onboard", "gcs"];

    try {
      const res = await sourceCompare(source, {
        index: selectedIndex,
        configs,
        labels,
        model_key: null,
      });

      // Parse compare response
      const rawTable = res.table as Record<string, string | number | null>[] | undefined;
      if (rawTable) {
        setTableRows(rawTable as CompareTableRow[]);
      }

      const rawRows = res.rows as Record<string, unknown>[] | undefined;
      if (rawRows) {
        const bRows: BeliefRow[] = rawRows.map((r, i) => ({
          label: labels[i] ?? `config-${i}`,
          cell_occupancy_probabilities: r.cell_occupancy_probabilities as number[][],
          target_locations: r.target_locations as number[],
          belief_threshold: r.belief_threshold as number,
        }));
        setBeliefRows(bRows);
        setKnownRows(bRows as TargetsKnownRow[]);
      }
    } catch (err: unknown) {
      const msg = err instanceof Error ? err.message : String(err);
      toast.error("Comparison failed", { description: msg });
    } finally {
      setComparing(false);
    }
  }

  const sections: PanelSection[] = front
    ? [
        {
          id: "pareto",
          label: "PARETO FRONT",
          estimatedHeight: PARETO_HEIGHT,
          controls: (
            <ParetoControls
              source={source}
              front={front}
              selectedIndex={selectedIndex}
              onSelectIndex={handleSelectIndex}
              combinationControls={combinationControls}
            />
          ),
          content: (
            <ParetoFrontsCard
              front={front}
              footer={paretoFooter}
              scatter2D={
                <ParetoScatter
                  front={front}
                  selectedIndex={selectedIndex}
                  onSelectIndex={handleSelectIndex}
                  xObj={xObj}
                  yObj={yObj}
                  onXChange={setXObj}
                  onYChange={setYObj}
                />
              }
              // Same predicate the card renders on; building the node early
              // would map over every solution for a plot that isn't shown.
              scatter3D={
                has3DView(front) ? (
                  <ParetoScatter3D
                    objectives={front.objectives}
                    points={front.solutions.map((s) => ({
                      index: s.index,
                      values: s.objectives_abs,
                    }))}
                    polarities={front.polarities}
                    selectedIndex={selectedIndex}
                    onSelectIndex={handleSelectIndex}
                    xObj={x3DObj}
                    yObj={y3DObj}
                    zObj={z3DObj}
                    onXChange={setX3DObj}
                    onYChange={setY3DObj}
                    onZChange={setZ3DObj}
                  />
                ) : null
              }
            />
          ),
        },
        {
          id: "merging",
          label: "MERGING",
          estimatedHeight: MERGING_HEIGHT,
          controls: (
            <MergingControls
              front={front}
              selectedIndex={selectedIndex}
              timeModel={timeModel}
              onTimeModelChange={setTimeModel}
              detProb={detProb}
              onDetProbChange={setDetProb}
              faProb={faProb}
              onFaProbChange={setFaProb}
              beliefThresh={beliefThresh}
              onBeliefThreshChange={setBeliefThresh}
              targetsInput={targetsInput}
              onTargetsInputChange={setTargetsInput}
              pqInvalid={pqInvalid}
              targetList={targetList}
              canCompare={canCompare}
              comparing={comparing}
              onCompare={runCompare}
            />
          ),
          content: (
            <MergingContent
              comparing={comparing}
              tableRows={tableRows}
              beliefRows={beliefRows}
              knownRows={knownRows}
            />
          ),
        },
        {
          id: "animation",
          label: "ANIMATION",
          estimatedHeight: ANIMATION_HEIGHT,
          controls: null,
          content: (
            <GridPlayback
              source={source}
              front={front}
              selectedIndex={selectedIndex}
            />
          ),
        },
      ]
    : [];

  return { front, loading, error, sections };
}
