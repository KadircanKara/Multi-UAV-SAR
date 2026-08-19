"use client";

/**
 * useStagedFilters — filter state in two halves: what the reader is editing
 * (`draft`) and what the page is actually showing (`applied`).
 *
 * Controls bind to `draft`; everything a filter change COSTS — the fetch, the
 * recompute, the URL write — reads `applied`. Nothing moves until `apply()`.
 *
 * Why: on /compare every chip was its own POST /api/comparison, and a reader
 * working through the picker spent the endpoint's whole per-minute budget on
 * intermediate selections nobody asked to see. A debounce only narrowed that
 * window and made a deliberate single click feel laggy; staging removes the
 * intermediate requests entirely, because the reader says when they are done.
 *
 * `commit` exists for changes that are NOT the reader editing a filter:
 *   - seeding, once the library/grid that defines the options has loaded;
 *   - a structural change that redefines what the filters even mean (switching
 *     the swept dimension re-seeds its x-values — leaving those staged would
 *     show a Drones chart under a pending Comm axis).
 * It patches BOTH halves, so a structural change takes effect at once while
 * the reader's unrelated pending edits stay pending.
 */

import { useCallback, useMemo, useState } from "react";

export interface StagedFilters<T extends object> {
  /** what the controls show and edit */
  draft: T;
  /** what the page renders and fetches from */
  applied: T;
  dirty: boolean;
  /** how many fields differ — the Apply button says this out loud */
  changeCount: number;
  setDraft: React.Dispatch<React.SetStateAction<T>>;
  apply: () => void;
  /** patch both halves at once — seeding and structural changes only */
  commit: (patch: Partial<T>) => void;
}

// Filter values are string lists and scalars, so a serialised compare is both
// correct and cheaper than a bespoke deep-equal. Key ORDER is stable here:
// both sides come from the same object shape.
function sameValue(a: unknown, b: unknown): boolean {
  return a === b || JSON.stringify(a) === JSON.stringify(b);
}

export function useStagedFilters<T extends object>(initial: T): StagedFilters<T> {
  const [draft, setDraft] = useState<T>(initial);
  const [applied, setApplied] = useState<T>(initial);

  const apply = useCallback(() => setApplied(draft), [draft]);

  const commit = useCallback((patch: Partial<T>) => {
    setDraft((d) => ({ ...d, ...patch }));
    setApplied((a) => ({ ...a, ...patch }));
  }, []);

  const changeCount = useMemo(() => {
    let n = 0;
    for (const key of Object.keys(draft) as (keyof T)[]) {
      if (!sameValue(draft[key], applied[key])) n++;
    }
    return n;
  }, [draft, applied]);

  return {
    draft,
    applied,
    dirty: changeCount > 0,
    changeCount,
    setDraft,
    apply,
    commit,
  };
}
