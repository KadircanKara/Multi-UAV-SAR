"use client";

import { useCallback, useEffect, useRef, useState } from "react";

/** Nav height (h-14 = 56px). A section becomes active once its top clears it. */
const NAV_OFFSET_PX = 56;

/**
 * Tracks which of `ids` is the section currently being read, and which have been
 * on screen at least once.
 *
 * `seenIds` is the lazy-mount signal: Merging and Animation each run expensive
 * backend work, and stacking them as scroll sections would otherwise fire all of
 * it on page load. A section's content mounts on first intersection, not before.
 */
export function useScrollSpy(ids: string[]) {
  const [activeId, setActiveId] = useState<string>(ids[0] ?? "");
  const [seenIds, setSeenIds] = useState<Set<string>>(new Set());
  const elements = useRef(new Map<string, HTMLElement>());

  // The effect below must key off the SET of ids, not the array's identity —
  // a consumer that passes an inline literal re-creates `ids` every render.
  // `idsRef` carries the latest array in without being a reactive dependency;
  // `idsKey` is the primitive that actually decides when the observer is torn
  // down and rebuilt (react-hooks/exhaustive-deps requires deps to be simple
  // identifiers, not inline expressions like `ids.join("|")`).
  const idsRef = useRef(ids);
  idsRef.current = ids;
  const idsKey = ids.join("|");

  const register = useCallback(
    (id: string) => (el: HTMLElement | null) => {
      if (el) elements.current.set(id, el);
      else elements.current.delete(id);
    },
    []
  );

  useEffect(() => {
    const currentIds = idsRef.current;
    const nodes = currentIds
      .map((id) => elements.current.get(id))
      .filter((el): el is HTMLElement => Boolean(el));
    if (nodes.length === 0) return;

    const observer = new IntersectionObserver(
      (entries) => {
        const nowSeen: string[] = [];
        for (const entry of entries) {
          if (entry.isIntersecting) nowSeen.push(entry.target.id);
        }
        if (nowSeen.length > 0) {
          setSeenIds((prev) => {
            const next = new Set(prev);
            let changed = false;
            for (const id of nowSeen) if (!next.has(id)) { next.add(id); changed = true; }
            return changed ? next : prev;
          });
        }

        // Active = the topmost section whose top has passed under the nav.
        // Read positions live rather than trusting entry order, which is not
        // guaranteed to be document order.
        const passed = currentIds
          .map((id) => ({ id, el: elements.current.get(id) }))
          .filter((s): s is { id: string; el: HTMLElement } => Boolean(s.el))
          .filter((s) => s.el.getBoundingClientRect().top <= NAV_OFFSET_PX + 8);
        setActiveId(passed.length > 0 ? passed[passed.length - 1]!.id : currentIds[0] ?? "");
      },
      { rootMargin: `-${NAV_OFFSET_PX}px 0px -60% 0px`, threshold: [0, 0.01] }
    );

    for (const node of nodes) observer.observe(node);
    return () => observer.disconnect();
  }, [idsKey]);

  return { activeId, seenIds, register };
}
