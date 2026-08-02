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
 *
 * `extraOffsetPx` is however much page chrome pins BELOW the nav — every route
 * using this pins its own identity header there. It has to be accounted for in
 * both places or the two disagree: the panel would switch to a section whose
 * heading is still hidden behind that header.
 *
 * The two outputs are driven by DIFFERENT mechanisms, deliberately.
 * `seenIds` is edge-triggered and an IntersectionObserver is exactly right for
 * it. `activeId` is not: it is a question about the current scroll position,
 * asked continuously. Computing it inside the observer callback — as this once
 * did — only answers it at threshold crossings, and the sections are separated
 * by a 40px gap, so when one section left the observation band the next one's
 * top was still below the header and no further callback was due. The panel
 * then sat on the previous section for most of a screen of scrolling, which is
 * precisely the bug it exists to prevent.
 */
export function useScrollSpy(ids: string[], extraOffsetPx = 0) {
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

  // Rounded so a sub-pixel ResizeObserver reading cannot rebuild the observer
  // on every scroll frame.
  const offset = NAV_OFFSET_PX + Math.round(extraOffsetPx);

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
        if (nowSeen.length === 0) return;
        setSeenIds((prev) => {
          const next = new Set(prev);
          let changed = false;
          for (const id of nowSeen) if (!next.has(id)) { next.add(id); changed = true; }
          return changed ? next : prev;
        });
      },
      { rootMargin: `-${offset}px 0px -60% 0px`, threshold: [0, 0.01] }
    );

    for (const node of nodes) observer.observe(node);
    return () => observer.disconnect();
  }, [idsKey, offset]);

  // Active = the last section whose top has passed under the header. Recomputed
  // on scroll (coalesced to one read per frame) rather than on intersection, so
  // it is right at every scroll position and not only at the boundaries.
  useEffect(() => {
    let frame: number | null = null;

    const recompute = () => {
      frame = null;
      const currentIds = idsRef.current;
      let active = currentIds[0] ?? "";
      for (const id of currentIds) {
        const el = elements.current.get(id);
        if (el && el.getBoundingClientRect().top <= offset + 8) active = id;
      }
      setActiveId((prev) => (prev === active ? prev : active));
    };

    const onScroll = () => {
      if (frame == null) frame = requestAnimationFrame(recompute);
    };

    // Sections mount lazily and change height as they do, which moves every
    // section below them — so re-read on layout changes, not just on scroll.
    const resize = new ResizeObserver(onScroll);
    for (const id of idsRef.current) {
      const el = elements.current.get(id);
      if (el) resize.observe(el);
    }

    window.addEventListener("scroll", onScroll, { passive: true });
    window.addEventListener("resize", onScroll);
    recompute();

    return () => {
      if (frame != null) cancelAnimationFrame(frame);
      resize.disconnect();
      window.removeEventListener("scroll", onScroll);
      window.removeEventListener("resize", onScroll);
    };
  }, [idsKey, offset]);

  return { activeId, seenIds, register };
}
