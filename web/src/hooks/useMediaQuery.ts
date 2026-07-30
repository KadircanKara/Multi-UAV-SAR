"use client";

import { useCallback, useSyncExternalStore } from "react";

/**
 * useMediaQuery — hydration-safe `matchMedia` boolean, backed by
 * useSyncExternalStore rather than useState+useEffect.
 *
 * Why useSyncExternalStore: a useState+useEffect version renders once with
 * whatever default you seed it with, paints, THEN corrects itself in an
 * effect after mount — a visible flash whenever the guess was wrong.
 * useSyncExternalStore instead reconciles the server/client snapshot
 * mismatch before the browser paints, so callers (e.g. SectionPanelLayout
 * choosing between a desktop aside and a mobile Sheet) never render the
 * wrong branch for a frame.
 *
 * Server snapshot is a fixed `false` (module-scope, so it's the exact same
 * value AND the exact same function reference on every call —
 * useSyncExternalStore requires that stability and warns/throws if the
 * "server" snapshot is instead recomputed per call). `false` (narrow) was
 * chosen as the safer default of the two: it renders the small Sheet
 * trigger button first, not a 320px desktop panel that would then have to
 * disappear once the client learns the real viewport is narrow.
 */

const getServerSnapshot = () => false;

export function useMediaQuery(query: string): boolean {
  const subscribe = useCallback(
    (onChange: () => void) => {
      const mql = window.matchMedia(query);
      mql.addEventListener("change", onChange);
      return () => mql.removeEventListener("change", onChange);
    },
    [query]
  );

  const getSnapshot = useCallback(() => window.matchMedia(query).matches, [query]);

  return useSyncExternalStore(subscribe, getSnapshot, getServerSnapshot);
}
