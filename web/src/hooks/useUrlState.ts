"use client";

/**
 * Filter state in the address bar: read once on mount, written back on change.
 *
 * The flow is deliberately ONE-WAY after mount. The URL seeds React state at
 * first render and is thereafter an output — a projection of state, never an
 * input to it. A two-way binding would fight itself: our own `replace` updates
 * `useSearchParams`, which would feed back into state, which would write the
 * URL again. Since filter changes `replace` rather than `push` (there is no
 * filter history to walk back through), nothing can change the query string
 * behind our back, so there is nothing a listener would legitimately catch.
 *
 * Reading goes through `window.location.search` rather than `useSearchParams`
 * for a second reason: in the app router `useSearchParams` forces any page
 * that calls it into a Suspense boundary or client-side rendering at build
 * time. We only ever need the value once, on the client, so the plain browser
 * API is both sufficient and cheaper.
 *
 * Values equal to their default are omitted, so an untouched page keeps a
 * clean URL and a shared link carries only what was actually chosen.
 */

import { useEffect, useRef, useState } from "react";

/** The query string as it was when the page mounted, before any of our writes. */
export function useInitialSearchParams(): URLSearchParams {
  const [params] = useState(() =>
    typeof window === "undefined"
      ? new URLSearchParams()
      : new URLSearchParams(window.location.search)
  );
  return params;
}

// ─── Readers ──────────────────────────────────────────────────────────────────

export function readString(
  params: URLSearchParams,
  key: string,
  fallback: string
): string {
  return params.get(key) ?? fallback;
}

/** One of a known set, or the fallback — so a hand-edited URL cannot put the
 *  page into a state its own controls could never produce. */
export function readEnum<T extends string>(
  params: URLSearchParams,
  key: string,
  allowed: readonly T[],
  fallback: T
): T {
  const raw = params.get(key);
  return raw != null && (allowed as readonly string[]).includes(raw)
    ? (raw as T)
    : fallback;
}

/** Comma-separated list. Empty string means "explicitly none", which is not
 *  the same as absent — absent falls back, empty selects nothing. */
export function readList(
  params: URLSearchParams,
  key: string,
  fallback: string[]
): string[] {
  const raw = params.get(key);
  if (raw == null) return fallback;
  if (raw === "") return [];
  return raw.split(",").filter(Boolean);
}

export function readNumber(
  params: URLSearchParams,
  key: string,
  fallback: number
): number {
  const raw = params.get(key);
  if (raw == null) return fallback;
  const n = Number(raw);
  return Number.isFinite(n) ? n : fallback;
}

// ─── Writer ───────────────────────────────────────────────────────────────────

/**
 * Keep the query string in step with `values`. A key whose value is null or
 * undefined is dropped from the URL — callers pass null for "same as default".
 *
 * Uses `history.replaceState` rather than `router.replace`: the URL here is a
 * record of what the page is showing, not a navigation, and Next's router
 * would re-render the route tree on every filter toggle to reach the same
 * component with the same state. This changes the address bar and what a
 * reload or a copied link resolves to, which is the whole requirement.
 */
export function useUrlSync(values: Record<string, string | null | undefined>) {
  // Serialised here so the effect below depends on the CONTENT of `values`,
  // not on the fresh object identity a caller creates each render.
  const query = buildQuery(values);
  const lastWritten = useRef<string | null>(null);

  useEffect(() => {
    if (typeof window === "undefined") return;
    // Never write on the first pass: at that point `query` is just the state
    // we seeded FROM the URL, and rewriting it would only normalise key order
    // in the address bar for no reason.
    if (lastWritten.current === null) {
      lastWritten.current = query;
      return;
    }
    if (lastWritten.current === query) return;
    lastWritten.current = query;
    const next = query
      ? `${window.location.pathname}?${query}`
      : window.location.pathname;
    window.history.replaceState(window.history.state, "", next);
  }, [query]);
}

function buildQuery(values: Record<string, string | null | undefined>): string {
  const params = new URLSearchParams();
  // Sorted so the same filter state always produces the same string —
  // otherwise the effect above would fire on key reordering alone.
  for (const key of Object.keys(values).sort()) {
    const v = values[key];
    if (v == null) continue;
    params.set(key, v);
  }
  return params.toString();
}

/** A list for the URL: null when it matches the default, so the key is
 *  omitted rather than spelled out in full on an untouched page. */
export function listParam(
  value: string[],
  defaultValue: string[]
): string | null {
  const same =
    value.length === defaultValue.length &&
    value.every((v, i) => v === defaultValue[i]);
  return same ? null : value.join(",");
}

/** A scalar for the URL, omitted when it matches the default. */
export function scalarParam<T extends string | number>(
  value: T,
  defaultValue: T
): string | null {
  return value === defaultValue ? null : String(value);
}
