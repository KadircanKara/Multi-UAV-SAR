"use client";

/**
 * The Apply button for a staged filter panel (see useStagedFilters).
 *
 * Disabled when nothing is pending — the disabled state IS the "you are
 * looking at your own selection" signal, so the panel needs no separate
 * clean/dirty badge. When something is pending it says how much, because a
 * reader who toggled four chips and scrolled away needs to know the charts
 * below are still the old selection.
 */

import { Button } from "@/components/ui/button";

export default function ApplyFiltersBar({
  dirty,
  changeCount,
  onApply,
  className,
}: {
  dirty: boolean;
  changeCount: number;
  onApply: () => void;
  className?: string;
}) {
  return (
    <Button
      type="button"
      size="sm"
      onClick={onApply}
      disabled={!dirty}
      className={className}
    >
      {dirty
        ? `Apply filters · ${changeCount} change${changeCount === 1 ? "" : "s"}`
        : "Filters applied"}
    </Button>
  );
}
