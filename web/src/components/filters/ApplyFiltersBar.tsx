"use client";

/**
 * The Apply button for a staged filter panel (see useStagedFilters).
 *
 * Two states, both carried by the label and the disabled attribute together:
 * pending edits ("Apply filters", enabled) or none ("Filters applied",
 * disabled). The panel needs no separate clean/dirty badge.
 */

import { Button } from "@/components/ui/button";

export default function ApplyFiltersBar({
  dirty,
  onApply,
  className,
}: {
  dirty: boolean;
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
      {dirty ? "Apply filters" : "Filters applied"}
    </Button>
  );
}
