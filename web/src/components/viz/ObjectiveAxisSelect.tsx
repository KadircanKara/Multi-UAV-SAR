"use client";

/** One labelled objective dropdown. Shared by the 2D and 3D axis pickers so the
 *  panel and the charts cannot drift in wording or sizing. */

import {
  Select, SelectContent, SelectItem, SelectTrigger, SelectValue,
} from "@/components/ui/select";

interface Props {
  label: string;
  value: string;
  onChange: (objective: string) => void;
  objectives: string[];
  polarities: Record<string, number>;
}

export default function ObjectiveAxisSelect({
  label, value, onChange, objectives, polarities,
}: Props) {
  return (
    <div className="flex items-center gap-2">
      <span className="text-xs text-muted-foreground tracking-widest font-mono">
        {label}
      </span>
      <Select value={value} onValueChange={onChange}>
        <SelectTrigger className="h-7 w-48 text-xs font-mono">
          <SelectValue />
        </SelectTrigger>
        <SelectContent>
          {objectives.map((obj) => (
            <SelectItem key={obj} value={obj} className="text-xs font-mono">
              {obj}
              {polarities[obj] === -1 && (
                <span className="ml-1 text-muted-foreground">(max)</span>
              )}
            </SelectItem>
          ))}
        </SelectContent>
      </Select>
    </div>
  );
}
