"use client";

import { useTheme } from "next-themes";
import { Toaster as Sonner } from "sonner";

type ToasterProps = React.ComponentProps<typeof Sonner>;

const Toaster = ({ ...props }: ToasterProps) => {
  // Follow the app's active theme (next-themes, class strategy). Hardcoding a
  // theme here leaves sonner's internal defaults mismatched with the page — in
  // light mode that made the (unclassed) toast title render near-white on the
  // white popover. resolvedTheme collapses "system" to light/dark.
  const { resolvedTheme } = useTheme();

  return (
    <Sonner
      theme={(resolvedTheme as ToasterProps["theme"]) ?? "light"}
      className="toaster group"
      toastOptions={{
        classNames: {
          toast:
            "group toast group-[.toaster]:bg-popover group-[.toaster]:text-popover-foreground group-[.toaster]:border-border group-[.toaster]:shadow-lg",
          // Title: pin to the high-contrast popover foreground so it never
          // depends on sonner's theme-derived default color.
          title: "group-[.toast]:text-popover-foreground group-[.toast]:font-semibold",
          // Description carries the actual error/status detail, so keep it
          // clearly legible (≈9.6:1 on the light popover) rather than muted.
          description: "group-[.toast]:text-popover-foreground/80",
          actionButton:
            "group-[.toast]:bg-primary group-[.toast]:text-primary-foreground",
          cancelButton:
            "group-[.toast]:bg-muted group-[.toast]:text-muted-foreground",
        },
      }}
      {...props}
    />
  );
};

export { Toaster };
