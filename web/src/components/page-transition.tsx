"use client";

import { usePathname } from "next/navigation";

/**
 * PageTransition — gives every route the landing's fade-rise entrance.
 *
 * Keyed by pathname so it remounts on each navigation and the CSS animation
 * replays. The landing ("/") runs its own staggered per-element entrance, so it
 * is excluded here to avoid double-animating.
 */
export function PageTransition({ children }: { children: React.ReactNode }) {
  const pathname = usePathname() ?? "/";
  const animate = pathname !== "/";
  return (
    <div key={pathname} className={animate ? "animate-page-enter" : undefined}>
      {children}
    </div>
  );
}
