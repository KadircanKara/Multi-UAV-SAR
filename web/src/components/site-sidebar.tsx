"use client";

/**
 * App navigation. A labelled sidebar at `lg` and above, a slim top bar with a
 * drawer below it.
 *
 * The page container is capped at `max-w-7xl`, so on a wide screen it already
 * floats with dead margin either side; a 224px sidebar takes that margin
 * rather than any content width. Content only narrows below ~1504px
 * (224 + 1280), and below `lg` the sidebar is not rendered at all.
 *
 * Typography is the sans stack throughout, like the rest of the app chrome.
 * The mono/uppercase vocabulary belongs to the mission and explorer panels,
 * where it labels data; borrowing it for navigation put two typefaces in one
 * 224px column for no reason.
 *
 * The breakpoint MUST stay in step with SectionPanelLayout's DESKTOP_QUERY:
 * that file decides how far below the viewport top its sticky panel pins, and
 * the answer is "under the top bar" only while the top bar exists. Both switch
 * at 1024px.
 */

import { useState } from "react";
import Link from "next/link";
import { usePathname } from "next/navigation";
import { Menu } from "lucide-react";
import { cn } from "@/lib/utils";
import { Button } from "@/components/ui/button";
import {
  Sheet,
  SheetContent,
  SheetHeader,
  SheetTitle,
  SheetTrigger,
} from "@/components/ui/sheet";
import { ThemeToggle } from "./theme-toggle";

// ─── Inline icons (currentColor, no icon dependency) ──────────────────────────

function DroneIcon() {
  return (
    <svg
      viewBox="0 0 24 24"
      className="size-[18px]"
      fill="none"
      stroke="currentColor"
      strokeWidth={1.6}
      strokeLinecap="round"
      strokeLinejoin="round"
      aria-hidden="true"
    >
      {/* propellers */}
      <line x1="2.5" y1="6" x2="9.5" y2="6" />
      <line x1="14.5" y1="6" x2="21.5" y2="6" />
      {/* arms / motor stems */}
      <line x1="6" y1="6" x2="7.5" y2="10" />
      <line x1="18" y1="6" x2="16.5" y2="10" />
      {/* body */}
      <rect x="7" y="10" width="10" height="3.4" rx="1.7" />
      {/* gimbal neck + camera */}
      <line x1="12" y1="13.4" x2="12" y2="14.3" />
      <circle cx="12" cy="16" r="1.7" />
    </svg>
  );
}

function MissionsIcon() {
  return (
    <svg viewBox="0 0 24 24" className="size-4" fill="none" stroke="currentColor" strokeWidth={1.8} strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
      <rect x="3" y="3" width="7" height="7" rx="1.5" />
      <rect x="14" y="3" width="7" height="7" rx="1.5" />
      <rect x="3" y="14" width="7" height="7" rx="1.5" />
      <rect x="14" y="14" width="7" height="7" rx="1.5" />
    </svg>
  );
}

function OptimizeIcon() {
  return (
    <svg viewBox="0 0 24 24" className="size-4" fill="none" stroke="currentColor" strokeWidth={1.8} strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
      <line x1="4" y1="6" x2="14" y2="6" />
      <line x1="18" y1="6" x2="20" y2="6" />
      <circle cx="16" cy="6" r="2" />
      <line x1="4" y1="12" x2="8" y2="12" />
      <line x1="12" y1="12" x2="20" y2="12" />
      <circle cx="10" cy="12" r="2" />
      <line x1="4" y1="18" x2="14" y2="18" />
      <line x1="18" y1="18" x2="20" y2="18" />
      <circle cx="16" cy="18" r="2" />
    </svg>
  );
}

function CompareIcon() {
  return (
    <svg viewBox="0 0 24 24" className="size-4" fill="none" stroke="currentColor" strokeWidth={1.8} strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
      <line x1="6" y1="20" x2="6" y2="12" />
      <line x1="12" y1="20" x2="12" y2="4" />
      <line x1="18" y1="20" x2="18" y2="9" />
    </svg>
  );
}

function AnalysisIcon() {
  return (
    <svg viewBox="0 0 24 24" className="size-4" fill="none" stroke="currentColor" strokeWidth={1.8} strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
      <circle cx="11" cy="11" r="6" />
      <line x1="15.5" y1="15.5" x2="20" y2="20" />
      <line x1="8.5" y1="12.5" x2="10.5" y2="10" />
      <line x1="10.5" y1="10" x2="12.5" y2="12" />
      <line x1="12.5" y1="12" x2="14" y2="8.5" />
    </svg>
  );
}

/**
 * Grouped by where the data comes from, which is the distinction that actually
 * governs what a reader can do on a page: the seeded library is precomputed
 * and browsable, whereas a run is something they produce in the session and
 * that is not persisted server-side.
 */
const NAV_GROUPS: {
  label: string;
  items: {
    href: string;
    label: string;
    icon: React.ReactNode;
    match: string[];
  }[];
}[] = [
  {
    label: "Seeded Results",
    items: [
      { href: "/missions", label: "Missions", icon: <MissionsIcon />, match: ["/missions", "/explore"] },
      { href: "/compare", label: "Compare", icon: <CompareIcon />, match: ["/compare"] },
    ],
  },
  {
    // PLACEHOLDER — the owner has not settled on this heading yet. Changing
    // the string is the whole change; nothing keys off it.
    label: "Your Runs",
    items: [
      { href: "/optimize", label: "Optimizer", icon: <OptimizeIcon />, match: ["/optimize"] },
      { href: "/analysis", label: "Analysis", icon: <AnalysisIcon />, match: ["/analysis"] },
    ],
  },
];

function isActive(pathname: string, match: string[]): boolean {
  return match.some((m) => pathname === m || pathname.startsWith(m + "/"));
}

function Wordmark() {
  return (
    <Link href="/" className="flex items-center gap-2.5">
      <span className="grid size-7 shrink-0 place-items-center rounded-lg bg-foreground text-background">
        <DroneIcon />
      </span>
      <span className="font-display text-[15px] font-semibold tracking-tight text-foreground">
        Multi-UAV SAR
      </span>
    </Link>
  );
}

function NavLinks({
  pathname,
  onNavigate,
}: {
  pathname: string;
  onNavigate?: () => void;
}) {
  return (
    <nav className="flex flex-col gap-5">
      {NAV_GROUPS.map((group) => (
        <div key={group.label} className="flex flex-col gap-1">
          <p className="px-3 pb-1 text-[11px] font-semibold uppercase tracking-wider text-muted-foreground">
            {group.label}
          </p>
          {group.items.map((n) => {
            const active = isActive(pathname, n.match);
            return (
              <Link
                key={n.href}
                href={n.href}
                onClick={onNavigate}
                aria-current={active ? "page" : undefined}
                className={cn(
                  "flex items-center gap-2.5 rounded-lg px-3 py-2 text-sm font-medium transition-colors",
                  active
                    ? "bg-secondary text-foreground"
                    : "text-muted-foreground hover:bg-accent hover:text-foreground"
                )}
              >
                {n.icon}
                {n.label}
              </Link>
            );
          })}
        </div>
      ))}
    </nav>
  );
}

export function SiteSidebar() {
  const pathname = usePathname() ?? "/";
  // Controlled so following a link closes the drawer; a plain <Sheet> would
  // leave it open over the page the reader just asked for.
  const [open, setOpen] = useState(false);

  return (
    <>
      {/* Desktop: fixed full-height sidebar. Fixed rather than sticky so it
          never scrolls with the page and needs no height bookkeeping. */}
      <aside className="fixed inset-y-0 left-0 z-50 hidden w-56 flex-col border-r border-border bg-background px-3 py-4 lg:flex">
        <div className="px-1">
          <Wordmark />
        </div>
        <div className="mt-6 flex-1">
          <NavLinks pathname={pathname} />
        </div>
        <div className="flex items-center justify-between border-t border-border px-1 pt-3">
          <span className="text-[11px] font-semibold uppercase tracking-wider text-muted-foreground">
            Theme
          </span>
          <ThemeToggle />
        </div>
      </aside>

      {/* Below lg: the sidebar is gone, so the same links live in a drawer
          behind a top bar. Everything that pins to the viewport top on these
          pages offsets by this bar's height at this width and by nothing at
          lg — see SectionPanelLayout. */}
      <header className="sticky top-0 z-50 flex h-14 items-center justify-between border-b border-border bg-background/80 px-4 backdrop-blur-md lg:hidden">
        <div className="flex items-center gap-3">
          <Sheet open={open} onOpenChange={setOpen}>
            <SheetTrigger asChild>
              <Button variant="ghost" size="icon" aria-label="Open navigation">
                <Menu className="size-5" aria-hidden="true" />
              </Button>
            </SheetTrigger>
            <SheetContent side="left" className="w-[min(80vw,16rem)]">
              <SheetHeader>
                <SheetTitle className="text-left">
                  <Wordmark />
                </SheetTitle>
              </SheetHeader>
              <div className="mt-6">
                <NavLinks pathname={pathname} onNavigate={() => setOpen(false)} />
              </div>
            </SheetContent>
          </Sheet>
          <Wordmark />
        </div>
        <ThemeToggle />
      </header>
    </>
  );
}
