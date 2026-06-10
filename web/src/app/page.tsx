import Link from "next/link";
import { cn } from "@/lib/utils";

/**
 * Landing page (route "/").
 *
 * Clean two-card layout: a bold heading + subtitle, then Mission Select and
 * Model Comparison as large, friendly cards. Fully token-based so it adapts to
 * light/dark. Subtle staggered fade-rise entrance (globals: hud-rise).
 */

// ─── Card glyphs ──────────────────────────────────────────────────────────────

function MissionsGlyph() {
  return (
    <svg viewBox="0 0 24 24" className="size-5" fill="none" stroke="currentColor" strokeWidth={1.7} strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
      <rect x="3" y="3" width="7" height="7" rx="1.5" />
      <rect x="14" y="3" width="7" height="7" rx="1.5" />
      <rect x="3" y="14" width="7" height="7" rx="1.5" />
      <rect x="14" y="14" width="7" height="7" rx="1.5" />
    </svg>
  );
}

function CompareGlyph() {
  return (
    <svg viewBox="0 0 24 24" className="size-5" fill="none" stroke="currentColor" strokeWidth={1.7} strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
      <line x1="6" y1="20" x2="6" y2="12" />
      <line x1="12" y1="20" x2="12" y2="4" />
      <line x1="18" y1="20" x2="18" y2="9" />
    </svg>
  );
}

function OptimizeGlyph() {
  return (
    <svg viewBox="0 0 24 24" className="size-5" fill="none" stroke="currentColor" strokeWidth={1.7} strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
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

function ArrowIcon() {
  return (
    <svg viewBox="0 0 24 24" className="size-5" fill="none" stroke="currentColor" strokeWidth={1.8} strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
      <line x1="5" y1="12" x2="19" y2="12" />
      <polyline points="12 5 19 12 12 19" />
    </svg>
  );
}

// ─── Deck card ────────────────────────────────────────────────────────────────

interface CardProps {
  href: string;
  title: string;
  description: string;
  bullets: string[];
  glyph: React.ReactNode;
  delay: number;
}

function DeckCard({ href, title, description, bullets, glyph, delay }: CardProps) {
  return (
    <Link
      href={href}
      style={{ animationDelay: `${delay}ms` }}
      className={cn(
        "group animate-hud-rise relative flex flex-col gap-5 rounded-2xl border border-border bg-card p-7",
        "transition-all duration-200 hover:-translate-y-0.5 hover:border-foreground/20 hover:shadow-lg hover:shadow-foreground/5"
      )}
    >
      <div className="flex items-start justify-between">
        <span className="grid size-11 place-items-center rounded-xl bg-muted text-foreground">
          {glyph}
        </span>
        <span className="text-muted-foreground transition-all duration-200 group-hover:translate-x-0.5 group-hover:text-foreground">
          <ArrowIcon />
        </span>
      </div>

      <div className="flex flex-col gap-2">
        <h2 className="text-xl font-semibold tracking-tight text-foreground">
          {title}
        </h2>
        <p className="text-[15px] leading-relaxed text-muted-foreground">
          {description}
        </p>
      </div>

      <ul className="flex flex-col gap-2 pt-1">
        {bullets.map((b) => (
          <li key={b} className="flex items-center gap-2.5 text-sm text-foreground/80">
            <span className="size-1.5 shrink-0 rounded-full bg-chart-1" aria-hidden="true" />
            {b}
          </li>
        ))}
      </ul>
    </Link>
  );
}

// ─── Page ─────────────────────────────────────────────────────────────────────

export default function LandingPage() {
  return (
    <div className="mx-auto max-w-5xl px-6 py-16 md:py-24">
      <div className="flex flex-col gap-3">
        <span
          className="animate-hud-rise text-sm font-medium text-chart-1"
          style={{ animationDelay: "0ms" }}
        >
          Interactive · Explainable optimiser
        </span>
        <h1
          className="animate-hud-rise max-w-2xl text-balance text-4xl font-bold tracking-tight text-foreground md:text-5xl"
          style={{ animationDelay: "60ms" }}
        >
          Two views. One mission.
        </h1>
        <p
          className="animate-hud-rise max-w-2xl text-[15px] leading-relaxed text-muted-foreground md:text-base"
          style={{ animationDelay: "120ms" }}
        >
          Browse and analyse 20 search-and-rescue path-optimisation models, or
          compare them head-to-head across objectives and sensing time-metrics —
          with Pareto fronts, belief-merging analysis, and live mission playback.
        </p>
      </div>

      <div className="mt-12 grid grid-cols-1 gap-5 md:grid-cols-3">
        <DeckCard
          href="/missions"
          title="Mission Select"
          description="Pick a model to explore its parameter sweeps, trade-offs, merging strategies, and live mission animations."
          bullets={[
            "Parameter-effect analysis",
            "Pareto front & solution selection",
            "Belief-merging & mission playback",
          ]}
          glyph={<MissionsGlyph />}
          delay={200}
        />
        <DeckCard
          href="/optimize"
          title="Optimizer"
          description="Configure and run your own optimization — pick the objectives, method, and algorithm, then watch it solve."
          bullets={[
            "SOO & MOO (NSGA-II / NSGA-III)",
            "Weighted-sum with custom weights",
            "Live generation progress",
          ]}
          glyph={<OptimizeGlyph />}
          delay={280}
        />
        <DeckCard
          href="/compare"
          title="Model Comparison"
          description="Put models head-to-head across every objective and sensing time-metric — even objectives a model never optimised."
          bullets={[
            "Bar, line, radar & table views",
            "Objective & time-metric comparison",
            "Cross-model, cross-parameter",
          ]}
          glyph={<CompareGlyph />}
          delay={360}
        />
      </div>
    </div>
  );
}
