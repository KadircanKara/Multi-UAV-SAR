import Link from "next/link";
import Image from "next/image";
import { cn } from "@/lib/utils";

/**
 * Landing page (route "/").
 *
 * Opens with a Motivation section that frames the research (multi-UAV search &
 * rescue, the conflicting objectives, and the non-obvious merit of optimising
 * time-between-visits), then presents the three interactive tools. Fully
 * token-based (light/dark), server-rendered, with the existing hud-rise entrance.
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

// ─── Inline citation → links to the references list ────────────────────────────

function Cite({ ids }: { ids: number[] }) {
  return (
    <sup className="ml-0.5 whitespace-nowrap text-[0.7em] font-medium text-chart-1">
      [
      {ids.map((id, i) => (
        <span key={id}>
          <a href={`#ref-${id}`} className="hover:underline">
            {id}
          </a>
          {i < ids.length - 1 ? ", " : ""}
        </span>
      ))}
      ]
    </sup>
  );
}

// ─── Static content ─────────────────────────────────────────────────────────────

const TENSIONS = [
  {
    label: "Coverage time",
    body: "Finding targets fast means spreading out and racing through the cells nobody has seen yet.",
  },
  {
    label: "Connectivity",
    body: "Sharing what each UAV senses means staying within multi-hop range of the GCS — which pulls the swarm back together.",
  },
  {
    label: "Revisit time",
    body: "Imperfect sensors miss targets, so high-uncertainty cells must be re-measured — and every revisit slows coverage.",
  },
];

const CONTRIBUTIONS = [
  "Two multi-objective formulations: one jointly optimising coverage time, connectivity to the GCS and UAV disconnectivity, the other adding cell revisit time.",
  "Custom genetic operators that drop into any genetic algorithm.",
  "Diverse Pareto fronts per scenario, so an operator can pick the solution that fits the mission.",
  "A benchmark of objective-optimal paths on cooperative search-and-inform time metrics.",
];

const REFERENCES: { id: number; text: string; thisWork?: boolean }[] = [
  { id: 1, text: "D. Kalafat, “Statistical evaluation of Turkey earthquake data (1900–2015): A case study,” Eastern Anatolian Journal of Science, vol. 2, no. 1, pp. 14–36, 2016." },
  { id: 2, text: "X. Ji, X. Wang, Y. Niu, and L. Shen, “Cooperative search by multiple unmanned aerial vehicles in a nonconvex environment,” Mathematical Problems in Engineering, vol. 2015, no. 1, p. 196730, 2015." },
  { id: 3, text: "A. Khan, E. Yanmaz, and B. Rinner, “Information exchange and decision making in micro aerial vehicle networks for cooperative search,” IEEE Transactions on Control of Network Systems, vol. 2, no. 4, pp. 335–347, 2015." },
  { id: 4, text: "S. Saha, A. E. Vasegaard, I. E. Nielsen, A. Hapka, and H. Budzisz, “UAVs path planning under a bi-objective optimization framework for smart cities,” Electronics, vol. 10, p. 1193, 2021." },
  { id: 5, text: "H. V. Nguyen, H. Rezatofighi, B.-N. Vo, and D. C. Ranasinghe, “Multi-objective multi-agent planning for jointly discovering and tracking mobile objects,” in AAAI Conf. on Artificial Intelligence, 2019." },
  { id: 6, text: "H. Ergezer and K. Leblebicioğlu, “Online path planning for unmanned aerial vehicles to maximize instantaneous information,” Intl. Journal of Advanced Robotic Systems, vol. 18, 2021." },
  { id: 7, text: "E. Yanmaz, H. M. Balanji, and İ. Güven, “Dynamic multi-UAV path planning for multi-target search and connectivity,” IEEE Transactions on Vehicular Technology, vol. 73, no. 7, pp. 10516–10528, 2024." },
  { id: 8, text: "Z. Liu, X. Gao, and X. Fu, “A cooperative search and coverage algorithm with controllable revisit and connectivity maintenance for multiple unmanned aerial vehicles,” Sensors, vol. 18, no. 5, p. 1472, 2018." },
  { id: 9, text: "S. Kazemdehbashi and Y. Liu, “An exact coverage path planning algorithm for UAV-based search and rescue operations,” arXiv preprint arXiv:2405.11399, 2024." },
  { id: 10, text: "P. Sujit and D. Ghose, “Search using multiple UAVs with flight time constraints,” IEEE Transactions on Aerospace and Electronic Systems, vol. 40, no. 2, pp. 491–509, 2004." },
  { id: 11, text: "J. Zheng, M. Ding, L. Sun, and H. Liu, “Distributed stochastic algorithm based on enhanced genetic algorithm for path planning of multi-UAV cooperative area search,” IEEE Transactions on Intelligent Transportation Systems, vol. 24, no. 8, pp. 8290–8303, 2023." },
  { id: 12, text: "W. Botes, “Grid-based coverage path planning for multiple UAVs in search and rescue applications,” PhD thesis, Stellenbosch University, 2023." },
  { id: 13, text: "A. Wolek, S. Cheng, D. Goswami, and D. A. Paley, “Cooperative mapping and target search over an unknown occupancy graph using mutual information,” IEEE Robotics and Automation Letters, vol. 5, no. 2, pp. 1071–1078, 2020." },
  { id: 14, text: "K. Kara and E. Yanmaz, “Joint optimization of connectivity, coverage, and revisit time in multi-UAV path planning,” in Proc. IEEE Vehicular Technology Conference (VTC), June 2025.", thisWork: true },
  { id: 15, text: "A. Khan, E. Yanmaz, and B. Rinner, “Information merging in multi-UAV cooperative search,” in IEEE International Conference on Robotics and Automation (ICRA), pp. 3122–3129, 2014." },
  { id: 16, text: "K. Kara, İ. Güven, and E. Yanmaz, “Cooperative multi-target search with UAV swarms: Evolutionary vs. reinforcement learning strategies,” in Proc. International Conference on Modeling, Analysis and Simulation of Wireless and Mobile Systems (MSWiM), Oct. 2025.", thisWork: true },
];

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
        <h3 className="text-xl font-semibold tracking-tight text-foreground">
          {title}
        </h3>
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
      {/* ── Motivation ─────────────────────────────────────────────────────── */}
      <section className="flex flex-col gap-7">
        <div className="flex flex-col gap-3">
          <span
            className="animate-hud-rise text-sm font-medium text-chart-1"
            style={{ animationDelay: "0ms" }}
          >
            Multi-UAV cooperative search · MSc thesis demo
          </span>
          <h1
            className="animate-hud-rise max-w-3xl text-balance text-4xl font-bold tracking-tight text-foreground md:text-5xl"
            style={{ animationDelay: "60ms" }}
          >
            Coordinated UAV swarms for disaster search &amp; rescue
          </h1>
          <p
            className="animate-hud-rise max-w-3xl text-[15px] leading-relaxed text-muted-foreground md:text-base"
            style={{ animationDelay: "120ms" }}
          >
            Natural disasters — earthquakes above all — strike Turkey more and
            more often.<Cite ids={[1]} /> When they do, searching the affected
            area quickly and reliably saves lives. A team of cooperating UAVs can
            sweep the ground far faster than responders on foot, stay connected to
            a ground control station (GCS), and improve detection reliability
            through redundancy and collaboration.<Cite ids={[2, 3]} />
          </p>
        </div>

        {/* Scenario figure */}
        <figure
          className="animate-hud-rise flex flex-col gap-3"
          style={{ animationDelay: "180ms" }}
        >
          <div className="overflow-hidden rounded-2xl border border-border bg-card">
            <Image
              src="/sar-scenario.png"
              alt="A cooperative multi-UAV search mission over a gridded disaster area: four UAVs sense cells and relay findings to a ground control station over multi-hop connectivity links; high-occupancy cells (points of interest) are revisited."
              width={2740}
              height={1536}
              priority
              sizes="(max-width: 1024px) 100vw, 1024px"
              className="h-auto w-full"
            />
          </div>
          <figcaption className="max-w-3xl text-sm leading-relaxed text-muted-foreground">
            A cooperative multi-UAV search mission. UAVs sweep a gridded disaster
            area and relay what they sense back to the GCS over multi-hop links.
            Cells with high occupancy probability — points of interest such as a
            fire or a survivor — are revisited until enough measurements confirm a
            target.
          </figcaption>
        </figure>

        {/* The core tension */}
        <div className="flex flex-col gap-4">
          <h2 className="text-xl font-semibold tracking-tight text-foreground">
            Three goals that pull against each other
          </h2>
          <p className="max-w-3xl text-[15px] leading-relaxed text-muted-foreground">
            Coordinating a swarm is a balancing act between conflicting
            requirements.<Cite ids={[7]} /> Push on one and the others give way:
          </p>
          <div className="grid grid-cols-1 gap-4 sm:grid-cols-3">
            {TENSIONS.map((t) => (
              <div
                key={t.label}
                className="flex flex-col gap-2 rounded-2xl border border-border bg-card p-5"
              >
                <div className="flex items-center gap-2">
                  <span className="size-1.5 rounded-full bg-chart-1" aria-hidden="true" />
                  <span className="text-sm font-semibold text-foreground">
                    {t.label}
                  </span>
                </div>
                <p className="text-sm leading-relaxed text-muted-foreground">
                  {t.body}
                </p>
              </div>
            ))}
          </div>
          <p className="max-w-3xl text-[15px] leading-relaxed text-muted-foreground">
            No single path plan wins on all three. We therefore produce a Pareto
            front of trade-offs and let the operator choose the plan that best
            fits the mission at hand.<Cite ids={[14]} />
          </p>
        </div>
      </section>

      {/* ── The non-obvious objective: time between visits ──────────────────── */}
      <section className="mt-12">
        <div className="flex flex-col gap-4 rounded-2xl border border-chart-1/30 bg-chart-1/5 p-7 md:p-9">
          <span className="text-sm font-medium text-chart-1">
            The non-obvious objective
          </span>
          <h2 className="max-w-3xl text-2xl font-semibold tracking-tight text-foreground">
            Why optimise the time between visits?
          </h2>
          <p className="max-w-3xl text-[15px] leading-relaxed text-muted-foreground">
            Real sensors are imperfect: a single pass over a cell can miss a
            target with non-negligible probability. The remedy is redundancy —
            visit each cell several times and fuse the measurements.<Cite ids={[3]} />{" "}
            But what really governs sensing quality is not <em>how many</em> times
            a cell is visited; it is <em>how much time</em> elapses between those
            visits. Long gaps let a cell&apos;s belief go stale and slow the
            swarm&apos;s convergence to a confident detection; shorter gaps keep
            every cell&apos;s information fresh and lift detection reliability — at
            the price of tighter, slower coverage.
          </p>
          <p className="max-w-3xl text-[15px] leading-relaxed text-muted-foreground">
            Most prior work fixes the number of revisits or ignores revisit
            timing altogether.<Cite ids={[8, 13]} /> Treating the maximum mean
            time between visits as a first-class optimisation objective — next to
            coverage time and connectivity — is the core novelty this work adds to
            the joint coverage–connectivity problem.<Cite ids={[14, 16]} /> It is a
            subtle lever, but a decisive one for how well a search mission actually
            finds what it is looking for.
          </p>
        </div>
      </section>

      {/* ── What this demo is built on ──────────────────────────────────────── */}
      <section className="mt-12 flex flex-col gap-4">
        <h2 className="text-xl font-semibold tracking-tight text-foreground">
          What this demo is built on
        </h2>
        <p className="max-w-3xl text-[15px] leading-relaxed text-muted-foreground">
          This interactive demo is the direct result of two publications from the
          underlying MSc thesis — a Vehicular Technology Conference paper<Cite ids={[14]} />{" "}
          and an MSWiM paper<Cite ids={[16]} /> — adopting the sensing models
          of<Cite ids={[3]} /> and the information-sharing paradigm of<Cite ids={[15]} />.
        </p>
        <ul className="flex max-w-3xl flex-col gap-2.5">
          {CONTRIBUTIONS.map((c) => (
            <li key={c} className="flex items-start gap-2.5 text-[15px] leading-relaxed text-foreground/80">
              <span className="mt-2 size-1.5 shrink-0 rounded-full bg-chart-1" aria-hidden="true" />
              {c}
            </li>
          ))}
        </ul>
      </section>

      {/* ── References ──────────────────────────────────────────────────────── */}
      <section className="mt-12 flex flex-col gap-4">
        <h2 className="text-xl font-semibold tracking-tight text-foreground">
          References
        </h2>
        <ol className="grid grid-cols-1 gap-x-8 gap-y-3 md:grid-cols-2">
          {REFERENCES.map((r) => (
            <li
              key={r.id}
              id={`ref-${r.id}`}
              className="scroll-mt-24 text-xs leading-relaxed text-muted-foreground"
            >
              <span className="font-medium text-foreground">[{r.id}]</span> {r.text}
              {r.thisWork && (
                <span className="ml-1.5 inline-flex items-center rounded-full border border-chart-1/40 bg-chart-1/10 px-1.5 py-0.5 text-[10px] font-medium text-chart-1">
                  This work
                </span>
              )}
            </li>
          ))}
        </ol>
      </section>

      {/* ── Divider ─────────────────────────────────────────────────────────── */}
      <div className="my-16 h-px w-full bg-border" aria-hidden="true" />

      {/* ── Explore the demo ────────────────────────────────────────────────── */}
      <div className="flex flex-col gap-3">
        <span className="text-sm font-medium text-chart-1">
          Interactive · Explainable optimiser
        </span>
        <h2 className="text-4xl font-bold tracking-tight text-foreground md:whitespace-nowrap md:text-5xl">
          Find the path that fits the mission.
        </h2>
        <p className="max-w-2xl text-[15px] leading-relaxed text-muted-foreground md:text-base">
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
          description="Configure and run your own optimization — then analyze any exported run right on the page."
          bullets={[
            "SOO & MOO (NSGA-II / NSGA-III / MOEA/D)",
            "Weighted-sum with custom weights",
            "Live progress + single-run analysis",
          ]}
          glyph={<OptimizeGlyph />}
          delay={280}
        />
        <DeckCard
          href="/compare"
          title="Model Comparison"
          description="Put models head-to-head across every objective and sensing time-metric — even objectives a model never optimised."
          bullets={[
            "Bar, line & table views",
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
