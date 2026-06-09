import type { Metadata } from "next";
import { JetBrains_Mono, Orbitron } from "next/font/google";
import "./globals.css";
import { Separator } from "@/components/ui/separator";
import { Toaster } from "@/components/ui/sonner";
import { TooltipProvider } from "@/components/ui/tooltip";
import { cn } from "@/lib/utils";

const jetbrainsMono = JetBrains_Mono({
  subsets: ["latin"],
  variable: "--font-mono",
  weight: ["100", "200", "300", "400", "500", "600", "700", "800"],
  display: "swap",
});

const orbitron = Orbitron({
  subsets: ["latin"],
  variable: "--font-display",
  weight: ["400", "500", "600", "700", "800", "900"],
  display: "swap",
});

export const metadata: Metadata = {
  title: "Multi-UAV SAR · OPS Console",
  description:
    "Tactical mission control interface for Multi-UAV Search-and-Rescue optimisation.",
};

export default function RootLayout({
  children,
}: Readonly<{ children: React.ReactNode }>) {
  return (
    <html
      lang="en"
      className={cn("dark", jetbrainsMono.variable, orbitron.variable)}
    >
      <body
        className={cn(
          "min-h-screen bg-background text-foreground font-mono antialiased bg-grid"
        )}
      >
        {/* Scanline CRT overlay */}
        <div className="scanline" aria-hidden="true" />

        <TooltipProvider>
          {/* ── OPS CONSOLE top bar ────────────────────────────────────── */}
          <header className="relative z-50 flex h-11 items-center justify-between px-4 bg-card border-b border-border">
            {/* Left: brand + live indicator */}
            <div className="flex items-center gap-3">
              <span
                className="font-display text-sm font-semibold tracking-widest text-primary uppercase"
                style={{ fontFamily: "var(--font-display)" }}
              >
                MULTI-UAV SAR
              </span>
              <span className="text-muted-foreground text-xs tracking-widest select-none">
                ·
              </span>
              <span
                className="font-display text-xs font-medium tracking-widest text-muted-foreground uppercase"
                style={{ fontFamily: "var(--font-display)" }}
              >
                OPS CONSOLE
              </span>
              <span className="flex items-center gap-1.5 ml-2">
                <span
                  className="size-2 rounded-full bg-accent animate-pulse-dot"
                  aria-hidden="true"
                />
                <span className="text-xs text-accent tracking-widest font-semibold">
                  ● LIVE
                </span>
              </span>
            </div>

            {/* Right: mono readout slot */}
            <div className="text-xs text-muted-foreground tracking-wider font-mono tabular-nums">
              SYS:NOMINAL
            </div>
          </header>

          <Separator />

          {/* ── Page content ──────────────────────────────────────────── */}
          <main className="relative z-10">{children}</main>
        </TooltipProvider>

        <Toaster />
      </body>
    </html>
  );
}
