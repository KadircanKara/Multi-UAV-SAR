"use client";

/**
 * GridPlayback — orchestrator component for the ANIMATION tab.
 * Manages:
 *   - Config panel (sensing config inputs)
 *   - Fetch state (loading / error / loaded)
 *   - Canvas + playback controls (play/pause, scrubber, speed, labels toggle)
 *
 * The canvas is loaded via next/dynamic({ ssr: false }) because it touches
 * the window/canvas APIs.
 */

import { useState, useRef, useCallback } from "react";
import { toast } from "sonner";
import { sourcePlayback, type ExplorerSource } from "@/lib/source";
import type { PlaybackPayload, SensingConfig, ParetoFront } from "@/lib/types";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Slider } from "@/components/ui/slider";
import { Skeleton } from "@/components/ui/skeleton";
import { Separator } from "@/components/ui/separator";
import { ToggleGroup, ToggleGroupItem } from "@/components/ui/toggle-group";
import { usePlaybackColors } from "./usePlaybackColors";
// Import the canvas directly (NOT via next/dynamic): next/dynamic returns a
// function-component wrapper that does NOT forward refs, which left the
// imperative GridCanvasHandle ref null (so play/pause/scrub did nothing).
// GridCanvas is SSR-safe — it only touches window/canvas inside effects.
import GridCanvas, { type GridCanvasHandle } from "./GridCanvas";

// ─── Types ────────────────────────────────────────────────────────────────────

interface Props {
  source: ExplorerSource;
  front: ParetoFront | null;
  selectedIndex: number;
}

// ─── Slider field (reusable within this file) ─────────────────────────────────

function SliderField({
  label,
  value,
  onChange,
  min,
  max,
  step,
  hint,
}: {
  label: string;
  value: number;
  onChange: (v: number) => void;
  min: number;
  max: number;
  step: number;
  hint?: string;
}) {
  return (
    <div className="flex flex-col gap-2">
      <div className="flex justify-between items-center">
        <Label className="text-xs text-muted-foreground tracking-widest uppercase font-mono">
          {label}
        </Label>
        <span className="text-xs font-mono tabular-nums text-primary">
          {value.toFixed(2)}
        </span>
      </div>
      <Slider
        min={min}
        max={max}
        step={step}
        value={[value]}
        onValueChange={([v]) => onChange(v!)}
      />
      {hint && (
        <p className="text-xs text-destructive font-mono">{hint}</p>
      )}
    </div>
  );
}

// ─── Speed options ─────────────────────────────────────────────────────────────

const SPEED_OPTIONS: { label: string; value: number }[] = [
  { label: "0.1×",   value: 0.1   },
  { label: "0.175×", value: 0.175 },
  { label: "0.25×",  value: 0.25  },
  { label: "0.5×",  value: 0.5  },
  { label: "1×",    value: 1    },
  { label: "2×",    value: 2    },
  { label: "4×",    value: 4    },
];

// ─── GridPlayback component ───────────────────────────────────────────────────

export default function GridPlayback({ source, front, selectedIndex }: Props) {
  const colors = usePlaybackColors();

  // ── Config state ──────────────────────────────────────────────────────────
  const [mergeTopology, setMergeTopology] = useState<"none" | "onboard" | "gcs">("none");
  const [timeModel, setTimeModel] = useState<"discrete" | "realtime">("discrete");
  const [detProb, setDetProb] = useState(0.8);
  const [faProb, setFaProb] = useState(0.1);
  const [beliefThresh, setBeliefThresh] = useState(0.9);
  const [targetsInput, setTargetsInput] = useState("12");
  const [stride, setStride] = useState(1);

  // ── Load state ────────────────────────────────────────────────────────────
  const [loadingPayload, setLoadingPayload] = useState(false);
  const [payload, setPayload] = useState<PlaybackPayload | null>(null);

  // ── Playback UI state (kept small; canvas reads frameRef directly) ────────
  const [displayStep, setDisplayStep] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [speedMultiplier, setSpeedMultiplier] = useState(0.5);
  const [showAllLabels, setShowAllLabels] = useState(false);

  const canvasHandle = useRef<GridCanvasHandle>(null);

  // ── Validation ────────────────────────────────────────────────────────────
  const pqInvalid = detProb <= faProb;
  const targetList = targetsInput
    .split(",")
    .map((s) => parseInt(s.trim(), 10))
    .filter((n) => !isNaN(n) && n >= 0);
  const canLoad = !pqInvalid && targetList.length > 0 && !loadingPayload;

  // ── Load handler ──────────────────────────────────────────────────────────
  const handleLoad = useCallback(async () => {
    if (!canLoad) return;
    setLoadingPayload(true);
    setPayload(null);
    setDisplayStep(0);
    setPlaying(false);

    const config: SensingConfig = {
      merge_topology: mergeTopology,
      time_model: timeModel,
      detection_prob: detProb,
      false_alarm_prob: faProb,
      belief_threshold: beliefThresh,
      target_locations: targetList,
    };

    try {
      const raw = await sourcePlayback(source, {
        model_key: front?.model_key ?? null,
        index: selectedIndex,
        config,
        stride,
      });

      // Cast the open record to our typed payload
      setPayload(raw as unknown as PlaybackPayload);
    } catch (err: unknown) {
      const msg = err instanceof Error ? err.message : String(err);
      toast.error("Couldn't load playback", { description: msg });
    } finally {
      setLoadingPayload(false);
    }
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [canLoad, source, front, selectedIndex, mergeTopology, timeModel,
      detProb, faProb, beliefThresh, targetsInput, stride]);

  // ── Playback controls ─────────────────────────────────────────────────────
  const handlePlayPause = useCallback(() => {
    const handle = canvasHandle.current;
    if (!handle) return;
    if (playing) {
      handle.pause();
    } else {
      handle.play();
    }
  }, [playing]);

  const handleScrub = useCallback((values: number[]) => {
    const step = values[0] ?? 0;
    setDisplayStep(step);
    canvasHandle.current?.seekTo(step);
  }, []);

  // Called (throttled) by canvas — updates slider/readout without re-rendering canvas
  const handleFrameChange = useCallback((step: number, isPlaying: boolean) => {
    setDisplayStep(step);
    setPlaying(isPlaying);
  }, []);

  const totalSteps = payload?.steps ?? 0;
  const targetsKnown = payload?.targets_known[displayStep] ?? 0;
  const totalTargets = payload?.targets.length ?? 0;

  return (
    <div className="flex flex-col gap-6">
      {/* ── Config card ──────────────────────────────────────────────────── */}
      <Card>
        <CardHeader>
          <CardTitle
            className="text-xs font-semibold tracking-widest uppercase text-primary font-display"
            style={{ fontFamily: "var(--font-display)" }}
          >
            ANIMATION CONFIG
          </CardTitle>
        </CardHeader>
        <CardContent className="flex flex-col gap-5">
          {/* Merge topology */}
          <div className="flex flex-col gap-2">
            <Label className="text-xs text-muted-foreground tracking-widest uppercase font-mono">
              MERGE TOPOLOGY
            </Label>
            <ToggleGroup
              type="single"
              value={mergeTopology}
              onValueChange={(v) => {
                if (v === "none" || v === "onboard" || v === "gcs") setMergeTopology(v);
              }}
              className="justify-start gap-2"
            >
              {(["none", "onboard", "gcs"] as const).map((t) => (
                <ToggleGroupItem
                  key={t}
                  value={t}
                  className="h-7 text-xs font-mono tracking-widest uppercase"
                >
                  {t.toUpperCase()}
                </ToggleGroupItem>
              ))}
            </ToggleGroup>
          </div>

          {/* Time model */}
          <div className="flex flex-col gap-2">
            <Label className="text-xs text-muted-foreground tracking-widest uppercase font-mono">
              TIME MODEL
            </Label>
            <ToggleGroup
              type="single"
              value={timeModel}
              onValueChange={(v) => {
                if (v === "discrete" || v === "realtime") {
                  setTimeModel(v);
                  // Suggest higher stride for realtime
                  if (v === "realtime" && stride < 10) setStride(10);
                  if (v === "discrete") setStride(1);
                }
              }}
              className="justify-start gap-2"
            >
              <ToggleGroupItem value="discrete" className="h-7 text-xs font-mono tracking-widest uppercase">
                DISCRETE
              </ToggleGroupItem>
              <ToggleGroupItem value="realtime" className="h-7 text-xs font-mono tracking-widest uppercase">
                REALTIME
              </ToggleGroupItem>
            </ToggleGroup>
            {timeModel === "realtime" && (
              <p className="text-xs text-muted-foreground font-mono">
                REALTIME CAN PRODUCE 1000+ STEPS — USE STRIDE ≥ 10
              </p>
            )}
          </div>

          {/* Detection prob */}
          <SliderField
            label="DETECTION PROB (p)"
            value={detProb}
            onChange={setDetProb}
            min={0.01}
            max={0.99}
            step={0.01}
          />

          {/* False alarm prob */}
          <SliderField
            label="FALSE ALARM PROB (q)"
            value={faProb}
            onChange={setFaProb}
            min={0.01}
            max={0.99}
            step={0.01}
            hint={pqInvalid ? "⚠ REQUIRES p > q — adjust sliders" : undefined}
          />

          {/* Belief threshold */}
          <SliderField
            label="BELIEF THRESHOLD (B)"
            value={beliefThresh}
            onChange={setBeliefThresh}
            min={0.01}
            max={0.99}
            step={0.01}
          />

          {/* Target cells */}
          <div className="flex flex-col gap-2">
            <Label className="text-xs text-muted-foreground tracking-widest uppercase font-mono">
              TARGET CELLS (COMMA-SEPARATED)
            </Label>
            <Input
              value={targetsInput}
              onChange={(e) => setTargetsInput(e.target.value)}
              placeholder="e.g. 12,34,56"
              className="h-7 text-xs font-mono"
            />
            {targetList.length === 0 && (
              <p className="text-xs text-destructive font-mono">
                ENTER AT LEAST ONE VALID CELL INDEX
              </p>
            )}
          </div>

          {/* Stride */}
          <div className="flex flex-col gap-2">
            <div className="flex justify-between items-center">
              <Label className="text-xs text-muted-foreground tracking-widest uppercase font-mono">
                STRIDE
              </Label>
              <span className="text-xs font-mono tabular-nums text-primary">{stride}</span>
            </div>
            <Slider
              min={1}
              max={50}
              step={1}
              value={[stride]}
              onValueChange={([v]) => setStride(v!)}
            />
            <p className="text-xs text-muted-foreground font-mono">
              STRIDE {stride} — EVERY {stride}TH STEP RETURNED
            </p>
          </div>

          <Separator />

          <Button
            onClick={handleLoad}
            disabled={!canLoad}
            size="sm"
            className="w-full text-xs tracking-widest font-mono font-semibold"
          >
            {loadingPayload ? "LOADING ANIMATION…" : "LOAD ANIMATION"}
          </Button>

          <p className="text-xs text-muted-foreground font-mono">
            SOLUTION INDEX {selectedIndex}
            {front?.model_key && ` — MODEL ${front.model_key}`}
          </p>
        </CardContent>
      </Card>

      {/* ── Loading skeleton ──────────────────────────────────────────────── */}
      {loadingPayload && (
        <div className="flex flex-col gap-3">
          <Skeleton className="h-6 w-48" />
          <Skeleton className="aspect-square w-full max-h-[560px]" />
          <Skeleton className="h-8 w-full" />
        </div>
      )}

      {/* ── Canvas + controls ─────────────────────────────────────────────── */}
      {!loadingPayload && payload && (
        <Card>
          <CardHeader>
            <CardTitle
              className="text-xs font-semibold tracking-widest uppercase text-primary font-display flex items-center justify-between"
              style={{ fontFamily: "var(--font-display)" }}
            >
              <span>MISSION PLAYBACK</span>
              <span className="font-mono tabular-nums text-muted-foreground font-normal text-xs">
                {payload.time_model.toUpperCase()} · {payload.merge_topology.toUpperCase()} · STRIDE {payload.stride}
              </span>
            </CardTitle>
          </CardHeader>
          <CardContent className="flex flex-col gap-4">
            {/* Canvas area */}
            <div
              className="w-full rounded border border-border overflow-hidden bg-card"
              style={{ aspectRatio: "1 / 1", maxHeight: 560, minHeight: 300 }}
            >
              <GridCanvas
                key={`${payload.scenario}-${payload.index}-${payload.time_model}-${payload.merge_topology}-${payload.stride}`}
                payload={payload}
                colors={colors}
                showAllBeliefLabels={showAllLabels}
                speedMultiplier={speedMultiplier}
                onFrameChange={handleFrameChange}
                ref={canvasHandle}
              />
            </div>

            {/* Timeline scrubber */}
            <div className="flex flex-col gap-2">
              <div className="flex justify-between items-center">
                <span className="text-xs font-mono text-muted-foreground tabular-nums">
                  STEP {displayStep} / {totalSteps - 1}
                </span>
                <span className="text-xs font-mono text-primary tabular-nums">
                  FOUND {targetsKnown} / {totalTargets}
                </span>
              </div>
              <Slider
                min={0}
                max={Math.max(0, totalSteps - 1)}
                step={1}
                value={[displayStep]}
                onValueChange={handleScrub}
              />
            </div>

            {/* Controls row */}
            <div className="flex flex-wrap items-center gap-3">
              {/* Play/Pause */}
              <Button
                size="sm"
                variant="outline"
                onClick={handlePlayPause}
                className="h-8 min-w-20 text-xs font-mono tracking-widest"
              >
                {playing ? "⏸ PAUSE" : "▶ PLAY"}
              </Button>

              {/* Speed */}
              <div className="flex items-center gap-2">
                <span className="text-xs text-muted-foreground font-mono">SPEED</span>
                <ToggleGroup
                  type="single"
                  value={String(speedMultiplier)}
                  onValueChange={(v) => {
                    const n = parseFloat(v);
                    if (!isNaN(n)) setSpeedMultiplier(n);
                  }}
                  className="gap-1"
                >
                  {SPEED_OPTIONS.map(({ label, value }) => (
                    <ToggleGroupItem
                      key={label}
                      value={String(value)}
                      className="h-7 px-2 text-xs font-mono"
                    >
                      {label}
                    </ToggleGroupItem>
                  ))}
                </ToggleGroup>
              </div>

              {/* Show all labels toggle */}
              <Button
                size="sm"
                variant={showAllLabels ? "default" : "outline"}
                onClick={() => setShowAllLabels((v) => !v)}
                className="h-7 text-xs font-mono tracking-widest"
              >
                {showAllLabels ? "ALL LABELS ON" : "ALL LABELS OFF"}
              </Button>
            </div>

            {/* Payload info row */}
            <p className="text-xs text-muted-foreground font-mono">
              {payload.grid_size}×{payload.grid_size} GRID · {payload.number_of_nodes - 1} DRONES · {totalSteps} STEPS ·{" "}
              {payload.targets.length} TARGETS · THRESHOLD {payload.belief_threshold.toFixed(2)}
            </p>
          </CardContent>
        </Card>
      )}
    </div>
  );
}
