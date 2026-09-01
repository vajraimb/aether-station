import { useEffect, useMemo, useRef, useState } from "react";
import { Link } from "@tanstack/react-router";
import { Pause, Play, Repeat, RotateCcw, SkipForward } from "lucide-react";
import { PHASE_LABEL, isLunarPhase, type Phase } from "../../domains/cislunar/constants";
import { buildCislunarMission, sampleAt } from "../../domains/cislunar/trajectory";
import { CislunarCanvas, CraftInset } from "@/viz/CislunarCanvas";
import type { CameraMode } from "@/viz/cislunar-types";

const AUTO_WARP: Record<Phase, number> = {
  leo: 70,
  tli: 20,
  coast: 14000,
  loi: 40,
  llo: 2500,
  revolution: 120000,
};

const WARPS: { label: string; value: number | "auto" }[] = [
  { label: "Auto", value: "auto" },
  { label: "1k×", value: 1_000 },
  { label: "10k×", value: 10_000 },
  { label: "100k×", value: 100_000 },
  { label: "250k×", value: 250_000 },
];

function fmtTime(t: number): string {
  const d = Math.floor(t / 86400);
  const h = Math.floor((t % 86400) / 3600);
  const m = Math.floor((t % 3600) / 60);
  if (d > 0) return `${d}d ${h}h ${m}m`;
  return `${h}h ${m}m`;
}

function Chip({
  label,
  value,
  tone = "muted",
}: {
  label: string;
  value: string;
  tone?: "muted" | "ok" | "warn";
}) {
  const c = tone === "ok" ? "text-ok" : tone === "warn" ? "text-warn" : "text-fg";
  return (
    <div className="rounded-lg bg-bg-subtle px-3 py-2 shadow-[var(--shadow-border)]">
      <div className="text-2xs font-medium uppercase tracking-[0.14em] text-fg-subtle">{label}</div>
      <div className={`mt-0.5 font-mono text-sm tabular-nums ${c}`}>{value}</div>
    </div>
  );
}

function Btn({
  children,
  onClick,
  active,
  title,
}: {
  children: React.ReactNode;
  onClick?: () => void;
  active?: boolean;
  title?: string;
}) {
  return (
    <button
      type="button"
      title={title}
      onClick={onClick}
      className={
        "inline-flex h-11 min-w-11 items-center justify-center gap-1.5 rounded-lg px-3 text-sm font-medium transition-colors duration-[var(--motion-quick)] " +
        (active ? "bg-accent text-accent-fg" : "bg-bg-subtle text-fg hover:bg-bg-hover")
      }
    >
      {children}
    </button>
  );
}

const CAMERAS: { id: CameraMode; label: string }[] = [
  { id: "overview", label: "System" },
  { id: "earth", label: "Earth" },
  { id: "moon", label: "Moon" },
  { id: "craft", label: "Craft" },
];

export function CislunarApp() {
  const mission = useMemo(() => buildCislunarMission(), []);
  const [briefing, setBriefing] = useState(true);
  const [playing, setPlaying] = useState(false);
  const [loop, setLoop] = useState(true);
  const [warp, setWarp] = useState<number | "auto">("auto");
  const [t, setT] = useState(0);
  const [mode, setMode] = useState<CameraMode>("earth");
  const prevPhase = useRef<Phase | null>(null);

  useEffect(() => {
    if (!playing || briefing) return;
    let last = performance.now();
    let raf = 0;
    const loopFrame = (now: number) => {
      const dt = Math.min(0.1, (now - last) / 1000);
      last = now;
      setT((prev) => {
        const nowSample = sampleAt(mission, prev);
        const rate = warp === "auto" ? AUTO_WARP[nowSample.phase] : warp;
        let next = prev + dt * rate;
        if (next >= mission.duration) {
          if (loop) return next % mission.duration;
          setPlaying(false);
          return mission.duration;
        }
        return next;
      });
      raf = requestAnimationFrame(loopFrame);
    };
    raf = requestAnimationFrame(loopFrame);
    return () => cancelAnimationFrame(raf);
  }, [playing, warp, briefing, mission, loop]);

  const sample = useMemo(() => sampleAt(mission, t), [mission, t]);
  const progress = t / mission.duration;
  const phase: Phase = sample.phase;
  const moonDeg =
    sample.t <= mission.tCapture
      ? 0
      : Math.min(360, ((sample.t - mission.tCapture) / mission.periodMoon) * 360);

  useEffect(() => {
    const prev = prevPhase.current;
    prevPhase.current = phase;
    if (prev === phase || prev === null) return;
    if (phase === "llo" && (mode === "earth" || mode === "overview")) setMode("moon");
    if (phase === "revolution") setMode("overview");
  }, [phase, mode]);

  const start = () => {
    setBriefing(false);
    setPlaying(true);
  };

  const skipPhase = () => {
    const next = mission.phases.find((p) => p.t0 > t + 1);
    setT(next ? next.t0 : mission.duration);
  };

  return (
    <main className="flex min-h-dvh flex-col bg-bg text-fg">
      <header className="flex flex-wrap items-center gap-3 border-b border-border px-4 py-3">
        <div className="min-w-0 flex-1">
          <div className="text-2xs font-medium uppercase tracking-[0.22em] text-fg-subtle">Cislunar flight</div>
          <h1 className="font-display text-xl font-semibold tracking-tight">Earth → Moon</h1>
        </div>
        <nav className="flex gap-1">
          <Link
            to="/station"
            className="inline-flex h-11 items-center rounded-lg px-3 text-sm text-fg-muted hover:bg-bg-hover hover:text-fg"
          >
            Station
          </Link>
          <span className="inline-flex h-11 items-center rounded-lg bg-bg-subtle px-3 text-sm text-fg shadow-[var(--shadow-border)]">
            Cislunar
          </span>
        </nav>
      </header>

      <section className="relative min-h-0 flex-1">
        <div className="absolute inset-0 touch-none">
          <CislunarCanvas mission={mission} sample={sample} mode={mode} />
        </div>

        <div className="pointer-events-none absolute right-3 top-3 z-10 w-40 sm:w-52">
          <div className="pointer-events-auto overflow-hidden rounded-xl bg-bg-elevated/90 shadow-[var(--shadow-border)] backdrop-blur-sm">
            <div className="h-40 sm:h-52">
              <CraftInset sample={sample} />
            </div>
            <div className="px-2 py-1.5 text-center text-2xs uppercase tracking-[0.14em] text-fg-subtle">
              Attitude · drag / pinch
            </div>
          </div>
        </div>

        {briefing && (
          <div className="pointer-events-none absolute inset-x-0 bottom-0 z-10 flex items-end p-4 sm:inset-0 sm:items-center sm:justify-center">
            <div className="pointer-events-auto w-full max-w-lg rounded-xl bg-bg-elevated/95 p-5 shadow-[var(--shadow-border)] sm:p-6">
              <div className="text-2xs font-medium uppercase tracking-[0.18em] text-fg-subtle">
                Patched-conic visualization
              </div>
              <h2 className="mt-1 text-2xl font-semibold tracking-tight">Earth to lunar orbit</h2>
              <p className="mt-3 text-sm leading-relaxed text-fg-muted">
                LEO parking, trans-lunar injection, capture into a 100 km lunar orbit, then
                the probe stays in LLO while the Moon completes one full revolution around
                Earth. Drag to orbit, scroll to zoom. System view shows the whole circuit.
              </p>
              <ul className="mt-4 space-y-1.5 font-mono text-xs text-fg-muted">
                <li>TLI Δv ≈ {mission.dvTli.toFixed(2)} km/s</li>
                <li>Coast {(mission.tofCoast / 86400).toFixed(2)} days · Moon 384,400 km</li>
                <li>LLO 100 km · {(mission.periodLlo / 3600).toFixed(1)} h period</li>
                <li>Moon around Earth {(mission.periodMoon / 86400).toFixed(2)} days</li>
              </ul>
              <div className="mt-5">
                <Btn onClick={start} active>
                  Launch
                </Btn>
              </div>
            </div>
          </div>
        )}

        {!briefing && (
          <div className="pointer-events-none absolute inset-x-0 bottom-0 p-3 sm:p-4">
            <div className="pointer-events-auto mx-auto flex max-w-3xl flex-col gap-3 rounded-xl bg-bg-elevated/90 p-3 shadow-[var(--shadow-border)] backdrop-blur-sm">
              <div className="h-1 overflow-hidden rounded-full bg-bg-subtle">
                <div className="h-full bg-accent" style={{ width: `${Math.min(100, progress * 100)}%` }} />
              </div>
              <div className="grid grid-cols-2 gap-2 sm:grid-cols-4">
                <Chip label="Phase" value={PHASE_LABEL[phase]} tone="ok" />
                <Chip label="Mission time" value={fmtTime(sample.t)} />
                <Chip
                  label={isLunarPhase(phase) ? "Moon altitude" : "Earth altitude"}
                  value={
                    (isLunarPhase(phase) ? sample.altMoon : sample.altEarth) > 2000
                      ? `${((isLunarPhase(phase) ? sample.altMoon : sample.altEarth) / 1000).toFixed(0)} Mm`
                      : `${(isLunarPhase(phase) ? sample.altMoon : sample.altEarth).toFixed(0)} km`
                  }
                />
                <Chip
                  label={phase === "revolution" ? "Moon orbit" : "Speed"}
                  value={
                    phase === "revolution" ? `${moonDeg.toFixed(0)}°` : `${sample.speed.toFixed(2)} km/s`
                  }
                  tone={phase === "revolution" ? "ok" : "muted"}
                />
              </div>
              <div className="flex flex-wrap items-center gap-2">
                <Btn onClick={() => setPlaying((p) => !p)} active={playing} title={playing ? "Pause" : "Play"}>
                  {playing ? <Pause className="size-4" /> : <Play className="size-4" />}
                </Btn>
                <Btn
                  onClick={() => {
                    setT(0);
                    setPlaying(false);
                  }}
                  title="Restart"
                >
                  <RotateCcw className="size-4" />
                </Btn>
                <Btn onClick={skipPhase} title="Next phase">
                  <SkipForward className="size-4" />
                </Btn>
                <Btn onClick={() => setLoop((v) => !v)} active={loop} title="Loop">
                  <Repeat className="size-4" />
                </Btn>
                <div className="mx-1 h-6 w-px bg-border" />
                {WARPS.map((w) => (
                  <Btn key={w.label} onClick={() => setWarp(w.value)} active={warp === w.value}>
                    {w.label}
                  </Btn>
                ))}
              </div>
              <div className="flex flex-wrap items-center gap-2">
                {CAMERAS.map((c) => (
                  <Btn key={c.id} onClick={() => setMode(c.id)} active={mode === c.id}>
                    {c.label}
                  </Btn>
                ))}
                <span className="px-1 text-2xs uppercase tracking-[0.12em] text-fg-subtle">
                  drag · scroll to zoom
                </span>
              </div>
            </div>
          </div>
        )}
      </section>
    </main>
  );
}
