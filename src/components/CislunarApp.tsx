import { useEffect, useMemo, useState } from "react";
import { Link } from "@tanstack/react-router";
import { Pause, Play, RotateCcw, SkipForward } from "lucide-react";
import { PHASE_LABEL, type Phase } from "../../domains/cislunar/constants";
import { buildCislunarMission, sampleAt } from "../../domains/cislunar/trajectory";
import { CislunarCanvas } from "@/viz/CislunarCanvas";
import type { CameraMode } from "@/viz/cislunar-types";

const AUTO_WARP: Record<Phase, number> = {
  leo: 70,
  tli: 20,
  coast: 14000,
  loi: 40,
  llo: 220,
};

const WARPS: { label: string; value: number | "auto" }[] = [
  { label: "Auto", value: "auto" },
  { label: "100×", value: 100 },
  { label: "1k×", value: 1_000 },
  { label: "10k×", value: 10_000 },
  { label: "50k×", value: 50_000 },
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
  { id: "cinematic", label: "Cinematic" },
  { id: "follow", label: "Follow" },
  { id: "earth", label: "Earth" },
  { id: "moon", label: "Moon" },
  { id: "overview", label: "Overview" },
  { id: "free", label: "Free" },
];

export function CislunarApp() {
  const mission = useMemo(() => buildCislunarMission(), []);
  const [briefing, setBriefing] = useState(true);
  const [playing, setPlaying] = useState(false);
  const [warp, setWarp] = useState<number | "auto">("auto");
  const [t, setT] = useState(0);
  const [mode, setMode] = useState<CameraMode>("cinematic");

  useEffect(() => {
    if (!playing || briefing) return;
    let last = performance.now();
    let raf = 0;
    const loop = (now: number) => {
      const dt = Math.min(0.1, (now - last) / 1000);
      last = now;
      setT((prev) => {
        const nowSample = sampleAt(mission, prev);
        const rate = warp === "auto" ? AUTO_WARP[nowSample.phase] : warp;
        const next = prev + dt * rate;
        if (next >= mission.duration) {
          setPlaying(false);
          return mission.duration;
        }
        return next;
      });
      raf = requestAnimationFrame(loop);
    };
    raf = requestAnimationFrame(loop);
    return () => cancelAnimationFrame(raf);
  }, [playing, warp, briefing, mission]);

  const sample = useMemo(() => sampleAt(mission, t), [mission, t]);
  const progress = t / mission.duration;
  const phase: Phase = sample.phase;

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
            to="/"
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

        {briefing && (
          <div className="absolute inset-0 flex items-end bg-overlay p-5 sm:items-center sm:justify-center">
            <div className="w-full max-w-lg rounded-xl bg-bg-elevated p-5 shadow-[var(--shadow-border)] sm:p-6">
              <div className="text-2xs font-medium uppercase tracking-[0.18em] text-fg-subtle">
                Patched-conic visualization
              </div>
              <h2 className="mt-1 text-2xl font-semibold tracking-tight">Earth to lunar orbit</h2>
              <p className="mt-3 text-sm leading-relaxed text-fg-muted">
                A 300 km LEO parking orbit, a trans-lunar injection, a multi-day coast, then capture
                into a 100 km lunar orbit. Distances and speeds are physical. This is a visualization,
                not the AETHER attitude plant.
              </p>
              <ul className="mt-4 space-y-1.5 font-mono text-xs text-fg-muted">
                <li>TLI Δv ≈ {mission.dvTli.toFixed(2)} km/s</li>
                <li>Coast {(mission.tofCoast / 86400).toFixed(2)} days · Moon 384,400 km</li>
                <li>LLO 100 km · {PHASE_LABEL.llo}</li>
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
                  label={phase === "llo" ? "Moon altitude" : "Earth altitude"}
                  value={
                    (phase === "llo" ? sample.altMoon : sample.altEarth) > 2000
                      ? `${((phase === "llo" ? sample.altMoon : sample.altEarth) / 1000).toFixed(0)} Mm`
                      : `${(phase === "llo" ? sample.altMoon : sample.altEarth).toFixed(0)} km`
                  }
                />
                <Chip label="Speed" value={`${sample.speed.toFixed(2)} km/s`} />
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
                <div className="mx-1 h-6 w-px bg-border" />
                {WARPS.map((w) => (
                  <Btn key={w.label} onClick={() => setWarp(w.value)} active={warp === w.value}>
                    {w.label}
                  </Btn>
                ))}
              </div>
              <div className="flex flex-wrap gap-2">
                {CAMERAS.map((c) => (
                  <Btn key={c.id} onClick={() => setMode(c.id)} active={mode === c.id}>
                    {c.label}
                  </Btn>
                ))}
              </div>
            </div>
          </div>
        )}
      </section>
    </main>
  );
}
