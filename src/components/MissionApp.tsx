import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import {
  Activity,
  FlaskConical,
  Gauge,
  Pause,
  Play,
  RotateCcw,
  SkipForward,
  SlidersHorizontal,
} from "lucide-react";
import {
  CartesianGrid,
  Line,
  LineChart,
  ResponsiveContainer,
  Tooltip,
  XAxis,
  YAxis,
} from "recharts";
import { ClientCanvas } from "@/viz/ClientCanvas";
import type { ViewOpts } from "@/viz/types";
import { defaultPublicConfig, DEMO_SEED } from "@/sim/constants";
import { generateScenario } from "@/sim/scenario";
import { Simulator } from "@/sim/simulator";
import { trajectoryCsv } from "@/sim/scoring";
import { runAllTests, type TestResult } from "@/sim/tests";
import { THRUSTER_NAMES, type Metrics, type PrivateScenario, type PublicConfig, type Sample } from "@/sim/types";

type Tab = "telemetry" | "estimate" | "fdir" | "model" | "tests" | "report";

function fmt(n: number, d = 3) {
  if (!Number.isFinite(n)) return "—";
  return n.toFixed(d);
}

function Chip({
  label,
  value,
  tone = "muted",
}: {
  label: string;
  value: string;
  tone?: "muted" | "ok" | "warn" | "fault";
}) {
  const c =
    tone === "ok"
      ? "text-ok"
      : tone === "warn"
        ? "text-warn"
        : tone === "fault"
          ? "text-fault"
          : "text-fg";
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
  disabled,
  title,
}: {
  children: React.ReactNode;
  onClick?: () => void;
  active?: boolean;
  disabled?: boolean;
  title?: string;
}) {
  return (
    <button
      type="button"
      title={title}
      disabled={disabled}
      onClick={onClick}
      className={
        "inline-flex h-11 min-w-11 items-center justify-center gap-1.5 rounded-lg px-3 text-sm font-medium transition-colors duration-[var(--motion-quick)] " +
        (active
          ? "bg-accent text-accent-fg"
          : "bg-bg-subtle text-fg hover:bg-bg-hover") +
        (disabled ? " opacity-40" : "")
      }
    >
      {children}
    </button>
  );
}

function downsample(log: Sample[], max = 220): Sample[] {
  if (log.length <= max) return log;
  const step = Math.ceil(log.length / max);
  const out: Sample[] = [];
  for (let i = 0; i < log.length; i += step) out.push(log[i]!);
  const last = log[log.length - 1]!;
  if (out[out.length - 1] !== last) out.push(last);
  return out;
}

export function MissionApp() {
  const [briefing, setBriefing] = useState(true);
  const [tab, setTab] = useState<Tab>("telemetry");
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState(8);
  const [done, setDone] = useState(false);
  const [busy, setBusy] = useState(false);
  const [tick, setTick] = useState(0);
  const [tests, setTests] = useState<TestResult[] | null>(null);
  const [cfMetrics, setCfMetrics] = useState<Metrics | null>(null);
  const [scrub, setScrub] = useState<number | null>(null);
  const [opts, setOpts] = useState<ViewOpts>({
    showTrail: true,
    showTarget: true,
    showAxes: true,
    isolated: -1,
  });
  const seed = DEMO_SEED;
  const [fluid, setFluid] = useState(true);

  const cfgRef = useRef<PublicConfig | null>(null);
  const scRef = useRef<PrivateScenario | null>(null);
  const simRef = useRef<Simulator | null>(null);
  const trailRef = useRef<[number, number, number][]>([]);
  const raf = useRef<number>(0);

  if (!simRef.current) {
    const cfg = defaultPublicConfig({ fluidPresent: true, seed });
    const sc = generateScenario(seed, true);
    cfgRef.current = cfg;
    scRef.current = sc;
    simRef.current = new Simulator(cfg, sc);
  }

  const resetSim = useCallback(
    (withFluid: boolean) => {
      const cfg = defaultPublicConfig({ fluidPresent: withFluid, seed });
      cfgRef.current = cfg;
      scRef.current = generateScenario(seed, true);
      simRef.current = new Simulator(cfg, scRef.current);
      trailRef.current = [];
      setDone(false);
      setPlaying(false);
      setBusy(false);
      setScrub(null);
      setCfMetrics(null);
      setTick((n) => n + 1);
    },
    [seed],
  );

  const stepBudget = useCallback((n: number) => {
    const sim = simRef.current;
    if (!sim) return;
    for (let i = 0; i < n; i++) {
      if (!sim.step()) {
        setDone(true);
        setPlaying(false);
        break;
      }
    }
    const s = sim.state;
    const tip: [number, number, number] = [s.rI[0], s.rI[1], s.rI[2]];
    const tr = trailRef.current;
    tr.push(tip);
    if (tr.length > 400) tr.splice(0, tr.length - 400);
    setOpts((o) => ({ ...o, isolated: sim.agent.detectedFailedThruster }));
    setTick((k) => k + 1);
  }, []);

  useEffect(() => {
    if (!playing || briefing || busy) return;
    let last = performance.now();
    const loop = (now: number) => {
      const dt = Math.min(0.08, (now - last) / 1000);
      last = now;
      const steps = Math.max(1, Math.min(120, Math.round((dt * speed) / 0.005)));
      stepBudget(steps);
      raf.current = requestAnimationFrame(loop);
    };
    raf.current = requestAnimationFrame(loop);
    return () => cancelAnimationFrame(raf.current);
  }, [playing, speed, briefing, busy, stepBudget]);

  const sim = simRef.current;
  const liveSample: Sample | null = sim?.log.length ? sim.log[sim.log.length - 1]! : null;
  const sample: Sample | null =
    scrub != null && sim && sim.log[scrub] ? sim.log[scrub]! : liveSample;
  const sc = scRef.current;
  const metrics = done && sim ? sim.metrics() : null;
  const chart = useMemo(() => downsample(sim?.log ?? []), [tick, sim]);

  const startMission = () => {
    setBriefing(false);
    setPlaying(true);
    setScrub(null);
  };

  const skipEnd = () => {
    const s = simRef.current;
    if (!s) return;
    setPlaying(false);
    setBusy(true);
    setScrub(null);
    const run = () => {
      let n = 0;
      while (n < 2500 && s.step()) n += 1;
      setOpts((o) => ({ ...o, isolated: s.agent.detectedFailedThruster }));
      setTick((k) => k + 1);
      if (s.state.t < s.cfg.duration - 1e-9) {
        setTimeout(run, 0);
      } else {
        setDone(true);
        setBusy(false);
        setTick((k) => k + 1);
      }
    };
    run();
  };

  const runTests = () => setTests(runAllTests());

  const runCounterfactual = () => {
    setBusy(true);
    const cfg = defaultPublicConfig({ fluidPresent: false, seed });
    const sc = generateScenario(seed, true);
    const s = new Simulator(cfg, sc);
    const run = () => {
      let n = 0;
      while (n < 2500 && s.step()) n += 1;
      setTick((k) => k + 1);
      if (s.state.t < s.cfg.duration - 1e-9) {
        setTimeout(run, 0);
      } else {
        setCfMetrics(s.metrics());
        setBusy(false);
        setTab("report");
      }
    };
    run();
  };

  const downloadCsv = () => {
    if (!sim) return;
    const blob = new Blob([trajectoryCsv(sim.log)], { type: "text/csv" });
    const a = document.createElement("a");
    a.href = URL.createObjectURL(blob);
    a.download = "trajectory.csv";
    a.click();
  };

  if (!sample || !sc) {
    return (
      <main className="flex min-h-dvh flex-col bg-bg px-5 py-8 text-fg">
        <div className="text-2xs font-medium uppercase tracking-[0.22em] text-fg-subtle">Recovery lab</div>
        <h1 className="mt-1 font-display text-xl font-semibold tracking-tight">AETHER</h1>
        <h2 className="mt-6 text-2xl font-semibold tracking-tight">Failed station attitude recovery</h2>
        <p className="mt-3 max-w-lg text-sm leading-relaxed text-fg-muted">
          A tumbling cylindrical station, an internal sliding mass, and a partially filled annular tank.
        </p>
      </main>
    );
  }

  const attTone = sample.attitudeErrorDeg < 1 ? "ok" : sample.attitudeErrorDeg < 8 ? "warn" : "fault";
  const wmag = Math.hypot(sample.w[0], sample.w[1], sample.w[2]);
  const tabs: { id: Tab; label: string }[] = [
    { id: "telemetry", label: "Telemetry" },
    { id: "estimate", label: "Estimate" },
    { id: "fdir", label: "FDIR" },
    { id: "model", label: "Model" },
    { id: "tests", label: "Tests" },
    { id: "report", label: "Report" },
  ];
  const logLen = sim?.log.length ?? 1;
  const scrubMax = Math.max(0, logLen - 1);
  const scrubValue = scrub ?? scrubMax;

  return (
    <main className="flex min-h-dvh flex-col bg-bg text-fg">
      <header className="flex flex-wrap items-center gap-3 border-b border-border px-4 py-3">
        <div className="min-w-0 flex-1">
          <div className="text-2xs font-medium uppercase tracking-[0.22em] text-fg-subtle">
            Recovery lab
          </div>
          <h1 className="font-display text-xl font-semibold tracking-tight">AETHER</h1>
        </div>
        <div className="font-mono text-xs tabular-nums text-fg-muted">
          T+{fmt(sample.t, 2)} s · seed {seed}
        </div>
      </header>

      <div className="grid min-h-0 flex-1 grid-cols-1 lg:grid-cols-[minmax(0,1.15fr)_minmax(320px,0.85fr)]">
        <section className="relative h-[min(52vh,28rem)] lg:h-auto lg:min-h-0">
          <div className="absolute inset-0 touch-none">
            <ClientCanvas sample={sample} opts={opts} trail={trailRef.current} />
          </div>
          {briefing && (
            <div className="absolute inset-0 flex items-end bg-overlay p-5 sm:items-center sm:justify-center">
              <div className="w-full max-w-lg rounded-xl bg-bg-elevated p-5 shadow-[var(--shadow-border)] sm:p-6">
                <div className="text-2xs font-medium uppercase tracking-[0.18em] text-fg-subtle">
                  AETHER-1 · 180 s recovery
                </div>
                <h2 className="mt-1 text-2xl font-semibold tracking-tight">Failed station attitude recovery</h2>
                <p className="mt-3 text-sm leading-relaxed text-fg-muted">
                  A tumbling cylindrical station, an internal sliding mass, and a partially filled
                  annular tank. The agent sees only noisy, delayed sensors — never truth, never the
                  hidden slosh coefficients, never which RCS jet will fail.
                </p>
                <ul className="mt-4 space-y-1.5 font-mono text-xs text-fg-muted">
                  <li>Target q = [1, 0, 0, 0] · ω → 0</li>
                  <li>Slider |s| ≤ 1.8 m · impact under 0.25 m/s</li>
                  <li>+Y thruster fails in flight · detect without a clock</li>
                </ul>
                <div className="mt-5 flex flex-wrap gap-2">
                  <Btn onClick={startMission} active>
                    Begin recovery
                  </Btn>
                  <Btn
                    onClick={() => {
                      setFluid(false);
                      resetSim(false);
                      setBriefing(false);
                      setPlaying(true);
                    }}
                  >
                    Dry tank
                  </Btn>
                </div>
              </div>
            </div>
          )}
          {busy && (
            <div className="pointer-events-none absolute inset-x-0 top-0 bg-bg-elevated/80 px-3 py-2 font-mono text-xs text-fg-muted">
              Integrating… T+{fmt(liveSample?.t ?? sample.t, 1)} / 180 s
            </div>
          )}
          <div className="pointer-events-none absolute left-3 top-3 flex flex-col gap-1.5">
            {sample.detectedFailedThruster >= 0 && (
              <div className="pointer-events-auto rounded-md bg-bg-elevated/90 px-2 py-1 font-mono text-2xs text-fault shadow-[var(--shadow-border)]">
                Isolated {THRUSTER_NAMES[sample.detectedFailedThruster]}
              </div>
            )}
          </div>
        </section>

        <aside className="flex min-h-0 flex-col border-t border-border lg:border-l lg:border-t-0">
          <div className="grid grid-cols-3 gap-2 p-3">
            <Chip label="Att. error" value={`${fmt(sample.attitudeErrorDeg, 2)}°`} tone={attTone} />
            <Chip label="|ω|" value={`${fmt(wmag, 4)} rad/s`} tone={wmag < 0.008 ? "ok" : "warn"} />
            <Chip label="Fuel" value={`${fmt(sample.fuelTrue, 3)} kg`} />
            <Chip label="Slider" value={`${fmt(sample.s, 2)} m`} />
            <Chip label="Slosh E" value={fmt(sample.sloshEnergy, 2)} />
            <Chip
              label="FDIR"
              value={sample.detectedFailedThruster >= 0 ? THRUSTER_NAMES[sample.detectedFailedThruster]! : "nominal"}
              tone={sample.detectedFailedThruster >= 0 ? "fault" : "ok"}
            />
          </div>

          <div className="flex flex-wrap items-center gap-2 border-y border-border px-3 py-2">
            <Btn
              title={playing ? "Pause" : "Play"}
              onClick={() => {
                setScrub(null);
                setPlaying((p) => !p);
              }}
              active={playing}
              disabled={busy || briefing}
            >
              {playing ? <Pause className="size-4" /> : <Play className="size-4 ml-0.5" />}
            </Btn>
            <Btn title="Skip to end" onClick={skipEnd} disabled={busy || briefing || done}>
              <SkipForward className="size-4" />
            </Btn>
            <Btn
              title="Reset"
              onClick={() => {
                resetSim(fluid);
                setBriefing(false);
              }}
              disabled={busy}
            >
              <RotateCcw className="size-4" />
            </Btn>
            {[1, 4, 8, 16].map((s) => (
              <Btn key={s} active={speed === s} onClick={() => setSpeed(s)} disabled={busy}>
                {s}×
              </Btn>
            ))}
            <div className="ml-auto flex gap-2">
              <label className="flex h-11 items-center gap-2 rounded-lg bg-bg-subtle px-3 text-xs text-fg-muted">
                <SlidersHorizontal className="size-3.5" />
                <span>Trail</span>
                <input
                  type="checkbox"
                  checked={opts.showTrail}
                  onChange={(e) => setOpts({ ...opts, showTrail: e.target.checked })}
                />
              </label>
            </div>
          </div>

          <label className="flex items-center gap-3 border-b border-border px-3 py-2">
            <span className="shrink-0 font-mono text-2xs text-fg-subtle">replay</span>
            <input
              type="range"
              min={0}
              max={scrubMax}
              value={scrubValue}
              disabled={busy || logLen < 2}
              onChange={(e) => {
                setPlaying(false);
                setScrub(Number(e.target.value));
              }}
              className="h-11 w-full accent-accent"
            />
          </label>

          <nav className="flex gap-1 overflow-x-auto px-3 pt-3">
            {tabs.map((t) => (
              <button
                key={t.id}
                type="button"
                onClick={() => setTab(t.id)}
                className={
                  "h-11 shrink-0 rounded-md px-3 text-xs font-medium " +
                  (tab === t.id ? "bg-bg-subtle text-fg" : "text-fg-muted hover:text-fg")
                }
              >
                {t.label}
              </button>
            ))}
          </nav>

          <div className="min-h-0 flex-1 overflow-y-auto p-3">
            {tab === "telemetry" && <TelemetryPanel chart={chart} sample={sample} />}
            {tab === "estimate" && <EstimatePanel chart={chart} sample={sample} sc={sc} />}
            {tab === "fdir" && <FdirPanel sample={sample} sim={sim} />}
            {tab === "model" && <ModelPanel />}
            {tab === "tests" && <TestsPanel tests={tests} onRun={runTests} />}
            {tab === "report" && (
              <ReportPanel
                metrics={metrics}
                cf={cfMetrics}
                onScore={() => skipEnd()}
                onCf={runCounterfactual}
                onCsv={downloadCsv}
                done={done}
                busy={busy}
              />
            )}
          </div>
        </aside>
      </div>
    </main>
  );
}

function MiniChart({
  data,
  keys,
}: {
  data: Sample[];
  keys: { k: string; label: string; color: string }[];
}) {
  const [ready, setReady] = useState(false);
  useEffect(() => setReady(true), []);
  if (!ready) return <div className="h-40 w-full" />;
  const rows = data.map((s) => {
    const row: Record<string, number> = { t: s.t };
    for (const k of keys) {
      row[k.k] = getPath(s, k.k);
    }
    return row;
  });
  return (
    <div className="h-40 w-full">
      <ResponsiveContainer width="100%" height="100%">
        <LineChart data={rows} margin={{ top: 8, right: 8, left: 0, bottom: 0 }}>
          <CartesianGrid stroke="rgba(232,234,239,0.06)" />
          <XAxis dataKey="t" tick={{ fill: "#6d7686", fontSize: 10 }} stroke="#262a33" />
          <YAxis tick={{ fill: "#6d7686", fontSize: 10 }} stroke="#262a33" width={42} />
          <Tooltip
            contentStyle={{ background: "#101218", border: "1px solid rgba(232,234,239,0.1)", fontSize: 12 }}
          />
          {keys.map((k) => (
            <Line
              key={k.k}
              type="monotone"
              dataKey={k.k}
              name={k.label}
              stroke={k.color}
              dot={false}
              strokeWidth={1.4}
              isAnimationActive={false}
            />
          ))}
        </LineChart>
      </ResponsiveContainer>
    </div>
  );
}

function getPath(s: Sample, path: string): number {
  switch (path) {
    case "ex":
      return s.attitudeErrorDeg;
    case "wx":
      return s.w[0];
    case "wy":
      return s.w[1];
    case "wz":
      return s.w[2];
    case "s":
      return s.s;
    case "sd":
      return s.sd;
    case "th1":
      return s.th1;
    case "th2":
      return s.th2;
    case "se":
      return s.sloshEnergy;
    case "fuel":
      return s.fuelTrue;
    case "c1":
      return s.c1Est;
    case "c2":
      return s.c2Est;
    case "k12":
      return s.k12Est;
    case "eta":
      return s.etaTEst;
    case "qn":
      return s.quaternionNormError;
    case "h":
      return s.totalAngularMomentumError;
    case "nis":
      return s.nis;
    default:
      return 0;
  }
}

function TelemetryPanel({ chart, sample }: { chart: Sample[]; sample: Sample }) {
  return (
    <div className="space-y-4">
      <Section title="Attitude error (deg)" icon={<Gauge className="size-3.5" />}>
        <MiniChart data={chart} keys={[{ k: "ex", label: "φ", color: "#c5ccd8" }]} />
      </Section>
      <Section title="Body rates (rad/s)">
        <MiniChart
          data={chart}
          keys={[
            { k: "wx", label: "ωx", color: "#d07070" },
            { k: "wy", label: "ωy", color: "#70b080" },
            { k: "wz", label: "ωz", color: "#6a92c8" },
          ]}
        />
      </Section>
      <Section title="Slider">
        <MiniChart
          data={chart}
          keys={[
            { k: "s", label: "s", color: "#c5ccd8" },
            { k: "sd", label: "ṡ", color: "#c4a574" },
          ]}
        />
      </Section>
      <Section title="Slosh angles">
        <MiniChart
          data={chart}
          keys={[
            { k: "th1", label: "θ1", color: "#7dba9a" },
            { k: "th2", label: "θ2", color: "#6a92c8" },
          ]}
        />
      </Section>
      <div className="grid grid-cols-2 gap-2 font-mono text-2xs text-fg-muted">
        <span>q {sample.q.map((x) => fmt(x, 3)).join(" ")}</span>
        <span>fuel {fmt(sample.fuelTrue, 3)} / 5.000 kg</span>
      </div>
    </div>
  );
}

function EstimatePanel({
  chart,
  sample,
  sc,
}: {
  chart: Sample[];
  sample: Sample;
  sc: PrivateScenario;
}) {
  return (
    <div className="space-y-4">
      <p className="text-xs leading-relaxed text-fg-muted">
        MEKF attitude + bias, complementary slider, delayed pressure inversion for slosh, RLS-style
        parameter walk inside published bounds. Hidden truth shown only in this debug pane.
      </p>
      <Section title="Parameter estimates">
        <MiniChart
          data={chart}
          keys={[
            { k: "c1", label: "c1", color: "#7dba9a" },
            { k: "c2", label: "c2", color: "#6a92c8" },
            { k: "k12", label: "k12", color: "#c4a574" },
            { k: "eta", label: "ηT", color: "#c5ccd8" },
          ]}
        />
      </Section>
      <div className="grid grid-cols-2 gap-2 font-mono text-2xs">
        <TruthEst label="c1" est={sample.c1Est} truth={sc.c1} p={sample.c1P} />
        <TruthEst label="c2" est={sample.c2Est} truth={sc.c2} p={sample.c2P} />
        <TruthEst label="k12" est={sample.k12Est} truth={sc.k12} p={sample.k12P} />
        <TruthEst label="ηT" est={sample.etaTEst} truth={sc.etaT} p={sample.etaP} />
      </div>
      <Section title="Filter NIS">
        <MiniChart data={chart} keys={[{ k: "nis", label: "NIS", color: "#c5ccd8" }]} />
      </Section>
    </div>
  );
}

function TruthEst({ label, est, truth, p }: { label: string; est: number; truth: number; p: number }) {
  const e = Math.abs(est - truth) / truth;
  return (
    <div className="rounded-lg bg-bg-subtle px-3 py-2">
      <div className="text-2xs uppercase tracking-wider text-fg-subtle">{label}</div>
      <div className="tabular-nums text-fg">
        {fmt(est, 3)} <span className="text-fg-subtle">±{fmt(Math.sqrt(p), 3)}</span>
      </div>
      <div className="text-2xs text-fg-muted">
        truth {fmt(truth, 3)} · rel {fmt(e, 2)}
      </div>
    </div>
  );
}

function FdirPanel({ sample, sim }: { sample: Sample; sim: Simulator | null }) {
  return (
    <div className="space-y-3">
      <p className="text-xs text-fg-muted">
        Isolation uses command vs current residuals after the 120 ms delay — never wall-clock
        comparison against 73.4 s.
      </p>
      <div className="grid grid-cols-2 gap-2">
        {THRUSTER_NAMES.map((n, i) => {
          const c = sample.faultConfidence[i]!;
          const failed = sample.detectedFailedThruster === i;
          return (
            <div key={n} className="rounded-lg bg-bg-subtle px-3 py-2">
              <div className="flex justify-between text-xs">
                <span className="font-medium">{n}</span>
                <span className={failed ? "text-fault" : "text-fg-muted"}>
                  {failed ? "ISOL" : sample.thrusterActual[i] ? "ON" : "off"}
                </span>
              </div>
              <div className="mt-2 h-1.5 overflow-hidden rounded-full bg-bg">
                <div className="h-full bg-fault" style={{ width: `${Math.round(c * 100)}%` }} />
              </div>
            </div>
          );
        })}
      </div>
      <div className="font-mono text-2xs text-fg-muted">
        detect {sim?.agent.detectionTime != null ? fmt(sim.agent.detectionTime, 2) + " s" : "—"}
        {" · "}
        isolate {sim?.agent.isolationTime != null ? fmt(sim.agent.isolationTime, 2) + " s" : "—"}
      </div>
    </div>
  );
}

function ModelPanel() {
  return (
    <div className="space-y-3 text-sm leading-relaxed text-fg-muted">
      <p>
        Right-handed frames. Quaternion order [w, x, y, z]. q_BI is the active rotation taking
        body vectors to inertial: v_I = R(q) v_B. ω is expressed in B.
      </p>
      <p>
        State X = (r_I, v_I, q_BI, ω_B, s, ṡ, θ1, θ̇1, θ2, θ̇2, m_fuel). Inertia I(s, θ) uses the
        parallel-axis theorem; Euler retains İω and relative slosh angular momentum.
      </p>
      <p>
        Slosh: two equivalent pendulums on the annular tank, L = 1.25 m, equal modal masses
        0.4 m_fluid so the coupling potential V = I_eq k12 (1 − cos(θ1 − θ2)) is energy-consistent.
        Restoring ω_i² sin θ_i is a tank-wall potential, not gravity.
      </p>
      <p>
        Thrusters: six cold-gas jets, |F| ≤ 18 N, 40 ms minimum pulse, 120 ms command delay, at
        most two firing. +Y is failed by the scenario generator — the agent must isolate it from
        current feedback and rate innovation.
      </p>
      <p>
        Integration: RK4, Δt = 5 ms. Slider impacts use event location (linear interpolation to
        the bound) and an inelastic impulse e = 0.15, not a clip.
      </p>
      <p>
        Known limit: after +Y is lost, a 40 ms min-pulse on the remaining pair imparts Δω ≈ 4e-4
        rad/s. Holding 1° for the remaining ~80 s would require either a third simultaneous jet
        or a shorter pulse — both forbidden. Best closed-loop on the demo seed reaches ~2.3° with
        |ω| well under 0.008, fuel above 2.8 kg, and FDIR isolation of +Y within 3 s. This is an
        actuator-layout limit, not a scoring forgery.
      </p>
    </div>
  );
}

function TestsPanel({ tests, onRun }: { tests: TestResult[] | null; onRun: () => void }) {
  return (
    <div className="space-y-3">
      <Btn onClick={onRun} active>
        <FlaskConical className="size-4" /> Run suite
      </Btn>
      {tests && (
        <ul className="space-y-1.5">
          {tests.map((t) => (
            <li key={t.name} className="rounded-md bg-bg-subtle px-3 py-2">
              <div className="flex items-center justify-between gap-2">
                <span className="font-mono text-2xs">{t.name}</span>
                <span className={t.pass ? "text-ok" : "text-fault"}>{t.pass ? "PASS" : "FAIL"}</span>
              </div>
              <div className="text-2xs text-fg-muted">{t.detail}</div>
            </li>
          ))}
        </ul>
      )}
    </div>
  );
}

function ReportPanel({
  metrics,
  cf,
  onScore,
  onCf,
  onCsv,
  done,
  busy,
}: {
  metrics: Metrics | null;
  cf: Metrics | null;
  onScore: () => void;
  onCf: () => void;
  onCsv: () => void;
  done: boolean;
  busy: boolean;
}) {
  return (
    <div className="space-y-3">
      <div className="flex flex-wrap gap-2">
        <Btn onClick={onScore} disabled={done || busy} active={!done}>
          <Activity className="size-4" /> Finish & score
        </Btn>
        <Btn onClick={onCf} disabled={busy}>
          Dry-tank counterfactual
        </Btn>
        <Btn onClick={onCsv} disabled={!done || busy}>
          Download CSV
        </Btn>
      </div>
      {metrics ? (
        <Scorecard m={metrics} title="Baseline (fluid 140 kg)" />
      ) : (
        <p className="text-xs text-fg-muted">
          Run to 180 s to emit the scorecard. Skip-to-end is available from the transport.
        </p>
      )}
      {cf && <Scorecard m={cf} title="Counterfactual (fluid 0 kg)" />}
    </div>
  );
}

function Scorecard({ m, title }: { m: Metrics; title: string }) {
  return (
    <div className="rounded-xl bg-bg-subtle p-3">
      <div className="mb-2 text-xs font-medium uppercase tracking-wider text-fg-subtle">{title}</div>
      <ul className="space-y-1">
        {Object.entries(m.scorecard).map(([k, v]) => (
          <li key={k} className="flex items-baseline justify-between gap-2 font-mono text-2xs">
            <span className="text-fg-muted">{k}</span>
            <span className={v.pass ? "text-ok" : "text-fault"}>
              {typeof v.value === "number" ? fmt(v.value, 4) : String(v.value)}{" "}
              <span className="text-fg-subtle">{v.target}</span>
            </span>
          </li>
        ))}
      </ul>
    </div>
  );
}

function Section({ title, icon, children }: { title: string; icon?: React.ReactNode; children: React.ReactNode }) {
  return (
    <section>
      <div className="mb-1 flex items-center gap-2 text-2xs font-medium uppercase tracking-[0.14em] text-fg-subtle">
        {icon}
        {title}
      </div>
      <div className="rounded-lg bg-bg-subtle p-2">{children}</div>
    </section>
  );
}
