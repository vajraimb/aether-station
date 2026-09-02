import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { ArrowLeft, Pause, Play, RotateCcw } from "lucide-react";
import { ClientCanvas } from "@/viz/ClientCanvas";
import type { ViewOpts } from "@/viz/types";
import {
  decodeFrame,
  loadReplay,
  omegaNorm,
  replaySeeds,
  type Frame,
  type ReplayFile,
} from "@/lib/challenge-v3/replay-data";
import { canonical, provenance } from "@/lib/challenge-v3/report-data";
import { DEFAULT_LINKS, type PageLinks } from "./ChallengeV3Report";

/* ------------------------------------------------------------------ helpers */

function fmt(x: number, d: number): string {
  return x.toFixed(d);
}

/** Attitude spans 0.05°-160°, so the trace is drawn on a log axis. */
const LOG_MIN = 0.02;
const LOG_MAX = 200;
function logY(deg: number): number {
  const v = Math.max(LOG_MIN, Math.min(LOG_MAX, deg));
  return 1 - Math.log10(v / LOG_MIN) / Math.log10(LOG_MAX / LOG_MIN);
}

const NOZZLE_LABEL = ["0", "1", "2", "3", "4", "5"];

/* -------------------------------------------------------------------- atoms */

function Readout({
  label,
  value,
  unit,
  tone = "",
  hint,
}: {
  label: string;
  value: string;
  unit?: string;
  tone?: string;
  hint?: string;
}) {
  return (
    <div className="rounded-sm border border-border bg-bg-elevated px-3 py-2">
      <div className="text-2xs uppercase tracking-[0.14em] text-fg-subtle">{label}</div>
      <div className={`mt-1 font-mono text-lg leading-none ${tone || "text-fg"}`}>
        {value}
        {unit ? <span className="ml-1 text-xs text-fg-subtle">{unit}</span> : null}
      </div>
      {hint ? <div className="mt-1 text-2xs text-fg-subtle">{hint}</div> : null}
    </div>
  );
}

function GateChip({ label, pass }: { label: string; pass: boolean }) {
  return (
    <span
      className={`inline-flex items-center gap-1 rounded-xs border px-1.5 py-0.5 font-mono text-2xs ${
        pass ? "border-ok/40 text-ok" : "border-fault/50 text-fault"
      }`}
    >
      {label}
      {pass ? " 通过" : " 未通过"}
    </span>
  );
}

/* --------------------------------------------------------------- the viewer */

const SPEEDS = [1, 2, 4, 8, 16];

export function ChallengeV3Replay({ links = DEFAULT_LINKS }: { links?: PageLinks } = {}) {
  const seedList = replaySeeds.length ? replaySeeds : [];
  const [seed, setSeed] = useState<number>(seedList[0] ?? 800000);
  const [file, setFile] = useState<ReplayFile | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [idx, setIdx] = useState(0);
  const [playing, setPlaying] = useState(true);
  const [speed, setSpeed] = useState(4);
  const [showTarget, setShowTarget] = useState(true);

  // Load the requested trajectory. Each seed is its own lazy chunk.
  useEffect(() => {
    let live = true;
    setFile(null);
    setError(null);
    setIdx(0);
    loadReplay(seed)
      .then((f) => {
        if (live) setFile(f);
      })
      .catch((e: unknown) => {
        if (live) setError(e instanceof Error ? e.message : String(e));
      });
    return () => {
      live = false;
    };
  }, [seed]);

  const frames = useMemo<Frame[]>(
    () => (file ? file.frames.map(decodeFrame) : []),
    [file],
  );

  /**
   * Nominal frame spacing, taken from the frames rather than from the file's
   * own field: the simulator log is not perfectly uniform, so playback is
   * driven by mission time and a lookup, never by "index += constant".
   */
  const interval = useMemo(() => {
    if (frames.length < 2) return 0.2;
    const gaps = frames.slice(1).map((f, i) => f.t - frames[i]!.t).sort((a, b) => a - b);
    return gaps[Math.floor(gaps.length / 2)] ?? 0.2;
  }, [frames]);

  /** Index of the last frame at or before mission time `t`. */
  const indexAt = useCallback(
    (t: number) => {
      let lo = 0;
      let hi = frames.length - 1;
      while (lo < hi) {
        const mid = (lo + hi + 1) >> 1;
        if ((frames[mid]?.t ?? 0) <= t) lo = mid;
        else hi = mid - 1;
      }
      return lo;
    },
    [frames],
  );

  // Playback clock. One frame is `frame_interval_s` of mission time, so at
  // speed s the viewer advances s x real time.
  const idxRef = useRef(0);
  idxRef.current = idx;
  useEffect(() => {
    if (!playing || frames.length === 0) return;
    let raf = 0;
    let last = performance.now();
    let t = frames[Math.round(idxRef.current)]?.t ?? 0;
    const tEnd = frames[frames.length - 1]?.t ?? 0;
    const step = (now: number) => {
      t += ((now - last) / 1000) * speed;
      last = now;
      if (t >= tEnd) {
        idxRef.current = frames.length - 1;
        setIdx(frames.length - 1);
        setPlaying(false);
        return;
      }
      const i = indexAt(t);
      idxRef.current = i;
      setIdx(i);
      raf = requestAnimationFrame(step);
    };
    raf = requestAnimationFrame(step);
    return () => cancelAnimationFrame(raf);
  }, [playing, speed, frames, indexAt]);

  const frame = frames[Math.round(idx)] ?? frames[0];
  const trail = useMemo<[number, number, number][]>(() => [], []);

  /**
   * The camera is body-centred. Over 180 s the centre of mass translates tens
   * of metres (the recovery burns are not momentum-balanced), which would take
   * the station out of frame within the first minute; the scored quantities are
   * attitude, rate and fuel, not position. The drift is not hidden - it is read
   * out beside the scene from the same recorded frame.
   */
  const sceneSample = useMemo(
    () => (frame ? { ...frame, r: [0, 0, 0] as [number, number, number] } : null),
    [frame],
  );
  const drift = frame ? Math.hypot(frame.r[0], frame.r[1], frame.r[2]) : 0;

  const jump = useCallback(
    (t: number) => {
      if (frames.length === 0) return;
      const i = indexAt(Math.max(0, t));
      idxRef.current = i;
      setIdx(i);
    },
    [frames.length, indexAt],
  );

  const row = canonical.per_seed.find((r) => r.seed === seed);

  // View options are derived, never stored: the isolated nozzle is whatever the
  // recorded FDIR estimate says at this instant, so it must not live in state
  // that an effect keeps chasing.
  const isolated = frame?.detectedFailedThruster ?? -1;
  const opts = useMemo<ViewOpts>(
    () => ({ showTrail: false, showTarget, showAxes: true, isolated }),
    [showTarget, isolated],
  );

  const t = frame?.t ?? 0;
  const attNow = frame?.attitudeErrorDeg ?? 0;
  const omgNow = frame ? omegaNorm(frame.w) : 0;
  const replansSoFar = file ? file.replans.filter((r) => r.t <= t).length : 0;
  const faultActive = file ? t >= file.scenario.faultTime : false;

  return (
    <div className="min-h-dvh bg-bg text-fg">
      <header className="sticky top-0 z-20 border-b border-border bg-overlay backdrop-blur">
        <div className="mx-auto flex max-w-7xl items-center gap-3 px-4 py-3 sm:px-6">
          <a
            href={links.report}
            className="inline-flex shrink-0 items-center gap-1.5 whitespace-nowrap rounded-xs px-2 py-1 text-xs text-fg-muted transition-colors hover:bg-bg-hover hover:text-fg"
          >
            <ArrowLeft className="size-3.5" />
            结果报告
          </a>
          <div className="min-w-0">
            <div className="truncate text-xs font-medium [word-break:keep-all] text-fg">
              Challenge V3 · 三维回放
            </div>
            <div className="truncate font-mono text-2xs text-fg-subtle">
              {canonical.short} · Train-10 · 分支 {provenance.branch}
            </div>
          </div>
        </div>
      </header>

      <main className="mx-auto max-w-7xl px-4 py-6 sm:px-6">
        <h1 className="font-display text-2xl font-semibold tracking-tight [word-break:keep-all] text-fg sm:text-3xl">
          把实测轨迹放回飞行器里
        </h1>
        <p className="mt-3 max-w-3xl text-sm leading-relaxed text-fg-muted">
          下面回放的是<strong className="font-medium text-fg">保留配置</strong>（
          {canonical.short}）在 Train-10 十个种子上跑出来的那一次真值态任务，不是浏览器里重新算的结果。
          规划器一次任务要几分钟 CPU，浏览器跑不动，也不该产生第二套没经过审计的数字；导出脚本
          <code className="mx-1 font-mono text-xs text-fg">src/sim/cli/challenge-v3-replay.ts</code>
          用官方 Simulator 重跑后，把终端指标与已提交的原始行逐项比对，差值写进文件里。
        </p>

        {/* seed selector */}
        <div className="mt-6 flex flex-wrap gap-1.5">
          {seedList.map((s) => {
            const r = canonical.per_seed.find((x) => x.seed === s);
            const ok = r?.passed ?? false;
            const on = s === seed;
            return (
              <button
                key={s}
                type="button"
                onClick={() => setSeed(s)}
                className={`rounded-xs border px-2 py-1 font-mono text-2xs transition-colors ${
                  on
                    ? "border-accent bg-accent text-accent-fg"
                    : ok
                      ? "border-ok/35 text-ok hover:bg-bg-hover"
                      : "border-fault/45 text-fault hover:bg-bg-hover"
                }`}
              >
                {s} · {r ? fmt(r.final_attitude_error_deg, 3) : "?"}°
              </button>
            );
          })}
        </div>

        {error ? (
          <p className="mt-6 rounded-sm border border-fault/40 bg-bg-elevated p-4 text-sm text-fault">
            这个种子没有已提交的轨迹文件：{error}
          </p>
        ) : null}

        <div className="mt-5 grid gap-4 lg:grid-cols-[minmax(0,1fr)_20rem]">
          {/* 3D scene */}
          <div className="overflow-hidden rounded-sm border border-border bg-bg-elevated">
            <div className="relative h-[26rem] w-full sm:h-[34rem]">
              {sceneSample ? (
                <ClientCanvas sample={sceneSample} opts={opts} trail={trail} />
              ) : (
                <div className="flex h-full items-center justify-center text-xs text-fg-subtle">
                  正在载入轨迹…
                </div>
              )}
              {frame ? (
                <div className="pointer-events-none absolute left-3 top-3 rounded-sm bg-overlay px-2.5 py-1.5 font-mono text-2xs leading-relaxed text-fg-muted backdrop-blur">
                  <div>t = {fmt(t, 1)} s / {file?.duration_s ?? 180} s</div>
                  <div className={attNow < 1 ? "text-ok" : "text-fault"}>
                    指向误差 {fmt(attNow, 3)}°
                  </div>
                  <div>角速度 {omgNow.toExponential(2)} rad/s</div>
                </div>
              ) : null}
              {faultActive && file ? (
                <div className="pointer-events-none absolute right-3 top-3 rounded-sm bg-overlay px-2.5 py-1.5 font-mono text-2xs text-fault backdrop-blur">
                  {file.scenario.faultThruster} 号喷口已失效 · {fmt(file.scenario.faultTime, 1)} s
                </div>
              ) : null}
            </div>

            {/* transport */}
            <div className="flex flex-wrap items-center gap-2 border-t border-border px-3 py-2.5">
              <button
                type="button"
                onClick={() => {
                  if (Math.round(idx) >= frames.length - 1) jump(0);
                  setPlaying((p) => !p);
                }}
                className="inline-flex items-center gap-1.5 rounded-xs border border-border-strong px-2.5 py-1 text-xs text-fg transition-colors hover:bg-bg-hover"
              >
                {playing ? <Pause className="size-3.5" /> : <Play className="size-3.5" />}
                {playing ? "暂停" : "播放"}
              </button>
              <button
                type="button"
                onClick={() => {
                  jump(0);
                  setPlaying(false);
                }}
                className="inline-flex items-center gap-1.5 rounded-xs border border-border px-2.5 py-1 text-xs text-fg-muted transition-colors hover:bg-bg-hover hover:text-fg"
              >
                <RotateCcw className="size-3.5" />
                回到 0 s
              </button>
              <div className="flex items-center gap-1">
                {SPEEDS.map((s) => (
                  <button
                    key={s}
                    type="button"
                    onClick={() => setSpeed(s)}
                    className={`rounded-xs px-1.5 py-1 font-mono text-2xs transition-colors ${
                      s === speed ? "bg-accent text-accent-fg" : "text-fg-subtle hover:bg-bg-hover"
                    }`}
                  >
                    {s}×
                  </button>
                ))}
              </div>
              {file ? (
                <div className="flex items-center gap-1">
                  <button
                    type="button"
                    onClick={() => jump(file.scenario.faultTime - 2)}
                    className="rounded-xs border border-border px-2 py-1 text-2xs text-fg-muted transition-colors hover:bg-bg-hover hover:text-fg"
                  >
                    跳到故障前
                  </button>
                  <button
                    type="button"
                    onClick={() => jump(file.duration_s - file.dwell_window_s)}
                    className="rounded-xs border border-border px-2 py-1 text-2xs text-fg-muted transition-colors hover:bg-bg-hover hover:text-fg"
                  >
                    跳到保持窗口
                  </button>
                </div>
              ) : null}
              <label className="flex items-center gap-1.5 text-2xs text-fg-subtle">
                <input
                  type="checkbox"
                  checked={showTarget}
                  onChange={(e) => setShowTarget(e.target.checked)}
                  className="size-3"
                />
                目标姿态
              </label>
              <input
                type="range"
                min={0}
                max={Math.max(0, frames.length - 1)}
                value={Math.round(idx)}
                onChange={(e) => {
                  const i = Number(e.target.value);
                  idxRef.current = i;
                  setIdx(i);
                  setPlaying(false);
                }}
                className="ml-auto h-1 w-full min-w-40 flex-1 accent-accent"
                aria-label="任务时间"
              />
            </div>
          </div>

          {/* HUD */}
          <div className="space-y-3">
            <div className="grid grid-cols-2 gap-2">
              <Readout
                label="指向误差"
                value={fmt(attNow, 3)}
                unit="°"
                tone={attNow < 1 ? "text-ok" : "text-fault"}
                hint="门限 < 1°"
              />
              <Readout
                label="角速度"
                value={omgNow.toExponential(1)}
                unit="rad/s"
                tone={omgNow < 0.008 ? "text-ok" : "text-fault"}
                hint="门限 < 0.008"
              />
              <Readout
                label="剩余燃料"
                value={fmt(frame?.fuelTrue ?? 0, 3)}
                unit="kg"
                tone={(frame?.fuelTrue ?? 0) > 2.8 ? "text-ok" : "text-fault"}
                hint="硬底 2.8 kg"
              />
              <Readout
                label="质心漂移"
                value={fmt(drift, 1)}
                unit="m"
                hint="不计分，镜头随体固定"
              />
              <Readout
                label="已重规划"
                value={String(replansSoFar)}
                unit={file ? `/ ${file.replan_count}` : undefined}
                hint="真值态滚动优化"
              />
            </div>

            {/* nozzles */}
            <div className="rounded-sm border border-border bg-bg-elevated p-3">
              <div className="text-2xs uppercase tracking-[0.14em] text-fg-subtle">喷口</div>
              <div className="mt-2 flex gap-1.5">
                {NOZZLE_LABEL.map((label, i) => {
                  const firing = frame ? ((frame.thrusterMask >> i) & 1) === 1 : false;
                  const dead = file ? faultActive && file.scenario.faultThruster === i : false;
                  return (
                    <div
                      key={label}
                      className={`flex h-8 flex-1 items-center justify-center rounded-xs border font-mono text-2xs ${
                        dead
                          ? "border-fault bg-fault/15 text-fault"
                          : firing
                            ? "border-[color:var(--color-thruster)] bg-[color:var(--color-thruster)]/20 text-[color:var(--color-thruster)]"
                            : "border-border text-fg-subtle"
                      }`}
                      title={dead ? "已失效" : firing ? "本帧窗口内点火" : "未点火"}
                    >
                      {label}
                    </div>
                  );
                })}
              </div>
              <p className="mt-2 text-2xs leading-relaxed text-fg-subtle">
                最短脉冲 40 ms，帧间隔 {fmt(interval, 2)} s，所以点火格显示的是
                「这一帧窗口内点过火」，不是某一瞬间的通断。
              </p>
            </div>

            {/* verdict */}
            {file ? (
              <div className="rounded-sm border border-border bg-bg-elevated p-3">
                <div className="flex items-center justify-between">
                  <div className="text-2xs uppercase tracking-[0.14em] text-fg-subtle">终端判定</div>
                  <span
                    className={`font-mono text-2xs ${file.metrics.passed ? "text-ok" : "text-fault"}`}
                  >
                    {file.metrics.passed ? "六门全过" : "未过门"}
                  </span>
                </div>
                <div className="mt-2 flex flex-wrap gap-1">
                  <GateChip label="姿态" pass={file.metrics.gate.attitude ?? false} />
                  <GateChip label="角速度" pass={file.metrics.gate.rate ?? false} />
                  <GateChip label="燃料" pass={file.metrics.gate.fuel ?? false} />
                  <GateChip label="晃动" pass={file.metrics.gate.slosh ?? false} />
                  <GateChip label="撞击" pass={file.metrics.gate.impact ?? false} />
                  <GateChip label="四元数" pass={file.metrics.gate.quat ?? false} />
                </div>
                <dl className="mt-3 space-y-1 font-mono text-2xs text-fg-muted">
                  <div className="flex justify-between gap-2">
                    <dt className="text-fg-subtle">终端指向</dt>
                    <dd>{fmt(file.metrics.final_attitude_error_deg, 3)}°</dd>
                  </div>
                  <div className="flex justify-between gap-2">
                    <dt className="text-fg-subtle">终端角速度</dt>
                    <dd>{file.metrics.final_angular_speed_rad_s.toExponential(2)} rad/s</dd>
                  </div>
                  <div className="flex justify-between gap-2">
                    <dt className="text-fg-subtle">脉冲数</dt>
                    <dd>{file.metrics.pulse_count}</dd>
                  </div>
                  <div className="flex justify-between gap-2">
                    <dt className="text-fg-subtle">规划耗时</dt>
                    <dd>{fmt(file.wall_clock_s, 0)} s</dd>
                  </div>
                </dl>
                {file.determinism_check ? (
                  <p className="mt-3 border-t border-border pt-2 text-2xs leading-relaxed text-fg-subtle">
                    与已提交原始行比对：指向差{" "}
                    <span className="font-mono">
                      {file.determinism_check.attitude_abs_diff_deg.toExponential(1)}
                    </span>
                    °，燃料差{" "}
                    <span className="font-mono">
                      {file.determinism_check.fuel_abs_diff_kg.toExponential(1)}
                    </span>{" "}
                    kg，判定{file.determinism_check.pass_matches ? "一致" : "不一致"}。
                  </p>
                ) : null}
                {row && !row.passed ? (
                  <p className="mt-2 text-2xs leading-relaxed text-fault">
                    这是一个失败种子：终端指向 {fmt(row.final_attitude_error_deg, 3)}° 超过 1° 门限。
                  </p>
                ) : null}
              </div>
            ) : null}
          </div>
        </div>

        {/* timeline */}
        {file && frames.length > 1 ? (
          <Timeline file={file} frames={frames} idx={Math.round(idx)} onSeek={jump} />
        ) : null}

        <p className="mt-8 max-w-3xl text-2xs leading-relaxed text-fg-subtle">
          轨迹文件：outputs/challenge-v3/replay/seed-*.json，由 {provenance.branch} 分支上的导出脚本生成；
          物理与审计文件未改动。回放本身不产生任何新结论，全部数字来自已提交的产物。
        </p>
      </main>
    </div>
  );
}

/* ----------------------------------------------------------------- timeline */

function Timeline({
  file,
  frames,
  idx,
  onSeek,
}: {
  file: ReplayFile;
  frames: Frame[];
  idx: number;
  onSeek: (t: number) => void;
}) {
  const W = 1000;
  const H = 150;
  const dur = file.duration_s;

  const path = useMemo(() => {
    let d = "";
    for (let i = 0; i < frames.length; i++) {
      const f = frames[i]!;
      const x = (f.t / dur) * W;
      const y = logY(f.attitudeErrorDeg) * H;
      d += `${i === 0 ? "M" : "L"}${x.toFixed(1)},${y.toFixed(1)}`;
    }
    return d;
  }, [frames, dur]);

  // Nozzle raster: consecutive firing frames merged into one bar.
  const bars = useMemo(() => {
    const out: { i: number; x: number; w: number }[] = [];
    for (let n = 0; n < 6; n++) {
      let run = -1;
      for (let k = 0; k <= frames.length; k++) {
        const on = k < frames.length ? ((frames[k]!.thrusterMask >> n) & 1) === 1 : false;
        if (on && run < 0) run = k;
        if (!on && run >= 0) {
          const x0 = (frames[run]!.t / dur) * W;
          const x1 = (frames[Math.min(k, frames.length - 1)]!.t / dur) * W;
          out.push({ i: n, x: x0, w: Math.max(1, x1 - x0) });
          run = -1;
        }
      }
    }
    return out;
  }, [frames, dur]);

  const gateY = logY(file.gates.attitudeDeg) * H;
  const faultX = (file.scenario.faultTime / dur) * W;
  const dwellX = ((dur - file.dwell_window_s) / dur) * W;
  const headX = ((frames[idx]?.t ?? 0) / dur) * W;

  return (
    <div className="mt-6 rounded-sm border border-border bg-bg-elevated p-3">
      <div className="flex flex-wrap items-baseline justify-between gap-2">
        <div className="text-2xs uppercase tracking-[0.14em] text-fg-subtle">
          指向误差与喷口点火（点击可跳转）
        </div>
        <div className="font-mono text-2xs text-fg-subtle">
          纵轴对数 · 灰线 = 1° 门限 · 红线 = 故障注入 · 右端阴影 = 3 s 保持窗口
        </div>
      </div>
      <svg
        viewBox={`0 0 ${W} ${H + 60}`}
        className="mt-2 w-full cursor-crosshair"
        preserveAspectRatio="none"
        onClick={(e) => {
          const box = e.currentTarget.getBoundingClientRect();
          onSeek(((e.clientX - box.left) / box.width) * dur);
        }}
      >
        <rect x={dwellX} y={0} width={W - dwellX} height={H} fill="rgba(125,186,154,0.10)" />
        <line x1={0} y1={gateY} x2={W} y2={gateY} stroke="#6d7686" strokeWidth={1} strokeDasharray="4 4" />
        {[0.1, 1, 10, 100].map((v) => (
          <g key={v}>
            <line
              x1={0}
              y1={logY(v) * H}
              x2={W}
              y2={logY(v) * H}
              stroke="rgba(232,234,239,0.06)"
              strokeWidth={1}
            />
            <text x={4} y={logY(v) * H - 3} fill="#6d7686" fontSize={9} fontFamily="monospace">
              {v}°
            </text>
          </g>
        ))}
        <line x1={faultX} y1={0} x2={faultX} y2={H} stroke="#d07272" strokeWidth={1} />
        {file.replans.map((r, i) => (
          <line
            key={i}
            x1={(r.t / dur) * W}
            y1={H - 6}
            x2={(r.t / dur) * W}
            y2={H}
            stroke="#6a92c8"
            strokeWidth={1}
          />
        ))}
        <path d={path} fill="none" stroke="#c5ccd8" strokeWidth={1.4} vectorEffect="non-scaling-stroke" />
        {bars.map((b, i) => (
          <rect
            key={i}
            x={b.x}
            y={H + 8 + b.i * 8}
            width={b.w}
            height={6}
            fill="#c88858"
            opacity={0.85}
          />
        ))}
        <line x1={headX} y1={0} x2={headX} y2={H + 56} stroke="#e8eaef" strokeWidth={1} />
      </svg>
      <div className="mt-1 flex justify-between font-mono text-2xs text-fg-subtle">
        <span>0 s</span>
        <span>{Math.round(dur / 2)} s</span>
        <span>{dur} s</span>
      </div>
      <div className="mt-2 text-2xs leading-relaxed text-fg-subtle">
        蓝色短竖线是重规划时刻（共 {file.replan_count} 次）；下方橙色栅格自上而下是 0-5 号喷口的点火区间。
      </div>
    </div>
  );
}
