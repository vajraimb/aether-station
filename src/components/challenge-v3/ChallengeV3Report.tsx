import { useMemo, useState } from "react";
import {
  ArrowLeft,
  Ban,
  Box,
  CircleAlert,
  Hash,
  Minus,
  ShieldCheck,
  Target,
  TrendingDown,
} from "lucide-react";
import {
  ATTITUDE_GATE_DEG,
  DWELL_WINDOW_S,
  ablationAnswer,
  ablationQuestion,
  canonical,
  configs,
  convergenceConsequence,
  convergenceFinding,
  convergenceMethod,
  convergenceNotClaimed,
  everFailingSeeds,
  failingSeeds,
  findings,
  gateRows,
  identicalLadders,
  level1,
  level1Range,
  limitation,
  provenance,
  seeds,
  selectorNote,
  studies,
  terminalAuthority,
  type ClaimType,
  type Config,
} from "@/lib/challenge-v3/report-data";
import { ZH } from "@/lib/challenge-v3/zh";

/* ------------------------------------------------------------------ atoms */

function pct(x: number): string {
  return `${Math.round(x * 100)}%`;
}

function deg(x: number, d = 3): string {
  return `${x.toFixed(d)}°`;
}

function sci(x: number): string {
  return x.toExponential(2).replace("e", "e");
}

const CLAIM_LABEL: Record<ClaimType, string> = {
  measured: "实测",
  search_unreached: "搜索未达",
  proven: "已证明",
  hypothesis: "假设",
  deprecated: "已废弃",
};

function ClaimBadge({ type }: { type: ClaimType }) {
  const tone =
    type === "measured"
      ? "text-ok"
      : type === "search_unreached"
        ? "text-warn"
        : type === "proven"
          ? "text-ok"
          : "text-fg-subtle";
  return (
    <span
      className={`inline-flex shrink-0 items-center rounded-xs bg-bg-subtle px-1.5 py-0.5 font-mono text-2xs uppercase tracking-[0.1em] ${tone}`}
      title={type}
    >
      {CLAIM_LABEL[type]}
    </span>
  );
}

/**
 * Artefact prose, read in Chinese with the committed English original one click
 * away. If a translation is missing the original is shown rather than nothing,
 * so a gap is visible instead of silent.
 */
function Prose({
  zhKey,
  original,
  className = "text-xs leading-relaxed text-fg-muted",
}: {
  zhKey: string;
  original: string;
  className?: string;
}) {
  const zh = ZH[zhKey];
  if (!zh) return <p className={className}>{original}</p>;
  return (
    <div>
      <p className={className}>{zh}</p>
      <details className="group mt-1.5">
        <summary className="cursor-pointer list-none font-mono text-2xs text-fg-subtle transition-colors hover:text-fg-muted">
          <span className="group-open:hidden">▸ 产物原文</span>
          <span className="hidden group-open:inline">▾ 产物原文</span>
        </summary>
        <p className="mt-1.5 border-l border-border pl-3 font-mono text-2xs leading-relaxed text-fg-subtle">
          {original}
        </p>
      </details>
    </div>
  );
}

function Section({
  id,
  kicker,
  title,
  lede,
  children,
}: {
  id: string;
  kicker: string;
  title: string;
  lede?: React.ReactNode;
  children: React.ReactNode;
}) {
  return (
    <section id={id} className="scroll-mt-20 border-t border-border pt-10">
      <div className="text-2xs font-medium uppercase tracking-[0.18em] text-fg-subtle">
        {kicker}
      </div>
      <h2 className="mt-1.5 font-display text-xl font-semibold tracking-tight [word-break:keep-all] text-fg sm:text-2xl">
        {title}
      </h2>
      {lede ? (
        <div className="mt-3 max-w-3xl text-sm leading-relaxed text-fg-muted">{lede}</div>
      ) : null}
      <div className="mt-6">{children}</div>
    </section>
  );
}

function Card({ children, className = "" }: { children: React.ReactNode; className?: string }) {
  return (
    <div className={`rounded-md bg-bg-elevated p-4 shadow-[var(--shadow-border)] ${className}`}>
      {children}
    </div>
  );
}

/* ------------------------------------------------------------- log helper */

const LOG_MIN = 0.02;
const LOG_MAX = 10;

/**
 * Position on the log axis, as a percentage rounded to 4 decimals. Rounding is
 * required, not cosmetic: an unrounded float is serialised differently by the
 * server and the client and React reports a hydration mismatch.
 */
function logPct(v: number): number {
  const c = Math.min(Math.max(v, LOG_MIN), LOG_MAX);
  const f = (Math.log10(c) - Math.log10(LOG_MIN)) / (Math.log10(LOG_MAX) - Math.log10(LOG_MIN));
  return Math.round(f * 1e6) / 1e4;
}

const LOG_TICKS = [0.02, 0.1, 1, 10];

/* --------------------------------------------------------------- verdict */

function Verdict() {
  return (
    <div className="rounded-lg bg-bg-elevated p-5 shadow-[var(--shadow-border)] sm:p-6">
      <div className="flex flex-wrap items-center gap-3">
        <span className="inline-flex items-center gap-1.5 rounded-sm bg-fault/12 px-2.5 py-1 font-mono text-xs font-medium uppercase tracking-[0.12em] text-fault">
          <CircleAlert size={13} strokeWidth={2.2} />
          Gate 未通过
        </span>
        <span className="font-mono text-xs uppercase tracking-[0.12em] text-fg-subtle">
          TASK SOLVED: NO
        </span>
        <ClaimBadge type={provenance.claimType} />
      </div>
      <p className="mt-4 max-w-3xl text-sm leading-relaxed text-fg-muted">
        A（持续捕获）、B（committed-prefix 排序）、C（显式时间量化搜索）三项已实施并在同一 10 个
        Train seed 上全量实测；D（收敛梯）已完成；E（leave-one-seed-out
        selector）按其自身前提不适用。 四个配置中
        <span className="text-fg"> 没有任何一个超过 7/10</span>，因此按评审约定的停止规则， L2
        搜索局限已正式记录。
      </p>
      <div className="mt-5 grid gap-2 sm:grid-cols-2 lg:grid-cols-4">
        {gateRows.map((g) => {
          const ok = g.measured >= g.required;
          return (
            <div key={g.key} className="rounded-sm bg-bg-subtle p-3">
              <div className="flex items-baseline justify-between gap-2">
                <span className="text-2xs font-medium uppercase tracking-[0.12em] text-fg-subtle">
                  {g.label}
                </span>
                <span className={`font-mono text-lg tabular-nums ${ok ? "text-ok" : "text-fault"}`}>
                  {pct(g.measured)}
                </span>
              </div>
              <div className="relative mt-2.5 h-1.5 overflow-hidden rounded-full bg-bg">
                <div
                  className={`absolute inset-y-0 left-0 rounded-full ${ok ? "bg-ok" : "bg-fault"}`}
                  style={{ width: `${g.measured * 100}%` }}
                />
              </div>
              <div className="relative mt-1 h-3">
                <div
                  className="absolute top-0 -translate-x-1/2 font-mono text-2xs tabular-nums text-fg-subtle"
                  style={{ left: `${g.required * 100}%` }}
                >
                  ▲{pct(g.required)}
                </div>
              </div>
              <div className="mt-1 font-mono text-2xs text-fg-subtle">{g.detail}</div>
            </div>
          );
        })}
      </div>
    </div>
  );
}

/* --------------------------------------------------- per-seed attitude bars */

function SeedBars({ config }: { config: Config }) {
  const rows = useMemo(
    () =>
      seeds.map((s) => {
        const r = config.per_seed.find((x) => x.seed === s)!;
        return r;
      }),
    [config],
  );

  return (
    <div className="rounded-md bg-bg-elevated p-4 shadow-[var(--shadow-border)] sm:p-5">
      <div className="flex flex-wrap items-baseline justify-between gap-x-4 gap-y-1">
        <div className="font-mono text-xs text-fg">
          {config.short} · <span className="text-fg-muted">{config.title}</span>
        </div>
        <div className="font-mono text-2xs uppercase tracking-[0.1em] text-fg-subtle">
          终端指向误差 · 对数轴 · 门限 {ATTITUDE_GATE_DEG.toFixed(0)}°
        </div>
      </div>

      {/* One flex row: a fixed label column, then a single positioning context
          shared by the axis, the gate line and every bar, so all three are
          aligned by construction rather than by a hand-tuned calc(). */}
      <div className="mt-4 flex gap-2">
        <div className="w-14 shrink-0">
          <div className="h-6" />
          <div className="space-y-1">
            {rows.map((r) => (
              <div
                key={r.seed}
                className={`flex h-5 items-center justify-end font-mono text-2xs tabular-nums ${
                  r.passed ? "text-fg-muted" : "text-fault"
                }`}
              >
                {r.seed}
              </div>
            ))}
          </div>
        </div>

        <div className="relative min-w-0 flex-1">
          <div className="relative h-6 border-b border-border">
            {LOG_TICKS.map((t) => (
              <div
                key={t}
                className="absolute bottom-1 -translate-x-1/2 font-mono text-2xs tabular-nums text-fg-subtle"
                style={{ left: `${logPct(t)}%` }}
              >
                {t < 1 ? t.toFixed(2) : t.toFixed(0)}°
              </div>
            ))}
          </div>

          <div
            className="pointer-events-none absolute top-6 bottom-0 z-1 w-px bg-warn/50"
            style={{ left: `${logPct(ATTITUDE_GATE_DEG)}%` }}
          />

          <div className="mt-1 space-y-1">
            {rows.map((r) => {
              const w = logPct(r.final_attitude_error_deg);
              return (
                <div key={r.seed} className="group relative h-5 rounded-xs bg-bg-subtle/60">
                  <div
                    className={`absolute inset-y-0 left-0 rounded-xs ${r.passed ? "bg-ok/70" : "bg-fault/70"}`}
                    style={{ width: `${w}%` }}
                  />
                  <div
                    className="absolute inset-y-0 flex items-center pl-1.5 font-mono text-2xs tabular-nums whitespace-nowrap text-fg"
                    style={{ left: `${w}%` }}
                  >
                    {deg(r.final_attitude_error_deg)}
                    <span className="ml-2 hidden text-fg-subtle group-hover:inline">
                      角速度 {sci(r.final_angular_speed_rad_s)} · 燃料 {r.remaining_fuel_kg.toFixed(2)}{" "}
                      kg · 重规划 {r.replans} · rollout {r.rollouts.toLocaleString()}
                    </span>
                  </div>
                </div>
              );
            })}
          </div>
        </div>
      </div>

      <div className="mt-4 flex flex-wrap gap-x-5 gap-y-1 font-mono text-2xs text-fg-subtle">
        <span>
          通过 <span className="text-ok">{config.passCount}</span>/10
        </span>
        <span>最好 {deg(config.attitude_deg.best)}</span>
        <span>中位 {deg(config.attitude_deg.median)}</span>
        <span>最差 {deg(config.attitude_deg.worst)}</span>
        <span className="text-fg-subtle/70">原始行 {config.raw_rows}</span>
      </div>
    </div>
  );
}

/* ------------------------------------------------------------- pass matrix */

function PassMatrix({
  selected,
  onSelect,
}: {
  selected: number | null;
  onSelect: (s: number | null) => void;
}) {
  return (
    <div className="overflow-x-auto rounded-md bg-bg-elevated p-4 shadow-[var(--shadow-border)] sm:p-5">
      <table className="w-full min-w-[720px] border-separate border-spacing-0">
        <thead>
          <tr>
            <th className="pb-2 text-left text-2xs font-medium uppercase tracking-[0.12em] text-fg-subtle">
              配置
            </th>
            {seeds.map((s) => (
              <th key={s} className="px-0.5 pb-2">
                <button
                  type="button"
                  onClick={() => onSelect(selected === s ? null : s)}
                  className={`w-full font-mono text-2xs tabular-nums transition-colors hover:text-fg ${
                    selected === s
                      ? "text-fg"
                      : everFailingSeeds.includes(s)
                        ? "text-warn/80"
                        : "text-fg-subtle"
                  }`}
                >
                  {String(s).slice(-3)}
                </button>
              </th>
            ))}
            <th className="pb-2 pl-3 text-right text-2xs font-medium uppercase tracking-[0.12em] text-fg-subtle">
              通过
            </th>
          </tr>
        </thead>
        <tbody>
          {configs.map((c) => (
            <tr key={c.id}>
              <td className="py-1 pr-3 align-middle">
                <div className="flex items-center gap-2 whitespace-nowrap">
                  <span className="font-mono text-xs text-fg">{c.short}</span>
                  <span className="text-xs text-fg-muted">{c.title}</span>
                  {c.retained ? (
                    <span className="inline-flex items-center gap-1 rounded-xs bg-ok/12 px-1.5 py-0.5 font-mono text-2xs uppercase tracking-[0.1em] text-ok">
                      <ShieldCheck size={10} strokeWidth={2.4} />
                      保留
                    </span>
                  ) : (
                    <span className="inline-flex items-center gap-1 rounded-xs bg-bg-subtle px-1.5 py-0.5 font-mono text-2xs uppercase tracking-[0.1em] text-fg-subtle">
                      <Minus size={10} strokeWidth={2.4} />
                      未采用
                    </span>
                  )}
                  <span className="font-mono text-2xs text-fg-subtle">{c.items}</span>
                </div>
              </td>
              {seeds.map((s) => {
                const r = c.per_seed.find((x) => x.seed === s)!;
                const dim = selected !== null && selected !== s;
                return (
                  <td key={s} className="px-0.5 py-1">
                    <button
                      type="button"
                      onClick={() => onSelect(selected === s ? null : s)}
                      title={`${c.short} · seed ${s} · ${deg(r.final_attitude_error_deg)} · ${
                        r.passed ? "PASS" : "FAIL"
                      }`}
                      className={`flex h-7 w-full items-center justify-center rounded-xs font-mono text-2xs tabular-nums transition-opacity ${
                        r.passed ? "bg-ok/18 text-ok" : "bg-fault/18 text-fault"
                      } ${dim ? "opacity-30" : "opacity-100"}`}
                    >
                      {r.final_attitude_error_deg < 1
                        ? r.final_attitude_error_deg.toFixed(2).slice(1)
                        : r.final_attitude_error_deg.toFixed(1)}
                    </button>
                  </td>
                );
              })}
              <td className="py-1 pl-3 text-right font-mono text-xs tabular-nums text-fg">
                {c.passCount}/10
              </td>
            </tr>
          ))}
        </tbody>
      </table>
      <div className="mt-3 font-mono text-2xs leading-relaxed text-fg-subtle">
        单元格为终端指向误差（度，小于 1° 省略前导零）。列标签为 seed 后三位。
        <span className="text-warn/80"> 黄色 seed</span> 至少在一个配置下失败：共{" "}
        {everFailingSeeds.length} 个， 说明短板不是某几个固定 seed 的属性。
      </div>
    </div>
  );
}

function SeedDetail({ seed }: { seed: number }) {
  return (
    <Card className="mt-3">
      <div className="flex items-baseline gap-3">
        <span className="font-mono text-sm text-fg">seed {seed}</span>
        <span className="text-2xs uppercase tracking-[0.12em] text-fg-subtle">四配置逐项对照</span>
      </div>
      <div className="mt-3 overflow-x-auto">
        <table className="w-full min-w-[560px] text-left">
          <thead>
            <tr className="text-2xs uppercase tracking-[0.1em] text-fg-subtle">
              <th className="pb-1.5 pr-4 font-medium">配置</th>
              <th className="pb-1.5 pr-4 text-right font-medium">指向</th>
              <th className="pb-1.5 pr-4 text-right font-medium">角速度</th>
              <th className="pb-1.5 pr-4 text-right font-medium">燃料</th>
              <th className="pb-1.5 pr-4 text-right font-medium">重规划</th>
              <th className="pb-1.5 text-right font-medium">rollout</th>
            </tr>
          </thead>
          <tbody className="font-mono text-xs tabular-nums">
            {configs.map((c) => {
              const r = c.per_seed.find((x) => x.seed === seed)!;
              return (
                <tr key={c.id} className="border-t border-border">
                  <td className="py-1.5 pr-4 text-fg-muted">{c.short}</td>
                  <td className={`py-1.5 pr-4 text-right ${r.passed ? "text-ok" : "text-fault"}`}>
                    {deg(r.final_attitude_error_deg)}
                  </td>
                  <td className="py-1.5 pr-4 text-right text-fg-muted">
                    {sci(r.final_angular_speed_rad_s)}
                  </td>
                  <td className="py-1.5 pr-4 text-right text-fg-muted">
                    {r.remaining_fuel_kg.toFixed(2)}
                  </td>
                  <td className="py-1.5 pr-4 text-right text-fg-muted">{r.replans}</td>
                  <td className="py-1.5 text-right text-fg-muted">{r.rollouts.toLocaleString()}</td>
                </tr>
              );
            })}
          </tbody>
        </table>
      </div>
    </Card>
  );
}

/* ------------------------------------------------------- convergence ladder */

function Convergence() {
  const [idx, setIdx] = useState(0);
  const study = studies[idx]!;
  const ladder = study.budget_ladder;
  const maxRollouts = Math.max(...ladder.map((r) => r.rollouts));
  const attValues = ladder.map((r) => r.best_attitude_deg);
  const attMin = Math.min(...attValues);
  const attMax = Math.max(...attValues);
  const span = Math.max(attMax - attMin, 0.2);
  const lo = attMin - span * 0.35;
  const hi = attMax + span * 0.35;
  const H = 128;
  const y = (v: number) => H - ((v - lo) / (hi - lo)) * H;
  const x = (i: number) => (i / (ladder.length - 1)) * 100;

  return (
    <div>
      <div className="flex flex-wrap gap-1.5">
        {studies.map((s, i) => (
          <button
            key={s.key}
            type="button"
            onClick={() => setIdx(i)}
            className={`rounded-sm px-2.5 py-1.5 font-mono text-2xs tabular-nums transition-colors ${
              i === idx
                ? "bg-bg-hover text-fg shadow-[var(--shadow-border)]"
                : "bg-bg-subtle text-fg-muted hover:text-fg"
            }`}
          >
            {s.seed} @ {s.epoch_s.toFixed(0)}s
            {s.best_sequence_unchanged_across_ladder ? (
              <span className="ml-1.5 text-warn">≡</span>
            ) : null}
          </button>
        ))}
      </div>

      <div className="mt-3 grid gap-3 lg:grid-cols-[1fr_1.15fr]">
        <Card>
          <div className="text-2xs uppercase tracking-[0.12em] text-fg-subtle">
            最优指向误差 vs 搜索预算
          </div>
          <div className="relative mt-4 pl-10">
            <svg viewBox={`0 0 100 ${H}`} preserveAspectRatio="none" className="h-32 w-full">
              <line
                x1="0"
                y1={y(attMin)}
                x2="100"
                y2={y(attMin)}
                stroke="rgb(232 234 239 / 0.08)"
                strokeWidth="0.4"
              />
              <polyline
                points={ladder.map((r, i) => `${x(i)},${y(r.best_attitude_deg)}`).join(" ")}
                fill="none"
                stroke="var(--color-fault)"
                strokeWidth="0.8"
                vectorEffect="non-scaling-stroke"
              />
              {ladder.map((r, i) => (
                <circle
                  key={r.budget_multiplier}
                  cx={x(i)}
                  cy={y(r.best_attitude_deg)}
                  r="1.6"
                  fill="var(--color-fault)"
                />
              ))}
            </svg>
            <div className="absolute top-0 left-0 flex h-32 flex-col justify-between font-mono text-2xs tabular-nums text-fg-subtle">
              <span>{hi.toFixed(2)}°</span>
              <span>{lo.toFixed(2)}°</span>
            </div>
            <div className="mt-1.5 flex justify-between font-mono text-2xs tabular-nums text-fg-subtle">
              {ladder.map((r) => (
                <span key={r.budget_multiplier}>{r.budget_multiplier}×</span>
              ))}
            </div>
          </div>
          <div className="mt-4 space-y-1.5 font-mono text-2xs text-fg-subtle">
            <div>
              rollout 增长 <span className="text-fg">{study.rollout_growth.toFixed(1)}×</span>（
              {ladder[0]!.rollouts.toLocaleString()} → {maxRollouts.toLocaleString()}）
            </div>
            <div>
              指向改善{" "}
              <span className={study.attitude_improvement_deg === 0 ? "text-fault" : "text-warn"}>
                {study.attitude_improvement_deg === 0
                  ? "0.000°"
                  : `${study.attitude_improvement_deg.toFixed(3)}°`}
              </span>
              {" · "}不同最优序列 <span className="text-fg">{study.distinct_best_sequences}</span>{" "}
              个
            </div>
            <div className="flex items-center gap-1.5">
              <Hash size={11} strokeWidth={2.2} />
              {study.best_sequence_unchanged_across_ladder ? (
                <span className="text-warn">1× 与 8× 的最优动作表逐位相同</span>
              ) : (
                <span className="text-fg-muted">最优动作表在梯上发生变化</span>
              )}
            </div>
          </div>
        </Card>

        <Card>
          <div className="text-2xs uppercase tracking-[0.12em] text-fg-subtle">预算梯逐档记录</div>
          <div className="mt-3 overflow-x-auto">
            <table className="w-full min-w-[440px] text-left">
              <thead>
                <tr className="text-2xs uppercase tracking-[0.1em] text-fg-subtle">
                  <th className="pb-1.5 pr-3 font-medium">预算</th>
                  <th className="pb-1.5 pr-3 text-right font-medium">B&amp;B 节点</th>
                  <th className="pb-1.5 pr-3 text-right font-medium">CEM 代</th>
                  <th className="pb-1.5 pr-3 text-right font-medium">rollout</th>
                  <th className="pb-1.5 pr-3 text-right font-medium">指向</th>
                  <th className="pb-1.5 pr-3 text-center font-medium">dwell</th>
                  <th className="pb-1.5 font-medium">序列哈希</th>
                </tr>
              </thead>
              <tbody className="font-mono text-2xs tabular-nums">
                {ladder.map((r) => (
                  <tr key={r.budget_multiplier} className="border-t border-border">
                    <td className="py-1.5 pr-3 text-fg">{r.budget_multiplier}×</td>
                    <td className="py-1.5 pr-3 text-right text-fg-muted">
                      {r.bnb_node_expansions}
                    </td>
                    <td className="py-1.5 pr-3 text-right text-fg-muted">
                      {r.cem_generations}/{r.cem_population}
                    </td>
                    <td className="py-1.5 pr-3 text-right text-fg-muted">
                      {r.rollouts.toLocaleString()}
                    </td>
                    <td className="py-1.5 pr-3 text-right text-fault">
                      {r.best_attitude_deg.toFixed(3)}
                    </td>
                    <td className="py-1.5 pr-3 text-center">
                      {r.dwell_held ? (
                        <span className="text-ok">保持</span>
                      ) : (
                        <span className="text-fault">未保持</span>
                      )}
                    </td>
                    <td className="py-1.5 text-fg-subtle">
                      {r.sequence_sha256_16}
                      {r.segments === 0 ? <span className="ml-1.5 text-warn">空序列</span> : null}
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
          <div className="mt-3 font-mono text-2xs leading-relaxed text-fg-subtle">
            预算按 B&amp;B 节点扩展、CEM 种群与代数、精化宽度、槽位与 polish 种子缩放。
            <span className="text-fg-muted"> 任何停止条件都不读时钟。</span>
            {ladder.some((r) => r.segments === 0) ? (
              <span className="text-warn">
                {" "}
                哈希 4f53cda18c2baa0c 即空动作表的 SHA-256——获胜方案是纯 coast。
              </span>
            ) : null}
          </div>
        </Card>
      </div>
    </div>
  );
}

/* ------------------------------------------------------------------- page */

/**
 * Where the header links point. The router app uses paths; the standalone
 * bundle has no router and passes hash targets instead, so the same component
 * ships in both without a second copy of the header.
 */
export interface PageLinks {
  home: string;
  report: string;
  replay: string;
}

export const DEFAULT_LINKS: PageLinks = {
  home: "/",
  report: "/challenge-v3",
  replay: "/challenge-v3/replay",
};

export function ChallengeV3Report({ links = DEFAULT_LINKS }: { links?: PageLinks } = {}) {
  const [cfgIdx, setCfgIdx] = useState(configs.findIndex((c) => c.retained));
  const [seed, setSeed] = useState<number | null>(null);
  const config = configs[cfgIdx] ?? canonical;

  return (
    <div className="min-h-dvh bg-bg">
      <header className="sticky top-0 z-10 border-b border-border bg-overlay backdrop-blur-md">
        <div className="mx-auto flex max-w-6xl items-center justify-between gap-4 px-4 py-3 sm:px-6">
          <div className="flex items-center gap-3">
            {/* A plain anchor, not a router Link: this component is also mounted
                by the standalone report bundle, which has no router. */}
            <a
              href={links.home}
              className="inline-flex items-center gap-1.5 rounded-sm bg-bg-subtle px-2.5 py-1.5 text-2xs uppercase tracking-[0.12em] text-fg-muted transition-colors hover:text-fg"
            >
              <ArrowLeft size={12} strokeWidth={2.4} />
              任务控制台
            </a>
            <a
              href={links.replay}
              className="inline-flex items-center gap-1.5 rounded-sm bg-bg-subtle px-2.5 py-1.5 text-2xs uppercase tracking-[0.12em] text-fg-muted transition-colors hover:text-fg"
            >
              <Box size={12} strokeWidth={2.4} />
              三维回放
            </a>
            <div className="font-mono text-xs text-fg-subtle">
              AETHER · Challenge V3 · 真值态 L2
            </div>
          </div>
          <div className="hidden font-mono text-2xs text-fg-subtle sm:block">
            {provenance.branch} @ {provenance.commitShort}
          </div>
        </div>
      </header>

      <main className="mx-auto max-w-6xl px-4 pb-24 sm:px-6">
        <div className="pt-10 pb-8">
          <div className="text-2xs font-medium uppercase tracking-[0.18em] text-fg-subtle">
            后续轮次 A–E · Train-10 公开划分
          </div>
          {/* keep-all stops the line breaking inside a Chinese word: without it
              the browser splits 排|序, which is legal CJK wrapping but reads badly. */}
          <h1 className="mt-2 max-w-3xl font-display text-2xl leading-tight font-semibold tracking-tight text-balance [word-break:keep-all] text-fg sm:text-4xl">
            持续捕获、committed-prefix 排序与显式时间量化搜索的实测结果
          </h1>
          <p className="mt-4 max-w-3xl text-sm leading-relaxed text-fg-muted">
            本页的每个数字都直接来自仓库中已提交的产物文件，不在前端重新计算任何指标；
            派生量只有图形几何与通过计数，后者与产物自述的通过率互相校验。
          </p>
          <div className="mt-6">
            <Verdict />
          </div>
        </div>

        <div className="space-y-12">
          <Section
            id="seeds"
            kicker="保留配置"
            title="逐 seed 终端指向误差"
            lede={
              <>
                持续捕获要求在整个{" "}
                <span className="font-mono text-fg">[T−{DWELL_WINDOW_S.toFixed(0)}, T]</span>{" "}
                窗口内采样指向与角速度并取窗口最差值，因此自由漂移掠过目标无法得分——掠过意味着角速度大到会离开窗口。
                这一意图确实生效：通过的 seed 终端角速度降到 1e-4 量级；但它没有把任何失败 seed
                转成通过。
              </>
            }
          >
            <div className="mb-3 flex flex-wrap gap-1.5">
              {configs.map((c, i) => (
                <button
                  key={c.id}
                  type="button"
                  onClick={() => setCfgIdx(i)}
                  className={`rounded-sm px-2.5 py-1.5 font-mono text-2xs transition-colors ${
                    i === cfgIdx
                      ? "bg-bg-hover text-fg shadow-[var(--shadow-border)]"
                      : "bg-bg-subtle text-fg-muted hover:text-fg"
                  }`}
                >
                  {c.short} {c.passCount}/10
                  {c.retained ? <span className="ml-1.5 text-ok">●</span> : null}
                </button>
              ))}
            </div>
            <SeedBars config={config} />
            <div className="mt-3 font-mono text-2xs text-fg-subtle">
              保留配置下失败的 seed：
              <span className="text-fault"> {failingSeeds.join(" · ")}</span>
              。悬停任一行可看角速度、燃料、重规划次数与 rollout 数。
            </div>
          </Section>

          <Section
            id="ablation"
            kicker="目标函数消融"
            title="四个配置，同一批 seed"
            lede={
              <>
                <div className="text-fg">{ZH.question ?? ablationQuestion}</div>
                <div className="mt-3 flex items-start gap-2">
                  <ClaimBadge type={ablationAnswer.claim_type} />
                  <Prose
                    zhKey="answer"
                    original={ablationAnswer.text}
                    className="text-sm leading-relaxed text-fg-muted"
                  />
                </div>
              </>
            }
          >
            <PassMatrix selected={seed} onSelect={setSeed} />
            {seed !== null ? <SeedDetail seed={seed} /> : null}
            <div className="mt-4 grid gap-3 md:grid-cols-2">
              {configs.map((c) => (
                <Card key={c.id}>
                  <div className="flex items-center gap-2">
                    <span className="font-mono text-xs text-fg">{c.short}</span>
                    <span className="text-xs text-fg-muted">{c.title}</span>
                    <ClaimBadge type={c.claim_type} />
                  </div>
                  <div className="mt-2">
                    <Prose zhKey={c.id} original={c.description} />
                  </div>
                </Card>
              ))}
            </div>
          </Section>

          <Section
            id="convergence"
            kicker="后续项 D"
            title="收敛梯：预算不是短板的原因"
            lede={
              <Prose
                zhKey="method"
                original={convergenceMethod.text}
                className="text-sm leading-relaxed text-fg-muted"
              />
            }
          >
            <Convergence />
            <div className="mt-4 grid gap-3 md:grid-cols-2">
              <Card>
                <div className="flex items-center gap-2">
                  <TrendingDown size={13} strokeWidth={2.2} className="text-warn" />
                  <span className="text-xs font-medium tracking-wide text-fg">结论</span>
                  <ClaimBadge type={convergenceFinding.claim_type} />
                </div>
                <div className="mt-2">
                  <Prose zhKey="finding" original={convergenceFinding.text} />
                </div>
                <div className="mt-3 font-mono text-2xs text-fg-subtle">
                  四条梯中 <span className="text-warn">{identicalLadders}</span> 条在 1× 与 8×
                  返回逐位相同的最优序列。
                </div>
              </Card>
              <Card>
                <div className="flex items-center gap-2">
                  <Target size={13} strokeWidth={2.2} className="text-warn" />
                  <span className="text-xs font-medium tracking-wide text-fg">推论</span>
                  <ClaimBadge type={convergenceConsequence.claim_type} />
                </div>
                <div className="mt-2">
                  <Prose zhKey="consequence" original={convergenceConsequence.text} />
                </div>
              </Card>
            </div>
            <Card className="mt-3">
              <div className="flex items-center gap-2">
                <Ban size={13} strokeWidth={2.2} className="text-fg-subtle" />
                <span className="text-xs font-medium tracking-wide text-fg">明确不声称</span>
              </div>
              <div className="mt-2">
                <Prose zhKey="explicitly_not_claimed" original={convergenceNotClaimed} />
              </div>
            </Card>
          </Section>

          <Section
            id="findings"
            kicker="负结果"
            title="本轮建立的发现，包括未采用的修复"
            lede="改动一律按实测结果保留或回退；被回退的修复不删除，作为负结果记录在 failure-analysis.json 中。"
          >
            <div className="space-y-2.5">
              {findings.map((f) => (
                <Card key={f.id}>
                  <div className="flex flex-wrap items-center gap-2">
                    <span className="text-xs font-medium tracking-wide text-fg">{f.title}</span>
                    <ClaimBadge type={f.claim_type} />
                    <span className="font-mono text-2xs text-fg-subtle">{f.id}</span>
                  </div>
                  <div className="mt-2">
                    <Prose zhKey={f.id} original={f.finding} />
                  </div>
                </Card>
              ))}
            </div>
          </Section>

          <Section
            id="levels"
            kicker="能力层级对照"
            title="差距在脉冲量化的候选文法，不在物理"
            lede="L1 是能力探针，不是提交方案：它用同一套受审计的积分器，但把力矩作为理想连续量注入。它 10/10 通过，而同一物理下的脉冲序列只有 7/10，说明差距在候选文法与终端权限，不在动力学。"
          >
            <div className="grid gap-3 md:grid-cols-2">
              <Card>
                <div className="flex items-baseline justify-between">
                  <span className="text-xs font-medium tracking-wide text-fg">L1 理想连续力矩</span>
                  <span className="font-mono text-lg tabular-nums text-ok">
                    {pct(level1Range.passRate)}
                  </span>
                </div>
                <div className="mt-3 space-y-1 font-mono text-2xs text-fg-muted">
                  <div>
                    终端指向 {deg(level1Range.attMin, 4)} – {deg(level1Range.attMax, 4)}
                  </div>
                  <div>
                    终端角速度 {sci(level1Range.omMin)} – {sci(level1Range.omMax)} rad/s
                  </div>
                </div>
                <div className="mt-3">
                  <Prose
                    zhKey="L1def"
                    original={level1.definition}
                    className="text-2xs leading-relaxed text-fg-subtle"
                  />
                </div>
              </Card>
              <Card>
                <div className="flex items-baseline justify-between">
                  <span className="text-xs font-medium tracking-wide text-fg">
                    L2 真值态脉冲序列（保留配置）
                  </span>
                  <span className="font-mono text-lg tabular-nums text-fault">
                    {pct(canonical.all_gates_pass_rate)}
                  </span>
                </div>
                <div className="mt-3 space-y-1 font-mono text-2xs text-fg-muted">
                  <div>
                    终端指向 {deg(canonical.attitude_deg.best)} –{" "}
                    {deg(canonical.attitude_deg.worst)}
                  </div>
                  <div>角速度门与燃料门 100% 通过，指向是唯一绑定门</div>
                </div>
                <p className="mt-3 text-2xs leading-relaxed text-fg-subtle">
                  终端权限的量化表述：最小 {(terminalAuthority.minPulseS * 1000).toFixed(0)} ms
                  脉冲对应最小可指令角速度变化约 {sci(terminalAuthority.deltaOmega)}{" "}
                  rad/s，用单量子抹掉 1° 残差需要约 {terminalAuthority.leverArmS} s
                  力臂，而候选时序表已经吃掉了 horizon。
                </p>
              </Card>
            </div>
          </Section>

          <Section
            id="limitation"
            kicker="后续项 E 与停止规则"
            title="正式记录的局限"
            lede="按评审约定：8× 预算与 dwell、committed-prefix 修复之后仍低于 9/10，则停止并正式记录局限。"
          >
            <Card>
              <div className="flex flex-wrap items-center gap-2">
                <span className="text-xs font-medium tracking-wide text-fg">
                  {limitation.title}
                </span>
                <ClaimBadge type={limitation.claim_type} />
              </div>
              <div className="mt-2">
                <Prose zhKey="limitation" original={limitation.statement} />
              </div>
            </Card>
            <Card className="mt-3">
              <div className="flex flex-wrap items-center gap-2">
                <span className="text-xs font-medium tracking-wide text-fg">
                  E · leave-one-seed-out selector 未构建
                </span>
                <ClaimBadge type={selectorNote.claim_type} />
              </div>
              <div className="mt-2">
                <Prose zhKey="selector" original={selectorNote.text} />
              </div>
            </Card>
          </Section>

          <Section
            id="provenance"
            kicker="溯源"
            title="这些数字来自哪里"
            lede={
              <Prose
                zhKey="claim"
                original={provenance.claim}
                className="text-sm leading-relaxed text-fg-muted"
              />
            }
          >
            <div className="grid gap-2 sm:grid-cols-2 lg:grid-cols-3">
              {[
                { k: "分支", v: provenance.branch },
                { k: "产物记录的提交", v: provenance.commitShort },
                { k: "起始提交", v: provenance.startCommitShort },
                { k: "物理基线", v: provenance.physicsBaseline },
                { k: "样本数", v: `Train-${provenance.n}` },
                {
                  k: "不可变文件 diff",
                  v: provenance.immutableDiff === "none" ? "无差异" : provenance.immutableDiff,
                },
              ].map((r) => (
                <div key={r.k} className="rounded-sm bg-bg-subtle px-3 py-2">
                  <div className="text-2xs font-medium uppercase tracking-[0.12em] text-fg-subtle">
                    {r.k}
                  </div>
                  <div className="mt-0.5 font-mono text-xs break-all text-fg">{r.v}</div>
                </div>
              ))}
            </div>
            <Card className="mt-3">
              <div className="text-2xs font-medium uppercase tracking-[0.12em] text-fg-subtle">
                确定性
              </div>
              <div className="mt-1.5 flex flex-wrap gap-1.5">
                <span className="rounded-xs bg-ok/12 px-1.5 py-0.5 font-mono text-2xs text-ok">
                  固定随机种子 {provenance.fixedSeed ? "是" : "否"}
                </span>
                <span className="rounded-xs bg-ok/12 px-1.5 py-0.5 font-mono text-2xs text-ok">
                  时钟停止条件 {provenance.wallClockDeadlines ? "有" : "无"}
                </span>
              </div>
              <div className="mt-2">
                <Prose zhKey="determinism" original={provenance.determinism} />
              </div>
              <div className="mt-3 text-2xs font-medium uppercase tracking-[0.12em] text-fg-subtle">
                受哈希锁定的不可变文件
              </div>
              <div className="mt-1.5 flex flex-wrap gap-1.5">
                {provenance.immutableFiles.map((f) => (
                  <span
                    key={f}
                    className="rounded-xs bg-bg-subtle px-1.5 py-0.5 font-mono text-2xs text-fg-muted"
                  >
                    {f}
                  </span>
                ))}
              </div>
            </Card>
            <div className="mt-3 font-mono text-2xs leading-relaxed text-fg-subtle">
              产物：outputs/challenge-v3/objective-ablation.json · search-convergence.json ·
              truth-optimizer-train10.json · failure-analysis.json · level-baseline.json；
              原始行：outputs/challenge-v3/raw/*.jsonl
            </div>
          </Section>
        </div>
      </main>
    </div>
  );
}
