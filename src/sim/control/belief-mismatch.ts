/**
 * Belief-versus-truth mismatch metrics. Pure functions: no Simulator,
 * no scenario object, no wall-clock fault identity. The closed-loop
 * harvest lives in the CLI.
 */
import {
  attitudeErrorAngle,
  deg,
  vdot,
  vnorm,
  vsub,
  type Quat,
  type Vec3,
} from "../math3d";
import { occupancyAt, type PendingPulse } from "./discrete-actions";
import { TERMINAL_ENTRY_DEG } from "./terminal-reachable";

export const AUDIT_PHASES = ["nominal", "pre_fault", "post_fault", "terminal"] as const;
export type AuditPhase = (typeof AUDIT_PHASES)[number];

export const PRE_FAULT_WINDOW_S = 10;

export interface BeliefSnapshot {
  readonly q: Quat;
  readonly w: Vec3;
  readonly s: number;
  readonly sd: number;
  readonly th1: number;
  readonly th1d: number;
  readonly th2: number;
  readonly th2d: number;
  readonly fuel: number;
  readonly bias: Vec3;
  readonly c1: number;
  readonly c2: number;
  readonly k12: number;
  readonly etaT: number;
  readonly isolated: readonly number[];
}

export interface TruthSnapshot {
  readonly q: Quat;
  readonly w: Vec3;
  readonly s: number;
  readonly sd: number;
  readonly th1: number;
  readonly th1d: number;
  readonly th2: number;
  readonly th2d: number;
  readonly fuel: number;
  readonly gyroBias: Vec3;
  readonly c1: number;
  readonly c2: number;
  readonly k12: number;
  readonly etaT: number;
  readonly failedThruster: number;
  readonly faultHasOccurred: boolean;
}

export interface MismatchSample {
  readonly attGeodesicDeg: number;
  readonly attTruthDeg: number;
  readonly attBeliefDeg: number;
  readonly wErr: number;
  readonly wAngleDeg: number;
  readonly fuelErr: number;
  readonly sloshAngleErr: number;
  readonly sloshRateErr: number;
  readonly sliderErr: number;
  readonly sliderRateErr: number;
  readonly biasErr: number;
  readonly fdirMaskMismatch: boolean;
  readonly pendingMismatchCount: number;
  readonly pendingExpectedOn: number;
  readonly pendingActualOn: number;
  readonly paramRelErr: number;
}

export function classifyPhase(args: {
  readonly t: number;
  readonly faultTime: number;
  readonly attDegTruth: number;
  readonly terminalEntryDeg?: number;
  readonly preFaultWindowS?: number;
}): AuditPhase {
  const entry = args.terminalEntryDeg ?? TERMINAL_ENTRY_DEG;
  if (args.attDegTruth <= entry + 1e-12) return "terminal";
  if (args.t + 1e-12 >= args.faultTime) return "post_fault";
  const window = args.preFaultWindowS ?? PRE_FAULT_WINDOW_S;
  if (args.t + 1e-12 >= args.faultTime - window) return "pre_fault";
  return "nominal";
}

function angleBetween(a: Vec3, b: Vec3): number {
  const na = vnorm(a);
  const nb = vnorm(b);
  if (na < 1e-12 || nb < 1e-12) return na < 1e-12 && nb < 1e-12 ? 0 : Math.PI;
  const c = Math.min(1, Math.max(-1, vdot(a, b) / (na * nb)));
  return Math.acos(c);
}

function relErr(est: number, truth: number): number {
  return Math.abs(est - truth) / Math.max(Math.abs(truth), 1e-6);
}

export function mismatchAt(
  belief: BeliefSnapshot,
  truth: TruthSnapshot,
  pending: readonly PendingPulse[],
  actualOn: readonly number[],
  nowS: number,
  maxActive = 2,
): MismatchSample {
  const attGeodesicDeg = deg(attitudeErrorAngle(belief.q, truth.q));
  const attTruthDeg = deg(attitudeErrorAngle(truth.q));
  const attBeliefDeg = deg(attitudeErrorAngle(belief.q));
  const wErr = vnorm(vsub(belief.w, truth.w));
  const wAngleDeg = deg(angleBetween(belief.w, truth.w));
  const isolated = new Set(belief.isolated);
  const expected = occupancyAt(pending, nowS, isolated, maxActive);
  const expectedSet = new Set(expected);
  let actualCount = 0;
  let mismatch = 0;
  for (let i = 0; i < 6; i += 1) {
    const on = (actualOn[i] ?? 0) > 0.5;
    if (on) actualCount += 1;
    const exp = expectedSet.has(i);
    if (on !== exp) mismatch += 1;
  }
  const truthFailed = truth.faultHasOccurred ? truth.failedThruster : -1;
  const estFailed = belief.isolated.length === 1 ? belief.isolated[0]! : belief.isolated.length === 0 ? -1 : -2;
  const fdirMaskMismatch = truthFailed !== estFailed;
  const paramRelErr =
    (relErr(belief.c1, truth.c1) +
      relErr(belief.c2, truth.c2) +
      relErr(belief.k12, truth.k12) +
      relErr(belief.etaT, truth.etaT)) /
    4;
  return {
    attGeodesicDeg,
    attTruthDeg,
    attBeliefDeg,
    wErr,
    wAngleDeg,
    fuelErr: belief.fuel - truth.fuel,
    sloshAngleErr: Math.hypot(belief.th1 - truth.th1, belief.th2 - truth.th2),
    sloshRateErr: Math.hypot(belief.th1d - truth.th1d, belief.th2d - truth.th2d),
    sliderErr: belief.s - truth.s,
    sliderRateErr: belief.sd - truth.sd,
    biasErr: vnorm(vsub(belief.bias, truth.gyroBias)),
    fdirMaskMismatch,
    pendingMismatchCount: mismatch,
    pendingExpectedOn: expected.length,
    pendingActualOn: actualCount,
    paramRelErr,
  };
}

export interface Quantiles {
  readonly n: number;
  readonly mean: number;
  readonly p50: number;
  readonly p90: number;
  readonly worst: number;
}

export function quantiles(values: readonly number[]): Quantiles {
  if (values.length === 0) return { n: 0, mean: NaN, p50: NaN, p90: NaN, worst: NaN };
  const s = [...values].sort((a, b) => a - b);
  const mean = s.reduce((a, b) => a + b, 0) / s.length;
  const at = (p: number) => s[Math.min(s.length - 1, Math.max(0, Math.ceil(p * s.length) - 1))]!;
  const absWorst = s.reduce((w, v) => (Math.abs(v) > Math.abs(w) ? v : w), s[0]!);
  return { n: s.length, mean, p50: at(0.5), p90: at(0.9), worst: absWorst };
}

export interface PhaseSummary {
  readonly phase: AuditPhase;
  readonly n: number;
  readonly attGeodesicDeg: Quantiles;
  readonly wErr: Quantiles;
  readonly fuelErr: Quantiles;
  readonly sloshAngleErr: Quantiles;
  readonly fdirMismatchRate: number;
  readonly pendingMismatchRate: number;
  readonly paramRelErr: Quantiles;
  readonly meanAttTruthDeg: number;
  readonly meanAttBeliefDeg: number;
}

export function summarizePhase(phase: AuditPhase, samples: readonly MismatchSample[]): PhaseSummary {
  const n = samples.length;
  const rate = (pred: (s: MismatchSample) => boolean) => (n === 0 ? NaN : samples.filter(pred).length / n);
  return {
    phase,
    n,
    attGeodesicDeg: quantiles(samples.map((s) => s.attGeodesicDeg)),
    wErr: quantiles(samples.map((s) => s.wErr)),
    fuelErr: quantiles(samples.map((s) => s.fuelErr)),
    sloshAngleErr: quantiles(samples.map((s) => s.sloshAngleErr)),
    fdirMismatchRate: rate((s) => s.fdirMaskMismatch),
    pendingMismatchRate: rate((s) => s.pendingMismatchCount > 0),
    paramRelErr: quantiles(samples.map((s) => s.paramRelErr)),
    meanAttTruthDeg: n === 0 ? NaN : samples.reduce((a, s) => a + s.attTruthDeg, 0) / n,
    meanAttBeliefDeg: n === 0 ? NaN : samples.reduce((a, s) => a + s.attBeliefDeg, 0) / n,
  };
}
