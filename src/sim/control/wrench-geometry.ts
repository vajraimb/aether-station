/**
 * Offline wrench-geometry study. Not a controller.
 *
 * For each isolation mask, build the body-frame torque matrix of legal
 * singles (and pairs), report rank / singular values / condition, and
 * how well a random eigenaxis can be reached without perpendicular leak.
 */
import { FMAX, THRUSTERS } from "../constants";
import { massState } from "../dynamics";
import { vdot, vnorm, vsub, vscale, type Mat3, type Vec3 } from "../math3d";
import type { PublicConfig } from "../types";
import {
  generatePulsePrimitives,
  netWrenchForPrimitive,
  type PulseDurationS,
} from "./discrete-actions";

export const WRENCH_PULSE_S = 0.04 as PulseDurationS;
const EPS = 1e-12;
const SIGMA_FLOOR = 1e-8;

export interface Svd3 {
  readonly values: Vec3;
  readonly rank: number;
  readonly cond: number;
}

export interface AxisRho {
  readonly axis: Vec3;
  readonly bestRho: number;
  readonly bestPar: number;
  readonly bestPerp: number;
  readonly bestId: string;
}

export interface MaskWrenchReport {
  readonly mask: string;
  readonly isolated: readonly number[];
  readonly nSingles: number;
  readonly nPairs: number;
  readonly singles: Svd3;
  readonly all: Svd3;
  readonly axisCount: number;
  readonly rhoSingles: { median: number; p10: number; p90: number; mean: number };
  readonly rhoAll: { median: number; p10: number; p90: number; mean: number };
  readonly fractionRhoGt4: number;
  readonly fractionRhoGt10: number;
  readonly meanBestPar: number;
  readonly meanBestPerp: number;
}

function cloneMat(m: Mat3): number[][] {
  return [
    [m[0][0], m[0][1], m[0][2]],
    [m[1][0], m[1][1], m[1][2]],
    [m[2][0], m[2][1], m[2][2]],
  ];
}

/** Jacobi eigen-decomposition of a 3×3 symmetric matrix. Values descending. */
export function jacobiEigen3(S: Mat3): { values: Vec3; vectors: Mat3 } {
  const a = cloneMat(S);
  const v = [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
  ];
  for (let iter = 0; iter < 40; iter += 1) {
    let p = 0;
    let q = 1;
    let max = Math.abs(a[0][1]!);
    const offs: Array<[number, number]> = [
      [0, 2],
      [1, 2],
    ];
    for (const [i, j] of offs) {
      const x = Math.abs(a[i]![j]!);
      if (x > max) {
        max = x;
        p = i;
        q = j;
      }
    }
    if (max < 1e-15) break;
    const app = a[p]![p]!;
    const aqq = a[q]![q]!;
    const apq = a[p]![q]!;
    const tau = (aqq - app) / (2 * apq);
    const t = Math.abs(tau) < 1e-18 ? 1 : Math.sign(tau) / (Math.abs(tau) + Math.hypot(1, tau));
    const c = 1 / Math.hypot(1, t);
    const s = t * c;
    a[p]![p] = app - t * apq;
    a[q]![q] = aqq + t * apq;
    a[p]![q] = 0;
    a[q]![p] = 0;
    for (let k = 0; k < 3; k += 1) {
      if (k === p || k === q) continue;
      const akp = a[k]![p]!;
      const akq = a[k]![q]!;
      a[k]![p] = c * akp - s * akq;
      a[p]![k] = a[k]![p]!;
      a[k]![q] = s * akp + c * akq;
      a[q]![k] = a[k]![q]!;
    }
    for (let k = 0; k < 3; k += 1) {
      const vkp = v[k]![p]!;
      const vkq = v[k]![q]!;
      v[k]![p] = c * vkp - s * vkq;
      v[k]![q] = s * vkp + c * vkq;
    }
  }
  const vals: Vec3 = [a[0][0], a[1][1], a[2][2]];
  const order = [0, 1, 2].sort((i, j) => vals[j]! - vals[i]!);
  const values: Vec3 = [vals[order[0]!], vals[order[1]!], vals[order[2]!]];
  const vectors: Mat3 = [
    [v[0]![order[0]!]!, v[0]![order[1]!]!, v[0]![order[2]!]!],
    [v[1]![order[0]!]!, v[1]![order[1]!]!, v[1]![order[2]!]!],
    [v[2]![order[0]!]!, v[2]![order[1]!]!, v[2]![order[2]!]!],
  ];
  return { values, vectors };
}

export function svdFromColumns(columns: readonly Vec3[]): Svd3 {
  let aat: Mat3 = [
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
  ];
  for (const col of columns) {
    for (let i = 0; i < 3; i += 1) {
      for (let j = 0; j < 3; j += 1) {
        aat[i]![j] = aat[i]![j]! + col[i]! * col[j]!;
      }
    }
  }
  const eig = jacobiEigen3(aat);
  const values: Vec3 = [
    Math.sqrt(Math.max(0, eig.values[0])),
    Math.sqrt(Math.max(0, eig.values[1])),
    Math.sqrt(Math.max(0, eig.values[2])),
  ];
  const rank = values.filter((s) => s > SIGMA_FLOOR).length;
  const cond = rank < 3 || values[2] < SIGMA_FLOOR ? Number.POSITIVE_INFINITY : values[0] / values[2];
  return { values, rank, cond };
}

export function fibonacciSphere(n: number): Vec3[] {
  const out: Vec3[] = [];
  const count = Math.max(1, n);
  const golden = Math.PI * (3 - Math.sqrt(5));
  for (let i = 0; i < count; i += 1) {
    const y = count === 1 ? 0 : 1 - (i / (count - 1)) * 2;
    const r = Math.sqrt(Math.max(0, 1 - y * y));
    const theta = golden * i;
    out.push([Math.cos(theta) * r, y, Math.sin(theta) * r]);
  }
  return out;
}

export function rhoOfTorque(tau: Vec3, e: Vec3): { rho: number; par: number; perp: number } {
  const par = vdot(tau, e);
  const perpV = vsub(tau, vscale(e, par));
  const perp = vnorm(perpV);
  return { rho: Math.abs(par) / (perp + EPS), par, perp };
}

export interface TorqueColumn {
  readonly id: string;
  readonly thrusterIds: readonly number[];
  readonly tau: Vec3;
  readonly force: Vec3;
  readonly propellantKg: number;
}

export function torqueColumnsForMask(
  plant: PublicConfig,
  isolated: ReadonlySet<number>,
  etaT: number,
  rCmB: Vec3,
  kinds: "singles" | "all" = "all",
): TorqueColumn[] {
  const all = generatePulsePrimitives(THRUSTERS, {
    isolatedThrusters: isolated,
    durationsS: [WRENCH_PULSE_S],
    includeCoast: false,
    commandedThrustN: Math.min(plant.maxThrust, FMAX),
  });
  const picked =
    kinds === "singles" ? all.filter((p) => p.thrusterIds.length === 1) : all.filter((p) => p.thrusterIds.length > 0);
  const out: TorqueColumn[] = [];
  for (const p of picked) {
    const w = netWrenchForPrimitive(p, THRUSTERS, etaT, rCmB, plant.maxThrust);
    out.push({
      id: p.id,
      thrusterIds: p.thrusterIds,
      tau: w.torqueB,
      force: w.forceB,
      propellantKg: w.propellantKg,
    });
  }
  return out;
}

function quantiles(values: readonly number[]): { median: number; p10: number; p90: number; mean: number } {
  if (values.length === 0) return { median: NaN, p10: NaN, p90: NaN, mean: NaN };
  const s = [...values].sort((a, b) => a - b);
  const at = (p: number) => s[Math.min(s.length - 1, Math.max(0, Math.ceil(p * s.length) - 1))]!;
  return {
    median: at(0.5),
    p10: at(0.1),
    p90: at(0.9),
    mean: s.reduce((a, b) => a + b, 0) / s.length,
  };
}

function bestRho(columns: readonly TorqueColumn[], e: Vec3): AxisRho {
  let best: AxisRho = { axis: e, bestRho: 0, bestPar: 0, bestPerp: Infinity, bestId: "" };
  for (const c of columns) {
    const r = rhoOfTorque(c.tau, e);
    if (r.rho > best.bestRho) {
      best = { axis: e, bestRho: r.rho, bestPar: r.par, bestPerp: r.perp, bestId: c.id };
    }
  }
  return best;
}

export function isolationMasks(): Array<{ name: string; isolated: number[] }> {
  const masks: Array<{ name: string; isolated: number[] }> = [{ name: "healthy", isolated: [] }];
  for (let i = 0; i < 6; i += 1) masks.push({ name: `isolated-${i}`, isolated: [i] });
  return masks;
}

export function analyzeMask(
  plant: PublicConfig,
  isolated: readonly number[],
  axes: readonly Vec3[],
  etaT: number,
  rCmB: Vec3,
): MaskWrenchReport {
  const set = new Set(isolated);
  const singles = torqueColumnsForMask(plant, set, etaT, rCmB, "singles");
  const all = torqueColumnsForMask(plant, set, etaT, rCmB, "all");
  const svdS = svdFromColumns(singles.map((c) => c.tau));
  const svdA = svdFromColumns(all.map((c) => c.tau));
  const rhoS = axes.map((e) => bestRho(singles, e));
  const rhoA = axes.map((e) => bestRho(all, e));
  const rhos = rhoA.map((r) => r.bestRho);
  return {
    mask: isolated.length === 0 ? "healthy" : `isolated-${isolated.join("+")}`,
    isolated: [...isolated],
    nSingles: singles.length,
    nPairs: all.length - singles.length,
    singles: svdS,
    all: svdA,
    axisCount: axes.length,
    rhoSingles: quantiles(rhoS.map((r) => r.bestRho)),
    rhoAll: quantiles(rhos),
    fractionRhoGt4: rhos.filter((x) => x > 4).length / Math.max(1, rhos.length),
    fractionRhoGt10: rhos.filter((x) => x > 10).length / Math.max(1, rhos.length),
    meanBestPar: rhoA.reduce((a, r) => a + Math.abs(r.bestPar), 0) / Math.max(1, rhoA.length),
    meanBestPerp: rhoA.reduce((a, r) => a + r.bestPerp, 0) / Math.max(1, rhoA.length),
  };
}

export function defaultCm(plant: PublicConfig): Vec3 {
  return massState(plant, 0.28, 0.08, -0.05, 4.2).rCmB;
}

export function runWrenchStudy(
  plant: PublicConfig,
  opts: { axisCount?: number; etaT?: number } = {},
): { etaT: number; rCmB: Vec3; pulseS: number; masks: MaskWrenchReport[] } {
  const etaT = opts.etaT ?? 0.873;
  const rCmB = defaultCm(plant);
  const axes = fibonacciSphere(opts.axisCount ?? 200);
  const masks = isolationMasks().map((m) => analyzeMask(plant, m.isolated, axes, etaT, rCmB));
  return { etaT, rCmB, pulseS: WRENCH_PULSE_S, masks };
}


