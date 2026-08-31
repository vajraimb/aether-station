/**
 * Post-fault reachable torque set under the RCS constraints.
 * Pure geometry — does not touch the controller or a wall-clock fault time.
 */
import { vadd, vdot, vnorm, vscale, type Vec3 } from "./math3d";
import { torqueColumns } from "./allocate";
import { MIN_PULSE, THRUSTERS } from "./constants";
import { defaultPublicConfig } from "./constants";
import type { PublicConfig } from "./types";

function svd3xN(cols: Vec3[]): { sv: Vec3; rank: number; cond: number } {
  // Singular values of 3×N B are sqrt(eig(B Bᵀ)). Jacobi on the 3×3 Gram.
  const G = [
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
  ];
  for (const c of cols) {
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) G[i]![j]! += c[i]! * c[j]!;
  }
  let A = G.map((r) => r.slice()) as number[][];
  for (let iter = 0; iter < 32; iter++) {
    let p = 0, q = 1, best = Math.abs(A[0]![1]!);
    const pairs: [number, number][] = [[0, 1], [0, 2], [1, 2]];
    for (const [i, j] of pairs) {
      const a = Math.abs(A[i]![j]!);
      if (a > best) {
        best = a;
        p = i;
        q = j;
      }
    }
    if (best < 1e-14) break;
    const app = A[p]![p]!;
    const aqq = A[q]![q]!;
    const apq = A[p]![q]!;
    const tau = (aqq - app) / (2 * apq);
    const t = Math.sign(tau) / (Math.abs(tau) + Math.hypot(1, tau));
    const c = 1 / Math.hypot(1, t);
    const s = t * c;
    for (let k = 0; k < 3; k++) {
      if (k === p || k === q) continue;
      const aik = A[k]![p]!;
      const aiq = A[k]![q]!;
      A[k]![p] = A[p]![k] = c * aik - s * aiq;
      A[k]![q] = A[q]![k] = s * aik + c * aiq;
    }
    A[p]![p] = app - t * apq;
    A[q]![q] = aqq + t * apq;
    A[p]![q] = A[q]![p] = 0;
  }
  const ev = [A[0]![0]!, A[1]![1]!, A[2]![2]!].map((x) => Math.max(0, x)).sort((a, b) => b - a);
  const sv: Vec3 = [Math.sqrt(ev[0]!), Math.sqrt(ev[1]!), Math.sqrt(ev[2]!)];
  const rank = sv.filter((x) => x > 1e-6).length;
  const cond = sv[2]! > 1e-9 ? sv[0]! / sv[2]! : Infinity;
  return { sv, rank, cond };
}

function twoJetHull(cols: Vec3[], isolated: Set<number>): Vec3[] {
  const live: number[] = [];
  for (let i = 0; i < cols.length; i++) if (!isolated.has(i)) live.push(i);
  const verts: Vec3[] = [[0, 0, 0]];
  for (const i of live) {
    verts.push(cols[i]!);
    for (const j of live) {
      if (j <= i) continue;
      verts.push(vadd(cols[i]!, cols[j]!));
    }
  }
  return verts;
}

function minAxisImpulse(cols: Vec3[], isolated: Set<number>, pulse = MIN_PULSE): Vec3 {
  const live: number[] = [];
  for (let i = 0; i < cols.length; i++) if (!isolated.has(i)) live.push(i);
  const acc: Vec3 = [Infinity, Infinity, Infinity];
  const consider = (H: Vec3) => {
    for (let a = 0; a < 3; a++) {
      const v = Math.abs(H[a]!);
      if (v > 1e-8 && v < acc[a]!) acc[a] = v;
    }
  };
  for (const i of live) consider(vscale(cols[i]!, pulse));
  for (let a = 0; a < live.length; a++) {
    for (let b = a + 1; b < live.length; b++) {
      consider(vscale(vadd(cols[live[a]!]!, cols[live[b]!]!), pulse));
    }
  }
  return acc.map((x) => (Number.isFinite(x) ? x : 0)) as Vec3;
}

/** Brute-force sequential min-pulse net impulse (n_i in 0..3). */
function minNetSequentialImpulse(cols: Vec3[], isolated: Set<number>, pulse = MIN_PULSE): {
  minAbs: number;
  vector: Vec3;
  counts: number[];
} {
  const live: number[] = [];
  for (let i = 0; i < 6; i++) if (!isolated.has(i)) live.push(i);
  let best = Infinity;
  let bestV: Vec3 = [0, 0, 0];
  let bestN: number[] = [];
  const nLive = live.length;
  const maxN = 3;
  const counts = new Array(nLive).fill(0);
  const rec = (k: number, remaining: number) => {
    if (k === nLive) {
      let H: Vec3 = [0, 0, 0];
      let tot = 0;
      for (let i = 0; i < nLive; i++) {
        tot += counts[i]!;
        if (counts[i]) H = vadd(H, vscale(cols[live[i]!]!, counts[i]! * pulse));
      }
      if (tot === 0) return;
      const n = vnorm(H);
      if (n > 1e-8 && n < best) {
        best = n;
        bestV = H;
        bestN = counts.slice();
      }
      return;
    }
    for (let c = 0; c <= maxN && c <= remaining; c++) {
      counts[k] = c;
      rec(k + 1, remaining - c);
    }
  };
  rec(0, 6);
  const full = new Array(6).fill(0);
  for (let i = 0; i < nLive; i++) full[live[i]!] = bestN[i] ?? 0;
  return { minAbs: best, vector: bestV, counts: full };
}

export interface ReachabilityReport {
  preFault: {
    columns: number[][];
    names: string[];
    rank: number;
    singularValues: number[];
    conditionNumber: number;
    twoJetVertices: number[][];
    minAxisImpulse: number[];
    minSequentialNetImpulse: number;
  };
  postFault: {
    failed: number;
    columns: number[][];
    names: string[];
    rank: number;
    singularValues: number[];
    conditionNumber: number;
    twoJetVertices: number[][];
    minAxisImpulse: number[];
    minSequentialNetImpulse: number;
    minSequentialVector: number[];
    sequentialCounts: number[];
  };
}

export function analyseReachability(
  failed: number,
  cfg: PublicConfig = defaultPublicConfig(),
  s = 0,
  eta = 0.873,
): ReachabilityReport {
  const cols = torqueColumns(cfg, s, 0, 0, cfg.initialFuelMass, eta);
  const names = THRUSTERS.map((g) => g.name);
  const pack = (isolated: Set<number>) => {
    const liveCols = cols.map((c, i) => (isolated.has(i) ? ([0, 0, 0] as Vec3) : c));
    const used = cols.filter((_, i) => !isolated.has(i));
    const svd = svd3xN(used);
    const hull = twoJetHull(cols, isolated);
    const axis = minAxisImpulse(cols, isolated);
    const seq = minNetSequentialImpulse(cols, isolated);
    return {
      columns: liveCols.map((c) => [...c]),
      names: names.filter((_, i) => !isolated.has(i)),
      rank: svd.rank,
      singularValues: [...svd.sv],
      conditionNumber: svd.cond,
      twoJetVertices: hull.map((v) => [...v]),
      minAxisImpulse: [...axis],
      minSequentialNetImpulse: seq.minAbs,
      minSequentialVector: [...seq.vector],
      sequentialCounts: seq.counts,
    };
  };
  const pre = pack(new Set());
  const post = pack(new Set([failed]));
  return {
    preFault: {
      columns: cols.map((c) => [...c]),
      names,
      rank: pre.rank,
      singularValues: pre.singularValues,
      conditionNumber: pre.conditionNumber,
      twoJetVertices: pre.twoJetVertices,
      minAxisImpulse: pre.minAxisImpulse,
      minSequentialNetImpulse: pre.minSequentialNetImpulse,
    },
    postFault: {
      failed,
      columns: post.columns,
      names: post.names,
      rank: post.rank,
      singularValues: post.singularValues,
      conditionNumber: post.conditionNumber,
      twoJetVertices: post.twoJetVertices,
      minAxisImpulse: post.minAxisImpulse,
      minSequentialNetImpulse: post.minSequentialNetImpulse,
      minSequentialVector: post.minSequentialVector,
      sequentialCounts: post.sequentialCounts,
    },
  };
}

void vdot;
