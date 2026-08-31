/**
 * Public-state reachability dataset.
 * Split is grouped by family id (shared axis / fault / fuel). States from
 * the same family never cross train and validation.
 */
import { defaultPublicConfig } from "../constants";
import { qmul, qnormalize, vnorm, vscale, type Quat, type Vec3 } from "../math3d";
import type { PublicConfig } from "../types";
import {
  capturedGates,
  searchState,
  type HarvestedState,
} from "./capture-reachability";
import { rolloutFromSimLike, type RolloutState } from "./rollout-model";
import {
  captureCostFromLabel,
  captureFeatures,
  proveInfeasible,
  type CaptureLabel,
  type FeatureVector,
} from "./reachability-label";

function qAxisAngle(axis: Vec3, angle: number): Quat {
  const n = vnorm(axis) || 1;
  const a: Vec3 = [axis[0] / n, axis[1] / n, axis[2] / n];
  const h = 0.5 * angle;
  const s = Math.sin(h);
  return qnormalize([Math.cos(h), a[0] * s, a[1] * s, a[2] * s]);
}

function u01(i: number, salt: number): number {
  let x = Math.imul(i + 1, 0x9e3779b1) ^ Math.imul(salt, 0x85ebca6b);
  x = Math.imul(x ^ (x >>> 16), 0x7feb352d);
  x = Math.imul(x ^ (x >>> 15), 0x846ca68b);
  return ((x ^ (x >>> 16)) >>> 0) / 4294967296;
}

export interface DatasetSample {
  id: string;
  groupId: string;
  split: "train" | "val";
  isolated: readonly number[];
  features: FeatureVector;
  label: CaptureLabel;
  cost: number;
  firstPrimitiveId: string | null;
  captureTimeS: number | null;
  minAttDeg: number;
  finalOmega: number;
  fuelUsedKg: number;
  method: string;
  horizonS: number;
}

export interface DatasetReport {
  n: number;
  nTrain: number;
  nVal: number;
  nGroups: number;
  nTrainGroups: number;
  nValGroups: number;
  split: "grouped-family";
  counts: Record<CaptureLabel, number>;
  samples: DatasetSample[];
}

export function splitOfGroup(groupId: string): "train" | "val" {
  let h = 2166136261;
  for (let i = 0; i < groupId.length; i += 1) h = Math.imul(h ^ groupId.charCodeAt(i), 16777619);
  return ((h >>> 0) % 5 === 0 ? "val" : "train") as "train" | "val";
}

/** @deprecated id-hash split leaked nearby states; use splitOfGroup */
export function splitOf(id: string): "train" | "val" {
  const g = id.includes("::") ? id.slice(0, id.indexOf("::")) : id;
  return splitOfGroup(g);
}

const TRAJ_ATT_DEG = [22, 14, 8, 4, 2] as const;

function sampleFamilyMember(group: number, step: number, plant: PublicConfig): HarvestedState {
  const ax: Vec3 = [0.2 + u01(group, 2), 0.4 + u01(group, 3), 0.1 + u01(group, 4)];
  const attDeg = TRAJ_ATT_DEG[step % TRAJ_ATT_DEG.length]!;
  const qErr = qAxisAngle(ax, (attDeg * Math.PI) / 180);
  const q = qnormalize(qmul(plant.qTarget, qErr));
  const n = vnorm(ax) || 1;
  const eN: Vec3 = [ax[0] / n, ax[1] / n, ax[2] / n];
  const mode = u01(group, 5);
  const closing0 = mode < 0.45 ? -0.014 : mode < 0.75 ? 0 : 0.01;
  const closing = closing0 * (0.4 + 0.6 * (1 - step / 4));
  const perp = (u01(group, 7) - 0.5) * 0.024 * (1 - 0.15 * step);
  const pAx: Vec3 = [eN[1], -eN[0], 0];
  const pn = vnorm(pAx) || 1;
  const w: Vec3 = [
    eN[0] * closing + (pAx[0] / pn) * perp,
    eN[1] * closing + (pAx[1] / pn) * perp,
    eN[2] * closing,
  ];
  const faultRoll = u01(group, 8);
  const isolated = faultRoll < 0.55 ? [] : [Math.floor(u01(group, 9) * 6)];
  const fuel = 2.9 + u01(group, 10) * 1.5;
  const pending =
    u01(group, 11) < 0.18 && isolated[0] !== 0
      ? [{ id: 0, tOn: 0.04, tOff: 0.16 }]
      : [];
  const state: RolloutState = rolloutFromSimLike({
    time: 0,
    q,
    w,
    s: (u01(group, 12) - 0.5) * 0.8,
    sd: 0,
    th1: (u01(group, 13) - 0.5) * 0.1,
    th1d: 0,
    th2: (u01(group, 14) - 0.5) * 0.08,
    th2d: 0,
    fuel,
    pendingPulses: pending,
  });
  const groupId = `fam-${group}`;
  return {
    id: `${groupId}::${step}`,
    bucketDeg: attDeg,
    rateMode: closing < -1e-4 ? "closing" : "rest",
    fault: isolated.length ? "plusY-isolated" : "healthy",
    isolated,
    state,
  };
}

export function labelOne(harvested: HarvestedState, plant: PublicConfig, horizonS: number, budget: number): DatasetSample {
  const groupId = harvested.id.includes("::") ? harvested.id.slice(0, harvested.id.indexOf("::")) : harvested.id;
  const split = splitOfGroup(groupId);
  const features = captureFeatures(harvested.state, harvested.isolated, plant);
  if (proveInfeasible(harvested.state, plant, harvested.isolated)) {
    const g = capturedGates(harvested.state, plant.qTarget);
    return {
      id: harvested.id,
      groupId,
      split,
      isolated: harvested.isolated,
      features,
      label: "proven_infeasible",
      cost: captureCostFromLabel("proven_infeasible", null, 0, g.attDeg, g.omega),
      firstPrimitiveId: null,
      captureTimeS: null,
      minAttDeg: g.attDeg,
      finalOmega: g.omega,
      fuelUsedKg: 0,
      method: "certificate",
      horizonS,
    };
  }
  const methods = ["eigen", "beam"] as const;
  let best = searchState(harvested, plant, "eigen", horizonS, budget);
  for (const m of methods.slice(1)) {
    const r = searchState(harvested, plant, m, horizonS, budget);
    const better =
      Number(r.captured) > Number(best.captured) ||
      (r.captured === best.captured && r.minAttDeg < best.minAttDeg - 1e-9);
    if (better) best = r;
  }
  const label: CaptureLabel = best.captured ? "captured" : "search_unreached";
  return {
    id: harvested.id,
    groupId,
    split,
    isolated: harvested.isolated,
    features,
    label,
    cost: captureCostFromLabel(label, best.captureTimeS, best.fuelUsedKg, best.minAttDeg, best.finalOmega),
    firstPrimitiveId: best.firstPrimitiveId,
    captureTimeS: best.captureTimeS,
    minAttDeg: best.minAttDeg,
    finalOmega: best.finalOmega,
    fuelUsedKg: best.fuelUsedKg,
    method: best.method,
    horizonS,
  };
}

export function buildDataset(
  n: number,
  plant: PublicConfig = defaultPublicConfig(),
  opts: { horizonS?: number; expansionBudget?: number; perGroup?: number } = {},
): DatasetReport {
  const horizonS = opts.horizonS ?? 8;
  const budget = opts.expansionBudget ?? 20;
  const perGroup = opts.perGroup ?? TRAJ_ATT_DEG.length;
  const nGroups = Math.max(1, Math.ceil(n / perGroup));
  const samples: DatasetSample[] = [];
  const counts: Record<CaptureLabel, number> = { captured: 0, search_unreached: 0, proven_infeasible: 0 };
  for (let g = 0; g < nGroups && samples.length < n; g += 1) {
    for (let step = 0; step < perGroup && samples.length < n; step += 1) {
      const harvested = sampleFamilyMember(g, step, plant);
      const row = labelOne(harvested, plant, horizonS, budget);
      counts[row.label] += 1;
      samples.push(row);
    }
  }
  const groups = new Set(samples.map((s) => s.groupId));
  const trainG = new Set(samples.filter((s) => s.split === "train").map((s) => s.groupId));
  const valG = new Set(samples.filter((s) => s.split === "val").map((s) => s.groupId));
  return {
    n: samples.length,
    nTrain: samples.filter((s) => s.split === "train").length,
    nVal: samples.filter((s) => s.split === "val").length,
    nGroups: groups.size,
    nTrainGroups: trainG.size,
    nValGroups: valG.size,
    split: "grouped-family",
    counts,
    samples,
  };
}

export function assertGroupedSplit(samples: readonly DatasetSample[]): string[] {
  const map = new Map<string, "train" | "val">();
  const leaks: string[] = [];
  for (const s of samples) {
    const prev = map.get(s.groupId);
    if (prev && prev !== s.split) leaks.push(s.groupId);
    map.set(s.groupId, s.split);
  }
  return [...new Set(leaks)];
}
