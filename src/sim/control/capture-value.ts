/**
 * Auditable k-NN capture-cost model.
 * V(q, ω, fuel, mask, pending) ≈ neighbor-weighted capture cost.
 * If the nearest neighbor is farther than the OOD threshold, the cost
 * falls back to a transparent eigen heuristic and must not be trusted.
 */
import type { PublicConfig } from "../types";
import type { RolloutState } from "./rollout-model";
import { captureFeatures, FEATURE_NAMES, type CaptureLabel, type FeatureVector } from "./reachability-label";
import type { DatasetSample } from "./reachability-dataset";
import bundled from "./data/capture-value-knn.v1.json" with { type: "json" };

export const FEATURE_SCHEMA_VERSION = "v1";
export const TRAINER_VERSION = "knn-7-l2-scaled";

export interface KnnNeighbor {
  x: FeatureVector;
  cost: number;
  label: CaptureLabel;
  firstPrimitiveId: string | null;
}

export interface KnnTable {
  k: number;
  scale: number[];
  neighbors: KnnNeighbor[];
  nTrain: number;
  nVal: number;
  valMae: number | null;
  oodThreshold: number;
  nnTrainP50: number;
  nnTrainP90: number;
  featureSchemaVersion: string;
  trainerVersion: string;
  datasetHash: string | null;
  splitHash: string | null;
  modelHash: string | null;
  notes: string;
}

export interface ValueQuery {
  cost: number;
  label: CaptureLabel | "heuristic";
  firstPrimitiveId: string | null;
  nUsed: number;
  nnDist: number;
  ood: boolean;
  capturedProb: number;
}

const DEFAULT_SCALE = [0.4, 0.04, 0.02, 0.08, 1.2, 2, 1, 1.2];

export function dist2(a: FeatureVector, b: FeatureVector, scale: number[]): number {
  let s = 0;
  const n = Math.min(a.length, b.length, scale.length);
  for (let i = 0; i < n; i += 1) {
    const d = (a[i]! - b[i]!) / (scale[i] || 1);
    s += d * d;
  }
  return s;
}

export function nnDistance(a: FeatureVector, b: FeatureVector, scale: number[]): number {
  return Math.sqrt(dist2(a, b, scale));
}

function percentile(sorted: number[], p: number): number {
  if (sorted.length === 0) return 0;
  const i = Math.min(sorted.length - 1, Math.max(0, Math.ceil(p * sorted.length) - 1));
  return sorted[i]!;
}

function trainNnDistances(neighbors: readonly KnnNeighbor[], scale: number[]): number[] {
  const out: number[] = [];
  for (let i = 0; i < neighbors.length; i += 1) {
    let best = Infinity;
    for (let j = 0; j < neighbors.length; j += 1) {
      if (i === j) continue;
      const d = nnDistance(neighbors[i]!.x, neighbors[j]!.x, scale);
      if (d < best) best = d;
    }
    if (Number.isFinite(best)) out.push(best);
  }
  out.sort((a, b) => a - b);
  return out;
}

export function fitKnn(samples: readonly DatasetSample[], k = 7, hashes?: { datasetHash?: string; splitHash?: string }): KnnTable {
  const train = samples.filter((s) => s.split === "train");
  const val = samples.filter((s) => s.split === "val");
  const neighbors: KnnNeighbor[] = train.map((s) => ({
    x: s.features.slice(),
    cost: s.cost,
    label: s.label,
    firstPrimitiveId: s.firstPrimitiveId,
  }));
  const nn = trainNnDistances(neighbors, DEFAULT_SCALE);
  const p50 = percentile(nn, 0.5);
  const p90 = percentile(nn, 0.9);
  const oodThreshold = Math.max(p90 * 1.5, p50 * 3, 1e-3);
  const table: KnnTable = {
    k,
    scale: DEFAULT_SCALE.slice(),
    neighbors,
    nTrain: train.length,
    nVal: val.length,
    valMae: null,
    oodThreshold,
    nnTrainP50: p50,
    nnTrainP90: p90,
    featureSchemaVersion: FEATURE_SCHEMA_VERSION,
    trainerVersion: TRAINER_VERSION,
    datasetHash: hashes?.datasetHash ?? null,
    splitHash: hashes?.splitHash ?? null,
    modelHash: null,
    notes: "k-NN on grouped-split public labels. OOD falls back to eigen heuristic. search_unreached is not proven_infeasible.",
  };
  if (val.length > 0 && neighbors.length > 0) {
    let mae = 0;
    for (const s of val) {
      const q = queryTable(table, s.features);
      mae += Math.abs(q.cost - s.cost);
    }
    table.valMae = mae / val.length;
  }
  return table;
}

export function heuristicCost(x: FeatureVector): number {
  const attRad = x[0] ?? 0;
  const wPar = x[1] ?? 0;
  const wPerp = x[2] ?? 0;
  const wMag = x[3] ?? 0;
  const fuelMargin = x[4] ?? 0;
  const nIso = x[5] ?? 0;
  const attDeg = (attRad * 180) / Math.PI;
  const restIso = nIso > 0 && wMag < 0.004 && attDeg > 1;
  if (fuelMargin <= 0 && wMag < 1e-4 && attDeg >= 1) return 200;
  if (restIso && attDeg > 1.2) return 90 + attDeg;
  return Math.max(0.5, 0.35 * attDeg + 40 * Math.max(0, wPar) + 80 * wPerp + 8 * Math.max(0, -fuelMargin));
}

export function queryTable(table: KnnTable, x: FeatureVector): ValueQuery {
  if (table.neighbors.length === 0) {
    return {
      cost: heuristicCost(x),
      label: "heuristic",
      firstPrimitiveId: null,
      nUsed: 0,
      nnDist: Infinity,
      ood: true,
      capturedProb: 0,
    };
  }
  const scored = table.neighbors.map((nb, i) => ({ i, d: dist2(x, nb.x, table.scale) }));
  scored.sort((a, b) => a.d - b.d || a.i - b.i);
  const nnDist = Math.sqrt(scored[0]!.d);
  const ood = nnDist > (table.oodThreshold || Infinity);
  if (ood) {
    return {
      cost: heuristicCost(x),
      label: "heuristic",
      firstPrimitiveId: null,
      nUsed: 0,
      nnDist,
      ood: true,
      capturedProb: 0,
    };
  }
  const take = scored.slice(0, Math.min(table.k, scored.length));
  let wsum = 0;
  let csum = 0;
  let capW = 0;
  const labels: Record<string, number> = {};
  let bestFirst: string | null = take[0] ? table.neighbors[take[0].i]!.firstPrimitiveId : null;
  for (const t of take) {
    const nb = table.neighbors[t.i]!;
    const w = 1 / (t.d + 1e-6);
    wsum += w;
    csum += w * nb.cost;
    if (nb.label === "captured") capW += w;
    labels[nb.label] = (labels[nb.label] ?? 0) + w;
    if (bestFirst == null) bestFirst = nb.firstPrimitiveId;
  }
  let label: CaptureLabel = "search_unreached";
  let bestW = -1;
  for (const [k, w] of Object.entries(labels)) {
    if (w > bestW) {
      bestW = w;
      label = k as CaptureLabel;
    }
  }
  return {
    cost: csum / Math.max(wsum, 1e-12),
    label,
    firstPrimitiveId: bestFirst,
    nUsed: take.length,
    nnDist,
    ood: false,
    capturedProb: capW / Math.max(wsum, 1e-12),
  };
}

let ACTIVE: KnnTable = {
  k: 7,
  scale: DEFAULT_SCALE.slice(),
  neighbors: [],
  nTrain: 0,
  nVal: 0,
  valMae: null,
  oodThreshold: 1,
  nnTrainP50: 0,
  nnTrainP90: 0,
  featureSchemaVersion: FEATURE_SCHEMA_VERSION,
  trainerVersion: TRAINER_VERSION,
  datasetHash: null,
  splitHash: null,
  modelHash: null,
  notes: "empty — heuristic fallback",
};

const bundledTable = bundled as KnnTable;
if (Array.isArray(bundledTable.neighbors) && bundledTable.neighbors.length > 0) {
  ACTIVE = bundledTable;
}

export function setCaptureValueTable(table: KnnTable): void {
  ACTIVE = table;
}

export function getCaptureValueTable(): KnnTable {
  return ACTIVE;
}

export function captureCost(state: RolloutState, isolated: readonly number[], plant: PublicConfig): number {
  const x = captureFeatures(state, isolated, plant);
  return queryTable(ACTIVE, x).cost;
}

export { FEATURE_NAMES };
