/**
 * Grouped-split validation for the capture-value k-NN.
 * Accuracy is not reported as a headline — captured is the rare class.
 */
import type { DatasetSample } from "./reachability-dataset";
import { FEATURE_SCHEMA_VERSION, TRAINER_VERSION, nnDistance, queryTable, type KnnTable } from "./capture-value";
import type { CaptureLabel } from "./reachability-label";

function rocAuc(y: number[], score: number[]): number {
  const n = y.length;
  if (n === 0) return NaN;
  const pos = y.reduce((a, b) => a + b, 0);
  const neg = n - pos;
  if (pos === 0 || neg === 0) return NaN;
  const order = score.map((s, i) => i).sort((a, b) => score[b]! - score[a]! || a - b);
  let tp = 0;
  let fp = 0;
  let prevTpr = 0;
  let prevFpr = 0;
  let auc = 0;
  let prevS: number | null = null;
  for (const i of order) {
    if (prevS !== null && score[i] !== prevS) {
      auc += (fp / neg - prevFpr) * (tp / pos + prevTpr) * 0.5;
      prevTpr = tp / pos;
      prevFpr = fp / neg;
    }
    prevS = score[i]!;
    if (y[i]) tp += 1;
    else fp += 1;
  }
  auc += (1 - prevFpr) * (1 + prevTpr) * 0.5;
  return auc;
}

function prAuc(y: number[], score: number[]): number {
  const n = y.length;
  const pos = y.reduce((a, b) => a + b, 0);
  if (n === 0 || pos === 0) return NaN;
  const order = score.map((s, i) => i).sort((a, b) => score[b]! - score[a]! || a - b);
  let tp = 0;
  let fp = 0;
  let prevRec = 0;
  let prevPrec = 1;
  let auc = 0;
  for (const i of order) {
    if (y[i]) tp += 1;
    else fp += 1;
    const rec = tp / pos;
    const prec = tp / (tp + fp);
    auc += (rec - prevRec) * (prec + prevPrec) * 0.5;
    prevRec = rec;
    prevPrec = prec;
  }
  return auc;
}

function confusion(y: number[], pred: number[]) {
  let tp = 0, fp = 0, tn = 0, fn = 0;
  for (let i = 0; i < y.length; i += 1) {
    if (y[i] && pred[i]) tp += 1;
    else if (!y[i] && pred[i]) fp += 1;
    else if (!y[i] && !pred[i]) tn += 1;
    else fn += 1;
  }
  const prec = tp + fp === 0 ? 0 : tp / (tp + fp);
  const rec = tp + fn === 0 ? 0 : tp / (tp + fn);
  const f1 = prec + rec === 0 ? 0 : (2 * prec * rec) / (prec + rec);
  return { tp, fp, tn, fn, precision: prec, recall: rec, f1 };
}

function calibration(y: number[], p: number[], bins = 5) {
  const out: Array<{ lo: number; hi: number; n: number; meanP: number; meanY: number }> = [];
  for (let b = 0; b < bins; b += 1) {
    const lo = b / bins;
    const hi = (b + 1) / bins;
    let n = 0, sp = 0, sy = 0;
    for (let i = 0; i < y.length; i += 1) {
      const ok = b === bins - 1 ? p[i]! >= lo && p[i]! <= hi : p[i]! >= lo && p[i]! < hi;
      if (!ok) continue;
      n += 1;
      sp += p[i]!;
      sy += y[i]!;
    }
    out.push({ lo, hi, n, meanP: n ? sp / n : NaN, meanY: n ? sy / n : NaN });
  }
  return out;
}

function attBucket(attRad: number): string {
  const d = (attRad * 180) / Math.PI;
  if (d < 3) return "<3";
  if (d < 8) return "3-8";
  if (d < 15) return "8-15";
  return ">=15";
}

export function validateCaptureValue(table: KnnTable, samples: readonly DatasetSample[]) {
  const train = samples.filter((s) => s.split === "train");
  const val = samples.filter((s) => s.split === "val");
  const count = (xs: readonly DatasetSample[]) => {
    const c: Record<CaptureLabel, number> = { captured: 0, search_unreached: 0, proven_infeasible: 0 };
    for (const s of xs) c[s.label] += 1;
    return c;
  };
  const y = val.map((s) => (s.label === "captured" ? 1 : 0));
  const queries = val.map((s) => queryTable(table, s.features));
  const score = queries.map((q) => q.capturedProb);
  const pred = queries.map((q) => (q.ood ? 0 : q.capturedProb >= 0.5 ? 1 : 0));
  const nn = queries.map((q) => q.nnDist).filter((d) => Number.isFinite(d)).sort((a, b) => a - b);
  const p = (arr: number[], t: number) => (arr.length ? arr[Math.min(arr.length - 1, Math.max(0, Math.ceil(t * arr.length) - 1))]! : NaN);
  const byAtt: Record<string, { n: number; captured: number; recall: number }> = {};
  for (let i = 0; i < val.length; i += 1) {
    const b = attBucket(val[i]!.features[0] ?? 0);
    const slot = byAtt[b] ?? (byAtt[b] = { n: 0, captured: 0, recall: 0 });
    slot.n += 1;
    if (y[i]) {
      slot.captured += 1;
      if (pred[i]) slot.recall += 1;
    }
  }
  for (const k of Object.keys(byAtt)) {
    const s = byAtt[k]!;
    s.recall = s.captured ? s.recall / s.captured : NaN;
  }
  const byFault: Record<string, ReturnType<typeof confusion> & { n: number }> = {};
  for (let i = 0; i < val.length; i += 1) {
    const key = (val[i]!.isolated.length ?? 0) === 0 ? "healthy" : "isolated";
    if (!byFault[key]) byFault[key] = { n: 0, tp: 0, fp: 0, tn: 0, fn: 0, precision: 0, recall: 0, f1: 0 };
    byFault[key]!.n += 1;
  }
  for (const key of Object.keys(byFault)) {
    const idx = val.map((s, i) => ((s.isolated.length === 0 ? "healthy" : "isolated") === key ? i : -1)).filter((i) => i >= 0);
    const c = confusion(idx.map((i) => y[i]!), idx.map((i) => pred[i]!));
    byFault[key] = { n: idx.length, ...c };
  }
  const oodRate = queries.filter((q) => q.ood).length / Math.max(1, queries.length);
  return {
    featureSchemaVersion: FEATURE_SCHEMA_VERSION,
    trainerVersion: TRAINER_VERSION,
    k: table.k,
    distance: "scaled-L2",
    featureScale: table.scale,
    oodThreshold: table.oodThreshold,
    datasetHash: table.datasetHash,
    splitHash: table.splitHash,
    modelHash: table.modelHash,
    split: "grouped-family",
    nTrain: train.length,
    nVal: val.length,
    trainLabelCounts: count(train),
    valLabelCounts: count(val),
    capturedClass: {
      ...confusion(y, pred),
      prAuc: prAuc(y, score),
      rocAuc: rocAuc(y, score),
    },
    confusionMatrix: confusion(y, pred),
    calibration: calibration(y, score, 5),
    byAttitudeBucket: byAtt,
    byFaultMask: byFault,
    nearestNeighborDistance: {
      p50: p(nn, 0.5),
      p90: p(nn, 0.9),
      worst: nn[nn.length - 1] ?? NaN,
      trainP50: table.nnTrainP50,
      trainP90: table.nnTrainP90,
    },
    ood: {
      threshold: table.oodThreshold,
      valRate: oodRate,
      policy: "if nnDist > threshold, ignore k-NN cost and use eigen heuristic",
    },
    valMae: table.valMae,
    note: "Labels are budgeted-search outcomes. search_unreached is not proven_infeasible. Accuracy is omitted because the negative class dominates.",
  };
}
