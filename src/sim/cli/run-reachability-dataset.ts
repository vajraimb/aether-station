#!/usr/bin/env npx tsx
/**
 * Grouped-split reachability dataset + single k-NN artifact + validation.
 *   npx tsx src/sim/cli/run-reachability-dataset.ts
 *   npx tsx src/sim/cli/run-reachability-dataset.ts --quick
 *
 * Authoritative model: src/sim/control/data/capture-value-knn.v1.json
 */
import { createHash } from "node:crypto";
import { execSync } from "node:child_process";
import { defaultPublicConfig } from "../constants.ts";
import { writeJson } from "../io.ts";
import { assertGroupedSplit, buildDataset } from "../control/reachability-dataset.ts";
import { fitKnn, setCaptureValueTable, type KnnTable } from "../control/capture-value.ts";
import { validateCaptureValue } from "../control/capture-value-validation.ts";

function hasFlag(flag: string): boolean {
  return process.argv.includes(flag);
}

function arg(flag: string, fallback?: string): string {
  const i = process.argv.indexOf(flag);
  if (i >= 0 && i + 1 < process.argv.length) return process.argv[i + 1]!;
  return fallback ?? "";
}

function gitSha(): string {
  try {
    return execSync("git rev-parse HEAD", { encoding: "utf8" }).trim();
  } catch {
    return "unknown";
  }
}

function sha256(value: unknown): string {
  return createHash("sha256").update(JSON.stringify(value)).digest("hex");
}

const quick = hasFlag("--quick");
const n = Number(arg("--n", quick ? "40" : "400"));
const horizonS = Number(arg("--horizon", quick ? "5" : "8"));
const budget = Number(arg("--budget", quick ? "12" : "20"));
const outDs = arg("--out", quick ? "outputs/reachability-dataset.quick.json" : "outputs/reachability-dataset.json");
const modelPath = "src/sim/control/data/capture-value-knn.v1.json";
const valPath = "outputs/capture-value-validation.json";

const plant = defaultPublicConfig();
const t0 = Date.now();
const ds = buildDataset(n, plant, { horizonS, expansionBudget: budget, perGroup: 5 });
const leaks = assertGroupedSplit(ds.samples);
if (leaks.length) {
  console.error("grouped split leaked families", leaks);
  process.exit(2);
}
const datasetHash = sha256(ds.samples.map((s) => ({ id: s.id, groupId: s.groupId, label: s.label, cost: s.cost, features: s.features })));
const splitHash = sha256(ds.samples.map((s) => ({ groupId: s.groupId, split: s.split })));
const knn = fitKnn(ds.samples, 7, { datasetHash, splitHash });
const modelCore = {
  k: knn.k,
  scale: knn.scale,
  neighbors: knn.neighbors,
  nTrain: knn.nTrain,
  nVal: knn.nVal,
  valMae: knn.valMae,
  oodThreshold: knn.oodThreshold,
  nnTrainP50: knn.nnTrainP50,
  nnTrainP90: knn.nnTrainP90,
  featureSchemaVersion: knn.featureSchemaVersion,
  trainerVersion: knn.trainerVersion,
};
knn.modelHash = sha256(modelCore);
setCaptureValueTable(knn);
const validation = validateCaptureValue(knn, ds.samples);
const elapsed = Date.now() - t0;
const payload = {
  commitSha: gitSha(),
  elapsed_ms: elapsed,
  timestamp: new Date().toISOString(),
  quick,
  horizonS,
  expansionBudget: budget,
  physicsBaselineSha: "bdfff5b4c62733d7156d2f9cdeeaa75661d6c9f4",
  datasetHash,
  splitHash,
  modelHash: knn.modelHash,
  taxonomy: {
    captured: "committed high-fid trajectory met att<1° AND |w|<0.008 AND fuel>2.8",
    search_unreached: "budgeted search found no sequence — not a proof",
    proven_infeasible: "no torque authority and near-zero rate with att>=1°",
  },
  ...ds,
};
writeJson(outDs, payload);
writeJson(modelPath, knn);
writeJson(valPath, { commitSha: gitSha(), elapsed_ms: elapsed, ...validation });
void (knn as KnnTable);
console.log(`wrote ${outDs}, ${modelPath}, ${valPath} in ${elapsed} ms`);
console.log("counts", JSON.stringify(ds.counts));
console.log("split", `grouped train=${ds.nTrain}/${ds.nTrainGroups}g val=${ds.nVal}/${ds.nValGroups}g`);
console.log("captured PR-AUC", validation.capturedClass.prAuc, "ROC-AUC", validation.capturedClass.rocAuc);
console.log("F1", validation.capturedClass.f1, "oodRate", validation.ood.valRate);
console.log("PHYSICS PASS / CONTROL FAIL until smoke+train-10. PR DRAFT.");
process.exit(0);
