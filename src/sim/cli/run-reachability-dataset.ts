#!/usr/bin/env npx tsx
/**
 * Offline reachability dataset + k-NN capture-value fit.
 *   npx tsx src/sim/cli/run-reachability-dataset.ts
 *   npx tsx src/sim/cli/run-reachability-dataset.ts --quick
 *
 * Labels are captured | search_unreached | proven_infeasible.
 * search_unreached is not a proof of physical infeasibility.
 * Does not run train-10 or hidden. Does not retune beam width.
 */
import { execSync } from "node:child_process";
import { defaultPublicConfig } from "../constants.ts";
import { writeJson } from "../io.ts";
import { buildDataset } from "../control/reachability-dataset.ts";
import { fitKnn, setCaptureValueTable } from "../control/capture-value.ts";

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

const quick = hasFlag("--quick");
const n = Number(arg("--n", quick ? "48" : "400"));
const horizonS = Number(arg("--horizon", quick ? "5" : "8"));
const budget = Number(arg("--budget", quick ? "12" : "20"));
const outDs = arg("--out", quick ? "outputs/reachability-dataset.quick.json" : "outputs/reachability-dataset.json");
const outKnn = arg("--knn", "src/sim/control/data/capture-value-knn.json");

const plant = defaultPublicConfig();
const t0 = Date.now();
const ds = buildDataset(n, plant, { horizonS, expansionBudget: budget });
const knn = fitKnn(ds.samples, 7);
setCaptureValueTable(knn);
const elapsed = Date.now() - t0;
const payload = {
  commitSha: gitSha(),
  elapsed_ms: elapsed,
  timestamp: new Date().toISOString(),
  quick,
  horizonS,
  expansionBudget: budget,
  physicsBaselineSha: "bdfff5b4c62733d7156d2f9cdeeaa75661d6c9f4",
  taxonomy: {
    captured: "committed high-fid trajectory met att<1° AND |w|<0.008 AND fuel>2.8",
    search_unreached: "budgeted search found no sequence — not a proof",
    proven_infeasible: "no torque authority and near-zero rate with att>=1°",
  },
  ...ds,
};
writeJson(outDs, payload);
writeJson("outputs/capture-value-knn.json", knn);
writeJson(outKnn, knn);
console.log(`wrote ${outDs} and ${outKnn} in ${elapsed} ms`);
console.log("counts", JSON.stringify(ds.counts));
console.log("split", `train=${ds.nTrain} val=${ds.nVal} valMae=${knn.valMae}`);
console.log("PHYSICS PASS / CONTROL FAIL until online gates move. PR DRAFT.");
process.exit(0);
