#!/usr/bin/env npx tsx
/**
 * Offline capture reachability study.
 *   npx tsx src/sim/cli/run-capture-reachability.ts
 *   npx tsx src/sim/cli/run-capture-reachability.ts --quick
 *   npx tsx src/sim/cli/run-capture-reachability.ts --out outputs/capture-reachability-study.json
 *
 * Does not run train-10 or hidden. Does not retune the online planner.
 */
import { execSync } from "node:child_process";
import { defaultPublicConfig } from "../constants.ts";
import { writeJson } from "../io.ts";
import { runCaptureStudy } from "../control/capture-reachability.ts";

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
const outPath = arg("--out", quick ? "outputs/capture-reachability-study.quick.json" : "outputs/capture-reachability-study.json");
const plant = defaultPublicConfig();
const t0 = Date.now();
const report = runCaptureStudy(plant, { quick });
const payload = {
  commitSha: gitSha(),
  elapsed_ms: Date.now() - t0,
  timestamp: new Date().toISOString(),
  quick,
  ...report,
};
writeJson(outPath, payload);
console.log(`wrote ${outPath} in ${payload.elapsed_ms} ms`);
console.log("captured", report.summary.capturedConjunctive, "attBall", report.summary.attBall);
console.log(JSON.stringify(report.summary.answers, null, 2));
console.log("verdict", JSON.stringify(report.verdict));
for (const b of report.summary.byBucket) {
  console.log(`bucket ${b.bucketDeg}° cap=${b.captured} attBall=${b.attBall} bestAtt=${b.bestFinalAttDeg.toFixed(2)}°`);
}
process.exit(report.verdict.overall === "PASS" ? 0 : 1);
