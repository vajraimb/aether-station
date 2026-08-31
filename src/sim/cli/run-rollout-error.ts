#!/usr/bin/env npx tsx
/**
 * Measure reduced-vs-frozen-kernel rollout error on the public state set.
 *   npx tsx src/sim/cli/run-rollout-error.ts
 *   npx tsx src/sim/cli/run-rollout-error.ts --out outputs/rollout-error-envelope.json
 */
import { execSync } from "node:child_process";
import { defaultPublicConfig } from "../constants.ts";
import { writeJson } from "../io.ts";
import { measureEnvelope } from "../control/rollout-error.ts";

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

const outPath = arg("--out", "outputs/rollout-error-envelope.json");
const plant = defaultPublicConfig();
const t0 = Date.now();
const report = measureEnvelope(plant);
const payload = {
  commitSha: gitSha(),
  elapsed_ms: Date.now() - t0,
  timestamp: new Date().toISOString(),
  ...report,
};
writeJson(outPath, payload);
console.log(`wrote ${outPath} in ${payload.elapsed_ms} ms`);
console.log("verdict", JSON.stringify(report.verdict, null, 2));
for (const g of report.byHorizon) {
  console.log(
    `${g.key} att p50=${g.attRad.p50.toExponential(3)} p90=${g.attRad.p90.toExponential(3)} worst=${g.attRad.worst.toExponential(3)} within=${g.withinToleranceRate.toFixed(2)} usable=${g.usableForStage}`,
  );
}
const terminalUsable = report.verdict.terminalHorizonUsable;
const fiveDeg = report.verdict.fiveSecondAttP50Deg;
console.log(
  fiveDeg >= 1
    ? `FAIL envelope: 5 s reduced att p50 = ${fiveDeg.toFixed(2)}° exceeds the 1° terminal gate`
    : `5 s reduced att p50 = ${fiveDeg.toFixed(2)}°`,
);
if (!report.verdict.longHorizonUsableForTerminal) {
  console.log("long (8–10 s) reduced rollout is NOT usable for sub-degree terminal capture");
}
process.exit(terminalUsable || report.verdict.guidanceHorizonUsable ? 0 : 1);
