#!/usr/bin/env npx tsx
/**
 * Offline action-macro library + Pareto study.
 *
 *   npx tsx src/sim/cli/run-action-macros.ts
 *
 * Does not wire macros into the online planner. Does not run hidden
 * or train-50. Public representative states only.
 */
import { execSync } from "node:child_process";
import { defaultPublicConfig } from "../constants.ts";
import { generateActionMacros, runMacroStudy, type MacroEval } from "../control/action-macros.ts";
import { writeJson } from "../io.ts";

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

function compact(e: MacroEval) {
  return {
    macroId: e.macroId,
    template: e.template,
    stateId: e.stateId,
    legal: e.legal,
    hParReduction: e.hParReduction,
    dtHPerp: e.dtHPerp,
    attDriftDeg: e.attDriftDeg,
    dWPar: e.dWPar,
    dWPerp: e.dWPerp,
    fuelKg: e.fuelKg,
    durationS: e.durationS,
    signAgree: e.robustness?.signAgree ?? null,
    hParStd: e.robustness?.hParReductionStd ?? null,
  };
}

const outPath = arg("--out", "outputs/action-macro-library.json");
const plant = defaultPublicConfig();
const t0 = Date.now();
const macros = generateActionMacros();
const study = runMacroStudy(plant);

const byTemplate: Record<string, number> = {};
for (const m of macros) byTemplate[m.template] = (byTemplate[m.template] ?? 0) + 1;

const legal = study.evaluations.filter((e) => e.legal);
const reducing = legal.filter((e) => e.hParReduction > 0 && e.dWPerp <= 0);
const robust = legal.filter((e) => (e.robustness?.signAgree ?? 0) >= 0.75 && e.hParReduction > 0);

const payload = {
  commitSha: gitSha(),
  elapsed_ms: Date.now() - t0,
  timestamp: new Date().toISOString(),
  wiredToController: false,
  library: {
    count: macros.length,
    byTemplate,
    minPulseS: 0.04,
    commandDelayS: 0.12,
    maxActive: 2,
    fuelFloorKg: 2.8,
  },
  study: {
    states: study.states,
    evaluations: legal.length,
    illegal: study.evaluations.length - legal.length,
    paretoCount: study.pareto.length,
    fractionStatesWithDominatingMacro: study.fractionStatesWithDominatingMacro,
    reducingAndNotInjectingPerp: reducing.length,
    robustReducing: robust.length,
    omegaParReduced: study.omegaParReduced,
    omegaParReducedNoPerpGrowth: study.omegaParReducedNoPerpGrowth,
    omegaParReducedModestPerp: study.omegaParReducedModestPerp,
    perState: study.perState,
  },
  baseline: study.baseline.map(compact),
  pareto: study.pareto
    .slice()
    .sort((a, b) => b.hParReduction - a.hParReduction)
    .slice(0, 40)
    .map(compact),
  topReducing: legal
    .slice()
    .sort((a, b) => b.hParReduction - a.hParReduction)
    .slice(0, 20)
    .map(compact),
  notes:
    "Macros are 2–4 segment pulse/coast sequences. Pareto is hParReduction / dtHPerp / fuel / attDrift. Not connected to DiscretePulseV2Controller.",
  status: {
    physics: "PASS",
    knnValueOnline: "FAIL",
    overall: "FAIL",
    readyToWire: false,
  },
};

writeJson(outPath, payload);
console.log(`wrote ${outPath} in ${payload.elapsed_ms} ms`);
console.log("macros", macros.length, "states", study.states, "pareto", study.pareto.length);
console.log("dominating-macro fraction", study.fractionStatesWithDominatingMacro.toFixed(3));
console.log("hPar reducing & dWPerp<=0", reducing.length, "/", legal.length);
console.log(
  "dWPar<0",
  study.omegaParReduced,
  "no perp growth",
  study.omegaParReducedNoPerpGrowth,
  "modest perp",
  study.omegaParReducedModestPerp,
);
