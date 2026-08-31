#!/usr/bin/env npx tsx
/**
 * Wrench geometry + 8–20 segment null-space sequence study.
 *
 *   npx tsx src/sim/cli/run-wrench-nullspace.ts
 *   npx tsx src/sim/cli/run-wrench-nullspace.ts --quick
 *
 * Does not retune or wire a controller. Does not run train-50 / hidden.
 */
import { execSync } from "node:child_process";
import { defaultPublicConfig } from "../constants.ts";
import { runNullspaceStudy } from "../control/nullspace-search.ts";
import { runWrenchStudy } from "../control/wrench-geometry.ts";
import { writeJson } from "../io.ts";

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
const outPath = arg("--out", quick ? "outputs/wrench-nullspace-study.quick.json" : "outputs/wrench-nullspace-study.json");
const plant = defaultPublicConfig();
const t0 = Date.now();
const wrench = runWrenchStudy(plant, { axisCount: quick ? 48 : 200 });
const seq = runNullspaceStudy(plant, { quick, maxSeg: quick ? 8 : 16, beamWidth: quick ? 8 : 18 });

const byMethod: Record<string, { n: number; medianRho: number; meanFrac: number; targetRate: number }> = {};
for (const r of seq.results) {
  const cur = byMethod[r.method] ?? { n: 0, medianRho: 0, meanFrac: 0, targetRate: 0 };
  cur.n += 1;
  cur.medianRho += r.rho;
  cur.meanFrac += r.fractionParReduced;
  cur.targetRate += r.targetMet ? 1 : 0;
  byMethod[r.method] = cur;
}
for (const k of Object.keys(byMethod)) {
  const c = byMethod[k]!;
  c.medianRho /= c.n;
  c.meanFrac /= c.n;
  c.targetRate /= c.n;
}

const payload = {
  commitSha: gitSha(),
  elapsed_ms: Date.now() - t0,
  timestamp: new Date().toISOString(),
  wiredToController: false,
  knnValueLine: "STOPPED",
  physicsBaselineSha: "bdfff5b4c62733d7156d2f9cdeeaa75661d6c9f4",
  wrench: {
    etaT: wrench.etaT,
    pulseS: wrench.pulseS,
    axisCount: wrench.masks[0]?.axisCount ?? 0,
    masks: wrench.masks.map((m) => ({
      mask: m.mask,
      isolated: m.isolated,
      nSingles: m.nSingles,
      nPairs: m.nPairs,
      singlesRank: m.singles.rank,
      singlesSigma: m.singles.values,
      singlesCond: m.singles.cond,
      allRank: m.all.rank,
      allSigma: m.all.values,
      allCond: m.all.cond,
      rhoSingles: m.rhoSingles,
      rhoAll: m.rhoAll,
      fractionRhoGt4: m.fractionRhoGt4,
      fractionRhoGt10: m.fractionRhoGt10,
      meanBestPar: m.meanBestPar,
      meanBestPerp: m.meanBestPerp,
    })),
  },
  sequences: {
    maxSeg: seq.maxSeg,
    beamWidth: seq.beamWidth,
    byMethod,
    results: seq.results.map((r) => ({
      method: r.method,
      stateId: r.stateId,
      isolated: r.isolated,
      nSeg: r.nSeg,
      dtHPar: r.dtHPar,
      dtHPerp: r.dtHPerp,
      peakHPerp: r.peakHPerp,
      rho: r.rho,
      fuelKg: r.fuelKg,
      slosh: r.slosh,
      durationS: r.durationS,
      targetMet: r.targetMet,
      fractionParReduced: r.fractionParReduced,
      ids: r.ids,
    })),
    robustness: seq.robustness,
  },
  notes:
    "Wrench SVD is body-frame torque at 40 ms. Sequence search allows intermediate ⊥ growth and scores net ΔH_⊥. Not wired to DiscretePulseV2Controller.",
  status: {
    physics: "PASS",
    onlineControl: "FAIL",
    overall: "FAIL",
    readyToWire: false,
  },
};

writeJson(outPath, payload);
console.log(`wrote ${outPath} in ${payload.elapsed_ms} ms`);
for (const m of wrench.masks) {
  console.log(
    `mask ${m.mask} rankS=${m.singles.rank} condS=${m.singles.cond.toFixed(2)} rhoAll p50=${m.rhoAll.median.toFixed(2)} p90=${m.rhoAll.p90.toFixed(2)} >10=${m.fractionRhoGt10.toFixed(2)}`,
  );
}
for (const [k, v] of Object.entries(byMethod)) {
  console.log(`seq ${k} n=${v.n} meanRho=${v.medianRho.toFixed(2)} frac=${v.meanFrac.toFixed(3)} target=${v.targetRate.toFixed(2)}`);
}
for (const r of seq.robustness) {
  console.log(`robust ${r.stateId} targetRate=${r.targetMetRate.toFixed(2)} medRho=${r.medianRho.toFixed(2)} minFrac=${r.minFraction.toFixed(3)}`);
}
