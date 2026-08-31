#!/usr/bin/env npx tsx
/**
 * Four-controller train-10 ablation.
 *   npx tsx src/sim/cli/run-ablation-train10.ts
 *
 * Families: baseline, original-v2, hierarchical, knn-value.
 * Does not touch hidden, beam width, or the demo seed.
 */
import { execSync } from "node:child_process";
import { defaultPublicConfig } from "../constants.ts";
import { createFlightController } from "../control/factory.ts";
import { DiscretePulseV2Controller, type PlannerChoiceSample } from "../control/controller-v2.ts";
import {
  getCaptureValueStats,
  resetCaptureValueStats,
} from "../control/capture-value.ts";
import type { PlannerFamily } from "../control/interface.ts";
import { generateScenario } from "../scenario.ts";
import { Simulator } from "../simulator.ts";
import { writeJson } from "../io.ts";
import { TRAIN_SEEDS } from "../evalset.ts";
import type { Metrics } from "../types.ts";

type VersionKey = "baseline" | "originalV2" | "hierarchicalV2" | "knnValueV2";

interface SeedRow {
  seed: number;
  att: number;
  w: number;
  fuel: number;
  param: number;
  isolationOk: number;
  allGates: boolean;
  sloshRatio: number;
  impact: number;
  pulseCount: number;
  knnQueryCount: number;
  oodFallbackCount: number;
  meanNnDist: number;
  meanCaptureCost: number | null;
  choices: PlannerChoiceSample[];
}

function gitSha(): string {
  try {
    return execSync("git rev-parse HEAD", { encoding: "utf8" }).trim();
  } catch {
    return "unknown";
  }
}

function pearson(xs: number[], ys: number[]): number | null {
  const n = Math.min(xs.length, ys.length);
  if (n < 3) return null;
  let mx = 0;
  let my = 0;
  for (let i = 0; i < n; i += 1) {
    mx += xs[i]!;
    my += ys[i]!;
  }
  mx /= n;
  my /= n;
  let num = 0;
  let dx = 0;
  let dy = 0;
  for (let i = 0; i < n; i += 1) {
    const a = xs[i]! - mx;
    const b = ys[i]! - my;
    num += a * b;
    dx += a * a;
    dy += b * b;
  }
  const den = Math.sqrt(dx * dy);
  return den < 1e-12 ? null : num / den;
}

function ratesOf(rows: SeedRow[]) {
  const n = rows.length;
  const frac = (p: (r: SeedRow) => boolean) => rows.filter(p).length / n;
  return {
    allGates: frac((r) => r.allGates),
    attitude: frac((r) => r.att < 1),
    rate: frac((r) => r.w < 0.008),
    fuel: frac((r) => r.fuel > 2.8),
    param: frac((r) => r.param < 0.15),
    fdir: frac((r) => r.isolationOk === 1),
    slosh: frac((r) => r.sloshRatio < 0.08),
    impact: frac((r) => r.impact < 0.25),
  };
}

function summarize(values: number[]) {
  const s = [...values].sort((a, b) => a - b);
  const at = (p: number) => s[Math.min(s.length - 1, Math.max(0, Math.ceil(p * s.length) - 1))] ?? NaN;
  return { median: at(0.5), p90: at(0.9), worst: s[s.length - 1] ?? NaN, best: s[0] ?? NaN };
}

function guidanceChangeRate(knn: SeedRow[], hier: SeedRow[]): number {
  let matched = 0;
  let changed = 0;
  for (const a of knn) {
    const b = hier.find((r) => r.seed === a.seed);
    if (!b) continue;
    const map = new Map<string, string>();
    for (const c of b.choices) map.set(c.t.toFixed(2), c.primitiveId);
    for (const c of a.choices) {
      const other = map.get(c.t.toFixed(2));
      if (other == null) continue;
      matched += 1;
      if (other !== c.primitiveId) changed += 1;
    }
  }
  return matched === 0 ? 0 : changed / matched;
}

function runOne(seed: number, version: VersionKey): SeedRow {
  const cfg = defaultPublicConfig({ seed, fluidPresent: true });
  const sc = generateScenario(seed, false);
  resetCaptureValueStats();
  const family: PlannerFamily | undefined =
    version === "knnValueV2" ? "knn-value" : version === "hierarchicalV2" ? "hierarchical" : version === "originalV2" ? "original-v2" : undefined;
  const flight =
    version === "baseline"
      ? createFlightController(cfg, { mode: "baseline", fuelFloorKg: 2.8, planningHorizonS: 8, replanPeriodS: 0.5, beamWidth: 28 })
      : createFlightController(cfg, {
          mode: "discrete-pulse-v2",
          fuelFloorKg: 2.8,
          planningHorizonS: 8,
          replanPeriodS: 0.5,
          beamWidth: 28,
          plannerFamily: family,
        });
  const plant = "asPlant" in flight && typeof flight.asPlant === "function" ? flight.asPlant() : flight;
  const sim = new Simulator(cfg, sc, plant);
  sim.runAll();
  const m: Metrics = sim.metrics();
  const fdirOk = m.isolatedThrusterId === sc.faultThruster && (m.isolationDelay ?? 99) < 3;
  const allOk = Object.values(m.scorecard).every((g) => g.pass) && fdirOk;
  const stats = getCaptureValueStats();
  const v2 = flight instanceof DiscretePulseV2Controller ? flight : null;
  const choices = v2 ? [...v2.getChoiceLog()] : [];
  const costs = choices.map((c) => c.captureCost).filter((x): x is number => x != null);
  return {
    seed,
    att: m.final_attitude_error_deg,
    w: m.final_angular_speed_rad_s,
    fuel: m.remaining_fuel_kg,
    param: m.parameter_relative_error,
    isolationOk: fdirOk ? 1 : 0,
    allGates: allOk,
    sloshRatio: m.final_slosh_energy_ratio,
    impact: m.max_slider_impact_speed_m_s,
    pulseCount: m.pulse_count,
    knnQueryCount: stats.queryCount,
    oodFallbackCount: stats.oodCount,
    meanNnDist: stats.meanNnDist,
    meanCaptureCost: costs.length ? costs.reduce((a, b) => a + b, 0) / costs.length : null,
    choices,
  };
}

const seeds = TRAIN_SEEDS.slice(0, 10);
const versions: VersionKey[] = ["baseline", "originalV2", "hierarchicalV2", "knnValueV2"];
const byVersion: Record<VersionKey, SeedRow[]> = {
  baseline: [],
  originalV2: [],
  hierarchicalV2: [],
  knnValueV2: [],
};

const t0 = Date.now();
const sha = gitSha();
for (const version of versions) {
  console.log(`\n=== ${version} ===`);
  for (const seed of seeds) {
    const row = runOne(seed, version);
    byVersion[version].push(row);
    const mark = row.allGates ? "PASS" : "fail";
    console.log(
      `${mark} ${version} seed=${seed} att=${row.att.toFixed(2)} w=${row.w.toFixed(4)} fuel=${row.fuel.toFixed(3)} param=${row.param.toFixed(3)} fdir=${row.isolationOk} knnQ=${row.knnQueryCount} ood=${row.oodFallbackCount}`,
    );
  }
}

const knn = byVersion.knnValueV2;
const hier = byVersion.hierarchicalV2;
const perSeed = seeds.map((seed) => {
  const pick = (v: VersionKey) => byVersion[v].find((r) => r.seed === seed)!;
  const b = pick("baseline");
  const o = pick("originalV2");
  const h = pick("hierarchicalV2");
  const k = pick("knnValueV2");
  return {
    seed,
    versions: {
      baseline: { att: b.att, w: b.w, fuel: b.fuel, param: b.param, fdir: b.isolationOk, allGates: b.allGates },
      originalV2: { att: o.att, w: o.w, fuel: o.fuel, param: o.param, fdir: o.isolationOk, allGates: o.allGates },
      hierarchicalV2: { att: h.att, w: h.w, fuel: h.fuel, param: h.param, fdir: h.isolationOk, allGates: h.allGates },
      knnValueV2: {
        att: k.att,
        w: k.w,
        fuel: k.fuel,
        param: k.param,
        fdir: k.isolationOk,
        allGates: k.allGates,
        knnQueryCount: k.knnQueryCount,
        oodFallbackCount: k.oodFallbackCount,
        meanNnDist: k.meanNnDist,
        meanCaptureCost: k.meanCaptureCost,
      },
    },
    deltaAttKnnVsHier: k.att - h.att,
    deltaFuelKnnVsHier: k.fuel - h.fuel,
    deltaWKnnVsHier: k.w - h.w,
  };
});

const knnCosts = knn.map((r) => r.meanCaptureCost).filter((x): x is number => x != null);
const knnAtt = knn.filter((r) => r.meanCaptureCost != null).map((r) => r.att);
const knnRates = ratesOf(knn);
const stageOne = knnRates.attitude >= 0.6 && knnRates.fuel >= 1 && knnRates.fdir >= 1 && knnRates.allGates > 0;

const report = {
  commitSha: sha,
  branch: "control/discrete-pulse-planner-v2",
  physicsBaselineSha: "bdfff5b4c62733d7156d2f9cdeeaa75661d6c9f4",
  set: "train",
  count: seeds.length,
  elapsed_ms: Date.now() - t0,
  timestamp: new Date().toISOString(),
  note: "Ablation of value heuristic. Beam width 28, horizon 8, hidden blocked, demo seed untouched.",
  rates: {
    baseline: ratesOf(byVersion.baseline),
    originalV2: ratesOf(byVersion.originalV2),
    hierarchicalV2: ratesOf(byVersion.hierarchicalV2),
    knnValueV2: knnRates,
  },
  stats: {
    baseline: { att: summarize(byVersion.baseline.map((r) => r.att)), fuel: summarize(byVersion.baseline.map((r) => r.fuel)), omega: summarize(byVersion.baseline.map((r) => r.w)) },
    originalV2: { att: summarize(byVersion.originalV2.map((r) => r.att)), fuel: summarize(byVersion.originalV2.map((r) => r.fuel)), omega: summarize(byVersion.originalV2.map((r) => r.w)) },
    hierarchicalV2: { att: summarize(hier.map((r) => r.att)), fuel: summarize(hier.map((r) => r.fuel)), omega: summarize(hier.map((r) => r.w)) },
    knnValueV2: { att: summarize(knn.map((r) => r.att)), fuel: summarize(knn.map((r) => r.fuel)), omega: summarize(knn.map((r) => r.w)) },
  },
  knnInstrumentation: {
    queryCountTotal: knn.reduce((s, r) => s + r.knnQueryCount, 0),
    oodFallbackTotal: knn.reduce((s, r) => s + r.oodFallbackCount, 0),
    meanNnDist: knn.reduce((s, r) => s + r.meanNnDist, 0) / Math.max(1, knn.length),
    captureCostVsFinalAttPearson: pearson(knnCosts, knnAtt),
    guidanceChoiceChangedVsHierarchical: guidanceChangeRate(knn, hier),
  },
  perSeed,
  stageOne: {
    attitude60: knnRates.attitude >= 0.6,
    fuel100: knnRates.fuel >= 1,
    fdir100: knnRates.fdir >= 1,
    allGatesPositive: knnRates.allGates > 0,
    allowedTrain50: stageOne,
  },
  verdict: stageOne
    ? "PHYSICS PASS / CAPTURE-VALUE VALIDATION PASS / ONLINE CONTROL TRAIN-10 PASS / PR DRAFT"
    : knnRates.attitude === 0
      ? "PHYSICS PASS / CAPTURE-VALUE VALIDATION PASS / ONLINE CONTROL UNPROVEN (0/10) / STOP kNN-VALUE LINE / OVERALL FAIL / PR DRAFT"
      : "PHYSICS PASS / CAPTURE-VALUE VALIDATION PASS / ONLINE CONTROL UNPROVEN / OVERALL FAIL / PR DRAFT",
};

writeJson("outputs/eval-ablation-train10.json", report);

writeJson("outputs/eval-v2-knn-train10.json", {
  commitSha: sha,
  branch: "control/discrete-pulse-planner-v2",
  controllerVersion: "discrete-pulse-v2-knn",
  physicsBaselineSha: "bdfff5b4c62733d7156d2f9cdeeaa75661d6c9f4",
  set: "train",
  count: knn.length,
  elapsed_ms: Date.now() - t0,
  timestamp: new Date().toISOString(),
  rates: knnRates,
  stats: report.stats.knnValueV2,
  targets: { allGates: 0.8, attitude: 0.9, fuel: 0.9, fdir: 0.95, param: 0.8 },
  pass: {
    allGates: knnRates.allGates >= 0.8,
    attitude: knnRates.attitude >= 0.9,
    fuel: knnRates.fuel >= 0.9,
    fdir: knnRates.fdir >= 0.95,
    param: knnRates.param >= 0.8,
  },
  rows: knn.map((r) => ({
    seed: r.seed,
    att: r.att,
    w: r.w,
    fuel: r.fuel,
    param: r.param,
    isolationOk: r.isolationOk,
    allGates: r.allGates,
    knnQueryCount: r.knnQueryCount,
    oodFallbackCount: r.oodFallbackCount,
    meanNnDist: r.meanNnDist,
  })),
});

console.log("\n--- ablation rates ---");
console.log(JSON.stringify(report.rates, null, 2));
console.log("knn instrumentation", report.knnInstrumentation);
console.log(report.verdict);
process.exit(stageOne ? 0 : 1);
