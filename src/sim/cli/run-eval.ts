#!/usr/bin/env npx tsx
/**
 * Multi-scenario evaluation.
 *   npm run eval -- --controller baseline --set train --count 10
 *   npm run eval -- --controller discrete-pulse-v2 --set train --count 10
 */
import { execSync } from "node:child_process";
import { defaultPublicConfig } from "../constants.ts";
import { createPlantController } from "../control/factory.ts";
import { generateScenario } from "../scenario.ts";
import { Simulator } from "../simulator.ts";
import { writeJson } from "../io.ts";
import { HIDDEN_SEEDS, TRAIN_SEEDS } from "../evalset.ts";
import { TruthFeedbackBaseline } from "../oracle.ts";
import type { ControllerMode } from "../control/interface.ts";
import type { Metrics } from "../types.ts";

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

function gitBranch(): string {
  try {
    return execSync("git rev-parse --abbrev-ref HEAD", { encoding: "utf8" }).trim();
  } catch {
    return "unknown";
  }
}

function percentile(sorted: number[], p: number): number {
  if (sorted.length === 0) return NaN;
  const i = Math.min(sorted.length - 1, Math.max(0, Math.ceil(p * sorted.length) - 1));
  return sorted[i]!;
}

function summarize(values: number[]) {
  const s = [...values].sort((a, b) => a - b);
  return {
    median: percentile(s, 0.5),
    p90: percentile(s, 0.9),
    worst: s[s.length - 1] ?? NaN,
    best: s[0] ?? NaN,
  };
}

const setName = (arg("--set", "train") || "train").toLowerCase();
const controllerFlag = (arg("--controller", "baseline") || "baseline").toLowerCase();
const seeds = (setName === "hidden" ? HIDDEN_SEEDS : TRAIN_SEEDS).slice();
const count = Math.max(1, Number(arg("--count", String(seeds.length))));
const useOracle = process.argv.includes("--oracle") || controllerFlag === "oracle";
const mode: ControllerMode = controllerFlag === "discrete-pulse-v2" ? "discrete-pulse-v2" : "baseline";
const defaultOut =
  setName === "hidden"
    ? "outputs/hidden-eval.json"
    : mode === "discrete-pulse-v2"
      ? "outputs/eval-v2-train10.json"
      : "outputs/eval-baseline-train10.json";
const outPath = arg("--out", count === 10 && setName === "train" ? defaultOut : `outputs/eval-${controllerFlag}-${setName}${count}.json`);
const picked = seeds.slice(0, count);

if (setName === "hidden" && !process.argv.includes("--force-hidden")) {
  console.error("Hidden set is blocked until train-10 and train-50 gates pass. Pass --force-hidden to override.");
  process.exit(2);
}

const gates = { att: 0, rate: 0, fuel: 0, param: 0, fdir: 0, all: 0, slosh: 0, impact: 0 };
const rows: Array<Record<string, unknown>> = [];
const paramRows: Array<Record<string, unknown>> = [];
const t0 = Date.now();
const sha = gitSha();
const branch = gitBranch();

for (const seed of picked) {
  const cfg = defaultPublicConfig({ seed, fluidPresent: true });
  const sc = generateScenario(seed, false);
  const controller = useOracle
    ? new TruthFeedbackBaseline(cfg)
    : createPlantController(cfg, {
        mode,
        fuelFloorKg: 2.8,
        planningHorizonS: 8,
        replanPeriodS: 0.5,
        beamWidth: 28,
      });
  const sim = new Simulator(cfg, sc, controller);
  sim.runAll();
  const m: Metrics = sim.metrics();
  const last = sim.log[sim.log.length - 1]!;
  const attOk = m.final_attitude_error_deg < 1;
  const rateOk = m.final_angular_speed_rad_s < 0.008;
  const fuelOk = m.remaining_fuel_kg > 2.8;
  const paramOk = m.parameter_relative_error < 0.15;
  const fdirOk = m.isolatedThrusterId === sc.faultThruster && (m.isolationDelay ?? 99) < 3;
  const allOk = Object.values(m.scorecard).every((g) => g.pass) && fdirOk;
  if (attOk) gates.att += 1;
  if (rateOk) gates.rate += 1;
  if (fuelOk) gates.fuel += 1;
  if (paramOk) gates.param += 1;
  if (fdirOk) gates.fdir += 1;
  if (allOk) gates.all += 1;
  if (m.final_slosh_energy_ratio < 0.08) gates.slosh += 1;
  if (m.max_slider_impact_speed_m_s < 0.25) gates.impact += 1;
  rows.push({
    seed,
    faultTime: sc.faultTime,
    faultThruster: sc.faultThruster,
    att: m.final_attitude_error_deg,
    w: m.final_angular_speed_rad_s,
    fuel: m.remaining_fuel_kg,
    impact: m.max_slider_impact_speed_m_s,
    sloshRatio: m.final_slosh_energy_ratio,
    param: m.parameter_relative_error,
    isolationDelay: m.isolationDelay,
    isolated: m.isolatedThrusterId,
    isolationOk: fdirOk ? 1 : 0,
    allGates: allOk,
    pulseCount: m.pulse_count,
    totalOnTime: m.total_thruster_on_time,
    c1Est: last.c1Est,
    c2Est: last.c2Est,
    k12Est: last.k12Est,
    etaTEst: last.etaTEst,
    c1True: sc.c1,
    c2True: sc.c2,
    k12True: sc.k12,
    etaTTrue: sc.etaT,
  });
  paramRows.push({
    seed,
    estimate: { c1: last.c1Est, c2: last.c2Est, k12: last.k12Est, etaT: last.etaTEst, P: { c1: last.c1P, c2: last.c2P, k12: last.k12P, eta: last.etaP } },
    truth: { c1: sc.c1, c2: sc.c2, k12: sc.k12, etaT: sc.etaT },
    relativeError: m.parameter_relative_error,
  });
  const mark = allOk ? "PASS" : "fail";
  console.log(
    `${mark} seed=${seed} att=${m.final_attitude_error_deg.toFixed(2)} w=${m.final_angular_speed_rad_s.toFixed(4)} fuel=${m.remaining_fuel_kg.toFixed(3)} param=${m.parameter_relative_error.toFixed(3)} iso=${m.isolatedThrusterId}/${sc.faultThruster} Δ=${m.isolationDelay?.toFixed(2)}`,
  );
}

const n = picked.length;
const attVals = rows.map((r) => Number(r.att));
const fuelVals = rows.map((r) => Number(r.fuel));
const wVals = rows.map((r) => Number(r.w));
const summary = {
  commitSha: sha,
  branch,
  controllerVersion: useOracle ? "truthFeedbackBaseline" : mode,
  physicsBaselineSha: "bdfff5b4c62733d7156d2f9cdeeaa75661d6c9f4",
  set: setName,
  count: n,
  elapsed_ms: Date.now() - t0,
  timestamp: new Date().toISOString(),
  rates: {
    allGates: gates.all / n,
    attitude: gates.att / n,
    rate: gates.rate / n,
    fuel: gates.fuel / n,
    param: gates.param / n,
    fdir: gates.fdir / n,
    slosh: gates.slosh / n,
    impact: gates.impact / n,
  },
  stats: {
    att: summarize(attVals),
    fuel: summarize(fuelVals),
    omega: summarize(wVals),
  },
  targets: {
    allGates: 0.8,
    attitude: 0.9,
    fuel: 0.9,
    fdir: 0.95,
    param: 0.8,
  },
  pass: {
    allGates: gates.all / n >= 0.8,
    attitude: gates.att / n >= 0.9,
    fuel: gates.fuel / n >= 0.9,
    fdir: gates.fdir / n >= 0.95,
    param: gates.param / n >= 0.8,
  },
  rows,
};
writeJson(outPath, summary);
if (controllerFlag === "discrete-pulse-v2" || mode === "discrete-pulse-v2") {
  writeJson("outputs/parameter-diagnostics.json", {
    commitSha: sha,
    branch,
    physicsBaselineSha: "bdfff5b4c62733d7156d2f9cdeeaa75661d6c9f4",
    note: "Truth values are scoring-side only and are not visible to the flight controller.",
    rows: paramRows,
  });
}
console.log("\n--- eval ---");
console.log(JSON.stringify(summary.rates, null, 2));
console.log("pass", summary.pass);
const stageOne =
  setName === "train" &&
  n <= 10 &&
  gates.fuel / n >= 1 &&
  gates.fdir / n >= 1 &&
  gates.att / n >= 0.6 &&
  gates.all / n > 0;
const ok = n <= 10 ? stageOne : Object.values(summary.pass).every(Boolean);
console.log(ok ? "ACCEPT" : "FAIL (not a demo-seed substitute)");
process.exit(ok ? 0 : 1);
