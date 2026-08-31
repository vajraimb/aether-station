#!/usr/bin/env npx tsx
/**
 * Multi-scenario evaluation on the public train set or the hidden set.
 *   npm run eval -- --set hidden --count 50
 *   npm run eval -- --set train --count 50
 */
import { defaultPublicConfig } from "../constants.ts";
import { generateScenario } from "../scenario.ts";
import { Simulator } from "../simulator.ts";
import { writeJson } from "../io.ts";
import { HIDDEN_SEEDS, TRAIN_SEEDS } from "../evalset.ts";
import { TruthFeedbackBaseline } from "../oracle.ts";
import type { Metrics } from "../types.ts";

function arg(flag: string, fallback?: string): string {
  const i = process.argv.indexOf(flag);
  if (i >= 0 && i + 1 < process.argv.length) return process.argv[i + 1]!;
  return fallback ?? "";
}

const setName = (arg("--set", "hidden") || "hidden").toLowerCase();
const seeds = (setName === "train" ? TRAIN_SEEDS : HIDDEN_SEEDS).slice();
const count = Math.max(1, Number(arg("--count", String(seeds.length))));
const useOracle = process.argv.includes("--oracle");
const outPath = arg("--out", setName === "train" ? "outputs/train-eval.json" : "outputs/hidden-eval.json");
const picked = seeds.slice(0, count);

const gates = {
  att: 0,
  rate: 0,
  fuel: 0,
  param: 0,
  fdir: 0,
  all: 0,
};

const rows: Array<Record<string, unknown>> = [];
const t0 = Date.now();

for (const seed of picked) {
  const cfg = defaultPublicConfig({ seed, fluidPresent: true });
  const sc = generateScenario(seed, false);
  const controller = useOracle ? new TruthFeedbackBaseline(cfg) : undefined;
  const sim = new Simulator(cfg, sc, controller);
  sim.runAll();
  const m: Metrics = sim.metrics();
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
  rows.push({
    seed,
    faultTime: sc.faultTime,
    faultThruster: sc.faultThruster,
    att: m.final_attitude_error_deg,
    w: m.final_angular_speed_rad_s,
    fuel: m.remaining_fuel_kg,
    param: m.parameter_relative_error,
    isolationDelay: m.isolationDelay,
    isolated: m.isolatedThrusterId,
    isolationOk: fdirOk ? 1 : 0,
    scorecardPass: allOk,
  });
  const mark = allOk ? "PASS" : "fail";
  console.log(
    `${mark} seed=${seed} att=${m.final_attitude_error_deg.toFixed(2)} w=${m.final_angular_speed_rad_s.toFixed(4)} fuel=${m.remaining_fuel_kg.toFixed(3)} param=${m.parameter_relative_error.toFixed(3)} iso=${m.isolatedThrusterId}/${sc.faultThruster} Δ=${m.isolationDelay?.toFixed(2)}`,
  );
}

const n = picked.length;
const summary = {
  set: setName,
  count: n,
  elapsed_ms: Date.now() - t0,
  controller: useOracle ? "truthFeedbackBaseline" : "observation",
  rates: {
    allGates: gates.all / n,
    attitude: gates.att / n,
    rate: gates.rate / n,
    fuel: gates.fuel / n,
    param: gates.param / n,
    fdir: gates.fdir / n,
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
console.log("\n--- eval ---");
console.log(JSON.stringify(summary.rates, null, 2));
console.log("pass", summary.pass);
const ok = Object.values(summary.pass).every(Boolean);
console.log(ok ? "HIDDEN/TRAIN ACCEPT" : "FAIL (not a demo-seed substitute)");
process.exit(ok ? 0 : 1);
