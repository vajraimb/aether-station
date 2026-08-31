#!/usr/bin/env npx tsx
/**
 * Headless runner. No DOM / WebGL.
 *   npm run sim -- --seed 20260831 --scenario demo
 */
import { mkdirSync } from "node:fs";
import { defaultPublicConfig, DEMO_SEED } from "../constants.ts";
import { generateScenario } from "../scenario.ts";
import { Simulator } from "../simulator.ts";
import { OracleController } from "../oracle.ts";
import { analyseReachability } from "../reachability.ts";
import { scoreFromLog } from "../scoring.ts";
import { writeEvents, writeJson, writeMetrics, writeTrajectory } from "../io.ts";

function arg(flag: string, fallback?: string): string | undefined {
  const i = process.argv.indexOf(flag);
  if (i >= 0 && i + 1 < process.argv.length) return process.argv[i + 1];
  return fallback;
}

function has(flag: string): boolean {
  return process.argv.includes(flag);
}

const seed = Number(arg("--seed", String(DEMO_SEED)));
const scenarioName = (arg("--scenario", "demo") ?? "demo").toLowerCase();
const outDir = arg("--out", "outputs")!;
const wantOracle = has("--oracle") || !has("--no-oracle");
const demo = scenarioName === "demo";

mkdirSync(outDir, { recursive: true });

const cfg = defaultPublicConfig({ seed, fluidPresent: true });
const sc = generateScenario(seed, demo);
const t0 = Date.now();
const sim = new Simulator(cfg, sc);
sim.runAll();
const live = sim.metrics();
writeTrajectory(`${outDir}/trajectory.csv`, sim.log);
writeEvents(`${outDir}/events.jsonl`, sim.events);
writeMetrics(`${outDir}/metrics.json`, live);

const recomputed = scoreFromLog(sim.log, sim.events, sc);
writeMetrics(`${outDir}/recomputed-metrics.json`, recomputed);

const cfgDry = defaultPublicConfig({ seed, fluidPresent: false });
const scDry = generateScenario(seed, demo);
const cf = new Simulator(cfgDry, scDry);
cf.runAll();
writeMetrics(`${outDir}/counterfactual-metrics.json`, cf.metrics());

const reach = analyseReachability(sc.faultThruster, cfg, 0, sc.etaT);
writeJson(`${outDir}/reachability.json`, reach);

if (wantOracle) {
  const ocfg = defaultPublicConfig({ seed, fluidPresent: true });
  const osc = generateScenario(seed, demo);
  const oracle = new OracleController(ocfg);
  const osim = new Simulator(ocfg, osc, oracle);
  osim.runAll();
  writeMetrics(`${outDir}/oracle-metrics.json`, osim.metrics());
}

console.log(JSON.stringify({
  seed,
  scenario: scenarioName,
  demo: sc.demo,
  elapsed_ms: Date.now() - t0,
  scorecard: live.scorecard,
  fdir: {
    faultInjectionTime: live.faultInjectionTime,
    abnormalFlagTime: live.abnormalFlagTime,
    detectionTime: live.detectionTime,
    isolationTime: live.isolationTime,
    detectionDelay: live.detectionDelay,
    isolationDelay: live.isolationDelay,
    isolatedThrusterId: live.isolatedThrusterId,
    confidence: live.confidence,
  },
  outDir,
}, null, 2));
