#!/usr/bin/env npx tsx
/**
 * Closed-loop traces for documented V2 failure / near-miss train seeds.
 * Not a per-seed controller branch — analysis only.
 *
 *   npx tsx src/sim/cli/run-failure-traces.ts
 */
import { execSync } from "node:child_process";
import { mkdirSync } from "node:fs";
import { defaultPublicConfig } from "../constants.ts";
import { mergeFlightConfig } from "../control/baseline.ts";
import { DiscretePulseV2Controller, type PlannerTraceSample } from "../control/controller-v2.ts";
import { writeJson, writeText } from "../io.ts";
import { generateScenario } from "../scenario.ts";
import { Simulator } from "../simulator.ts";

function gitSha(): string {
  try {
    return execSync("git rev-parse HEAD", { encoding: "utf8" }).trim();
  } catch {
    return "unknown";
  }
}

const SEEDS = [800017, 800068, 800119, 800102];
const LABELS: Record<number, string> = {
  800017: "wrong-way / perp (prior 99.6°)",
  800068: "wrong-way / perp (prior 79.5°)",
  800119: "best / closest-to-1° (prior 2.48°)",
  800102: "next-closest (prior 5.86°)",
};

mkdirSync("outputs/v2-failure-traces", { recursive: true });
const sha = gitSha();
const summary: unknown[] = [];

for (const seed of SEEDS) {
  const cfg = defaultPublicConfig({ seed, fluidPresent: true });
  const sc = generateScenario(seed, false);
  const flight = new DiscretePulseV2Controller(
    cfg,
    mergeFlightConfig({
      mode: "discrete-pulse-v2",
      fuelFloorKg: 2.8,
      planningHorizonS: 8,
      replanPeriodS: 0.5,
      beamWidth: 28,
    }),
  );
  const samples: PlannerTraceSample[] = [];
  flight.setTraceSink((s) => samples.push({ ...s }));
  const t0 = Date.now();
  const sim = new Simulator(cfg, sc, flight.asPlant());
  sim.runAll();
  const m = sim.metrics();
  const elapsed = Date.now() - t0;
  const last = samples[samples.length - 1];
  const record = {
    seed,
    label: LABELS[seed],
    commitSha: sha,
    elapsed_ms: elapsed,
    faultTime: sc.faultTime,
    faultThruster: sc.faultThruster,
    final: {
      att: m.final_attitude_error_deg,
      w: m.final_angular_speed_rad_s,
      fuel: m.remaining_fuel_kg,
      param: m.parameter_relative_error,
      isolationDelay: m.isolationDelay,
      isolated: m.isolatedThrusterId,
    },
    phaseCounts: {
      guidance: samples.filter((s) => s.plannerPhase === "guidance").length,
      terminal: samples.filter((s) => s.plannerPhase === "terminal").length,
      fallback: samples.filter((s) => s.plannerPhase === "fallback").length,
    },
    terminalReachableRate: samples.filter((s) => s.terminalReachable).length / Math.max(1, samples.length),
    last,
    samples,
  };
  writeJson(`outputs/v2-failure-traces/${seed}.json`, record);
  const csv = [
    "t,attDeg,wParallel,wPerp,primitive,predAtt,actualAtt,predMinusActual,fuelMargin,phase,reachable,fdir",
    ...samples.map((s) =>
      [
        s.t.toFixed(3),
        s.attDeg.toFixed(4),
        s.wParallel.toFixed(5),
        s.wPerp.toFixed(5),
        s.selectedPrimitive ?? "",
        s.predictedNextAttDeg?.toFixed(4) ?? "",
        s.actualAttDeg.toFixed(4),
        s.predictedVsActualAttDeg?.toFixed(4) ?? "",
        s.fuelMarginKg.toFixed(4),
        s.plannerPhase,
        s.terminalReachable ? 1 : 0,
        s.fdirMask.join("+"),
      ].join(","),
    ),
  ].join("\n");
  writeText(`outputs/v2-failure-traces/${seed}.csv`, csv);
  summary.push({
    seed,
    label: LABELS[seed],
    att: m.final_attitude_error_deg,
    w: m.final_angular_speed_rad_s,
    fuel: m.remaining_fuel_kg,
    isolationDelay: m.isolationDelay,
    isolated: m.isolatedThrusterId,
    faultThruster: sc.faultThruster,
    phaseCounts: record.phaseCounts,
    terminalReachableRate: record.terminalReachableRate,
    elapsed_ms: elapsed,
  });
  console.log(
    `seed=${seed} att=${m.final_attitude_error_deg.toFixed(2)} w=${m.final_angular_speed_rad_s.toFixed(4)} fuel=${m.remaining_fuel_kg.toFixed(3)} iso=${m.isolatedThrusterId}/${sc.faultThruster} n=${samples.length} ${elapsed}ms`,
  );
}

writeJson("outputs/v2-failure-traces/summary.json", {
  commitSha: sha,
  physicsBaselineSha: "bdfff5b4c62733d7156d2f9cdeeaa75661d6c9f4",
  controllerVersion: "discrete-pulse-v2",
  note: "Analysis of public train seeds. Controller has no per-seed branch.",
  rows: summary,
});
console.log("wrote outputs/v2-failure-traces/");
