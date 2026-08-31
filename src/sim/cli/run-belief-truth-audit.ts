#!/usr/bin/env npx tsx
/**
 * Closed-loop belief-versus-truth audit on the public train set.
 *
 *   npx tsx src/sim/cli/run-belief-truth-audit.ts
 *   npx tsx src/sim/cli/run-belief-truth-audit.ts --quick
 *
 * Uses original-v2 (kNN-value line is stopped). Does not retune the
 * planner, does not run hidden, does not change beam width.
 */
import { execSync } from "node:child_process";
import { defaultPublicConfig } from "../constants.ts";
import { mergeFlightConfig } from "../control/baseline.ts";
import {
  AUDIT_PHASES,
  classifyPhase,
  mismatchAt,
  summarizePhase,
  type AuditPhase,
  type BeliefSnapshot,
  type MismatchSample,
  type TruthSnapshot,
} from "../control/belief-mismatch.ts";
import { DiscretePulseV2Controller } from "../control/controller-v2.ts";
import { occupancyAt, type PendingPulse } from "../control/discrete-actions.ts";
import { TRAIN_SEEDS } from "../evalset.ts";
import { writeJson } from "../io.ts";
import type { AnyController } from "../oracle.ts";
import { generateScenario } from "../scenario.ts";
import { Simulator } from "../simulator.ts";
import type { Command, SimState } from "../types.ts";

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

interface CycleRow {
  t: number;
  phase: AuditPhase;
  mismatch: MismatchSample;
  plannerPhase: string;
  expectedOn: number[];
  afterFault: boolean;
}

function wrapAudit(
  inner: DiscretePulseV2Controller,
  scenario: {
    faultTime: number;
    faultThruster: number;
    gyroBias0: [number, number, number];
    c1: number;
    c2: number;
    k12: number;
    etaT: number;
  },
  rows: CycleRow[],
): AnyController {
  let truth: SimState | null = null;
  return {
    name: inner.name,
    ingestTruth(state: SimState) {
      truth = state;
    },
    step(obs) {
      const cmd: Command = inner.stepPlant(obs);
      if (truth) {
        const est = inner.getEstimate();
        const isolated = [...inner.fdir.isolated].sort((a, b) => a - b);
        const belief: BeliefSnapshot = {
          q: est.q,
          w: est.w,
          s: est.s,
          sd: est.sd,
          th1: est.th1,
          th1d: est.th1d,
          th2: est.th2,
          th2d: est.th2d,
          fuel: est.fuel,
          bias: est.bias,
          c1: est.c1,
          c2: est.c2,
          k12: est.k12,
          etaT: est.etaT,
          isolated,
        };
        const tsnap: TruthSnapshot = {
          q: truth.q,
          w: truth.w,
          s: truth.s,
          sd: truth.sd,
          th1: truth.th1,
          th1d: truth.th1d,
          th2: truth.th2,
          th2d: truth.th2d,
          fuel: truth.fuel,
          gyroBias: scenario.gyroBias0,
          c1: scenario.c1,
          c2: scenario.c2,
          k12: scenario.k12,
          etaT: scenario.etaT,
          failedThruster: scenario.faultThruster,
          faultHasOccurred: obs.timestamp + 1e-12 >= scenario.faultTime,
        };
        const pending: readonly PendingPulse[] = inner.getPendingPulses();
        const expectedOn = occupancyAt(pending, obs.timestamp, new Set(isolated), 2);
        const mismatch = mismatchAt(belief, tsnap, pending, [0, 0, 0, 0, 0, 0], obs.timestamp);
        const phase = classifyPhase({
          t: obs.timestamp,
          faultTime: scenario.faultTime,
          attDegTruth: mismatch.attTruthDeg,
        });
        rows.push({
          t: obs.timestamp,
          phase,
          mismatch,
          plannerPhase: inner.diagnostics().plannerPhase ?? "guidance",
          expectedOn,
          afterFault: obs.timestamp + 1e-12 >= scenario.faultTime,
        });
      }
      return cmd;
    },
    getEstimate: () => inner.getEstimate(),
    getFdir: () => inner.getFdir(),
    get faultConfidence() {
      return inner.fdir.faultConfidence;
    },
    get detectedFailedThruster() {
      return inner.fdir.detectedFailedThruster;
    },
    get detectionTime() {
      return inner.fdir.detectionTime;
    },
    get isolationTime() {
      return inner.fdir.isolationTime;
    },
    get isolationConfidence() {
      return inner.fdir.isolationConfidence;
    },
  };
}

function attachActual(rows: CycleRow[], sim: Simulator): void {
  for (const row of rows) {
    let best = sim.log[0];
    let bestDt = Infinity;
    for (const s of sim.log) {
      const dt = Math.abs(s.t - row.t);
      if (dt < bestDt) {
        bestDt = dt;
        best = s;
      }
    }
    if (!best) continue;
    const actual = best.thrusterActual;
    let expected = 0;
    let mismatch = 0;
    let actualOn = 0;
    const expSet = new Set(row.expectedOn);
    for (let i = 0; i < 6; i += 1) {
      const on = (actual[i] ?? 0) > 0.5;
      const exp = expSet.has(i);
      if (on) actualOn += 1;
      if (exp) expected += 1;
      if (on !== exp) mismatch += 1;
    }
    row.mismatch = {
      ...row.mismatch,
      pendingExpectedOn: expected,
      pendingActualOn: actualOn,
      pendingMismatchCount: mismatch,
    };
  }
}

const quick = hasFlag("--quick");
const count = Math.max(1, Number(arg("--count", quick ? "2" : "10")));
const outPath = arg(
  "--out",
  quick ? "outputs/belief-truth-audit-train10.quick.json" : "outputs/belief-truth-audit-train10.json",
);
const seeds = TRAIN_SEEDS.slice(0, count);
const sha = gitSha();
const t0 = Date.now();

const perSeed: Array<Record<string, unknown>> = [];
const allRows: CycleRow[] = [];

for (const seed of seeds) {
  const cfg = defaultPublicConfig({ seed, fluidPresent: true });
  const sc = generateScenario(seed, false);
  const flight = mergeFlightConfig({
    mode: "discrete-pulse-v2",
    plannerFamily: "original-v2",
    fuelFloorKg: 2.8,
    planningHorizonS: 8,
    replanPeriodS: 0.5,
    beamWidth: 28,
  });
  const inner = new DiscretePulseV2Controller(cfg, flight);
  const rows: CycleRow[] = [];
  const controller = wrapAudit(inner, sc, rows);
  const sim = new Simulator(cfg, sc, controller);
  sim.runAll();
  attachActual(rows, sim);
  const m = sim.metrics();
  const byPhase: Record<string, unknown> = {};
  for (const phase of AUDIT_PHASES) {
    byPhase[phase] = summarizePhase(
      phase,
      rows.filter((r) => r.phase === phase).map((r) => r.mismatch),
    );
  }
  perSeed.push({
    seed,
    faultTime: sc.faultTime,
    faultThruster: sc.faultThruster,
    nCycles: rows.length,
    att: m.final_attitude_error_deg,
    w: m.final_angular_speed_rad_s,
    fuel: m.remaining_fuel_kg,
    isolated: m.isolatedThrusterId,
    byPhase,
  });
  allRows.push(...rows);
  console.log(
    `seed=${seed} cycles=${rows.length} att=${m.final_attitude_error_deg.toFixed(2)} fuel=${m.remaining_fuel_kg.toFixed(3)} phases=${AUDIT_PHASES.map((p) => `${p}:${rows.filter((r) => r.phase === p).length}`).join(",")}`,
  );
}

const overall: Record<string, unknown> = {};
for (const phase of AUDIT_PHASES) {
  overall[phase] = summarizePhase(
    phase,
    allRows.filter((r) => r.phase === phase).map((r) => r.mismatch),
  );
}
const byClock = {
  before_fault: summarizePhase(
    "nominal",
    allRows.filter((r) => !r.afterFault).map((r) => r.mismatch),
  ),
  after_fault: summarizePhase(
    "post_fault",
    allRows.filter((r) => r.afterFault).map((r) => r.mismatch),
  ),
};

const payload = {
  commitSha: sha,
  controller: "discrete-pulse-v2",
  plannerFamily: "original-v2",
  knnValueLine: "STOPPED",
  set: "train",
  count: seeds.length,
  seeds,
  elapsed_ms: Date.now() - t0,
  timestamp: new Date().toISOString(),
  notes:
    "Belief is estimator+FDIR. Truth is plant state. Pending mismatch is controller occupancy vs plant actual at the nearest 50 ms sample. kNN-value is not used. Terminal phase overrides the fault clock when truth att ≤ 12°.",
  overall,
  byClock,
  perSeed,
  status: {
    physics: "PASS",
    knnValueOnline: "FAIL",
    overall: "FAIL",
    wiredToController: false,
  },
};

writeJson(outPath, payload);
console.log(`wrote ${outPath} in ${payload.elapsed_ms} ms`);
for (const phase of AUDIT_PHASES) {
  const s = overall[phase] as { n: number; attGeodesicDeg: { mean: number; p90: number }; fdirMismatchRate: number };
  console.log(
    `phase ${phase} n=${s.n} attGeoMean=${s.attGeodesicDeg.mean?.toFixed?.(3)} p90=${s.attGeodesicDeg.p90?.toFixed?.(3)} fdirMismatch=${s.fdirMismatchRate?.toFixed?.(3)}`,
  );
}
console.log(
  `clock before n=${byClock.before_fault.n} after n=${byClock.after_fault.n}`,
);
