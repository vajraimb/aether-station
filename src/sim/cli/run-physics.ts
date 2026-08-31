#!/usr/bin/env npx tsx
/**
 * Physics + isolation contract tests. Headless.
 *   npm run test:physics
 */
import { readFileSync, readdirSync, statSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { runAllTests } from "../tests.ts";
import { runConservation, runConvergence } from "../numerics.ts";
import { writeJson } from "../io.ts";
import { fdirFromEvents } from "../scoring.ts";
import { Simulator } from "../simulator.ts";
import { AgentController } from "../controller.ts";
import { OBSERVATION_KEYS } from "../types.ts";
import type { Observation, PrivateScenario, SimEvent, ThrusterIndex } from "../types.ts";
import { Q0, W0, defaultPublicConfig } from "../constants.ts";
import { makeRng } from "../math3d.ts";

const here = dirname(fileURLToPath(import.meta.url));
const simDir = join(here, "..");

interface T {
  name: string;
  pass: boolean;
  detail: string;
}
const extra: T[] = [];
function check(name: string, pass: boolean, detail: string) {
  extra.push({ name, pass, detail });
  console.log(`${pass ? "PASS" : "FAIL"}  ${name}  ${detail}`);
}

const unit = runAllTests();
for (const r of unit) {
  console.log(`${r.pass ? "PASS" : "FAIL"}  ${r.name}  ${r.detail}`);
}

// --- source contract: no 73.4 s / fixed +Y branch in the controller path ---
function walk(dir: string, acc: string[] = []): string[] {
  for (const name of readdirSync(dir)) {
    const p = join(dir, name);
    const st = statSync(p);
    if (st.isDirectory()) walk(p, acc);
    else if (name.endsWith(".ts")) acc.push(p);
  }
  return acc;
}

const ctrlFiles = ["controller.ts", "allocate.ts", "fdir.ts", "estimator.ts"].map((f) => join(simDir, f));
for (const f of ctrlFiles) {
  const src = readFileSync(f, "utf8");
  const banned = [];
  if (/\b73\.4\b/.test(src)) banned.push("73.4");
  if (/\bFAULT_TIME\b/.test(src)) banned.push("FAULT_TIME");
  if (/\bFAULT_THRUSTER\b/.test(src)) banned.push("FAULT_THRUSTER");
  if (/\b\+Y\b/.test(src) && /if\s*\(.*\+Y/.test(src)) banned.push("+Y identity branch");
  check(
    `source_no_hardcoded_fault:${f.split("/").pop()}`,
    banned.length === 0,
    banned.length ? `found ${banned.join(",")}` : "clean",
  );
}

const cfg = defaultPublicConfig();
const agent = new AgentController(cfg);
check("controller_step_arity", agent.step.length === 1, `step.length=${agent.step.length}`);
check("controller_no_ingestTruth", !("ingestTruth" in agent), "no ingestTruth on flight controller");
check("controller_no_simulator_field", !("sim" in agent) && !("scenario" in agent), "no sim/scenario");

const allowed = new Set<string>(OBSERVATION_KEYS);
const accessed = new Set<string>();
const raw: Observation = {
  timestamp: 4.2,
  quaternionMeasured: Q0,
  gyroMeasured: [0.01, -0.01, 0.02],
  sliderPosition: 0.1,
  sliderVelocity: 0,
  tankWallPressure1: 2500,
  tankWallPressure2: 2500,
  remainingFuelEstimate: 5,
  thrusterCurrentFeedback: [0, 0, 0, 0, 0, 0],
  actuatorResponseAbnormal: false,
};
const proxy = new Proxy(raw, {
  get(t, prop) {
    const key = String(prop);
    accessed.add(key);
    if (!allowed.has(key) && key !== "then") {
      throw new Error(`controller read forbidden obs field ${key}`);
    }
    return t[prop as keyof Observation];
  },
  has(t, prop) {
    return prop in t;
  },
});
try {
  agent.step(proxy);
  const extraKeys = [...accessed].filter((k) => !allowed.has(k) && k !== "then");
  check("observation_proxy_no_truth", extraKeys.length === 0, `accessed=${[...accessed].join(",")}`);
} catch (e) {
  check("observation_proxy_no_truth", false, String(e));
}

// Randomised FDIR: fault time and thruster drawn independently of 73.4 / +Y.
function fdirCase(seed: number): PrivateScenario {
  const rng = makeRng(seed);
  return {
    seed,
    c1: 0.13,
    c2: 0.09,
    k12: 0.3,
    etaT: 0.87,
    faultTime: 2.4 + rng.u01() * 3.2,
    faultThruster: Math.floor(rng.u01() * 6) as ThrusterIndex,
    gyroBias0: [0.001, -0.0005, 0.0007],
    demo: false,
    q0: [Q0[0], Q0[1], Q0[2], Q0[3]],
    w0: [W0[0], W0[1], W0[2]],
    s0: 0.25,
    sd0: -0.05,
  };
}

let fdirOk = 0;
const fdirN = 6;
for (let i = 0; i < fdirN; i++) {
  const sc = fdirCase(8800 + i * 13);
  const c = defaultPublicConfig({ duration: 16, seed: sc.seed });
  const sim = new Simulator(c, sc);
  sim.runAll();
  const m = sim.metrics();
  const delay = m.isolationDelay;
  const okIso = m.isolatedThrusterId === sc.faultThruster;
  const delayOk = delay !== null && delay >= 0.05 && delay < 4.0;
  const notTiny = delay !== null && delay > 0.02;
  const pass = okIso && delayOk && notTiny;
  if (pass) fdirOk += 1;
  check(
    `random_fdir_seed_${sc.seed}`,
    pass,
    `thr ${sc.faultThruster}→${m.isolatedThrusterId} t_f=${sc.faultTime.toFixed(2)} isoΔ=${delay?.toFixed(3)} detΔ=${m.detectionDelay?.toFixed(3)}`,
  );
}
check("random_fdir_all", fdirOk === fdirN, `${fdirOk}/${fdirN}`);

// Isolation delay definition: 74.4 − 73.4 = 1.0, never 0.001.
{
  const events: SimEvent[] = [
    { t: 0, type: "scenario", data: { faultTime: 73.4, faultThruster: 2 } },
    { t: 73.4, type: "fault_injected", data: { thruster: 2 } },
    { t: 73.45, type: "abnormal_flag" },
    { t: 73.5, type: "fault_detected" },
    { t: 74.4, type: "fault_isolated", data: { thruster: 2, confidence: 0.84 } },
  ];
  const r = fdirFromEvents(events);
  check(
    "isolation_delay_definition",
    r.isolationDelay !== null && Math.abs(r.isolationDelay - 1.0) < 1e-9 && Math.abs(r.detectionDelay! - 0.1) < 1e-9,
    `isoΔ=${r.isolationDelay} detΔ=${r.detectionDelay}`,
  );
}

const cons = runConservation(2);
writeJson("outputs/conservation.json", cons);
check("conservation_energy", cons.energyRel < 0.08, `relΔE=${cons.energyRel.toExponential(2)}`);
check("conservation_H", cons.angularMomentumRel < 0.05, `relΔH=${cons.angularMomentumRel.toExponential(2)}`);
check("conservation_q", cons.quaternionNormMax < 1e-6, `|q|-1=${cons.quaternionNormMax.toExponential(2)}`);

const conv = runConvergence(0.8);
writeJson("outputs/convergence.json", conv);
check(
  "convergence_recorded",
  Number.isFinite(conv["trajectoryDiff_5ms_vs_1.25ms"]) && conv.collisionTime["5ms"] !== null,
  `order≈${conv.observedOrder_trajectory_coarse_to_mid?.toFixed(2)} dTcol=${conv.collisionTime.delta_5_vs_1_25?.toExponential(2)}`,
);

const all = [...unit, ...extra];
const fail = all.filter((t) => !t.pass).length;
console.log(`\n${all.length - fail}/${all.length} passed`);
process.exit(fail ? 1 : 0);
