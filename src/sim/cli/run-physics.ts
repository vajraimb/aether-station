#!/usr/bin/env npx tsx
/**
 * Physics + isolation contract tests. Headless.
 *   npm run test:physics
 *   npm run test:physics -- --full   # 180 s at every dt
 */
import { execSync } from "node:child_process";
import { readFileSync, readdirSync, statSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { runAllTests } from "../tests.ts";
import { runCollision, runReactionAudit, runSmooth, writeLedgers, observedOrder } from "../audit.ts";
import { writeJson } from "../io.ts";
import { fdirFromEvents } from "../scoring.ts";
import { Simulator } from "../simulator.ts";
import { AgentController } from "../controller.ts";
import { OBSERVATION_KEYS } from "../types.ts";
import type { Observation, PrivateScenario, SimEvent, ThrusterIndex } from "../types.ts";
import { Q0, W0, defaultPublicConfig } from "../constants.ts";
import { makeRng } from "../math3d.ts";
import { runDiscreteActionTests } from "../control/discrete-actions.test.ts";
import { runControllerProtocolTests } from "../control/interface.test.ts";
import { runControlV2Tests } from "../control/control-v2.test.ts";
import { runBenchmarkContractTests } from "../benchmark.test.ts";
import { runLedgerTests } from "../ledger/catalog.test.ts";

const here = dirname(fileURLToPath(import.meta.url));
const simDir = join(here, "..");
const full = process.argv.includes("--full");

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

for (const name of runDiscreteActionTests()) {
  extra.push({ name: `discrete:${name}`, pass: true, detail: "ok" });
  console.log(`PASS  discrete:${name}  ok`);
}

for (const r of runControllerProtocolTests()) {
  extra.push(r);
  console.log(`${r.pass ? "PASS" : "FAIL"}  ${r.name}  ${r.detail}`);
}

for (const r of runControlV2Tests()) {
  extra.push(r);
  console.log(`${r.pass ? "PASS" : "FAIL"}  ${r.name}  ${r.detail}`);
}

for (const r of runBenchmarkContractTests()) {
  extra.push(r);
  console.log(`${r.pass ? "PASS" : "FAIL"}  ${r.name}  ${r.detail}`);
}

for (const r of runLedgerTests()) {
  extra.push(r);
  console.log(`${r.pass ? "PASS" : "FAIL"}  ${r.name}  ${r.detail}`);
}

function walk(dir: string, acc: string[] = []): string[] {
  for (const name of readdirSync(dir)) {
    const p = join(dir, name);
    const st = statSync(p);
    if (st.isDirectory()) walk(p, acc);
    else if (name.endsWith(".ts")) acc.push(p);
  }
  return acc;
}
void walk;

const ctrlFiles = [
  "controller.ts",
  "allocate.ts",
  "fdir.ts",
  "estimator.ts",
  "planner.ts",
  "control/baseline.ts",
  "control/factory.ts",
  "control/interface.ts",
  "control/discrete-actions.ts",
  "control/rollout-model.ts",
  "control/rollout-error.ts",
  "control/beam-planner.ts",
  "control/lexicographic-cost.ts",
  "control/terminal-planner.ts",
  "control/terminal-reachable.ts",
  "control/guidance-planner.ts",
  "control/controller-v2.ts",
].map((f) => join(simDir, f));
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

const dts = [0.01, 0.005, 0.0025, 0.00125, 0.000625];
const orderDur = 4;
const orderRuns = dts.map((dt) => runSmooth(dt, orderDur, true));
const ref = orderRuns[orderRuns.length - 1]!;
const errors = orderRuns.map((r) => {
  if (r.dt === ref.dt) return 0;
  const dq = Math.hypot(r.qFinal[1]! - ref.qFinal[1]!, r.qFinal[2]! - ref.qFinal[2]!, r.qFinal[3]! - ref.qFinal[3]!);
  const dw = Math.hypot(r.wFinal[0]! - ref.wFinal[0]!, r.wFinal[1]! - ref.wFinal[1]!, r.wFinal[2]! - ref.wFinal[2]!);
  return dq + dw + Math.abs(r.sFinal - ref.sFinal);
});
const orders: number[] = [];
for (let i = 0; i < errors.length - 2; i++) orders.push(observedOrder(errors[i]!, errors[i + 1]!));
const minOrder = Math.min(...orders.filter((x) => Number.isFinite(x)));
check("rk4_observed_order", minOrder >= 3.7, `p=${orders.map((x) => x.toFixed(2)).join(",")} min=${minOrder.toFixed(2)}`);

const longDts = full ? dts : [0.01, 0.005];
const longRuns = longDts.map((dt) => runSmooth(dt, 180));
writeJson("outputs/conservation.json", {
  duration: 180,
  dts: longRuns.map((r) => ({
    dt: r.dt,
    energyRel: r.energyRel,
    angularMomentumRel: r.angularMomentumRel,
    linearMomentumAbs: r.linearMomentumAbs,
    energy0: r.energy0,
    energy1: r.energy1,
    collided: r.collided,
    maxSlider: r.maxSlider,
    qNormRawMax: r.qNormRawMax,
  })),
});
for (const r of longRuns) {
  check(
    `smooth_180s_dt_${r.dt * 1000}ms`,
    r.energyRel < 1e-4 && r.angularMomentumRel < 1e-4 && !r.collided,
    `relE=${r.energyRel.toExponential(2)} relH=${r.angularMomentumRel.toExponential(2)} |s|max=${r.maxSlider.toFixed(3)} |q|-1=${r.qNormRawMax.toExponential(2)}`,
  );
}

writeJson("outputs/convergence.json", {
  duration: orderDur,
  dts,
  errors,
  orders,
  minObservedOrder: minOrder,
  energyRel: Object.fromEntries(orderRuns.map((r) => [String(r.dt), r.energyRel])),
  angularMomentumRel: Object.fromEntries(orderRuns.map((r) => [String(r.dt), r.angularMomentumRel])),
});

const cols = dts.map((dt) => runCollision(dt, 0.5));
writeJson(
  "outputs/collision-audit.json",
  cols,
);
const tHits = cols.map((c) => c.tHit ?? NaN);
const tSpan = Math.abs(tHits[0]! - tHits[tHits.length - 1]!);
check(
  "collision_event_time_converges",
  tSpan < 5e-5 && cols.every((c) => c.tHit !== null),
  `tHit ${tHits[0]?.toFixed(6)} → ${tHits[tHits.length - 1]?.toFixed(6)} span=${tSpan.toExponential(2)}`,
);
check(
  "collision_no_penetration",
  cols.every((c) => !c.penetrated),
  `sMax=${Math.max(...cols.map((c) => c.sMax)).toFixed(4)}`,
);
check(
  "collision_H_conserved",
  cols.every((c) => c.HrelJump < 1e-8),
  `max |ΔH|=${Math.max(...cols.map((c) => c.HrelJump)).toExponential(2)}`,
);
check(
  "collision_energy_loss_positive",
  cols.every((c) => c.energyLoss > 0),
  `ΔE=${cols[1]!.energyLoss.toFixed(3)} naive=${cols[1]!.naiveLoss.toFixed(3)} (e=0.15)`,
);

const rx = runReactionAudit();
writeJson("outputs/reaction-audit.json", rx);
check("reaction_slider_Fext", rx.sliderNetForce < 1e-6, `F=${rx.sliderNetForce.toExponential(2)}`);
check("reaction_slosh_Fext", rx.slosh1NetForce < 1e-6 && rx.slosh2NetForce < 1e-6, `F1=${rx.slosh1NetForce} F2=${rx.slosh2NetForce}`);
check("reaction_dHdt", rx.dHdtResidual < 1e-6 && rx.sliderNetTorqueCm < 1e-6, `dH=${rx.dHdtResidual.toExponential(2)}`);
check(
  "reaction_dEdt",
  rx.dEdtResidualConservative < 1e-6 && rx.dEdtResidualDamped < 1e-3 && rx.dEdtResidualActuated < 2e-3,
  `cons=${rx.dEdtResidualConservative.toExponential(2)} damp=${rx.dEdtResidualDamped.toExponential(2)} act=${rx.dEdtResidualActuated.toExponential(2)}`,
);
writeLedgers(8, 0.005);

{
  let diff = "";
  try {
    diff = execSync(
      "git diff --name-only bdfff5b4c62733d7156d2f9cdeeaa75661d6c9f4 -- src/sim/math3d.ts src/sim/dynamics.ts src/sim/audit.ts",
      { encoding: "utf8" },
    ).trim();
  } catch (e) {
    diff = String(e);
  }
  check("test_physics_baseline_unchanged", diff === "", diff === "" ? "math3d/dynamics/audit match bdfff5b" : diff);
}

const all = [...unit, ...extra];
const fail = all.filter((t) => !t.pass).length;
console.log(`\n${all.length - fail}/${all.length} passed`);
process.exit(fail ? 1 : 0);
