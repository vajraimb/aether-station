#!/usr/bin/env npx tsx
import { AgentController } from "../controller.ts";
import { defaultPublicConfig } from "../constants.ts";
import { generateScenario } from "../scenario.ts";
import { Simulator } from "../simulator.ts";
import { estimateAlphaMax, switchCommand } from "../planner.ts";
import { torqueColumns } from "../allocate.ts";
import { massState } from "../dynamics.ts";
import { minv3, vnorm } from "../math3d.ts";

const seed = Number(process.argv[2] ?? 800017);
const demo = process.argv.includes("--demo");
const cfg = defaultPublicConfig({ seed, fluidPresent: true });
const sc = generateScenario(seed, demo);
const sim = new Simulator(cfg, sc);
let lastPrint = -1;
while (sim.step()) {
  const t = sim.state.t;
  if (t < 96 && t - lastPrint < 2.0 && t < cfg.duration - 0.2) continue;
  if (t >= 96 && t - lastPrint < 4.0 && t < cfg.duration - 0.2) continue;
  lastPrint = t;
  const est = sim.controller.getEstimate();
  const cols = torqueColumns(cfg, est.s, est.th1, est.th2, est.fuel, est.etaT);
  const ms = massState(cfg, est.s, est.th1, est.th2, est.fuel);
  const agent = sim.controller as AgentController;
  const aMax = estimateAlphaMax(cols, agent.isolated, minv3(ms.Icm));
  const cmd = switchCommand(cfg, est.q, est.w, aMax, { wCap: 0.05, alphaScale: 0.5 });
  const att = sim.log[sim.log.length - 1]?.attitudeErrorDeg ?? 0;
  const on = sim.lastCmd.pulseWidth.filter((w) => w > 0).length;
  console.log(
    `t=${t.toFixed(1).padStart(5)} att=${att.toFixed(2).padStart(7)} w=${vnorm(est.w).toFixed(4)} wPar=${cmd.wPar.toFixed(3)} wPerp=${cmd.wPerpN.toFixed(3)} fuelT=${sim.state.fuel.toFixed(3)} fuelE=${est.fuel.toFixed(3)} phase=${cmd.phase.padEnd(8)} jets=${on} iso=${[...agent.isolated].join(",") || "-"}`,
  );
}
const m = sim.metrics();
console.log(
  `\nFINAL att=${m.final_attitude_error_deg.toFixed(3)} w=${m.final_angular_speed_rad_s.toFixed(4)} fuel=${m.remaining_fuel_kg.toFixed(3)} param=${m.parameter_relative_error.toFixed(3)} isoΔ=${m.isolationDelay?.toFixed(2)}`,
);
