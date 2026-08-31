/**
 * Offline pulse-sequence optimizer. May read the full truth trajectory model
 * but must obey every physical actuator constraint. Not used in flight.
 *
 * Search: truth-fed receding-horizon mixed pulse catalogue with multiple
 * restarts of (wCap, α-scale, horizon). Reports reachable attitude / fuel
 * Pareto bounds. Flight controller does not import this module.
 */
import { defaultPublicConfig } from "./constants";
import { generateScenario } from "./scenario";
import { Simulator } from "./simulator";
import { TruthFeedbackBaseline } from "./oracle";
import { FUEL_FLOOR, TRAIN_SEEDS } from "./evalset";
import type { PlannerOpts } from "./planner";
import { writeJson } from "./io";

/**
 * Truth-fed receding-horizon planner. Same actuator constraints as flight;
 * the only extra privilege is the true (q, ω, s, θ, fuel) fed to the law
 * and a restart-specific (wCap, α-scale, horizon).
 */
export class OfflinePulseController extends TruthFeedbackBaseline {
  override readonly name = "offlinePulseOptimizer";

  constructor(cfg: ReturnType<typeof defaultPublicConfig>, opts?: PlannerOpts) {
    super(cfg, {
      horizon: opts?.horizon ?? 12,
      wCap: opts?.wCap ?? 0.1,
      alphaScale: opts?.alphaScale ?? 0.7,
      ignoreDelay: false,
    });
  }
}

export interface OptimizerReport {
  seeds: number[];
  truthFeedback: { att: number; fuel: number; omega: number }[];
  offline: { att: number; fuel: number; omega: number; restart: PlannerOpts }[];
  minReachableAttDeg: number;
  fuelToReach1deg: number | null;
  bestAttAtFuelFloor: number;
  restarts: number;
  restartGrid: PlannerOpts[];
  note: string;
}

const RESTART_GRID: PlannerOpts[] = [
  { wCap: 0.09, alphaScale: 0.55, horizon: 8 },
  { wCap: 0.11, alphaScale: 0.7, horizon: 10 },
  { wCap: 0.08, alphaScale: 0.45, horizon: 12 },
  { wCap: 0.13, alphaScale: 0.85, horizon: 8 },
  { wCap: 0.1, alphaScale: 0.6, horizon: 15 },
];

function better(row: { att: number; fuel: number; omega: number }, best: { att: number; fuel: number; omega: number }) {
  const rowFloor = row.fuel >= FUEL_FLOOR;
  const bestFloor = best.fuel >= FUEL_FLOOR;
  if (rowFloor && !bestFloor) return true;
  if (!rowFloor && bestFloor) return false;
  if (rowFloor && bestFloor) {
    const rowRate = row.omega < 0.008;
    const bestRate = best.omega < 0.008;
    if (rowRate && !bestRate) return true;
    if (!rowRate && bestRate) return false;
    if (row.att < best.att - 1e-9) return true;
    if (Math.abs(row.att - best.att) < 1e-9 && row.fuel > best.fuel) return true;
    return false;
  }
  return row.fuel > best.fuel;
}

export function runOfflineOptimizer(n = 6): OptimizerReport {
  const seeds = TRAIN_SEEDS.slice(0, n);
  const truthFeedback: OptimizerReport["truthFeedback"] = [];
  const offline: OptimizerReport["offline"] = [];

  for (const seed of seeds) {
    const cfgB = defaultPublicConfig({ seed, fluidPresent: true });
    const scB = generateScenario(seed, false);
    const base = new TruthFeedbackBaseline(cfgB);
    const simB = new Simulator(cfgB, scB, base);
    simB.runAll();
    const mB = simB.metrics();
    truthFeedback.push({
      att: mB.final_attitude_error_deg,
      fuel: mB.remaining_fuel_kg,
      omega: mB.final_angular_speed_rad_s,
    });

    let best = { att: Infinity, fuel: 0, omega: Infinity, restart: RESTART_GRID[0]! };
    for (const r of RESTART_GRID) {
      const cfg = defaultPublicConfig({ seed, fluidPresent: true });
      const sc = generateScenario(seed, false);
      const opt = new OfflinePulseController(cfg, r);
      const sim = new Simulator(cfg, sc, opt);
      sim.runAll();
      const m = sim.metrics();
      const row = {
        att: m.final_attitude_error_deg,
        fuel: m.remaining_fuel_kg,
        omega: m.final_angular_speed_rad_s,
        restart: r,
      };
      if (better(row, best)) best = row;
    }
    offline.push(best);
  }

  const atts = offline.map((r) => r.att);
  const reached = offline.filter((r) => r.att < 1 && r.omega < 0.008 && r.fuel >= FUEL_FLOOR);
  const fuelToReach1deg = reached.length ? Math.min(...reached.map((r) => 5 - r.fuel)) : null;
  const nearFloor = offline.filter((r) => r.fuel >= FUEL_FLOOR - 0.05);
  const bestAttAtFuelFloor = nearFloor.length ? Math.min(...nearFloor.map((r) => r.att)) : Math.min(...atts);

  const report: OptimizerReport = {
    seeds,
    truthFeedback,
    offline,
    minReachableAttDeg: Math.min(...atts),
    fuelToReach1deg,
    bestAttAtFuelFloor,
    restarts: RESTART_GRID.length,
    restartGrid: RESTART_GRID,
    note:
      "Offline search = truth-fed receding-horizon mixed pulse planner with 5 restarts of (wCap, α-scale, horizon). Plant, delay, min-pulse, max-two and fuel floor are enforced. Flight controller does not import this module.",
  };
  writeJson("outputs/optimizer.json", report);
  return report;
}
