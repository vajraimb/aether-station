/**
 * High-fidelity terminal capture planner.
 * Frozen-kernel rollout (rk4 + collision), 40/80/120/160 ms pulses,
 * delay-complete pending queue, horizon 0.5–2 s.
 * Does not replace the reduced beam planner.
 */
import { torqueColumns } from "../allocate";
import { CMD_DELAY, MAX_ACTIVE, MIN_PULSE, THRUSTERS } from "../constants";
import { massState } from "../dynamics";
import {
  attitudeErrorAngle,
  attitudeErrorVector,
  deg,
  qnormalize,
  vdot,
  vnorm,
  vscale,
  vsub,
  type Vec3,
} from "../math3d";
import { pulseAlongWant } from "../planner";
import type { PublicConfig } from "../types";
import {
  filterExecutablePrimitives,
  generatePulsePrimitives,
  netWrenchForPrimitive,
  type PulseDurationS,
  type PulsePrimitive,
} from "./discrete-actions";
import {
  applyPrimitiveUntilComplete,
  cloneRolloutState,
  geodesicAttitudeError,
  rolloutAdvance,
  type RolloutConfig,
  type RolloutParameters,
  type RolloutState,
} from "./rollout-model";

export const TERMINAL_PULSE_DURATIONS = [0.04, 0.08, 0.12, 0.16] as const satisfies readonly PulseDurationS[];

export const TERMINAL_ROLLOUT_CONFIG: RolloutConfig = {
  dt: 0.01,
  commandDelayS: CMD_DELAY,
  maxActive: MAX_ACTIVE,
  useCollision: false,
  fast: false,
};

export const TERMINAL_ATT_GATE_RAD = Math.PI / 180;
export const TERMINAL_RATE_GATE = 0.008;
export const TERMINAL_FUEL_GATE = 2.8;

export interface TerminalSearchConfig {
  horizonS: number;
  beamWidth: number;
  expansionBudget: number;
  fuelFloorKg: number;
  fuelReserveKg: number;
  rollout: RolloutConfig;
}

export const DEFAULT_TERMINAL_CONFIG: TerminalSearchConfig = {
  horizonS: 1.0,
  beamWidth: 6,
  expansionBudget: 24,
  fuelFloorKg: TERMINAL_FUEL_GATE,
  fuelReserveKg: 0.04,
  rollout: TERMINAL_ROLLOUT_CONFIG,
};

export interface TerminalPlanResult {
  primitive: PulsePrimitive;
  plan: readonly PulsePrimitive[];
  captured: boolean;
  predictedAttRad: number;
  predictedAttDeg: number;
  predictedOmega: number;
  predictedFuelKg: number;
  expandedNodes: number;
  fallback: boolean;
  reason: string;
}

function coastPrimitive(isolated: ReadonlySet<number>): PulsePrimitive {
  return generatePulsePrimitives(THRUSTERS, {
    isolatedThrusters: isolated,
    durationsS: [MIN_PULSE as PulseDurationS],
  }).find((p) => p.thrusterIds.length === 0)!;
}

function eigenWant(state: RolloutState, plant: PublicConfig): Vec3 {
  const q = qnormalize(state.qBI);
  const attErr = attitudeErrorVector(q, plant.qTarget);
  const w = state.omegaB;
  const attDeg = deg(attitudeErrorAngle(q, plant.qTarget));
  const kRate = attDeg < 3 ? 0.35 : 0.18;
  const wDes = vscale(attErr, -2 * kRate);
  const wCap = attDeg < 3 ? 0.02 : 0.04;
  const n = vnorm(wDes);
  const wDesSat = n > wCap && n > 1e-9 ? vscale(wDes, wCap / n) : wDes;
  return vscale(vsub(w, wDesSat), -1);
}

export function selectTerminalPrimitives(
  state: RolloutState,
  params: RolloutParameters,
  plant: PublicConfig,
  fuelFloorKg: number,
  reserveKg: number,
): PulsePrimitive[] {
  const isolated = new Set(params.failedThrusterBeliefs);
  const attDeg = deg(attitudeErrorAngle(qnormalize(state.qBI), plant.qTarget));
  const durations: PulseDurationS[] =
    attDeg < 3 ? [0.04, 0.08, 0.12] : attDeg < 8 ? [0.04, 0.08, 0.12, 0.16] : [0.08, 0.12, 0.16];
  const generated = generatePulsePrimitives(THRUSTERS, {
    isolatedThrusters: isolated,
    durationsS: durations,
  });
  const executable = filterExecutablePrimitives(generated, {
    nowS: state.time,
    commandDelayS: plant.commandDelay,
    pendingPulses: state.pendingPulses,
    isolatedThrusters: isolated,
    estimatedFuelKg: state.fuelMass,
    fuelFloorKg,
    reserveKg,
    maxActive: plant.maxActiveThrusters,
  });
  const coasts = executable.filter((p) => p.thrusterIds.length === 0);
  const coast = coasts.sort((a, b) => a.durationS - b.durationS)[0] ?? coasts[0];
  const ms = massState(plant, state.sliderS, state.theta1, state.theta2, state.fuelMass);
  const want = eigenWant(state, plant);
  const wantN = vnorm(want);
  const eWant: Vec3 = wantN > 1e-9 ? vscale(want, 1 / wantN) : [1, 0, 0];
  type Ranked = { p: PulsePrimitive; proj: number };
  const ranked: Ranked[] = [];
  for (const p of executable) {
    if (p.thrusterIds.length === 0) continue;
    const w = netWrenchForPrimitive(p, THRUSTERS, params.etaTEstimate, ms.rCmB, plant.maxThrust);
    ranked.push({ p, proj: vdot(w.torqueB, eWant) });
  }
  ranked.sort((a, b) => (Math.abs(b.proj - a.proj) > 1e-9 ? b.proj - a.proj : a.p.id < b.p.id ? -1 : 1));
  const cols = torqueColumns(plant, state.sliderS, state.theta1, state.theta2, state.fuelMass, params.etaTEstimate);
  const width = durations[Math.min(1, durations.length - 1)] ?? 0.08;
  const analytic = pulseAlongWant(eWant, cols, isolated, width, 1);
  const analyticIds = [0, 1, 2, 3, 4, 5].filter((i) => (analytic[i] ?? 0) > 0);
  const analyticPrim = executable.find(
    (p) =>
      Math.abs(p.durationS - width) < 1e-9 &&
      p.thrusterIds.length === analyticIds.length &&
      analyticIds.every((id) => p.thrusterIds.includes(id)),
  );
  const picked = new Map<string, PulsePrimitive>();
  const take = (p: PulsePrimitive | undefined) => {
    if (p && !picked.has(p.id)) picked.set(p.id, p);
  };
  take(coast);
  take(analyticPrim);
  const bestBySingle = new Map<number, Ranked>();
  const pairs: Ranked[] = [];
  for (const r of ranked) {
    if (r.p.thrusterIds.length === 1) {
      const id = r.p.thrusterIds[0]!;
      const prev = bestBySingle.get(id);
      const longer = prev && Math.abs(r.proj - prev.proj) <= 1e-9 && r.p.durationS > prev.p.durationS;
      if (!prev || r.proj > prev.proj + 1e-9 || longer) bestBySingle.set(id, r);
    } else if (r.p.thrusterIds.length === 2) {
      pairs.push(r);
    }
  }
  const singles = [...bestBySingle.values()].sort((a, b) => b.proj - a.proj || b.p.durationS - a.p.durationS);
  for (const r of singles) take(r.p);
  pairs.sort((a, b) => b.proj - a.proj || (a.p.id < b.p.id ? -1 : 1));
  take(pairs[0]?.p);
  const out = [...picked.values()];
  out.sort((a, b) => (a.id < b.id ? -1 : a.id > b.id ? 1 : 0));
  return out;
}

interface Node {
  state: RolloutState;
  scored: RolloutState;
  actions: PulsePrimitive[];
  fuelUsed: number;
  hard: number;
  key: string;
}

function terminalRank(a: RolloutState, b: RolloutState, plant: PublicConfig, fuelFloor: number): number {
  const gates = (s: RolloutState) => {
    const att = geodesicAttitudeError(s.qBI, plant.qTarget);
    const w = vnorm(s.omegaB);
    const fuelFail = s.fuelMass + 1e-9 < fuelFloor ? 1 : 0;
    const near = att < 2 * TERMINAL_ATT_GATE_RAD;
    const rateFail = near && w > TERMINAL_RATE_GATE ? 1 : w > 0.04 ? 1 : 0;
    const attFail = att > TERMINAL_ATT_GATE_RAD ? 1 : 0;
    return { fuelFail, rateFail, attFail, att, w, fuel: s.fuelMass, near };
  };
  const A = gates(a);
  const B = gates(b);
  if (A.fuelFail !== B.fuelFail) return A.fuelFail - B.fuelFail;
  if (A.near && B.near) {
    if (A.rateFail !== B.rateFail) return A.rateFail - B.rateFail;
    if (A.attFail !== B.attFail) return A.attFail - B.attFail;
  } else {
    if (A.attFail !== B.attFail) return A.attFail - B.attFail;
    if (A.rateFail !== B.rateFail) return A.rateFail - B.rateFail;
  }
  if (Math.abs(A.att - B.att) > 1e-9) return A.att - B.att;
  if (Math.abs(A.w - B.w) > 1e-9) return A.w - B.w;
  return B.fuel - A.fuel;
}

function meetsGates(state: RolloutState, plant: PublicConfig, fuelFloor: number): boolean {
  return (
    geodesicAttitudeError(state.qBI, plant.qTarget) < TERMINAL_ATT_GATE_RAD &&
    vnorm(state.omegaB) < TERMINAL_RATE_GATE &&
    state.fuelMass > fuelFloor
  );
}

export function searchTerminal(
  rootState: RolloutState,
  params: RolloutParameters,
  plant: PublicConfig,
  cfg: TerminalSearchConfig = DEFAULT_TERMINAL_CONFIG,
): TerminalPlanResult {
  const isolated = new Set(params.failedThrusterBeliefs);
  const coast = coastPrimitive(isolated);
  const att0 = geodesicAttitudeError(rootState.qBI, plant.qTarget);
  const horizon = att0 < 3 * TERMINAL_ATT_GATE_RAD ? Math.min(cfg.horizonS, 0.8) : cfg.horizonS;
  const maxDepth = Math.max(2, Math.round(horizon / 0.16));
  const root: Node = {
    state: cloneRolloutState(rootState),
    scored: cloneRolloutState(rootState),
    actions: [],
    fuelUsed: 0,
    hard: 0,
    key: "root",
  };
  let frontier: Node[] = [root];
  let expanded = 0;
  let best = root;
  const tLimit = rootState.time + horizon;
  {
    const remain0 = tLimit - rootState.time;
    if (remain0 > 1e-3) root.scored = rolloutAdvance(root.state, params, plant, remain0, cfg.rollout);
    best = root;
  }

  depthLoop: for (let depth = 0; depth < maxDepth; depth += 1) {
    const children: Node[] = [];
    for (const node of frontier) {
      if (node.state.time + 1e-9 >= tLimit) continue;
      const cands = selectTerminalPrimitives(node.state, params, plant, cfg.fuelFloorKg, cfg.fuelReserveKg);
      for (const prim of cands) {
        if (expanded >= cfg.expansionBudget) break depthLoop;
        expanded += 1;
        const fired = applyPrimitiveUntilComplete(node.state, params, plant, prim, cfg.rollout);
        const remain = tLimit - fired.time;
        const scored = remain > 1e-3 ? rolloutAdvance(fired, params, plant, remain, cfg.rollout) : fired;
        if (scored.fuelMass + 1e-12 < cfg.fuelFloorKg && prim.thrusterIds.length > 0) continue;
        const hard =
          node.hard +
          (scored.fuelMass < 0 ? 1 : 0) +
          (Math.abs(scored.sliderS) > plant.sliderMax + 1e-6 ? 1 : 0) +
          (Number.isFinite(scored.omegaB[0]) ? 0 : 1);
        const child: Node = {
          state: fired,
          scored,
          actions: [...node.actions, prim],
          fuelUsed: node.fuelUsed + Math.max(0, node.state.fuelMass - fired.fuelMass),
          hard,
          key: [...node.actions, prim].map((p) => p.id).join("|"),
        };
        children.push(child);
        if (child.hard <= best.hard && terminalRank(child.scored, best.scored, plant, cfg.fuelFloorKg) < 0) {
          best = child;
        }
      }
    }
    if (children.length === 0) break;
    children.sort((a, b) => {
      const h = a.hard - b.hard;
      if (h !== 0) return h;
      const r = terminalRank(a.scored, b.scored, plant, cfg.fuelFloorKg);
      if (r !== 0) return r;
      return a.key < b.key ? -1 : 1;
    });
    frontier = children.slice(0, cfg.beamWidth);
    if (terminalRank(frontier[0]!.scored, best.scored, plant, cfg.fuelFloorKg) < 0) best = frontier[0]!;
    if (meetsGates(best.scored, plant, cfg.fuelFloorKg)) break;
  }

  const first = best.actions[0] ?? coast;
  const att = geodesicAttitudeError(best.scored.qBI, plant.qTarget);
  return {
    primitive: first,
    plan: best.actions,
    captured: meetsGates(best.scored, plant, cfg.fuelFloorKg),
    predictedAttRad: att,
    predictedAttDeg: deg(att),
    predictedOmega: vnorm(best.scored.omegaB),
    predictedFuelKg: best.scored.fuelMass,
    expandedNodes: expanded,
    fallback: best.actions.length === 0,
    reason: best.actions.length === 0 ? "empty-plan-coast" : "terminal-beam",
  };
}

export function planTerminal(
  rootState: RolloutState,
  params: RolloutParameters,
  plant: PublicConfig,
  cfg: TerminalSearchConfig = DEFAULT_TERMINAL_CONFIG,
): TerminalPlanResult {
  return searchTerminal(rootState, params, plant, cfg);
}
