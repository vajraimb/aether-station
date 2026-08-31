/**
 * Guidance planner. Reduced rollout, horizon 3–5 s.
 * Objective: enter the terminal reachable set / basin. Does not claim
 * a predicted final attitude below 1°.
 */
import { torqueColumns } from "../allocate";
import { MIN_PULSE, THRUSTERS } from "../constants";
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
  DEFAULT_ROLLOUT_CONFIG,
  geodesicAttitudeError,
  rolloutAdvance,
  type RolloutConfig,
  type RolloutParameters,
  type RolloutState,
} from "./rollout-model";
import { basinFlags, TERMINAL_ENTRY_DEG } from "./terminal-reachable";
import { TERMINAL_FUEL_GATE } from "./terminal-planner";

export const GUIDANCE_HORIZON_S = 4;

export interface GuidanceConfig {
  horizonS: number;
  beamWidth: number;
  expansionBudget: number;
  fuelFloorKg: number;
  fuelReserveKg: number;
  entryDeg: number;
  rollout: RolloutConfig;
}

export const DEFAULT_GUIDANCE_CONFIG: GuidanceConfig = {
  horizonS: GUIDANCE_HORIZON_S,
  beamWidth: 20,
  expansionBudget: 900,
  fuelFloorKg: TERMINAL_FUEL_GATE,
  fuelReserveKg: 0.08,
  entryDeg: TERMINAL_ENTRY_DEG,
  rollout: DEFAULT_ROLLOUT_CONFIG,
};

export interface GuidancePlanResult {
  primitive: PulsePrimitive;
  plan: readonly PulsePrimitive[];
  predictedAttDeg: number;
  predictedOmega: number;
  predictedFuelKg: number;
  predictedInBasin: boolean;
  expandedNodes: number;
  fallback: boolean;
  reason: string;
}

interface GuidanceScore {
  hard: number;
  fuelBelow: number;
  wrongWay: number;
  basinMiss: number;
  perpExcess: number;
  attRad: number;
  omega: number;
  perp: number;
  wPar: number;
  fuelUsed: number;
  switches: number;
}

function compareGuidance(a: GuidanceScore, b: GuidanceScore): number {
  const keys: (keyof GuidanceScore)[] = [
    "hard",
    "fuelBelow",
    "wrongWay",
    "basinMiss",
    "perpExcess",
    "attRad",
    "perp",
    "omega",
    "fuelUsed",
    "switches",
  ];
  for (const k of keys) {
    if (a[k] < b[k]) return -1;
    if (a[k] > b[k]) return 1;
  }
  return 0;
}

function scoreGuidance(
  state: RolloutState,
  plant: PublicConfig,
  extras: { fuelUsed: number; switches: number; hard: number },
  entryDeg: number,
  fuelFloor: number,
): GuidanceScore {
  const q = qnormalize(state.qBI);
  const attRad = attitudeErrorAngle(q, plant.qTarget);
  const attErr = attitudeErrorVector(q, plant.qTarget);
  const n = vnorm(attErr);
  const eN: Vec3 = n > 1e-9 ? vscale(attErr, 1 / n) : [1, 0, 0];
  const wPar = vdot(state.omegaB, eN);
  const perp = vnorm(vsub(state.omegaB, vscale(eN, wPar)));
  const flags = basinFlags(state, plant, entryDeg, fuelFloor);
  return {
    hard: extras.hard,
    fuelBelow: state.fuelMass + 1e-9 < fuelFloor ? 1 : 0,
    wrongWay: wPar > 0.01 ? 1 : 0,
    basinMiss: flags.inBasin ? 0 : 1,
    perpExcess: perp > 0.03 ? 1 : 0,
    attRad,
    omega: vnorm(state.omegaB),
    perp,
    wPar,
    fuelUsed: extras.fuelUsed,
    switches: extras.switches,
  };
}

function guidanceWant(state: RolloutState, plant: PublicConfig): Vec3 {
  const q = qnormalize(state.qBI);
  const attErr = attitudeErrorVector(q, plant.qTarget);
  const attDeg = deg(attitudeErrorAngle(q, plant.qTarget));
  const w = state.omegaB;
  const n = vnorm(attErr);
  const eN: Vec3 = n > 1e-9 ? vscale(attErr, 1 / n) : [1, 0, 0];
  const wPar = vdot(w, eN);
  const wPerp = vsub(w, vscale(eN, wPar));
  if (vnorm(w) > 0.08) return vscale(w, -1);
  if (wPar > 0.01) return vscale(eN, -1);
  if (vnorm(wPerp) > 0.02 && attDeg > 4) return vscale(wPerp, -1);
  const kRate = attDeg > 20 ? 0.1 : 0.16;
  const wDes = vscale(attErr, -2 * kRate);
  const wCap = attDeg > 25 ? 0.05 : 0.035;
  const nn = vnorm(wDes);
  const wDesSat = nn > wCap && nn > 1e-9 ? vscale(wDes, wCap / nn) : wDes;
  return vscale(vsub(w, wDesSat), -1);
}

export function selectGuidancePrimitives(
  state: RolloutState,
  params: RolloutParameters,
  plant: PublicConfig,
  fuelFloorKg: number,
  reserveKg: number,
): PulsePrimitive[] {
  const isolated = new Set(params.failedThrusterBeliefs);
  const attDeg = deg(attitudeErrorAngle(qnormalize(state.qBI), plant.qTarget));
  const wmag = vnorm(state.omegaB);
  const { wPerp } = { wPerp: (() => {
    const attErr = attitudeErrorVector(qnormalize(state.qBI), plant.qTarget);
    const n = vnorm(attErr);
    const eN: Vec3 = n > 1e-9 ? vscale(attErr, 1 / n) : [1, 0, 0];
    return vnorm(vsub(state.omegaB, vscale(eN, vdot(state.omegaB, eN))));
  })() };
  const noPairs = attDeg > 20 || wmag > 0.07 || wPerp > 0.03;
  const durations: PulseDurationS[] = attDeg > 20 || wmag > 0.06 ? [0.16, 0.24, 0.32] : [0.08, 0.16, 0.24];
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
  const coast = coasts.sort((a, b) => b.durationS - a.durationS)[0] ?? coasts[0];
  const ms = massState(plant, state.sliderS, state.theta1, state.theta2, state.fuelMass);
  const want = guidanceWant(state, plant);
  const wantN = vnorm(want);
  const eWant: Vec3 = wantN > 1e-9 ? vscale(want, 1 / wantN) : [1, 0, 0];
  type Ranked = { p: PulsePrimitive; proj: number };
  const ranked: Ranked[] = [];
  for (const p of executable) {
    if (p.thrusterIds.length === 0) continue;
    if (noPairs && p.thrusterIds.length === 2) continue;
    const w = netWrenchForPrimitive(p, THRUSTERS, params.etaTEstimate, ms.rCmB, plant.maxThrust);
    ranked.push({ p, proj: vdot(w.torqueB, eWant) });
  }
  ranked.sort((a, b) => (Math.abs(b.proj - a.proj) > 1e-9 ? b.proj - a.proj : a.p.id < b.p.id ? -1 : 1));
  const cols = torqueColumns(plant, state.sliderS, state.theta1, state.theta2, state.fuelMass, params.etaTEstimate);
  const width = durations[durations.length - 1] ?? 0.16;
  const analytic = pulseAlongWant(eWant, cols, isolated, width, noPairs ? 1 : 2);
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
  if (!(noPairs && analyticPrim && analyticPrim.thrusterIds.length === 2)) take(analyticPrim);
  for (const r of ranked) {
    if (picked.size >= 7) break;
    take(r.p);
  }
  const out = [...picked.values()];
  out.sort((a, b) => (a.id < b.id ? -1 : a.id > b.id ? 1 : 0));
  return out;
}

interface Node {
  state: RolloutState;
  actions: PulsePrimitive[];
  score: GuidanceScore;
  key: string;
}

export function planGuidance(
  rootState: RolloutState,
  params: RolloutParameters,
  plant: PublicConfig,
  cfg: GuidanceConfig = DEFAULT_GUIDANCE_CONFIG,
): GuidancePlanResult {
  const isolated = new Set(params.failedThrusterBeliefs);
  const coast = generatePulsePrimitives(THRUSTERS, {
    isolatedThrusters: isolated,
    durationsS: [MIN_PULSE as PulseDurationS],
  }).find((p) => p.thrusterIds.length === 0)!;
  const maxDepth = Math.max(2, Math.round(cfg.horizonS / 0.28));
  const tLimit = rootState.time + cfg.horizonS;
  const coastedRoot =
    tLimit - rootState.time > 1e-3
      ? rolloutAdvance(rootState, params, plant, tLimit - rootState.time, cfg.rollout)
      : rootState;
  const rootScore = scoreGuidance(coastedRoot, plant, { fuelUsed: 0, switches: 0, hard: 0 }, cfg.entryDeg, cfg.fuelFloorKg);
  let frontier: Node[] = [
    { state: cloneRolloutState(rootState), actions: [], score: rootScore, key: "root" },
  ];
  let expanded = 0;
  let best = frontier[0]!;

  depthLoop: for (let depth = 0; depth < maxDepth; depth += 1) {
    const children: Node[] = [];
    for (const node of frontier) {
      if (node.state.time + 1e-9 >= tLimit) continue;
      const cands = selectGuidancePrimitives(node.state, params, plant, cfg.fuelFloorKg, cfg.fuelReserveKg);
      for (const prim of cands) {
        if (expanded >= cfg.expansionBudget) break depthLoop;
        expanded += 1;
        const fired = applyPrimitiveUntilComplete(node.state, params, plant, prim, cfg.rollout);
        const remain = tLimit - fired.time;
        const scored = remain > 1e-3 ? rolloutAdvance(fired, params, plant, remain, cfg.rollout) : fired;
        if (scored.fuelMass + 1e-12 < cfg.fuelFloorKg + cfg.fuelReserveKg && prim.thrusterIds.length > 0) continue;
        const hard =
          node.score.hard +
          (scored.fuelMass < 0 ? 1 : 0) +
          (Math.abs(scored.sliderS) > plant.sliderMax + 1e-6 ? 1 : 0) +
          (Number.isFinite(scored.omegaB[0]) ? 0 : 1);
        const used = node.score.fuelUsed + Math.max(0, node.state.fuelMass - fired.fuelMass);
        const score = scoreGuidance(
          scored,
          plant,
          { fuelUsed: used, switches: node.score.switches + (prim.thrusterIds.length > 0 ? 1 : 0), hard },
          cfg.entryDeg,
          cfg.fuelFloorKg,
        );
        const child: Node = {
          state: fired,
          actions: [...node.actions, prim],
          score,
          key: [...node.actions, prim].map((p) => p.id).join("|"),
        };
        children.push(child);
      }
    }
    if (children.length === 0) break;
    children.sort((a, b) => {
      const c = compareGuidance(a.score, b.score);
      return c !== 0 ? c : a.key < b.key ? -1 : 1;
    });
    frontier = children.slice(0, cfg.beamWidth);
    if (compareGuidance(frontier[0]!.score, best.score) < 0) best = frontier[0]!;
  }

  frontier.sort((a, b) => {
    const c = compareGuidance(a.score, b.score);
    return c !== 0 ? c : a.key < b.key ? -1 : 1;
  });
  if (frontier[0] && compareGuidance(frontier[0].score, best.score) < 0) best = frontier[0];
  const first = best.actions[0] ?? coast;
  const att = best.score.attRad;
  return {
    primitive: first,
    plan: best.actions,
    predictedAttDeg: deg(att),
    predictedOmega: best.score.omega,
    predictedFuelKg: Math.max(0, rootState.fuelMass - best.score.fuelUsed),
    predictedInBasin: best.score.basinMiss === 0,
    expandedNodes: expanded,
    fallback: best.actions.length === 0,
    reason: best.actions.length === 0 ? "empty-plan-coast" : "guidance-beam",
  };
}
