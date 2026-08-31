import { pulseAlongWant } from "../planner";
import { torqueColumns } from "../allocate";
import { MIN_PULSE, THRUSTERS } from "../constants";
import {
  attitudeErrorAngle,
  attitudeErrorVector,
  deg,
  qnormalize,
  vadd,
  vdot,
  vnorm,
  vscale,
  vsub,
  type Vec3,
} from "../math3d";
import type { PublicConfig } from "../types";
import {
  filterExecutablePrimitives,
  generatePulsePrimitives,
  netWrenchForPrimitive,
  type PulseDurationS,
  type PulsePrimitive,
} from "./discrete-actions";
import {
  compareLexicographic,
  scoreRollout,
  type LexicographicScore,
  type ScoreContext,
} from "./lexicographic-cost";
import { captureCost as knnCaptureCost } from "./capture-value";
import {
  applyPrimitive,
  cloneRolloutState,
  evaluateRolloutWrench,
  type RolloutConfig,
  type RolloutParameters,
  type RolloutState,
  DEFAULT_ROLLOUT_CONFIG,
} from "./rollout-model";
import { massState } from "../dynamics";

export interface SearchNode {
  state: RolloutState;
  actions: readonly PulsePrimitive[];
  depth: number;
  fuelUsedKg: number;
  pulseTransitions: number;
  hardViolationCount: number;
  score: LexicographicScore;
  deterministicKey: string;
}

export interface BeamPlannerConfig {
  horizonS: number;
  beamWidth: number;
  expansionBudget: number;
  fuelFloorKg: number;
  fuelReserveKg: number;
  rollout: RolloutConfig;
}

export const DEFAULT_BEAM_CONFIG: BeamPlannerConfig = {
  horizonS: 8,
  beamWidth: 24,
  expansionBudget: 2800,
  fuelFloorKg: 2.8,
  fuelReserveKg: 0.08,
  rollout: DEFAULT_ROLLOUT_CONFIG,
};

export interface BeamDiagnostics {
  expandedNodes: number;
  retainedNodes: number;
  nodesPrunedForFuel: number;
  minimumPredictedFuel: number;
  reserveKg: number;
  selectedPlanFuelMargin: number;
  selectedPrimitiveId: string | null;
  predictedTerminalAttitudeErrorDeg: number | null;
  predictedTerminalAngularSpeedRadS: number | null;
  predictedTerminalFuelKg: number | null;
  fallback: boolean;
  reason: string;
}

export interface BeamResult {
  primitive: PulsePrimitive;
  plan: readonly PulsePrimitive[];
  diagnostics: BeamDiagnostics;
}

function actionKey(actions: readonly PulsePrimitive[]): string {
  return actions.map((a) => a.id).join("|") || "root";
}

function pendingSignature(state: RolloutState): string {
  if (state.pendingPulses.length === 0) return "-";
  return state.pendingPulses
    .map((p) => `${p.id}:${p.tOn.toFixed(3)}:${p.tOff.toFixed(3)}`)
    .join(",");
}

function failedMask(params: RolloutParameters): number {
  let m = 0;
  for (const id of params.failedThrusterBeliefs) m |= 1 << id;
  return m;
}

export function quantizeStateKey(
  state: RolloutState,
  params: RolloutParameters,
  plant: PublicConfig,
  depth: number,
): string {
  const q = qnormalize(state.qBI);
  const theta = attitudeErrorAngle(q, plant.qTarget);
  const e = attitudeErrorVector(q, plant.qTarget);
  const near = theta < 0.06;
  const aBucket = near ? 0.004 : 0.02;
  const wBucket = near ? 0.002 : 0.008;
  const fBucket = 0.02;
  const ax = Math.round(e[0] / aBucket);
  const ay = Math.round(e[1] / aBucket);
  const az = Math.round(e[2] / aBucket);
  const wx = Math.round(state.omegaB[0] / wBucket);
  const wy = Math.round(state.omegaB[1] / wBucket);
  const wz = Math.round(state.omegaB[2] / wBucket);
  const fuel = Math.round(state.fuelMass / fBucket);
  return `${depth}|${ax},${ay},${az}|${wx},${wy},${wz}|${fuel}|${pendingSignature(state)}|${failedMask(params)}`;
}

function wantVector(state: RolloutState, plant: PublicConfig): Vec3 {
  const q = qnormalize(state.qBI);
  const attErr = attitudeErrorVector(q, plant.qTarget);
  const theta = attitudeErrorAngle(q, plant.qTarget);
  const attDeg = deg(theta);
  const w = state.omegaB;
  const wmag = vnorm(w);
  const eNmag = vnorm(attErr);
  const eN: Vec3 = eNmag > 1e-9 ? vscale(attErr, 1 / eNmag) : [1, 0, 0];
  const wPar = vdot(w, eN);
  const wPerp = vsub(w, vscale(eN, wPar));
  if (wmag > 0.07) return vscale(w, -1);
  if (wPar > 0.012) return vscale(eN, -1);
  if (vnorm(wPerp) > 0.03 && attDeg > 2) return vscale(wPerp, -1);
  const kRate = attDeg > 8 ? 0.12 : 0.22;
  const wDes = vscale(attErr, -2 * kRate);
  const wCap = attDeg > 12 ? 0.05 : 0.035;
  const wDesN = vnorm(wDes);
  const wDesSat = wDesN > wCap && wDesN > 1e-9 ? vscale(wDes, wCap / wDesN) : wDes;
  return vscale(vsub(w, wDesSat), -1);
}

function durationSet(state: RolloutState, plant: PublicConfig): PulseDurationS[] {
  const attDeg = deg(attitudeErrorAngle(qnormalize(state.qBI), plant.qTarget));
  const wmag = vnorm(state.omegaB);
  if (attDeg > 12 || wmag > 0.06) return [0.16, 0.24, 0.32];
  if (attDeg < 2.2 && wmag < 0.02) return [MIN_PULSE as PulseDurationS, 0.08, 0.12];
  return [0.08, 0.16, 0.24];
}

export function selectCandidatePrimitives(
  state: RolloutState,
  params: RolloutParameters,
  plant: PublicConfig,
  fuelFloorKg: number,
  reserveKg: number,
): PulsePrimitive[] {
  const isolated = new Set(params.failedThrusterBeliefs);
  const durations = durationSet(state, plant);
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
  const want = wantVector(state, plant);
  const wantN = vnorm(want);
  const eWant: Vec3 = wantN > 1e-9 ? vscale(want, 1 / wantN) : [1, 0, 0];

  type Ranked = { p: PulsePrimitive; proj: number; align: number; perp: number };
  const ranked: Ranked[] = [];
  for (const p of executable) {
    if (p.thrusterIds.length === 0) continue;
    const w = netWrenchForPrimitive(p, THRUSTERS, params.etaTEstimate, ms.rCmB, plant.maxThrust);
    const n = vnorm(w.torqueB);
    const proj = vdot(w.torqueB, eWant);
    const align = n > 1e-9 ? proj / n : 0;
    const perp = Math.sqrt(Math.max(0, n * n - proj * proj));
    ranked.push({ p, proj, align, perp });
  }
  ranked.sort((a, b) => {
    if (Math.abs(b.proj - a.proj) > 1e-9) return b.proj - a.proj;
    return a.p.id < b.p.id ? -1 : 1;
  });
  const cols = torqueColumns(plant, state.sliderS, state.theta1, state.theta2, state.fuelMass, params.etaTEstimate);
  const width = durations[durations.length - 1] ?? 0.16;
  const analytic = pulseAlongWant(eWant, cols, isolated, width, 1);
  const analyticIds = [0, 1, 2, 3, 4, 5].filter((i) => (analytic[i] ?? 0) > 0);
  const analyticPrim = executable.find(
    (p) => p.durationS === width && p.thrusterIds.length === analyticIds.length && analyticIds.every((id) => p.thrusterIds.includes(id)),
  );
  const picked = new Map<string, PulsePrimitive>();
  const take = (p: PulsePrimitive | undefined) => {
    if (p && !picked.has(p.id)) picked.set(p.id, p);
  };
  take(coast);
  take(analyticPrim);
  take(ranked[0]?.p);
  const neg = [...ranked].sort((a, b) => a.proj - b.proj || (a.p.id < b.p.id ? -1 : 1));
  take(neg[0]?.p);
  const minPerp = [...ranked].filter((r) => r.align > 0.15).sort((a, b) => a.perp - b.perp || (a.p.id < b.p.id ? -1 : 1));
  take(minPerp[0]?.p);
  for (const r of ranked) {
    if (picked.size >= 8) break;
    take(r.p);
  }
  if (picked.size < 5) {
    for (const p of executable) {
      if (picked.size >= 6) break;
      take(p);
    }
  }
  const out = [...picked.values()];
  out.sort((a, b) => (a.id < b.id ? -1 : a.id > b.id ? 1 : 0));
  return out;
}

function nodeFrom(
  state: RolloutState,
  actions: readonly PulsePrimitive[],
  depth: number,
  fuelUsedKg: number,
  pulseTransitions: number,
  hardViolationCount: number,
  ctx: ScoreContext,
): SearchNode {
  const score = scoreRollout(state, ctx, { fuelUsedKg, pulseTransitions, hardViolationCount });
  return {
    state,
    actions,
    depth,
    fuelUsedKg,
    pulseTransitions,
    hardViolationCount,
    score,
    deterministicKey: actionKey(actions),
  };
}

function cmpNodes(a: SearchNode, b: SearchNode): number {
  const c = compareLexicographic(a.score, b.score);
  if (c !== 0) return c;
  if (a.deterministicKey < b.deterministicKey) return -1;
  if (a.deterministicKey > b.deterministicKey) return 1;
  return 0;
}

export function planBeam(
  rootState: RolloutState,
  params: RolloutParameters,
  plant: PublicConfig,
  cfg: BeamPlannerConfig = DEFAULT_BEAM_CONFIG,
): BeamResult {
  const isolated = new Set(params.failedThrusterBeliefs);
  const ctx: ScoreContext = {
    qTarget: plant.qTarget,
    durationS: plant.duration,
    fuelFloorKg: cfg.fuelFloorKg + cfg.fuelReserveKg,
    rateGate: 0.008,
    attGateRad: Math.PI / 180,
    sliderMax: plant.sliderMax,
    initialFuelKg: plant.initialFuelMass,
    alphaMax: 0.014,
    plant,
    k12: params.k12Estimate,
    scoreTimeIsTerminal: false,
    captureCostOf: (s) => knnCaptureCost(s, params.failedThrusterBeliefs, plant),
  };
  const coast = generatePulsePrimitives(THRUSTERS, {
    isolatedThrusters: isolated,
    durationsS: [MIN_PULSE as PulseDurationS],
  }).find((p) => p.thrusterIds.length === 0)!;

  const root = nodeFrom(cloneRolloutState(rootState), [], 0, 0, 0, 0, ctx);
  let frontier: SearchNode[] = [root];
  let expanded = 0;
  let retained = 1;
  let prunedFuel = 0;
  let minFuel = rootState.fuelMass;
  const maxDepth = Math.max(1, Math.round(cfg.horizonS / 0.32));
  const seenGlobal = new Set<string>();

  depthLoop: for (let depth = 0; depth < maxDepth; depth += 1) {
    const children: SearchNode[] = [];
    const seen = new Set<string>();
    for (const node of frontier) {
      const cands = selectCandidatePrimitives(node.state, params, plant, cfg.fuelFloorKg, cfg.fuelReserveKg);
      for (const prim of cands) {
        if (expanded >= cfg.expansionBudget) break depthLoop;
        expanded += 1;
        const next = applyPrimitive(node.state, params, plant, prim, cfg.rollout);
        minFuel = Math.min(minFuel, next.fuelMass);
        const hard =
          node.hardViolationCount +
          (next.fuelMass < 0 ? 1 : 0) +
          (Math.abs(next.sliderS) > plant.sliderMax + 1e-6 ? 1 : 0) +
          (Number.isFinite(next.omegaB[0]) ? 0 : 1);
        const used = node.fuelUsedKg + Math.max(0, node.state.fuelMass - next.fuelMass);
        const trans = node.pulseTransitions + (prim.thrusterIds.length > 0 ? 1 : 0);
        if (next.fuelMass + 1e-12 < cfg.fuelFloorKg + cfg.fuelReserveKg && prim.thrusterIds.length > 0) {
          prunedFuel += 1;
          continue;
        }
        const child = nodeFrom(next, [...node.actions, prim], depth + 1, used, trans, hard, ctx);
        const key = quantizeStateKey(next, params, plant, depth + 1);
        const prev = seen.has(key);
        if (prev) continue;
        seen.add(key);
        if (seenGlobal.has(key) && child.score.attRad > 0.02) continue;
        seenGlobal.add(key);
        children.push(child);
      }
    }
    if (children.length === 0) break;
    children.sort(cmpNodes);
    frontier = children.slice(0, cfg.beamWidth);
    retained = frontier.length;
  }

  frontier.sort(cmpNodes);
  const best = frontier[0] ?? root;
  const first = best.actions[0] ?? coast;
  const fallback = best.actions.length === 0;
  const margin = best.state.fuelMass - cfg.fuelFloorKg;
  return {
    primitive: first,
    plan: best.actions,
    diagnostics: {
      expandedNodes: expanded,
      retainedNodes: retained,
      nodesPrunedForFuel: prunedFuel,
      minimumPredictedFuel: minFuel,
      reserveKg: cfg.fuelReserveKg,
      selectedPlanFuelMargin: margin,
      selectedPrimitiveId: first.id,
      predictedTerminalAttitudeErrorDeg: deg(best.score.attRad),
      predictedTerminalAngularSpeedRadS: best.score.omega,
      predictedTerminalFuelKg: best.state.fuelMass,
      fallback,
      reason: fallback ? "empty-plan-coast" : "beam",
    },
  };
}

void evaluateRolloutWrench;
void vadd;
