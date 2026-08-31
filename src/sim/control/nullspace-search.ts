/**
 * Offline 8–20 segment null-space sequence search. Not a controller.
 *
 * Allows intermediate perpendicular growth and only scores the net
 * accumulated ΔH_⊥ after the whole sequence, plus fuel and slosh.
 */
import { CMD_DELAY, MAX_ACTIVE, MIN_PULSE, THRUSTERS } from "../constants";
import { massState } from "../dynamics";
import {
  qRotate,
  vadd,
  vdot,
  vnorm,
  vscale,
  vsub,
  type Vec3,
} from "../math3d";
import type { PublicConfig } from "../types";
import {
  generatePulsePrimitives,
  isLegalPulsePrimitive,
  leavesFuelFloor,
  netWrenchForPrimitive,
  type PulseDurationS,
  type PulsePrimitive,
} from "./discrete-actions";
import {
  applyPrimitiveUntilComplete,
  DEFAULT_ROLLOUT_CONFIG,
  predictedSloshEnergy,
  rolloutAdvance,
  type RolloutParameters,
  type RolloutState,
} from "./rollout-model";
import { representativeMacroStates } from "./action-macros";
import { eigenComponents } from "./terminal-reachable";

export const SEARCH_PULSE_S = [0.04, 0.08] as const satisfies readonly PulseDurationS[];
export const SEARCH_COAST_S = 0.08 as PulseDurationS;
export const FUEL_FLOOR = 2.8;
export const DEFAULT_MAX_SEG = 16;
export const DEFAULT_BEAM = 18;

export interface SequenceResult {
  readonly method: string;
  readonly stateId: string;
  readonly isolated: readonly number[];
  readonly nSeg: number;
  readonly ids: readonly string[];
  readonly dtHPar: number;
  readonly dtHPerp: number;
  readonly peakHPerp: number;
  readonly rho: number;
  readonly fuelKg: number;
  readonly slosh: number;
  readonly durationS: number;
  readonly targetMet: boolean;
  readonly hPar0: number;
  readonly hPar1: number;
  readonly fractionParReduced: number;
}

function impulseI(
  state: RolloutState,
  params: RolloutParameters,
  plant: PublicConfig,
  prim: PulsePrimitive,
): Vec3 {
  if (prim.thrusterIds.length === 0) return [0, 0, 0];
  const ms = massState(plant, state.sliderS, state.theta1, state.theta2, state.fuelMass);
  const w = netWrenchForPrimitive(prim, THRUSTERS, params.etaTEstimate, ms.rCmB, plant.maxThrust);
  return qRotate(state.qBI, w.angularImpulse);
}

export interface AxisFrame {
  readonly eI: Vec3;
  readonly hPar0: number;
  readonly target: number;
}

export function axisFrame(plant: PublicConfig, state: RolloutState): AxisFrame {
  const eigen = eigenComponents(state, plant);
  const eI = qRotate(state.qBI, eigen.eN);
  const ms = massState(plant, state.sliderS, state.theta1, state.theta2, state.fuelMass);
  const hB: Vec3 = [
    ms.Icm[0][0] * state.omegaB[0] + ms.Icm[0][1] * state.omegaB[1] + ms.Icm[0][2] * state.omegaB[2],
    ms.Icm[1][0] * state.omegaB[0] + ms.Icm[1][1] * state.omegaB[1] + ms.Icm[1][2] * state.omegaB[2],
    ms.Icm[2][0] * state.omegaB[0] + ms.Icm[2][1] * state.omegaB[1] + ms.Icm[2][2] * state.omegaB[2],
  ];
  const hPar0 = vdot(qRotate(state.qBI, hB), eI);
  const mag = Math.abs(hPar0);
  return { eI, hPar0, target: Math.min(0.35 * mag, 12) };
}

function applySeg(
  state: RolloutState,
  params: RolloutParameters,
  plant: PublicConfig,
  prim: PulsePrimitive,
): RolloutState {
  if (prim.thrusterIds.length === 0) {
    return rolloutAdvance(state, params, plant, prim.durationS, DEFAULT_ROLLOUT_CONFIG);
  }
  return applyPrimitiveUntilComplete(state, params, plant, prim, DEFAULT_ROLLOUT_CONFIG);
}

export function searchActionSet(isolated: ReadonlySet<number>): PulsePrimitive[] {
  const pulses = generatePulsePrimitives(THRUSTERS, {
    isolatedThrusters: isolated,
    durationsS: SEARCH_PULSE_S,
    includeCoast: false,
  });
  const singles = pulses.filter((p) => p.thrusterIds.length === 1);
  const pairs = pulses.filter((p) => p.thrusterIds.length === 2 && p.durationS === 0.04);
  const coast: PulsePrimitive = {
    id: `coast:${SEARCH_COAST_S.toFixed(3)}`,
    thrusterIds: [],
    durationS: SEARCH_COAST_S,
    commandedThrustN: 0,
    propellantKg: 0,
  };
  return [...singles, ...pairs, coast].filter((p) => isLegalPulsePrimitive(p, THRUSTERS, isolated));
}

function legalNext(
  prim: PulsePrimitive,
  isolated: ReadonlySet<number>,
  fuel: number,
): boolean {
  if (!isLegalPulsePrimitive(prim, THRUSTERS, isolated)) return false;
  if (prim.thrusterIds.length > MAX_ACTIVE) return false;
  if (prim.thrusterIds.length > 0 && prim.durationS + 1e-12 < MIN_PULSE) return false;
  if (!leavesFuelFloor(prim, fuel, FUEL_FLOOR, 0)) return false;
  return true;
}

interface Node {
  state: RolloutState;
  seq: PulsePrimitive[];
  accum: Vec3;
  dHPar: number;
  dHPerp: number;
  peakPerp: number;
  fuelUsed: number;
  slosh: number;
}

function splitAccum(accum: Vec3, frame: AxisFrame): { dHPar: number; dHPerp: number } {
  const dHPar = vdot(accum, frame.eI);
  const dHPerp = vnorm(vsub(accum, vscale(frame.eI, dHPar)));
  return { dHPar, dHPerp };
}

function nodeAfter(
  prev: Node,
  prim: PulsePrimitive,
  next: RolloutState,
  dImp: Vec3,
  startFuel: number,
  frame: AxisFrame,
  params: RolloutParameters,
  plant: PublicConfig,
): Node {
  const accum = vadd(prev.accum, dImp);
  const { dHPar, dHPerp } = splitAccum(accum, frame);
  return {
    state: next,
    seq: [...prev.seq, prim],
    accum,
    dHPar,
    dHPerp,
    peakPerp: Math.max(prev.peakPerp, dHPerp),
    fuelUsed: Math.max(0, startFuel - next.fuelMass),
    slosh: predictedSloshEnergy(next, plant, params.k12Estimate),
  };
}

function parReduction(frame: AxisFrame, dHPar: number): number {
  const s = frame.hPar0 >= 0 ? 1 : -1;
  return -s * dHPar;
}

function lexBetter(a: Node, b: Node, frame: AxisFrame): boolean {
  const ra = parReduction(frame, a.dHPar);
  const rb = parReduction(frame, b.dHPar);
  const va = Math.max(0, frame.target - ra);
  const vb = Math.max(0, frame.target - rb);
  if (va < vb - 1e-9) return true;
  if (va > vb + 1e-9) return false;
  if (a.dHPerp < b.dHPerp - 1e-9) return true;
  if (a.dHPerp > b.dHPerp + 1e-9) return false;
  if (a.fuelUsed < b.fuelUsed - 1e-9) return true;
  if (a.fuelUsed > b.fuelUsed + 1e-9) return false;
  if (a.peakPerp < b.peakPerp - 1e-9) return true;
  if (a.peakPerp > b.peakPerp + 1e-9) return false;
  if (a.slosh < b.slosh - 1e-9) return true;
  return false;
}

function toResult(method: string, stateId: string, isolated: readonly number[], node: Node, frame: AxisFrame, t0: number): SequenceResult {
  const reduced = parReduction(frame, node.dHPar);
  const rho = Math.abs(node.dHPar) / (node.dHPerp + 1e-12);
  return {
    method,
    stateId,
    isolated,
    nSeg: node.seq.length,
    ids: node.seq.map((s) => s.id),
    dtHPar: node.dHPar,
    dtHPerp: node.dHPerp,
    peakHPerp: node.peakPerp,
    rho,
    fuelKg: node.fuelUsed,
    slosh: node.slosh,
    durationS: node.state.time - t0,
    targetMet: reduced + 1e-9 >= frame.target,
    hPar0: frame.hPar0,
    hPar1: frame.hPar0 + node.dHPar,
    fractionParReduced: Math.abs(frame.hPar0) < 1e-9 ? 0 : reduced / Math.abs(frame.hPar0),
  };
}

function rootNode(state: RolloutState, params: RolloutParameters, plant: PublicConfig): Node {
  return {
    state,
    seq: [],
    accum: [0, 0, 0],
    dHPar: 0,
    dHPerp: 0,
    peakPerp: 0,
    fuelUsed: 0,
    slosh: predictedSloshEnergy(state, plant, params.k12Estimate),
  };
}

function expand(
  node: Node,
  actions: readonly PulsePrimitive[],
  isolated: ReadonlySet<number>,
  params: RolloutParameters,
  plant: PublicConfig,
  frame: AxisFrame,
  startFuel: number,
): Node[] {
  const out: Node[] = [];
  for (const a of actions) {
    if (!legalNext(a, isolated, node.state.fuelMass)) continue;
    const dImp = impulseI(node.state, params, plant, a);
    const next = applySeg(node.state, params, plant, a);
    out.push(nodeAfter(node, a, next, dImp, startFuel, frame, params, plant));
  }
  return out;
}

function pickGreedy(
  node: Node,
  actions: readonly PulsePrimitive[],
  isolated: ReadonlySet<number>,
  params: RolloutParameters,
  plant: PublicConfig,
  frame: AxisFrame,
  startFuel: number,
  mode: "par" | "perp",
): Node | null {
  const kids = expand(node, actions, isolated, params, plant, frame, startFuel);
  if (kids.length === 0) return null;
  if (mode === "par") {
    const thrusting = kids.filter((k) => (k.seq[k.seq.length - 1]?.thrusterIds.length ?? 0) > 0);
    const pool = thrusting.length > 0 ? thrusting : kids;
    pool.sort((a, b) => parReduction(frame, b.dHPar) - parReduction(frame, a.dHPar));
    return pool[0]!;
  }
  kids.sort((a, b) => a.dHPerp - b.dHPerp || parReduction(frame, b.dHPar) - parReduction(frame, a.dHPar));
  return kids[0]!;
}

export function greedyParSequence(
  state: RolloutState,
  params: RolloutParameters,
  plant: PublicConfig,
  stateId: string,
  maxSeg = DEFAULT_MAX_SEG,
): SequenceResult {
  const isolated = new Set(params.failedThrusterBeliefs);
  const actions = searchActionSet(isolated);
  const frame = axisFrame(plant, state);
  const startFuel = state.fuelMass;
  let node: Node = rootNode(state, params, plant);
  for (let k = 0; k < maxSeg; k += 1) {
    if (parReduction(frame, node.dHPar) >= frame.target) break;
    const nxt = pickGreedy(node, actions, isolated, params, plant, frame, startFuel, "par");
    if (!nxt) break;
    node = nxt;
  }
  return toResult("greedy-par", stateId, [...isolated], node, frame, state.time);
}

export function greedyThenCancel(
  state: RolloutState,
  params: RolloutParameters,
  plant: PublicConfig,
  stateId: string,
  maxSeg = DEFAULT_MAX_SEG,
): SequenceResult {
  const isolated = new Set(params.failedThrusterBeliefs);
  const actions = searchActionSet(isolated);
  const frame = axisFrame(plant, state);
  const startFuel = state.fuelMass;
  let node: Node = rootNode(state, params, plant);
  const half = Math.max(4, Math.floor(maxSeg * 0.6));
  for (let k = 0; k < half; k += 1) {
    if (parReduction(frame, node.dHPar) >= frame.target) break;
    const nxt = pickGreedy(node, actions, isolated, params, plant, frame, startFuel, "par");
    if (!nxt) break;
    node = nxt;
  }
  while (node.seq.length < maxSeg) {
    const nxt = pickGreedy(node, actions, isolated, params, plant, frame, startFuel, "perp");
    if (!nxt) break;
    if (nxt.dHPerp > node.dHPerp + 1e-6 && parReduction(frame, node.dHPar) >= frame.target * 0.8) break;
    node = nxt;
  }
  return toResult("greedy-then-cancel", stateId, [...isolated], node, frame, state.time);
}

export function beamNullspace(
  state: RolloutState,
  params: RolloutParameters,
  plant: PublicConfig,
  stateId: string,
  maxSeg = DEFAULT_MAX_SEG,
  beamWidth = DEFAULT_BEAM,
): SequenceResult {
  const isolated = new Set(params.failedThrusterBeliefs);
  const actions = searchActionSet(isolated);
  const frame = axisFrame(plant, state);
  const startFuel = state.fuelMass;
  const root = rootNode(state, params, plant);
  let beam: Node[] = [root];
  let best: Node | null = null;
  for (let depth = 0; depth < maxSeg; depth += 1) {
    const nxt: Node[] = [];
    for (const n of beam) {
      nxt.push(...expand(n, actions, isolated, params, plant, frame, startFuel));
    }
    if (nxt.length === 0) break;
    nxt.sort((a, b) => {
      if (lexBetter(a, b, frame)) return -1;
      if (lexBetter(b, a, frame)) return 1;
      return 0;
    });
    beam = nxt.slice(0, beamWidth);
    for (const n of beam) {
      if (n.seq.length === 0) continue;
      if (!best || lexBetter(n, best, frame)) best = n;
    }
  }
  return toResult("beam-nullspace", stateId, [...isolated], best ?? root, frame, state.time);
}

export interface RobustSample {
  readonly etaScale: number;
  readonly cScale: number;
  readonly k12Scale: number;
  readonly dtHPar: number;
  readonly dtHPerp: number;
  readonly rho: number;
  readonly fractionParReduced: number;
  readonly targetMet: boolean;
}

export function replaySequence(
  state: RolloutState,
  params: RolloutParameters,
  plant: PublicConfig,
  seq: readonly PulsePrimitive[],
): RolloutState {
  let s = state;
  for (const p of seq) s = applySeg(s, params, plant, p);
  return s;
}

export function robustnessOf(
  state: RolloutState,
  params: RolloutParameters,
  plant: PublicConfig,
  seq: readonly PulsePrimitive[],
): RobustSample[] {
  const frame = axisFrame(plant, state);
  const startFuel = state.fuelMass;
  const scales: Array<[number, number, number]> = [];
  for (const e of [0.8, 1.2]) {
    for (const c of [0.8, 1.2]) {
      for (const k of [0.8, 1.2]) scales.push([e, c, k]);
    }
  }
  const out: RobustSample[] = [];
  for (const [etaScale, cScale, k12Scale] of scales) {
    const p: RolloutParameters = {
      ...params,
      etaTEstimate: params.etaTEstimate * etaScale,
      c1Estimate: params.c1Estimate * cScale,
      c2Estimate: params.c2Estimate * cScale,
      k12Estimate: params.k12Estimate * k12Scale,
    };
    let node = rootNode(state, p, plant);
    for (const prim of seq) {
      const dImp = impulseI(node.state, p, plant, prim);
      const next = applySeg(node.state, p, plant, prim);
      node = nodeAfter(node, prim, next, dImp, startFuel, frame, p, plant);
    }
    const reduced = parReduction(frame, node.dHPar);
    out.push({
      etaScale,
      cScale,
      k12Scale,
      dtHPar: node.dHPar,
      dtHPerp: node.dHPerp,
      rho: Math.abs(node.dHPar) / (node.dHPerp + 1e-12),
      fractionParReduced: Math.abs(frame.hPar0) < 1e-9 ? 0 : reduced / Math.abs(frame.hPar0),
      targetMet: reduced + 1e-9 >= frame.target,
    });
  }
  return out;
}

const STUDY_STATE_IDS = [
  "high-rate|healthy|empty",
  "high-rate|one-isolated|empty",
  "medium-rate|healthy|empty",
  "terminal|healthy|empty",
  "closing|healthy|empty",
  "opening|healthy|empty",
  "closing|one-isolated|empty",
  "detumble|healthy|empty",
];

export function runNullspaceStudy(
  plant: PublicConfig,
  opts: { maxSeg?: number; beamWidth?: number; quick?: boolean } = {},
): {
  maxSeg: number;
  beamWidth: number;
  results: SequenceResult[];
  robustness: Array<{ stateId: string; method: string; n: number; targetMetRate: number; medianRho: number; minFraction: number }>;
} {
  const maxSeg = opts.maxSeg ?? (opts.quick ? 8 : DEFAULT_MAX_SEG);
  const beamWidth = opts.beamWidth ?? (opts.quick ? 8 : DEFAULT_BEAM);
  const states = representativeMacroStates(plant).filter((s) => STUDY_STATE_IDS.includes(s.id));
  const picked = opts.quick ? states.slice(0, 4) : states;
  const results: SequenceResult[] = [];
  const robustness: Array<{
    stateId: string;
    method: string;
    n: number;
    targetMetRate: number;
    medianRho: number;
    minFraction: number;
  }> = [];
  for (const st of picked) {
    const g = greedyParSequence(st.state, st.params, plant, st.id, maxSeg);
    const c = greedyThenCancel(st.state, st.params, plant, st.id, maxSeg);
    const b = beamNullspace(st.state, st.params, plant, st.id, maxSeg, beamWidth);
    results.push(g, c, b);
    const catalog = searchActionSet(new Set(st.params.failedThrusterBeliefs));
    const prims = b.ids
      .map((id) => catalog.find((p) => p.id === id))
      .filter((p): p is PulsePrimitive => Boolean(p));
    const samples = robustnessOf(st.state, st.params, plant, prims);
    const rhos = samples.map((s) => s.rho).sort((a, b) => a - b);
    robustness.push({
      stateId: st.id,
      method: b.method,
      n: samples.length,
      targetMetRate: samples.filter((s) => s.targetMet).length / samples.length,
      medianRho: rhos[Math.floor(rhos.length / 2)] ?? NaN,
      minFraction: Math.min(...samples.map((s) => s.fractionParReduced)),
    });
  }
  return { maxSeg, beamWidth, results, robustness };
}

void CMD_DELAY;
