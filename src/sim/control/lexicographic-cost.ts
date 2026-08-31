import {
  attitudeErrorAngle,
  attitudeErrorVector,
  deg,
  qnormalize,
  vdot,
  vnorm,
  vscale,
  vsub,
  type Quat,
  type Vec3,
} from "../math3d";
import { predictedSloshEnergy, type RolloutState } from "./rollout-model";
import type { PublicConfig } from "../types";

export interface LexicographicScore {
  hardViolationCount: number;
  predictedFuelBelowFloor: number;
  terminalRateGateFailure: number;
  terminalAttitudeGateFailure: number;
  sliderRisk: number;
  sloshRisk: number;
  attRad: number;
  omega: number;
  perpMomentum: number;
  sloshEnergy: number;
  fuelConsumedKg: number;
  pulseTransitions: number;
  constraintMargin: number;
}

export interface ScoreContext {
  qTarget: Quat;
  durationS: number;
  fuelFloorKg: number;
  rateGate: number;
  attGateRad: number;
  sliderMax: number;
  initialFuelKg: number;
  alphaMax: number;
  plant: PublicConfig;
  k12: number;
  scoreTimeIsTerminal: boolean;
}

export function compareLexicographic(a: LexicographicScore, b: LexicographicScore): -1 | 0 | 1 {
  const rank: (keyof LexicographicScore)[] = [
    "hardViolationCount",
    "predictedFuelBelowFloor",
    "terminalRateGateFailure",
    "terminalAttitudeGateFailure",
    "sliderRisk",
    "sloshRisk",
  ];
  for (const key of rank) {
    if (a[key] < b[key]) return -1;
    if (a[key] > b[key]) return 1;
  }
  const cont: (keyof LexicographicScore)[] = [
    "attRad",
    "omega",
    "perpMomentum",
    "sloshEnergy",
    "fuelConsumedKg",
    "pulseTransitions",
  ];
  for (const key of cont) {
    if (a[key] < b[key]) return -1;
    if (a[key] > b[key]) return 1;
  }
  if (a.constraintMargin > b.constraintMargin) return -1;
  if (a.constraintMargin < b.constraintMargin) return 1;
  return 0;
}

export function quaternionSignInvariantAtt(q: Quat, qTarget: Quat): number {
  const a = attitudeErrorAngle(qnormalize(q), qTarget);
  const nq: Quat = [-q[0], -q[1], -q[2], -q[3]];
  const b = attitudeErrorAngle(qnormalize(nq), qTarget);
  return Math.abs(a - b);
}

function remainingBurnFeasible(state: RolloutState, ctx: ScoreContext): { attFail: number; rateFail: number } {
  const tGo = Math.max(0, ctx.durationS - state.time);
  const q = qnormalize(state.qBI);
  const theta = attitudeErrorAngle(q, ctx.qTarget);
  const w = vnorm(state.omegaB);
  const attErr = attitudeErrorVector(q, ctx.qTarget);
  const tPred = Math.min(8, tGo);
  const predicted = vnorm([
    2 * attErr[0] + state.omegaB[0] * tPred,
    2 * attErr[1] + state.omegaB[1] * tPred,
    2 * attErr[2] + state.omegaB[2] * tPred,
  ]);
  const attMetric = tGo < 2 ? theta : Math.min(theta, predicted);
  const attFail = attMetric > ctx.attGateRad ? 1 : 0;
  const rateFail = tGo < 6 && w > ctx.rateGate ? 1 : w > 0.12 ? 1 : 0;
  return { attFail, rateFail };
}

export function scoreRollout(
  state: RolloutState,
  ctx: ScoreContext,
  extras: {
    fuelUsedKg: number;
    pulseTransitions: number;
    hardViolationCount: number;
  },
): LexicographicScore {
  const q = qnormalize(state.qBI);
  const attNow = attitudeErrorAngle(q, ctx.qTarget);
  const attErr = attitudeErrorVector(q, ctx.qTarget);
  const tPred = Math.min(8, Math.max(0, ctx.durationS - state.time));
  const predicted = vnorm([
    2 * attErr[0] + state.omegaB[0] * tPred,
    2 * attErr[1] + state.omegaB[1] * tPred,
    2 * attErr[2] + state.omegaB[2] * tPred,
  ]);
  const attRad = ctx.scoreTimeIsTerminal || tPred < 1.5 ? attNow : 0.35 * attNow + 0.65 * predicted;
  const omega = vnorm(state.omegaB);
  const eNmag = vnorm(attErr);
  const eN: Vec3 = eNmag > 1e-9 ? vscale(attErr, 1 / eNmag) : [1, 0, 0];
  const wPar = vdot(state.omegaB, eN);
  const wPerp = vsub(state.omegaB, vscale(eN, wPar));
  const perpMomentum = vnorm(wPerp);
  const slosh = predictedSloshEnergy(state, ctx.plant, ctx.k12);
  const sliderMargin = ctx.sliderMax - Math.abs(state.sliderS);
  const sliderRisk =
    Math.abs(state.sliderS) > ctx.sliderMax - 0.05 || (Math.abs(state.sliderS) > 1.5 && state.sliderS * state.sliderV > 0)
      ? 1
      : 0;
  const sloshRisk = slosh > 4 ? 1 : 0;
  const fuelBelow = state.fuelMass + 1e-9 < ctx.fuelFloorKg ? 1 : 0;
  const terminal = remainingBurnFeasible(state, ctx);
  return {
    hardViolationCount: extras.hardViolationCount,
    predictedFuelBelowFloor: fuelBelow,
    terminalRateGateFailure: terminal.rateFail,
    terminalAttitudeGateFailure: terminal.attFail,
    sliderRisk,
    sloshRisk,
    attRad,
    omega,
    perpMomentum,
    sloshEnergy: slosh,
    fuelConsumedKg: extras.fuelUsedKg,
    pulseTransitions: extras.pulseTransitions,
    constraintMargin: sliderMargin,
  };
}

export function attDegOf(score: LexicographicScore): number {
  return deg(score.attRad);
}
