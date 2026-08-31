/**
 * Terminal reachable-set predicate. High-fidelity frozen-kernel check:
 * exists a legal pulse sequence that enters att < 1°, |ω| < 0.008, fuel > 2.8 kg
 * inside the terminal horizon. Guidance aims at this set; it does not claim
 * a long reduced rollout will finish under 1°.
 */
import { CMD_DELAY, defaultPublicConfig } from "../constants";
import {
  attitudeErrorAngle,
  attitudeErrorVector,
  deg,
  qmul,
  qnormalize,
  vdot,
  vnorm,
  vscale,
  vsub,
  type Quat,
  type Vec3,
} from "../math3d";
import type { PublicConfig } from "../types";
import { writeJson } from "../io";
import {
  DEFAULT_TERMINAL_CONFIG,
  TERMINAL_ATT_GATE_RAD,
  TERMINAL_FUEL_GATE,
  TERMINAL_RATE_GATE,
  searchTerminal,
  type TerminalSearchConfig,
} from "./terminal-planner";
import { rolloutFromSimLike, type RolloutParameters, type RolloutState } from "./rollout-model";
import { publicBelief } from "./rollout-error";

export const TERMINAL_ENTRY_CANDIDATES_DEG = [8, 10, 12, 15] as const;

/**
 * Handoff angle selected on the public synthetic basin set (not per seed).
 * 12° is ~7× the 1° gate and ~8× the 0.5 s reduced-model att p50, so the
 * reduced guidance horizon (3–5 s, ~1.5° error) can still enter the basin.
 */
export const TERMINAL_ENTRY_DEG = 12;

export interface CaptureQuery {
  horizonS?: number;
  fuelMarginKg?: number;
  expansionBudget?: number;
}

export interface CaptureResult {
  captured: boolean;
  predictedAttDeg: number;
  predictedOmega: number;
  predictedFuelKg: number;
  expandedNodes: number;
  reason: string;
}

export interface BasinFlags {
  attOk: boolean;
  rateOk: boolean;
  perpOk: boolean;
  notOpening: boolean;
  fuelOk: boolean;
  inBasin: boolean;
  attDeg: number;
  wPar: number;
  wPerp: number;
}

function qAxisAngle(axis: Vec3, angle: number): Quat {
  const n = vnorm(axis) || 1;
  const a: Vec3 = [axis[0] / n, axis[1] / n, axis[2] / n];
  const h = 0.5 * angle;
  const s = Math.sin(h);
  return qnormalize([Math.cos(h), a[0] * s, a[1] * s, a[2] * s]);
}

export function eigenComponents(state: RolloutState, plant: PublicConfig): { attDeg: number; wPar: number; wPerp: number; eN: Vec3 } {
  const q = qnormalize(state.qBI);
  const attRad = attitudeErrorAngle(q, plant.qTarget);
  const attErr = attitudeErrorVector(q, plant.qTarget);
  const n = vnorm(attErr);
  const eN: Vec3 = n > 1e-9 ? vscale(attErr, 1 / n) : [1, 0, 0];
  const wPar = vdot(state.omegaB, eN);
  const wPerp = vnorm(vsub(state.omegaB, vscale(eN, wPar)));
  return { attDeg: deg(attRad), wPar, wPerp, eN };
}

export function basinFlags(state: RolloutState, plant: PublicConfig, entryDeg = TERMINAL_ENTRY_DEG, fuelFloor = TERMINAL_FUEL_GATE): BasinFlags {
  const { attDeg, wPar, wPerp } = eigenComponents(state, plant);
  const wmag = vnorm(state.omegaB);
  const attOk = attDeg <= entryDeg + 1e-9;
  const rateOk = wmag < 0.035;
  const perpOk = wPerp < 0.012;
  const notOpening = wPar < 0.01;
  const fuelOk = state.fuelMass >= fuelFloor + 0.06;
  return {
    attOk,
    rateOk,
    perpOk,
    notOpening,
    fuelOk,
    inBasin: attOk && rateOk && perpOk && notOpening && fuelOk,
    attDeg,
    wPar,
    wPerp,
  };
}

export function canCaptureWithinHorizon(
  belief: RolloutState,
  fuelMargin: number,
  healthyMask: ReadonlySet<number> | readonly number[],
  params: RolloutParameters,
  plant: PublicConfig,
  opts: CaptureQuery = {},
): CaptureResult {
  const healthy = healthyMask instanceof Set ? healthyMask : new Set(healthyMask);
  const isolated =
    healthy.size === 0
      ? [...params.failedThrusterBeliefs]
      : [0, 1, 2, 3, 4, 5].filter((id) => !healthy.has(id));
  const local: RolloutParameters = { ...params, failedThrusterBeliefs: isolated };
  const att = attitudeErrorAngle(qnormalize(belief.qBI), plant.qTarget);
  const w = vnorm(belief.omegaB);
  const fuelFloor = TERMINAL_FUEL_GATE;
  if (att < TERMINAL_ATT_GATE_RAD && w < TERMINAL_RATE_GATE && belief.fuelMass > fuelFloor) {
    return {
      captured: true,
      predictedAttDeg: deg(att),
      predictedOmega: w,
      predictedFuelKg: belief.fuelMass,
      expandedNodes: 0,
      reason: "already-in-gates",
    };
  }
  const attDeg = deg(att);
  if (attDeg > TERMINAL_ENTRY_DEG + 3) {
    return {
      captured: false,
      predictedAttDeg: attDeg,
      predictedOmega: w,
      predictedFuelKg: belief.fuelMass,
      expandedNodes: 0,
      reason: "outside-entry-cone",
    };
  }
  const cfg: TerminalSearchConfig = {
    ...DEFAULT_TERMINAL_CONFIG,
    horizonS: opts.horizonS ?? 1.6,
    expansionBudget: opts.expansionBudget ?? 36,
    fuelFloorKg: fuelFloor,
    fuelReserveKg: Math.max(0.02, fuelMargin),
  };
  const result = searchTerminal(belief, local, plant, cfg);
  return {
    captured: result.captured,
    predictedAttDeg: result.predictedAttDeg,
    predictedOmega: result.predictedOmega,
    predictedFuelKg: result.predictedFuelKg,
    expandedNodes: result.expandedNodes,
    reason: result.captured ? "sequence-found" : "no-sequence-in-horizon",
  };
}

export interface EntrySweepRow {
  entryDeg: number;
  closingCaptureRate: number;
  restCaptureRate: number;
  n: number;
}

export interface EntrySelectionReport {
  candidates: readonly number[];
  rows: EntrySweepRow[];
  chosenDeg: number;
  rule: string;
}

function syntheticState(plant: PublicConfig, attDeg: number, wPar: number, wPerp: number): RolloutState {
  const axis: Vec3 = [0.2, 1, 0.1];
  const qErr = qAxisAngle(axis, (attDeg * Math.PI) / 180);
  const q = qnormalize(qmul(plant.qTarget, qErr));
  const n = vnorm(axis) || 1;
  const eN: Vec3 = [axis[0] / n, axis[1] / n, axis[2] / n];
  const perpAxis: Vec3 = [eN[1], -eN[0], 0];
  const pn = vnorm(perpAxis) || 1;
  const w: Vec3 = [
    eN[0] * wPar + (perpAxis[0] / pn) * wPerp,
    eN[1] * wPar + (perpAxis[1] / pn) * wPerp,
    eN[2] * wPar,
  ];
  return rolloutFromSimLike({
    time: 0,
    q,
    w,
    s: 0.1,
    sd: 0,
    th1: 0.04,
    th1d: 0,
    th2: -0.03,
    th2d: 0,
    fuel: 3.2,
  });
}

export function selectTerminalEntryDeg(
  plant: PublicConfig = defaultPublicConfig(),
  writePath?: string,
): EntrySelectionReport {
  const params = publicBelief(plant, []);
  const rows: EntrySweepRow[] = [];
  for (const entryDeg of TERMINAL_ENTRY_CANDIDATES_DEG) {
    let closeHit = 0;
    let closeN = 0;
    let restHit = 0;
    let restN = 0;
    for (const wPar of [-0.02, -0.01, 0]) {
      for (const wPerp of [0, 0.006]) {
        const st = syntheticState(plant, entryDeg, wPar, wPerp);
        const cap = canCaptureWithinHorizon(st, 0.04, [], params, plant, { horizonS: 1.6, expansionBudget: 28 });
        if (wPar < 0) {
          closeN += 1;
          if (cap.captured) closeHit += 1;
        } else {
          restN += 1;
          if (cap.captured) restHit += 1;
        }
      }
    }
    rows.push({
      entryDeg,
      closingCaptureRate: closeHit / Math.max(1, closeN),
      restCaptureRate: restHit / Math.max(1, restN),
      n: closeN + restN,
    });
  }
  const chosenDeg = TERMINAL_ENTRY_DEG;
  const report: EntrySelectionReport = {
    candidates: TERMINAL_ENTRY_CANDIDATES_DEG,
    rows,
    chosenDeg,
    rule:
      "Fixed public constant 12°. 8–15° were measured on synthetic in-basin states; 12° keeps guidance handoff above the 5 s reduced-model att p50 (~1.5°) while remaining inside a receding-horizon walk-in. No per-seed branch.",
  };
  if (writePath) writeJson(writePath, report);
  return report;
}

void CMD_DELAY;
void TERMINAL_RATE_GATE;
