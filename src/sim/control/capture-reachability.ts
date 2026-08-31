/**
 * Offline high-fidelity capture-reachability study.
 *
 * Not an online planner. Does not change beam width, terminalEntryDeg,
 * physics, or the demo seed. Frozen-kernel rk4, legal coast/single/pair
 * primitives, 40–320 ms pulses, 120 ms delay, max-2 jets, fuel floor 2.8 kg.
 *
 * Capture is the conjunction att < 1° AND |ω| < 0.008 AND fuel > 2.8 kg.
 * Passing the attitude ball alone is recorded separately and is not capture.
 */
import { CMD_DELAY, MAX_ACTIVE, MIN_PULSE, THRUSTERS, defaultPublicConfig } from "../constants";
import { massState } from "../dynamics";
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
  rolloutFromSimLike,
  type RolloutConfig,
  type RolloutParameters,
  type RolloutState,
} from "./rollout-model";
import { publicBelief } from "./rollout-error";
import { TERMINAL_ATT_GATE_RAD, TERMINAL_FUEL_GATE, TERMINAL_RATE_GATE } from "./terminal-planner";

export const STUDY_BUCKETS_DEG = [15, 10, 5, 2.5, 1.5] as const;
export const STUDY_HORIZONS_S = [5, 10, 20, 30] as const;
export const STUDY_METHODS = ["eigen", "beam", "cem"] as const;
export type StudyMethod = (typeof STUDY_METHODS)[number];

export const OFFLINE_ROLLOUT: RolloutConfig = {
  dt: 0.01,
  commandDelayS: CMD_DELAY,
  maxActive: MAX_ACTIVE,
  useCollision: false,
  fast: false,
};

export const OFFLINE_PULSE_DURATIONS = [0.04, 0.08, 0.12, 0.16, 0.24, 0.32] as const satisfies readonly PulseDurationS[];

export interface CaptureGates {
  attOk: boolean;
  rateOk: boolean;
  fuelOk: boolean;
  captured: boolean;
  attDeg: number;
  omega: number;
  fuelKg: number;
}

export function capturedGates(state: RolloutState, qTarget: Quat, fuelFloor = TERMINAL_FUEL_GATE): CaptureGates {
  const attDeg = deg(attitudeErrorAngle(qnormalize(state.qBI), qTarget));
  const omega = vnorm(state.omegaB);
  const fuelKg = state.fuelMass;
  const attOk = attDeg < 1 + 1e-12;
  const rateOk = omega < TERMINAL_RATE_GATE;
  const fuelOk = fuelKg > fuelFloor;
  return { attOk, rateOk, fuelOk, captured: attOk && rateOk && fuelOk, attDeg, omega, fuelKg };
}

function qAxisAngle(axis: Vec3, angle: number): Quat {
  const n = vnorm(axis) || 1;
  const a: Vec3 = [axis[0] / n, axis[1] / n, axis[2] / n];
  const h = 0.5 * angle;
  const s = Math.sin(h);
  return qnormalize([Math.cos(h), a[0] * s, a[1] * s, a[2] * s]);
}

/** Replace attitude error magnitude, keep the current eigenaxis, rates, slosh, fuel. */
export function scaleAttitude(state: RolloutState, targetDeg: number, qTarget: Quat): RolloutState {
  const q = qnormalize(state.qBI);
  const err = attitudeErrorVector(q, qTarget);
  const n = vnorm(err);
  const axis: Vec3 = n > 1e-9 ? vscale(err, 1 / n) : [0.2, 1, 0.1];
  const qErr = qAxisAngle(axis, (targetDeg * Math.PI) / 180);
  const next = cloneRolloutState(state);
  next.qBI = qnormalize(qmul(qTarget, qErr));
  return next;
}

export interface HarvestedState {
  id: string;
  bucketDeg: number;
  rateMode: "closing" | "rest";
  fault: "healthy" | "plusY-isolated";
  isolated: readonly number[];
  state: RolloutState;
}

export function harvestStudyStates(plant: PublicConfig = defaultPublicConfig()): HarvestedState[] {
  const out: HarvestedState[] = [];
  let k = 0;
  for (const bucket of STUDY_BUCKETS_DEG) {
    for (const rateMode of ["closing", "rest"] as const) {
      for (const fault of ["healthy", "plusY-isolated"] as const) {
        const axis: Vec3 = k % 2 === 0 ? [0.25, 1, 0.15] : [0.9, 0.2, 0.35];
        const qErr = qAxisAngle(axis, (bucket * Math.PI) / 180);
        const q = qnormalize(qmul(plant.qTarget, qErr));
        const n = vnorm(axis) || 1;
        const eN: Vec3 = [axis[0] / n, axis[1] / n, axis[2] / n];
        const closing = rateMode === "closing" ? -0.012 : 0;
        const w: Vec3 = [eN[0] * closing, eN[1] * closing, eN[2] * closing];
        const isolated = fault === "plusY-isolated" ? [2] : [];
        const state = rolloutFromSimLike({
          time: 0,
          q,
          w,
          s: 0.12,
          sd: 0,
          th1: 0.05,
          th1d: 0,
          th2: -0.04,
          th2d: 0,
          fuel: 3.4,
        });
        out.push({
          id: `b${bucket}-${rateMode}-${fault}`,
          bucketDeg: bucket,
          rateMode,
          fault,
          isolated,
          state,
        });
        k += 1;
      }
    }
  }
  return out;
}

function executableAt(
  state: RolloutState,
  params: RolloutParameters,
  plant: PublicConfig,
  fuelFloor: number,
): PulsePrimitive[] {
  const isolated = new Set(params.failedThrusterBeliefs);
  const generated = generatePulsePrimitives(THRUSTERS, {
    isolatedThrusters: isolated,
    durationsS: OFFLINE_PULSE_DURATIONS,
  });
  return filterExecutablePrimitives(generated, {
    nowS: state.time,
    commandDelayS: plant.commandDelay,
    pendingPulses: state.pendingPulses,
    isolatedThrusters: isolated,
    estimatedFuelKg: state.fuelMass,
    fuelFloorKg: fuelFloor,
    reserveKg: 0.02,
    maxActive: plant.maxActiveThrusters,
  });
}

function rankByEigen(state: RolloutState, prims: readonly PulsePrimitive[], params: RolloutParameters, plant: PublicConfig): PulsePrimitive[] {
  const e = attitudeErrorVector(qnormalize(state.qBI), plant.qTarget);
  const att = attitudeErrorAngle(qnormalize(state.qBI), plant.qTarget);
  const kp = att > 0.08 ? 0.25 : att > 0.03 ? 0.45 : 0.7;
  const wDes = vscale(e, -kp);
  const need = vsub(wDes, state.omegaB);
  const ms = massState(plant, state.sliderS, state.theta1, state.theta2, state.fuelMass);
  const scored = prims.map((p) => {
    const w = netWrenchForPrimitive(p, THRUSTERS, params.etaTEstimate, ms.rCmB, plant.maxThrust);
    const proj = vdot(w.torqueB, need);
    const pairPen = p.thrusterIds.length === 2 ? 0.02 : 0;
    const longPen = att < 0.04 && p.durationS > 0.12 ? 0.4 : 0;
    return { p, score: proj - pairPen - longPen };
  });
  scored.sort((a, b) => b.score - a.score || (a.p.id < b.p.id ? -1 : 1));
  return scored.map((s) => s.p);
}

export interface SequenceReport {
  stateId: string;
  bucketDeg: number;
  horizonS: number;
  method: StudyMethod;
  captured: boolean;
  passedAttBall: boolean;
  captureTimeS: number | null;
  attBallTimeS: number | null;
  minAttDeg: number;
  finalAttDeg: number;
  finalOmega: number;
  finalFuelKg: number;
  fuelUsedKg: number;
  usedPair: boolean;
  attIncreased: boolean;
  nPulses: number;
  expanded: number;
  elapsedMs: number;
  firstPrimitiveId: string | null;
  note: string;
}

interface WalkOpts {
  horizonS: number;
  method: StudyMethod;
  expansionBudget: number;
  fuelFloor: number;
}

function hashU32(n: number): number {
  let x = n >>> 0;
  x = Math.imul(x ^ (x >>> 16), 0x7feb352d);
  x = Math.imul(x ^ (x >>> 15), 0x846ca68b);
  return (x ^ (x >>> 16)) >>> 0;
}

function choosePrimitive(
  state: RolloutState,
  params: RolloutParameters,
  plant: PublicConfig,
  opts: WalkOpts,
  stepIndex: number,
): { prim: PulsePrimitive; expanded: number } {
  const cands = executableAt(state, params, plant, opts.fuelFloor);
  const coasts = cands.filter((p) => p.thrusterIds.length === 0);
  const coast = coasts.sort((a, b) => a.durationS - b.durationS)[0] ?? cands[0]!;
  const att = attitudeErrorAngle(qnormalize(state.qBI), plant.qTarget);
  const wmag = vnorm(state.omegaB);

  if (opts.method === "eigen") {
    const ranked = rankByEigen(state, cands, params, plant);
    const shortlist: PulsePrimitive[] = [];
    const take = (p: PulsePrimitive | undefined) => {
      if (p && !shortlist.some((x) => x.id === p.id)) shortlist.push(p);
    };
    take(coast);
    for (const p of ranked) {
      if (shortlist.length >= 10) break;
      take(p);
    }
    if (att < 0.035 && wmag < 0.018) take(coast);
    let best = coast;
    let bestKey = walkScore(applyPrimitiveUntilComplete(state, params, plant, coast, OFFLINE_ROLLOUT), plant, opts.fuelFloor);
    let expanded = 1;
    for (const p of shortlist) {
      if (p.id === coast.id) continue;
      expanded += 1;
      const nxt = applyPrimitiveUntilComplete(state, params, plant, p, OFFLINE_ROLLOUT);
      const key = walkScore(nxt, plant, opts.fuelFloor);
      if (cmpScore(key, bestKey) < 0) {
        best = p;
        bestKey = key;
      }
    }
    return { prim: best, expanded };
  }

  if (opts.method === "cem") {
    const fires = cands.filter((p) => p.thrusterIds.length > 0);
    const pool = fires.length > 0 ? fires : cands;
    const rng = hashU32(stepIndex * 2654435761 + Math.round(state.time * 1e4) + opts.horizonS * 17);
    const pick = pool[rng % pool.length]!;
    const look = [coast, pick];
    const extra = rankByEigen(state, pool, params, plant).slice(0, 4);
    for (const p of extra) if (!look.some((x) => x.id === p.id)) look.push(p);
    let best = coast;
    let bestKey = walkScore(applyPrimitiveUntilComplete(state, params, plant, coast, OFFLINE_ROLLOUT), plant, opts.fuelFloor);
    let expanded = 1;
    for (const p of look) {
      if (p.id === coast.id) continue;
      expanded += 1;
      const nxt = applyPrimitiveUntilComplete(state, params, plant, p, OFFLINE_ROLLOUT);
      const key = walkScore(nxt, plant, opts.fuelFloor);
      if (cmpScore(key, bestKey) < 0) {
        best = p;
        bestKey = key;
      }
    }
    return { prim: best, expanded };
  }

  // beam: one-step lookahead over a width-capped shortlist. Does not touch
  // DEFAULT_BEAM_CONFIG.beamWidth of the online planner.
  const ranked = rankByEigen(state, cands, params, plant);
  const width = 8;
  const look: PulsePrimitive[] = [];
  const take = (p: PulsePrimitive | undefined) => {
    if (p && !look.some((x) => x.id === p.id)) look.push(p);
  };
  take(coast);
  for (const p of ranked) {
    if (look.length >= width) break;
    take(p);
  }
  let best = coast;
  let bestKey = walkScore(applyPrimitiveUntilComplete(state, params, plant, coast, OFFLINE_ROLLOUT), plant, opts.fuelFloor);
  let expanded = 1;
  for (const p of look) {
    if (p.id === coast.id) continue;
    if (expanded >= opts.expansionBudget) break;
    expanded += 1;
    const nxt = applyPrimitiveUntilComplete(state, params, plant, p, OFFLINE_ROLLOUT);
    const key = walkScore(nxt, plant, opts.fuelFloor);
    if (cmpScore(key, bestKey) < 0) {
      best = p;
      bestKey = key;
    }
  }
  return { prim: best, expanded };
}

interface WalkScore {
  fuelFail: number;
  captured: number;
  attFail: number;
  rateFail: number;
  att: number;
  omega: number;
  fuel: number;
}

function walkScore(s: RolloutState, plant: PublicConfig, fuelFloor: number): WalkScore {
  const g = capturedGates(s, plant.qTarget, fuelFloor);
  return {
    fuelFail: g.fuelOk ? 0 : 1,
    captured: g.captured ? 0 : 1,
    attFail: g.attOk ? 0 : 1,
    rateFail: g.attOk && !g.rateOk ? 1 : 0,
    att: g.attDeg,
    omega: g.omega,
    fuel: -g.fuelKg,
  };
}

function cmpScore(a: WalkScore, b: WalkScore): number {
  if (a.fuelFail !== b.fuelFail) return a.fuelFail - b.fuelFail;
  if (a.captured !== b.captured) return a.captured - b.captured;
  if (a.attFail !== b.attFail) return a.attFail - b.attFail;
  if (a.rateFail !== b.rateFail) return a.rateFail - b.rateFail;
  if (Math.abs(a.att - b.att) > 1e-9) return a.att - b.att;
  if (Math.abs(a.omega - b.omega) > 1e-9) return a.omega - b.omega;
  return a.fuel - b.fuel;
}

export function searchState(
  harvested: HarvestedState,
  plant: PublicConfig,
  method: StudyMethod,
  horizonS: number,
  expansionBudget = 48,
): SequenceReport {
  const t0 = Date.now();
  const params = publicBelief(plant, harvested.isolated);
  const fuelFloor = TERMINAL_FUEL_GATE;
  let state = cloneRolloutState(harvested.state);
  const tStart = state.time;
  const tEnd = tStart + horizonS;
  let expanded = 0;
  let nPulses = 0;
  let usedPair = false;
  let firstPrimitiveId: string | null = null;
  let passedAttBall = false;
  let attBallTimeS: number | null = null;
  let captureTimeS: number | null = null;
  let captured = false;
  let minAttDeg = capturedGates(state, plant.qTarget, fuelFloor).attDeg;
  let maxAttDeg = minAttDeg;
  const startAtt = minAttDeg;
  const startFuel = state.fuelMass;
  const opts: WalkOpts = { horizonS, method, expansionBudget, fuelFloor };

  const noteGate = (s: RolloutState) => {
    const g = capturedGates(s, plant.qTarget, fuelFloor);
    if (g.attDeg < minAttDeg) minAttDeg = g.attDeg;
    if (g.attDeg > maxAttDeg) maxAttDeg = g.attDeg;
    if (g.attOk && !passedAttBall) {
      passedAttBall = true;
      attBallTimeS = s.time - tStart;
    }
    if (g.captured && !captured) {
      captured = true;
      captureTimeS = s.time - tStart;
    }
  };
  noteGate(state);

  while (state.time + 1e-9 < tEnd && !captured && expanded < expansionBudget * 40) {
    const { prim, expanded: e } = choosePrimitive(state, params, plant, opts, nPulses);
    expanded += e;
    if (firstPrimitiveId == null) firstPrimitiveId = prim.id;
    if (prim.thrusterIds.length > 0) nPulses += 1;
    if (prim.thrusterIds.length === 2) usedPair = true;
    const remaining = tEnd - state.time;
    if (prim.durationS + OFFLINE_ROLLOUT.commandDelayS > remaining + 1e-9 && prim.thrusterIds.length > 0) {
      // Coast out the remainder rather than overshooting the study horizon.
      const coasts = executableAt(state, params, plant, fuelFloor).filter((p) => p.thrusterIds.length === 0);
      const c = coasts.sort((a, b) => a.durationS - b.durationS)[0];
      if (c) {
        state = applyPrimitiveUntilComplete(state, params, plant, c, OFFLINE_ROLLOUT);
        noteGate(state);
        if (state.time >= tEnd - 1e-9) break;
        continue;
      }
    }
    state = applyPrimitiveUntilComplete(state, params, plant, prim, OFFLINE_ROLLOUT);
    noteGate(state);
  }

  const g = capturedGates(state, plant.qTarget, fuelFloor);
  // Capture only from committed trajectory (noteGate), not peek candidates.
  return {
    stateId: harvested.id,
    bucketDeg: harvested.bucketDeg,
    horizonS,
    method,
    captured,
    passedAttBall,
    captureTimeS,
    attBallTimeS,
    minAttDeg,
    finalAttDeg: g.attDeg,
    finalOmega: g.omega,
    finalFuelKg: g.fuelKg,
    fuelUsedKg: Math.max(0, startFuel - g.fuelKg),
    usedPair,
    attIncreased: maxAttDeg > startAtt + 0.05,
    nPulses,
    expanded,
    elapsedMs: Date.now() - t0,
    firstPrimitiveId,
    note: captured
      ? "conjunctive-capture"
      : passedAttBall
        ? "att-ball-only"
        : "no-att-ball",
  };
}

export interface StudyOptions {
  quick?: boolean;
  horizonsS?: readonly number[];
  methods?: readonly StudyMethod[];
  expansionBudget?: number;
}

export interface StudyReport {
  physicsBaselineSha: string;
  controllerVersion: string;
  notes: string;
  constraints: {
    minPulseS: number;
    commandDelayS: number;
    maxActive: number;
    fuelFloorKg: number;
    attGateDeg: number;
    rateGate: number;
    pulseDurationsS: readonly number[];
  };
  bucketsDeg: readonly number[];
  horizonsS: readonly number[];
  methods: readonly StudyMethod[];
  nStates: number;
  states: Array<{
    id: string;
    bucketDeg: number;
    rateMode: string;
    fault: string;
    isolated: readonly number[];
    startAttDeg: number;
    startOmega: number;
    startFuelKg: number;
  }>;
  rows: SequenceReport[];
  summary: {
    capturedConjunctive: string;
    attBall: string;
    byBucket: Array<{
      bucketDeg: number;
      captured: string;
      attBall: string;
      bestFinalAttDeg: number;
      bestHorizonS: number | null;
    }>;
    dualThrusterUsedOnBest: boolean;
    anyNonMonotonic: boolean;
    minFuelUsedOnAttBall: number | null;
    minTimeToAttBallS: number | null;
    answers: {
      canEnterOneDegWithFuel: string;
      minTime: string;
      minFuel: string;
      needsTemporaryAttIncrease: string;
      mustUsePair: string;
      needsLongerThan1p6s: string;
      geometryReview: string;
    };
  };
  verdict: {
    physics: "PASS";
    control: "FAIL" | "PASS";
    overall: "FAIL" | "PASS";
    pr: string;
  };
}

export function runCaptureStudy(plant: PublicConfig = defaultPublicConfig(), opts: StudyOptions = {}): StudyReport {
  const quick = Boolean(opts.quick);
  const all = harvestStudyStates(plant);
  const states = quick
    ? STUDY_BUCKETS_DEG.map((b) => all.find((s) => s.bucketDeg === b && s.rateMode === "closing" && s.fault === "healthy")!).filter(Boolean)
    : all;
  const horizons = opts.horizonsS ?? (quick ? [5, 10] : STUDY_HORIZONS_S);
  const methods = opts.methods ?? (quick ? (["eigen", "beam"] as const) : STUDY_METHODS);
  const budget = opts.expansionBudget ?? (quick ? 24 : 48);
  const rows: SequenceReport[] = [];
  for (const st of states) {
    for (const h of horizons) {
      for (const m of methods) {
        rows.push(searchState(st, plant, m, h, budget));
      }
    }
  }

  const bestByState = new Map<string, SequenceReport>();
  for (const r of rows) {
    const prev = bestByState.get(r.stateId);
    if (!prev || cmpReport(r, prev) < 0) bestByState.set(r.stateId, r);
  }
  const bests = [...bestByState.values()];
  const nCap = bests.filter((r) => r.captured).length;
  const nBall = bests.filter((r) => r.passedAttBall).length;
  const byBucket = STUDY_BUCKETS_DEG.map((bucketDeg) => {
    const group = bests.filter((r) => r.bucketDeg === bucketDeg);
    const cap = group.filter((r) => r.captured).length;
    const ball = group.filter((r) => r.passedAttBall).length;
    let bestFinal = Infinity;
    let bestH: number | null = null;
    for (const r of group) {
      if (r.minAttDeg < bestFinal) {
        bestFinal = r.minAttDeg;
        bestH = r.horizonS;
      }
    }
    return {
      bucketDeg,
      captured: `${cap}/${group.length}`,
      attBall: `${ball}/${group.length}`,
      bestFinalAttDeg: Number.isFinite(bestFinal) ? bestFinal : NaN,
      bestHorizonS: bestH,
    };
  });

  const attBallRows = rows.filter((r) => r.passedAttBall);
  const minFuel = attBallRows.length ? Math.min(...attBallRows.map((r) => r.fuelUsedKg)) : null;
  const minTime = attBallRows.length
    ? Math.min(...attBallRows.map((r) => r.attBallTimeS ?? Infinity))
    : null;
  const fiveDeg = bests.filter((r) => r.bucketDeg <= 5 + 1e-9);
  const fiveCap = fiveDeg.filter((r) => r.captured).length;
  const dual = bests.some((r) => r.captured && r.usedPair);
  const nonMono = bests.some((r) => r.attIncreased && (r.captured || r.passedAttBall));
  const longNeeded = attBallRows.some((r) => (r.attBallTimeS ?? 0) > 1.6);

  const canEnter = nCap === bests.length ? "yes" : nCap > 0 ? `partial ${nCap}/${bests.length}` : "no";
  const geometry =
    fiveCap === 0
      ? "offline search did not conjunctively capture any ≤5° state — review thruster geometry and the 1°/rate/fuel conjunction before more planner patches"
      : "≤5° conjunctive capture observed offline; remaining gap is the online planner";

  return {
    physicsBaselineSha: "bdfff5b4c62733d7156d2f9cdeeaa75661d6c9f4",
    controllerVersion: "discrete-pulse-v2-offline-study",
    notes:
      "Offline frozen-kernel sequence search. Online hierarchical planner was not patched. Beam width of DEFAULT_BEAM_CONFIG is unchanged. Train-10 and hidden were not run.",
    constraints: {
      minPulseS: MIN_PULSE,
      commandDelayS: CMD_DELAY,
      maxActive: MAX_ACTIVE,
      fuelFloorKg: TERMINAL_FUEL_GATE,
      attGateDeg: 1,
      rateGate: TERMINAL_RATE_GATE,
      pulseDurationsS: OFFLINE_PULSE_DURATIONS,
    },
    bucketsDeg: STUDY_BUCKETS_DEG,
    horizonsS: horizons,
    methods,
    nStates: states.length,
    states: states.map((s) => {
      const g = capturedGates(s.state, plant.qTarget);
      return {
        id: s.id,
        bucketDeg: s.bucketDeg,
        rateMode: s.rateMode,
        fault: s.fault,
        isolated: s.isolated,
        startAttDeg: g.attDeg,
        startOmega: g.omega,
        startFuelKg: g.fuelKg,
      };
    }),
    rows,
    summary: {
      capturedConjunctive: `${nCap}/${bests.length}`,
      attBall: `${nBall}/${bests.length}`,
      byBucket,
      dualThrusterUsedOnBest: dual,
      anyNonMonotonic: nonMono,
      minFuelUsedOnAttBall: minFuel,
      minTimeToAttBallS: Number.isFinite(minTime ?? NaN) ? minTime : null,
      answers: {
        canEnterOneDegWithFuel: canEnter,
        minTime: minTime != null && Number.isFinite(minTime) ? `${minTime.toFixed(2)} s to att-ball (conjunctive capture may be later or never)` : "no att-ball",
        minFuel: minFuel != null ? `${minFuel.toFixed(3)} kg to att-ball` : "n/a",
        needsTemporaryAttIncrease: nonMono ? "yes on at least one successful att-ball/capture trajectory" : "not observed on recorded bests",
        mustUsePair: dual ? "best conjunctive captures used a pair" : "no conjunctive capture required a pair in this study (or none captured)",
        needsLongerThan1p6s: longNeeded ? "yes — att-ball time exceeded 1.6 s" : "att-ball, if any, was inside 1.6 s",
        geometryReview: geometry,
      },
    },
    verdict: {
      physics: "PASS",
      control: nCap === bests.length && bests.length > 0 ? "PASS" : "FAIL",
      overall: nCap === bests.length && bests.length > 0 ? "PASS" : "FAIL",
      pr: "DRAFT / DO NOT MERGE",
    },
  };
}

function cmpReport(a: SequenceReport, b: SequenceReport): number {
  if (Number(a.captured) !== Number(b.captured)) return Number(b.captured) - Number(a.captured);
  if (Number(a.passedAttBall) !== Number(b.passedAttBall)) return Number(b.passedAttBall) - Number(a.passedAttBall);
  if (Math.abs(a.minAttDeg - b.minAttDeg) > 1e-9) return a.minAttDeg - b.minAttDeg;
  if (Math.abs(a.finalOmega - b.finalOmega) > 1e-9) return a.finalOmega - b.finalOmega;
  return a.fuelUsedKg - b.fuelUsedKg;
}

void TERMINAL_ATT_GATE_RAD;
