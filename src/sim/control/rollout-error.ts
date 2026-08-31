/**
 * Reduced-versus-frozen-kernel rollout error envelope.
 *
 * Compares DEFAULT_ROLLOUT_CONFIG (fast rigid step, 40 ms) against
 * PARITY_ROLLOUT_CONFIG (frozen rk4 + collision, 5 ms) on a public
 * synthetic state set. Same belief parameters on both sides — this is
 * model-reduction error, not estimator error.
 *
 * The envelope is a diagnostic. It does not read Simulator, truth, or
 * private scenario fields.
 */
import { CMD_DELAY, Q0, THRUSTERS, defaultPublicConfig } from "../constants";
import {
  attitudeErrorAngle,
  deg,
  qmul,
  qnormalize,
  vnorm,
  vsub,
  type Quat,
  type Vec3,
} from "../math3d";
import type { PublicConfig } from "../types";
import {
  enqueuePrimitive,
  generatePulsePrimitives,
  type PendingPulse,
  type PulsePrimitive,
} from "./discrete-actions";
import {
  DEFAULT_ROLLOUT_CONFIG,
  PARITY_ROLLOUT_CONFIG,
  cloneRolloutState,
  geodesicAttitudeError,
  predictedSloshEnergy,
  rolloutAdvance,
  rolloutFromSimLike,
  type RolloutConfig,
  type RolloutParameters,
  type RolloutState,
} from "./rollout-model";

export const ENVELOPE_HORIZONS_S = [0.5, 1, 2, 3, 5, 8, 10] as const;
export type EnvelopeHorizonS = (typeof ENVELOPE_HORIZONS_S)[number];
export type ActionClass = "coast" | "single" | "pair" | "mixed";
export type RateRegime = "high-rate" | "medium-rate" | "terminal";
export type FaultState = "healthy" | "one-isolated";
export type PendingQueueState = "empty" | "non-empty";

export type ControlStage = "terminal" | "guidance" | "long";

export interface StageTolerance {
  readonly stage: ControlStage;
  readonly attRad: number;
  readonly omega: number;
  readonly fuel: number;
  readonly slosh: number;
}

/** Allowed reduced-vs-kernel error by the stage that would use that horizon. */
export const STAGE_TOLERANCE: Record<ControlStage, StageTolerance> = {
  terminal: { stage: "terminal", attRad: 0.005, omega: 0.002, fuel: 0.004, slosh: 0.04 },
  guidance: { stage: "guidance", attRad: 0.035, omega: 0.015, fuel: 0.015, slosh: 0.15 },
  long: { stage: "long", attRad: 0.08, omega: 0.03, fuel: 0.04, slosh: 0.3 },
};

export function stageForHorizon(horizonS: number): ControlStage {
  if (horizonS <= 2 + 1e-9) return "terminal";
  if (horizonS <= 5 + 1e-9) return "guidance";
  return "long";
}

export function toleranceFor(horizonS: number, regime: RateRegime): StageTolerance {
  if (regime === "terminal") {
    return horizonS <= 2 + 1e-9 ? STAGE_TOLERANCE.terminal : STAGE_TOLERANCE.guidance;
  }
  if (regime === "high-rate") return STAGE_TOLERANCE.guidance;
  return stageForHorizon(horizonS) === "long" ? STAGE_TOLERANCE.long : STAGE_TOLERANCE.guidance;
}

export interface EnvelopeStateSpec {
  readonly id: string;
  readonly regime: RateRegime;
  readonly fault: FaultState;
  readonly pending: PendingQueueState;
  readonly isolatedIds: readonly number[];
}

export interface RolloutErrorSample {
  readonly attRad: number;
  readonly attDeg: number;
  readonly omegaErr: number;
  readonly fuelErr: number;
  readonly sloshErr: number;
  readonly sloshEnergyErr: number;
}

export interface EnvelopeCell {
  readonly stateId: string;
  readonly regime: RateRegime;
  readonly actionClass: ActionClass;
  readonly fault: FaultState;
  readonly pending: PendingQueueState;
  readonly horizonS: number;
  readonly stage: ControlStage;
  readonly tolerance: StageTolerance;
  readonly error: RolloutErrorSample;
  readonly withinTolerance: boolean;
}

export interface ErrorQuantiles {
  readonly p50: number;
  readonly p90: number;
  readonly worst: number;
  readonly n: number;
}

export interface EnvelopeGroupSummary {
  readonly key: string;
  readonly attRad: ErrorQuantiles;
  readonly omegaErr: ErrorQuantiles;
  readonly fuelErr: ErrorQuantiles;
  readonly sloshErr: ErrorQuantiles;
  readonly withinToleranceRate: number;
  readonly usableForStage: boolean;
}

export interface EnvelopeReport {
  readonly physicsBaselineSha: string;
  readonly controllerVersion: string;
  readonly notes: string;
  readonly horizonsS: readonly number[];
  readonly actionClasses: readonly ActionClass[];
  readonly regimes: readonly RateRegime[];
  readonly faultStates: readonly FaultState[];
  readonly pendingQueues: readonly PendingQueueState[];
  readonly tolerances: typeof STAGE_TOLERANCE;
  readonly cells: EnvelopeCell[];
  readonly byHorizon: EnvelopeGroupSummary[];
  readonly byActionClass: EnvelopeGroupSummary[];
  readonly byRegime: EnvelopeGroupSummary[];
  readonly verdict: {
    readonly terminalHorizonUsable: boolean;
    readonly guidanceHorizonUsable: boolean;
    readonly longHorizonUsableForTerminal: boolean;
    readonly fiveSecondAttP50Rad: number;
    readonly fiveSecondAttP50Deg: number;
  };
}

function qAxisAngle(axis: Vec3, angle: number): Quat {
  const n = vnorm(axis);
  const a: Vec3 = n > 1e-12 ? [axis[0] / n, axis[1] / n, axis[2] / n] : [1, 0, 0];
  const h = 0.5 * angle;
  const s = Math.sin(h);
  return qnormalize([Math.cos(h), a[0] * s, a[1] * s, a[2] * s]);
}

function percentile(sorted: number[], p: number): number {
  if (sorted.length === 0) return NaN;
  const i = Math.min(sorted.length - 1, Math.max(0, Math.ceil(p * sorted.length) - 1));
  return sorted[i]!;
}

function quantiles(values: number[]): ErrorQuantiles {
  const s = [...values].sort((a, b) => a - b);
  return {
    p50: percentile(s, 0.5),
    p90: percentile(s, 0.9),
    worst: s[s.length - 1] ?? NaN,
    n: s.length,
  };
}

export function rolloutStateError(
  a: RolloutState,
  b: RolloutState,
  plant: PublicConfig,
  k12: number,
): RolloutErrorSample {
  const attRad = geodesicAttitudeError(a.qBI, b.qBI);
  const omegaErr = vnorm(vsub(a.omegaB, b.omegaB));
  const fuelErr = Math.abs(a.fuelMass - b.fuelMass);
  const sloshErr = Math.hypot(
    a.theta1 - b.theta1,
    a.theta1Dot - b.theta1Dot,
    a.theta2 - b.theta2,
    a.theta2Dot - b.theta2Dot,
  );
  const sloshEnergyErr = Math.abs(
    predictedSloshEnergy(a, plant, k12) - predictedSloshEnergy(b, plant, k12),
  );
  return { attRad, attDeg: deg(attRad), omegaErr, fuelErr, sloshErr, sloshEnergyErr };
}

export function withinTolerance(err: RolloutErrorSample, tol: StageTolerance): boolean {
  return (
    err.attRad <= tol.attRad &&
    err.omegaErr <= tol.omega &&
    err.fuelErr <= tol.fuel &&
    err.sloshErr <= tol.slosh
  );
}

export function publicBelief(plant: PublicConfig, isolated: readonly number[] = []): RolloutParameters {
  return {
    inertiaEstimate: [
      [620, 0, 0],
      [0, 710, 0],
      [0, 0, 540],
    ],
    etaTEstimate: 0.873,
    c1Estimate: 0.137,
    c2Estimate: 0.091,
    k12Estimate: 0.318,
    failedThrusterBeliefs: isolated,
  };
}

export function makePublicState(spec: EnvelopeStateSpec, plant: PublicConfig): RolloutState {
  const attRad =
    spec.regime === "high-rate" ? (40 * Math.PI) / 180 : spec.regime === "medium-rate" ? (12 * Math.PI) / 180 : (2.5 * Math.PI) / 180;
  const wmag = spec.regime === "high-rate" ? 0.14 : spec.regime === "medium-rate" ? 0.04 : 0.01;
  const axis: Vec3 =
    spec.regime === "high-rate" ? [1, 0.4, 0.2] : spec.regime === "medium-rate" ? [0.2, 1, -0.3] : [0.1, 0.2, 1];
  const qErr = qAxisAngle(axis, attRad);
  const q = qnormalize(qmul(plant.qTarget, qErr));
  const n = vnorm(axis) || 1;
  const w: Vec3 = [(axis[0] / n) * wmag, (axis[1] / n) * wmag, (axis[2] / n) * wmag];
  const pending: PendingPulse[] =
    spec.pending === "non-empty"
      ? [{ id: spec.isolatedIds.includes(0) ? 1 : 0, tOn: 0.05, tOff: 0.21 }]
      : [];
  const fuel = spec.regime === "terminal" ? 3.15 : 4.2;
  return rolloutFromSimLike({
    time: 0,
    q,
    w,
    s: 0.28,
    sd: 0.02,
    th1: 0.08,
    th1d: 0.01,
    th2: -0.05,
    th2d: -0.02,
    fuel,
    pendingPulses: pending,
  });
}

export function publicStateSpecs(): EnvelopeStateSpec[] {
  const specs: EnvelopeStateSpec[] = [];
  const regimes: RateRegime[] = ["high-rate", "medium-rate", "terminal"];
  const faults: FaultState[] = ["healthy", "one-isolated"];
  const pendings: PendingQueueState[] = ["empty", "non-empty"];
  for (const regime of regimes) {
    for (const fault of faults) {
      for (const pending of pendings) {
        const isolatedIds = fault === "one-isolated" ? [4] : [];
        specs.push({
          id: `${regime}|${fault}|${pending}`,
          regime,
          fault,
          pending,
          isolatedIds,
        });
      }
    }
  }
  return specs;
}

function pickPrimitive(
  all: readonly PulsePrimitive[],
  ids: readonly number[],
  durationS: number,
): PulsePrimitive {
  const want = [...ids].sort((a, b) => a - b);
  const found = all.find(
    (p) =>
      p.durationS === durationS &&
      p.thrusterIds.length === want.length &&
      want.every((id, i) => p.thrusterIds[i] === id),
  );
  if (!found) {
    return all.find((p) => p.thrusterIds.length === 0)!;
  }
  return found;
}

export function primitivesForActionClass(
  action: ActionClass,
  isolated: readonly number[],
): PulsePrimitive[] {
  const failed = new Set(isolated);
  const all = generatePulsePrimitives(THRUSTERS, {
    isolatedThrusters: failed,
    durationsS: [0.08, 0.16, 0.32],
  });
  const healthy = [0, 1, 2, 3, 4, 5].filter((id) => !failed.has(id));
  const a = healthy[0] ?? 0;
  const b = healthy[1] ?? a;
  const c = healthy[2] ?? b;
  switch (action) {
    case "coast":
      return [pickPrimitive(all, [], 0.32)];
    case "single":
      return [pickPrimitive(all, [a], 0.16)];
    case "pair":
      return [pickPrimitive(all, a === b ? [a] : [a, b], 0.16)];
    case "mixed":
      return [
        pickPrimitive(all, [a], 0.08),
        pickPrimitive(all, a === c ? [a] : [b, c], 0.16),
        pickPrimitive(all, [], 0.32),
      ];
  }
}

/**
 * Issue the primitive commands at t0 (staggered by duration) then
 * integrate the open-loop pending queue out to each horizon.
 */
export function commandAndSample(
  start: RolloutState,
  params: RolloutParameters,
  plant: PublicConfig,
  primitives: readonly PulsePrimitive[],
  horizons: readonly number[],
  rcfg: RolloutConfig,
): Map<number, RolloutState> {
  let pending = start.pendingPulses.map((p) => ({ ...p }));
  let tCmd = start.time;
  for (const prim of primitives) {
    pending = enqueuePrimitive(pending, prim, tCmd, rcfg.commandDelayS);
    tCmd += prim.durationS;
  }
  const started: RolloutState = { ...cloneRolloutState(start), pendingPulses: pending };
  const samples = new Map<number, RolloutState>();
  let s = started;
  const ordered = [...horizons].sort((a, b) => a - b);
  for (const h of ordered) {
    const goal = start.time + h;
    if (s.time + 1e-12 < goal) {
      s = rolloutAdvance(s, params, plant, goal - s.time, rcfg);
    }
    samples.set(h, cloneRolloutState(s));
  }
  return samples;
}

function summarize(key: string, cells: EnvelopeCell[], stage: ControlStage): EnvelopeGroupSummary {
  const att = quantiles(cells.map((c) => c.error.attRad));
  const omega = quantiles(cells.map((c) => c.error.omegaErr));
  const fuel = quantiles(cells.map((c) => c.error.fuelErr));
  const slosh = quantiles(cells.map((c) => c.error.sloshErr));
  const within = cells.filter((c) => c.withinTolerance).length / Math.max(1, cells.length);
  const tol = STAGE_TOLERANCE[stage];
  const usable = att.p50 <= tol.attRad && omega.p50 <= tol.omega;
  return {
    key,
    attRad: att,
    omegaErr: omega,
    fuelErr: fuel,
    sloshErr: slosh,
    withinToleranceRate: within,
    usableForStage: usable,
  };
}

export function measureEnvelope(
  plant: PublicConfig = defaultPublicConfig(),
  options: { horizons?: readonly number[]; actionClasses?: readonly ActionClass[] } = {},
): EnvelopeReport {
  const horizons = options.horizons ?? ENVELOPE_HORIZONS_S;
  const actions: ActionClass[] = options.actionClasses
    ? [...options.actionClasses]
    : ["coast", "single", "pair", "mixed"];
  const specs = publicStateSpecs();
  const cells: EnvelopeCell[] = [];
  const reduced = DEFAULT_ROLLOUT_CONFIG;
  const kernel = PARITY_ROLLOUT_CONFIG;

  for (const spec of specs) {
    const state = makePublicState(spec, plant);
    const params = publicBelief(plant, spec.isolatedIds);
    for (const action of actions) {
      const prims = primitivesForActionClass(action, spec.isolatedIds);
      const reducedSamples = commandAndSample(state, params, plant, prims, horizons, reduced);
      const kernelSamples = commandAndSample(state, params, plant, prims, horizons, kernel);
      for (const h of horizons) {
        const a = reducedSamples.get(h)!;
        const b = kernelSamples.get(h)!;
        const error = rolloutStateError(a, b, plant, params.k12Estimate);
        const stage = spec.regime === "terminal" && h <= 2 + 1e-9 ? "terminal" : stageForHorizon(h);
        const tol = toleranceFor(h, spec.regime);
        cells.push({
          stateId: spec.id,
          regime: spec.regime,
          actionClass: action,
          fault: spec.fault,
          pending: spec.pending,
          horizonS: h,
          stage,
          tolerance: tol,
          error,
          withinTolerance: withinTolerance(error, tol),
        });
      }
    }
  }

  const byHorizon = horizons.map((h) =>
    summarize(`horizon:${h}`, cells.filter((c) => Math.abs(c.horizonS - h) < 1e-9), stageForHorizon(h)),
  );
  const byActionClass = actions.map((a) =>
    summarize(`action:${a}`, cells.filter((c) => c.actionClass === a), "guidance"),
  );
  const byRegime = (["high-rate", "medium-rate", "terminal"] as RateRegime[]).map((r) =>
    summarize(`regime:${r}`, cells.filter((c) => c.regime === r), r === "terminal" ? "terminal" : "guidance"),
  );

  const five = byHorizon.find((g) => g.key === "horizon:5");
  const two = byHorizon.find((g) => g.key === "horizon:2");
  const half = byHorizon.find((g) => g.key === "horizon:0.5");
  const ten = byHorizon.find((g) => g.key === "horizon:10");
  const fiveP50 = five?.attRad.p50 ?? NaN;

  return {
    physicsBaselineSha: "bdfff5b4c62733d7156d2f9cdeeaa75661d6c9f4",
    controllerVersion: "discrete-pulse-v2",
    notes:
      "Reduced DEFAULT_ROLLOUT_CONFIG (fast, dt=40 ms) vs frozen PARITY_ROLLOUT_CONFIG (rk4+collision, dt=5 ms) on a public synthetic state grid. Same belief parameters. Terminal capture requires att error << 1° ≈ 0.0175 rad; a 5 s p50 near 0.028 rad already exceeds that gate.",
    horizonsS: [...horizons],
    actionClasses: actions,
    regimes: ["high-rate", "medium-rate", "terminal"],
    faultStates: ["healthy", "one-isolated"],
    pendingQueues: ["empty", "non-empty"],
    tolerances: STAGE_TOLERANCE,
    cells,
    byHorizon,
    byActionClass,
    byRegime,
    verdict: {
      terminalHorizonUsable: Boolean(half && half.attRad.p50 <= STAGE_TOLERANCE.terminal.attRad),
      guidanceHorizonUsable: Boolean(two && two.attRad.p50 <= STAGE_TOLERANCE.guidance.attRad),
      longHorizonUsableForTerminal: Boolean(ten && ten.attRad.p50 <= STAGE_TOLERANCE.terminal.attRad),
      fiveSecondAttP50Rad: fiveP50,
      fiveSecondAttP50Deg: deg(fiveP50),
    },
  };
}

void Q0;
void CMD_DELAY;
void attitudeErrorAngle;
