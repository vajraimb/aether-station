/**
 * Controller-side reduced rollout. Uses belief parameters only.
 *
 * Approximations (documented, covered by parity tests):
 * - parameters frozen over one horizon
 * - translation kept but not costed
 * - slider uses the same PD law as flight
 * - slosh uses estimated (c1,c2,k12) inside the frozen Newton–Euler kernel
 * - planning dt may be coarser than the 5 ms plant dt
 * - collisions are integrated with the frozen event locator when enabled
 */
import { sliderForceCommand } from "../allocate";
import {
  CMD_DELAY,
  FMAX,
  G0,
  ISP,
  MAX_ACTIVE,
  MIN_PULSE,
  OMEGA1,
  OMEGA2,
  THRUSTERS,
} from "../constants";
import {
  integrateWithCollision,
  massState,
  modalMasses,
  rk4Step,
  sloshEnergy,
  type ForceInput,
} from "../dynamics";
import {
  attitudeErrorAngle,
  minv3,
  mv,
  qdot,
  qnormalize,
  vadd,
  vcross,
  vnorm,
  vscale,
  vsub,
  type Mat3,
  type Quat,
  type Vec3,
} from "../math3d";
import type { PublicConfig, SimState } from "../types";
import {
  enqueuePrimitive,
  occupancyAt,
  type PendingPulse,
  type PulsePrimitive,
} from "./discrete-actions";

export interface RolloutState {
  time: number;
  qBI: Quat;
  omegaB: Vec3;
  sliderS: number;
  sliderV: number;
  theta1: number;
  theta1Dot: number;
  theta2: number;
  theta2Dot: number;
  fuelMass: number;
  pendingPulses: readonly PendingPulse[];
  rCmI: Vec3;
  vCmI: Vec3;
}

export interface RolloutParameters {
  inertiaEstimate: Mat3;
  etaTEstimate: number;
  c1Estimate: number;
  c2Estimate: number;
  k12Estimate: number;
  failedThrusterBeliefs: readonly number[];
}

export interface RolloutConfig {
  dt: number;
  commandDelayS: number;
  maxActive: number;
  useCollision: boolean;
  fast: boolean;
}

export const DEFAULT_ROLLOUT_CONFIG: RolloutConfig = {
  dt: 0.04,
  commandDelayS: CMD_DELAY,
  maxActive: MAX_ACTIVE,
  useCollision: false,
  fast: true,
};

export const PARITY_ROLLOUT_CONFIG: RolloutConfig = {
  dt: 0.005,
  commandDelayS: CMD_DELAY,
  maxActive: MAX_ACTIVE,
  useCollision: true,
  fast: false,
};

export function cloneRolloutState(s: RolloutState): RolloutState {
  return {
    time: s.time,
    qBI: [s.qBI[0], s.qBI[1], s.qBI[2], s.qBI[3]],
    omegaB: [s.omegaB[0], s.omegaB[1], s.omegaB[2]],
    sliderS: s.sliderS,
    sliderV: s.sliderV,
    theta1: s.theta1,
    theta1Dot: s.theta1Dot,
    theta2: s.theta2,
    theta2Dot: s.theta2Dot,
    fuelMass: s.fuelMass,
    pendingPulses: s.pendingPulses.map((p) => ({ id: p.id, tOn: p.tOn, tOff: p.tOff })),
    rCmI: [s.rCmI[0], s.rCmI[1], s.rCmI[2]],
    vCmI: [s.vCmI[0], s.vCmI[1], s.vCmI[2]],
  };
}

export function rolloutFromSimLike(args: {
  time: number;
  q: Quat;
  w: Vec3;
  s: number;
  sd: number;
  th1: number;
  th1d: number;
  th2: number;
  th2d: number;
  fuel: number;
  pendingPulses?: readonly PendingPulse[];
  rCmI?: Vec3;
  vCmI?: Vec3;
}): RolloutState {
  return {
    time: args.time,
    qBI: qnormalize(args.q),
    omegaB: [args.w[0], args.w[1], args.w[2]],
    sliderS: args.s,
    sliderV: args.sd,
    theta1: args.th1,
    theta1Dot: args.th1d,
    theta2: args.th2,
    theta2Dot: args.th2d,
    fuelMass: args.fuel,
    pendingPulses: args.pendingPulses ? args.pendingPulses.map((p) => ({ ...p })) : [],
    rCmI: args.rCmI ? [args.rCmI[0], args.rCmI[1], args.rCmI[2]] : [0, 0, 0],
    vCmI: args.vCmI ? [args.vCmI[0], args.vCmI[1], args.vCmI[2]] : [0, 0, 0],
  };
}

function toSimState(s: RolloutState): SimState {
  return {
    t: s.time,
    rI: [0, 0, 0],
    vI: [0, 0, 0],
    rCmI: [s.rCmI[0], s.rCmI[1], s.rCmI[2]],
    vCmI: [s.vCmI[0], s.vCmI[1], s.vCmI[2]],
    q: [s.qBI[0], s.qBI[1], s.qBI[2], s.qBI[3]],
    w: [s.omegaB[0], s.omegaB[1], s.omegaB[2]],
    s: s.sliderS,
    sd: s.sliderV,
    th1: s.theta1,
    th1d: s.theta1Dot,
    th2: s.theta2,
    th2d: s.theta2Dot,
    fuel: s.fuelMass,
  };
}

function fromSimState(st: SimState, pending: readonly PendingPulse[], fuel: number): RolloutState {
  return {
    time: st.t,
    qBI: qnormalize(st.q),
    omegaB: [st.w[0], st.w[1], st.w[2]],
    sliderS: st.s,
    sliderV: st.sd,
    theta1: st.th1,
    theta1Dot: st.th1d,
    theta2: st.th2,
    theta2Dot: st.th2d,
    fuelMass: fuel,
    pendingPulses: pending.map((p) => ({ id: p.id, tOn: p.tOn, tOff: p.tOff })),
    rCmI: [st.rCmI[0], st.rCmI[1], st.rCmI[2]],
    vCmI: [st.vCmI[0], st.vCmI[1], st.vCmI[2]],
  };
}

export interface ActiveWrench {
  Fb: Vec3;
  tauO: Vec3;
  nActive: number;
  fuelDot: number;
  ids: number[];
}

export function evaluateRolloutWrench(
  state: RolloutState,
  params: RolloutParameters,
  cfg: PublicConfig,
  rcfg: RolloutConfig,
): ActiveWrench {
  const failed = new Set(params.failedThrusterBeliefs);
  const ids = occupancyAt(state.pendingPulses, state.time, failed, rcfg.maxActive);
  let Fb: Vec3 = [0, 0, 0];
  let tauO: Vec3 = [0, 0, 0];
  const dry = state.fuelMass <= 1e-6;
  let nActive = 0;
  if (!dry) {
    for (const id of ids) {
      const geom = THRUSTERS[id];
      if (!geom) continue;
      const Fi = vscale(geom.dir, params.etaTEstimate * cfg.maxThrust);
      Fb = vadd(Fb, Fi);
      tauO = vadd(tauO, vcross(geom.pos, Fi));
      nActive += 1;
    }
  }
  const fuelDot = dry || nActive === 0 ? 0 : -(nActive * params.etaTEstimate * cfg.maxThrust) / (cfg.isp * cfg.g0);
  return { Fb, tauO, nActive, fuelDot, ids };
}

export function rolloutStep(
  state: RolloutState,
  params: RolloutParameters,
  plant: PublicConfig,
  rcfg: RolloutConfig = DEFAULT_ROLLOUT_CONFIG,
): RolloutState {
  const snap = evaluateRolloutWrench(state, params, plant, rcfg);
  if (rcfg.fast) return fastStep(state, params, plant, rcfg, snap);
  const u: ForceInput = {
    FthrB: snap.Fb,
    tauThrO: snap.tauO,
    Fslider: sliderForceCommand(state.sliderS, state.sliderV, plant),
    c1: plant.fluidPresent ? params.c1Estimate : 0,
    c2: plant.fluidPresent ? params.c2Estimate : 0,
    k12: plant.fluidPresent ? params.k12Estimate : 0,
  };
  const st = toSimState(state);
  const dt = rcfg.dt;
  const nextSt = rcfg.useCollision
    ? integrateWithCollision(plant, st, u, dt).state
    : rk4Step(st, plant, u, dt);
  const fuel = Math.max(0, state.fuelMass + snap.fuelDot * dt);
  const pending = state.pendingPulses.filter((p) => p.tOff > nextSt.t);
  return fromSimState(nextSt, pending, fuel);
}

function fastStep(
  state: RolloutState,
  params: RolloutParameters,
  plant: PublicConfig,
  rcfg: RolloutConfig,
  snap: ActiveWrench,
): RolloutState {
  const dt = rcfg.dt;
  const ms = massState(plant, state.sliderS, state.theta1, state.theta2, state.fuelMass);
  const Iinv = minv3(ms.Icm);
  const tauCm = vsub(snap.tauO, vcross(ms.rCmB, snap.Fb));
  const Iw = mv(ms.Icm, state.omegaB);
  const alpha = mv(Iinv, vsub(tauCm, vcross(state.omegaB, Iw)));
  const Fs = sliderForceCommand(state.sliderS, state.sliderV, plant);
  const sdd = Fs / Math.max(plant.sliderMass, 1);
  const th1dd =
    -OMEGA1 * OMEGA1 * Math.sin(state.theta1) -
    params.c1Estimate * state.theta1Dot -
    params.k12Estimate * Math.sin(state.theta1 - state.theta2);
  const th2dd =
    -OMEGA2 * OMEGA2 * Math.sin(state.theta2) -
    params.c2Estimate * state.theta2Dot -
    params.k12Estimate * Math.sin(state.theta2 - state.theta1);
  const qd = qdot(state.qBI, state.omegaB);
  const q = qnormalize([
    state.qBI[0] + dt * qd[0],
    state.qBI[1] + dt * qd[1],
    state.qBI[2] + dt * qd[2],
    state.qBI[3] + dt * qd[3],
  ]);
  let s = state.sliderS + dt * state.sliderV + 0.5 * dt * dt * sdd;
  let sd = state.sliderV + dt * sdd;
  if (s > plant.sliderMax) {
    s = plant.sliderMax;
    sd = Math.min(0, sd);
  } else if (s < plant.sliderMin) {
    s = plant.sliderMin;
    sd = Math.max(0, sd);
  }
  const pending = state.pendingPulses.filter((p) => p.tOff > state.time + dt);
  return {
    time: state.time + dt,
    qBI: q,
    omegaB: [
      state.omegaB[0] + dt * alpha[0],
      state.omegaB[1] + dt * alpha[1],
      state.omegaB[2] + dt * alpha[2],
    ],
    sliderS: s,
    sliderV: sd,
    theta1: state.theta1 + dt * state.theta1Dot,
    theta1Dot: state.theta1Dot + dt * th1dd,
    theta2: state.theta2 + dt * state.theta2Dot,
    theta2Dot: state.theta2Dot + dt * th2dd,
    fuelMass: Math.max(0, state.fuelMass + snap.fuelDot * dt),
    pendingPulses: pending,
    rCmI: state.rCmI,
    vCmI: state.vCmI,
  };
}

export function rolloutAdvance(
  state: RolloutState,
  params: RolloutParameters,
  plant: PublicConfig,
  durationS: number,
  rcfg: RolloutConfig = DEFAULT_ROLLOUT_CONFIG,
): RolloutState {
  let s = state;
  const end = state.time + durationS;
  const dt = rcfg.dt;
  while (s.time + 1e-12 < end) {
    const step = Math.min(dt, end - s.time);
    const local = step === dt ? rcfg : { ...rcfg, dt: step };
    s = rolloutStep(s, params, plant, local);
  }
  return s;
}

export function applyPrimitive(
  state: RolloutState,
  params: RolloutParameters,
  plant: PublicConfig,
  primitive: Readonly<PulsePrimitive>,
  rcfg: RolloutConfig = DEFAULT_ROLLOUT_CONFIG,
): RolloutState {
  const queued = enqueuePrimitive(state.pendingPulses, primitive, state.time, rcfg.commandDelayS);
  const started: RolloutState = { ...cloneRolloutState(state), pendingPulses: queued };
  return rolloutAdvance(started, params, plant, primitive.durationS, rcfg);
}

/** Advance through command delay plus pulse width so the queued firing is visible. */
export function applyPrimitiveUntilComplete(
  state: RolloutState,
  params: RolloutParameters,
  plant: PublicConfig,
  primitive: Readonly<PulsePrimitive>,
  rcfg: RolloutConfig = DEFAULT_ROLLOUT_CONFIG,
): RolloutState {
  const queued = enqueuePrimitive(state.pendingPulses, primitive, state.time, rcfg.commandDelayS);
  const started: RolloutState = { ...cloneRolloutState(state), pendingPulses: queued };
  const span = primitive.durationS + rcfg.commandDelayS;
  return rolloutAdvance(started, params, plant, span, rcfg);
}

export function predictedSloshEnergy(state: RolloutState, plant: PublicConfig, k12: number): number {
  const mm = modalMasses(plant.fluidMass);
  return sloshEnergy(
    state.theta1,
    state.theta1Dot,
    state.theta2,
    state.theta2Dot,
    mm.m1,
    mm.m2,
    plant.tankMeanRadius,
    k12,
  );
}

export function inertiaAboutCm(plant: PublicConfig, state: RolloutState): Mat3 {
  return massState(plant, state.sliderS, state.theta1, state.theta2, state.fuelMass).Icm;
}

export function geodesicAttitudeError(qBI: Quat, qTarget: Quat): number {
  return attitudeErrorAngle(qnormalize(qBI), qTarget);
}

void FMAX;
void ISP;
void G0;
void MIN_PULSE;
void vnorm;
