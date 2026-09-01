/**
 * Deterministic plant roll-out engine for Challenge V3.
 *
 * It reuses the audited physics core verbatim (`integrateWithCollision`,
 * `massState`, `sloshEnergy`, `totalAngularMomentumI`) and the audited
 * actuator model (`ThrusterSystem`: 120 ms command delay, >= 40 ms pulses,
 * two concurrent nozzles, Isp mass flow). It drops only the sensor, estimator,
 * FDIR and logging layers, which do not influence the physical trajectory.
 *
 * The same class is used three ways:
 *   1. truth roll-out      - parameters = PrivateScenario truth (stage 1)
 *   2. ensemble roll-out   - parameters = a belief particle (stage 3)
 *   3. calibration roll-out- planner prediction vs. truth plant outcome
 *
 * `dtMax` may be coarsened for search; every reported number is re-measured by
 * the official `Simulator` at the official 5 ms step and by the file scorer.
 */
import { attitudeErrorAngle, deg, qnorm, qnormalize, qRotate, vnorm, type Quat, type Vec3 } from "../math3d";
import { CTRL_DT, DT } from "../constants";
import {
  integrateWithCollision,
  massState,
  modalMasses,
  sloshEnergy,
  totalAngularMomentumI,
} from "../dynamics";
import { ThrusterSystem } from "../thrusters";
import { sliderForceCommand } from "../allocate";
import type { Command, PublicConfig, SimState } from "../types";
import { segmentTicks, type Segment } from "./action-space";

export interface PlantParams {
  c1: number;
  c2: number;
  k12: number;
  etaT: number;
  /** Absolute mission time at which `faultThruster` dies. Infinity = never. */
  faultTime: number;
  faultThruster: number;
  /** Thrusters already dead at roll-out start (e.g. an isolated nozzle). */
  preFailed?: readonly number[];
}

export interface PlantOptions {
  /** Integration step. Defaults to the official 5 ms. */
  dt?: number;
  /** Coarser step allowed while every queued pulse is off. 0 = disabled. */
  coastDt?: number;
  /** Stop at this absolute mission time. Defaults to cfg.duration. */
  until?: number;
  /** Slider law. Defaults to the audited `sliderForceCommand`. */
  sliderForce?: (s: number, sd: number, cfg: PublicConfig) => number;
  /**
   * Mission-initial slosh energy used as the denominator of the reported
   * ratio. Defaults to the slosh energy of `start`, which is only correct for
   * roll-outs that begin at mission t=0.
   */
  sloshRef?: number;
  /**
   * Per-step conservation audit (angular-momentum drift). On by default for
   * validation roll-outs, switched off inside the optimizers where it is pure
   * overhead. It never influences the trajectory.
   */
  audit?: boolean;
}

export interface RolloutResult {
  state: SimState;
  /** State at the instant the commanded schedule finishes burning. */
  atScheduleEnd: SimState;
  tEnd: number;
  attitudeErrorDeg: number;
  omega: number;
  omegaVec: Vec3;
  fuel: number;
  sloshEnergy: number;
  initialSloshEnergy: number;
  /** final / initial slosh energy, matching `final_slosh_energy_ratio`. */
  sloshRatio: number;
  maxImpactSpeed: number;
  maxQuatNormError: number;
  maxConstraintViolation: number;
  maxAngularMomentumError: number;
  pulseCount: number;
  totalOnTime: number;
  collisions: number;
  numericAnomaly: boolean;
  /** Peak |omega| seen during the roll-out. */
  peakOmega: number;
  steps: number;
}

export function cloneState(st: SimState): SimState {
  return {
    ...st,
    rI: [...st.rI] as Vec3,
    vI: [...st.vI] as Vec3,
    rCmI: [...st.rCmI] as Vec3,
    vCmI: [...st.vCmI] as Vec3,
    q: [...st.q] as Quat,
    w: [...st.w] as Vec3,
  };
}

/** Mission t=0 state, byte-identical to what `Simulator`'s constructor builds. */
export function initialState(cfg: PublicConfig, q0: Quat, w0: Vec3, s0: number, sd0: number): SimState {
  const st: SimState = {
    t: 0,
    rI: [0, 0, 0],
    vI: [0, 0, 0],
    rCmI: [0, 0, 0],
    vCmI: [0, 0, 0],
    q: qnormalize([q0[0], q0[1], q0[2], q0[3]]),
    w: [w0[0], w0[1], w0[2]],
    s: s0,
    sd: sd0,
    th1: cfg.fluidPresent ? 0.18 : 0,
    th1d: 0,
    th2: cfg.fluidPresent ? -0.11 : 0,
    th2d: cfg.fluidPresent ? 0.06 : 0,
    fuel: cfg.initialFuelMass,
  };
  const ms = massState(cfg, st.s, st.th1, st.th2, st.fuel);
  st.rCmI = qRotate(st.q, ms.rCmB);
  st.vCmI = [0, 0, 0];
  return st;
}

/** Expand a segment sequence into per-controller-tick pulse-width commands. */
export function expandSchedule(
  seq: readonly Segment[],
  ctrlDt: number = CTRL_DT,
): Array<[number, number, number, number, number, number]> {
  const ticks: Array<[number, number, number, number, number, number]> = [];
  for (const seg of seq) {
    const n = segmentTicks(seg.durationS, ctrlDt);
    const first: [number, number, number, number, number, number] = [0, 0, 0, 0, 0, 0];
    for (const id of seg.action) first[id] = seg.durationS;
    ticks.push(first);
    for (let k = 1; k < n; k++) ticks.push([0, 0, 0, 0, 0, 0]);
  }
  return ticks;
}

/**
 * Roll the plant forward from `start` under an explicit tick schedule.
 * `schedule[k]` is the pulse-width vector submitted at controller tick k
 * (mission time `start.t + k * cfg.controllerPeriod`). Once the schedule is
 * exhausted the plant coasts (slider law stays active) until `until`.
 */
export function rollout(
  cfg: PublicConfig,
  start: SimState,
  params: PlantParams,
  schedule: ReadonlyArray<readonly number[]>,
  opts: PlantOptions = {},
): RolloutResult {
  const dtFine = opts.dt ?? cfg.dtMax ?? DT;
  const dtCoast = opts.coastDt ?? 0;
  const until = opts.until ?? cfg.duration;
  const sliderLaw = opts.sliderForce ?? sliderForceCommand;

  let st = cloneState(start);
  const thr = new ThrusterSystem(params.etaT, {
    Fmax: cfg.maxThrust,
    delay: cfg.commandDelay,
    minPulse: cfg.minPulse,
    isp: cfg.isp,
    g0: cfg.g0,
  });
  for (const id of params.preFailed ?? []) thr.fail(id);
  let faultArmed = Number.isFinite(params.faultTime) && params.faultTime > st.t;
  if (Number.isFinite(params.faultTime) && params.faultTime <= st.t) thr.fail(params.faultThruster);

  const audit = opts.audit ?? true;
  const H0 = totalAngularMomentumI(cfg, st, params.k12);
  const mm = modalMasses(cfg.fluidMass);
  const se0 =
    opts.sloshRef ??
    sloshEnergy(st.th1, st.th1d, st.th2, st.th2d, mm.m1, mm.m2, cfg.tankMeanRadius, params.k12);

  const t0 = st.t;
  let tick = 0;
  const tickTime = (k: number) => t0 + k * cfg.controllerPeriod;
  let maxImpact = 0;
  let maxQerr = 0;
  let maxViol = 0;
  let maxHerr = 0;
  let collisions = 0;
  let peakOmega = vnorm(st.w);
  let numericAnomaly = false;
  let steps = 0;
  /** Slider force is a zero-order hold between controller ticks, as in the plant. */
  let sliderF = 0;
  let scheduleDone = schedule.length === 0;
  /** Latest mission time at which a queued pulse can still be burning. */
  let lastPulseOff = st.t;
  let atScheduleEnd: SimState | null = schedule.length === 0 ? cloneState(st) : null;

  while (st.t < until - 1e-12) {
    // Coarse step only once the schedule is exhausted and nothing is still
    // burning. Coarsening idle stretches *inside* a schedule was tried and
    // measured worse: the coast leg of a rest-to-rest capture is exactly where
    // the pointing prediction has to be accurate, and integrating it at the
    // coast step poisons candidate ranking (see failure-analysis.json,
    // `coarse_coast_inside_schedule`).
    const quiet = scheduleDone && st.t >= lastPulseOff;
    // While the schedule is live the step is exactly `dtFine`, which divides
    // the 0.1 s controller period, so ticks always land on a step boundary
    // exactly as in the official loop. Coarsening only happens after the last
    // queued pulse has burned out.
    let dt = quiet && dtCoast > 0 ? dtCoast : dtFine;
    if (st.t + dt > until) dt = until - st.t;
    // A coarse step must not skip a controller tick (slider law update).
    const gap = tickTime(tick) - st.t;
    if (dt > gap + 1e-9 && gap > 1e-9) dt = gap;

    if (faultArmed && st.t + dt >= params.faultTime) {
      thr.fail(params.faultThruster);
      faultArmed = false;
    }

    if (st.t + 1e-9 >= tickTime(tick)) {
      const widths = scheduleDone ? undefined : schedule[tick];
      const pulseWidth: [number, number, number, number, number, number] = [0, 0, 0, 0, 0, 0];
      if (widths) for (let i = 0; i < 6; i++) pulseWidth[i] = widths[i] ?? 0;
      sliderF = sliderLaw(st.s, st.sd, cfg);
      const cmd: Command = { sliderForce: sliderF, pulseWidth };
      thr.submit(st.t, cmd);
      let wMax = 0;
      for (let i = 0; i < 6; i++) {
        if (pulseWidth[i]! > 0) wMax = Math.max(wMax, Math.max(pulseWidth[i]!, cfg.minPulse));
      }
      if (wMax > 0) lastPulseOff = Math.max(lastPulseOff, st.t + cfg.commandDelay + wMax);
      tick += 1;
      if (tick >= schedule.length) scheduleDone = true;
    }

    const snap = thr.evaluate(st.t, dt, st.fuel);
    const u = {
      FthrB: snap.Fb,
      tauThrO: snap.tauO,
      Fslider: sliderF,
      c1: cfg.fluidPresent ? params.c1 : 0,
      c2: cfg.fluidPresent ? params.c2 : 0,
      k12: cfg.fluidPresent ? params.k12 : 0,
    };
    const col = integrateWithCollision(cfg, st, u, dt);
    st = col.state;
    st.fuel = Math.max(0, st.fuel + snap.fuelDot * dt);
    steps += 1;
    if (col.collided) {
      collisions += 1;
      maxImpact = Math.max(maxImpact, col.impactSpeed);
    }
    const qn = Math.abs(qnorm(st.q) - 1);
    if (qn > maxQerr) maxQerr = qn;
    const viol = Math.max(0, Math.abs(st.s) - cfg.sliderMax);
    if (viol > maxViol) maxViol = viol;
    if (audit) {
      const H = totalAngularMomentumI(cfg, st, params.k12);
      const herr = vnorm([H[0] - H0[0], H[1] - H0[1], H[2] - H0[2]]);
      if (herr > maxHerr) maxHerr = herr;
    }
    if (atScheduleEnd === null && scheduleDone && st.t >= lastPulseOff - 1e-12) {
      atScheduleEnd = cloneState(st);
    }
    const wm = vnorm(st.w);
    if (wm > peakOmega) peakOmega = wm;
    if (!Number.isFinite(wm) || !Number.isFinite(st.q[0]) || !Number.isFinite(st.fuel)) {
      numericAnomaly = true;
      break;
    }
  }

  const se = sloshEnergy(st.th1, st.th1d, st.th2, st.th2d, mm.m1, mm.m2, cfg.tankMeanRadius, params.k12);
  return {
    state: st,
    atScheduleEnd: atScheduleEnd ?? cloneState(st),
    tEnd: st.t,
    attitudeErrorDeg: deg(attitudeErrorAngle(st.q, cfg.qTarget)),
    omega: vnorm(st.w),
    omegaVec: [st.w[0], st.w[1], st.w[2]],
    fuel: st.fuel,
    sloshEnergy: se,
    initialSloshEnergy: se0,
    sloshRatio: se0 > 1e-9 ? se / se0 : 0,
    maxImpactSpeed: maxImpact,
    maxQuatNormError: maxQerr,
    maxConstraintViolation: maxViol,
    maxAngularMomentumError: maxHerr,
    pulseCount: thr.pulseCount,
    totalOnTime: thr.totalOnTime,
    collisions,
    numericAnomaly,
    peakOmega,
    steps,
  };
}

/** Convenience: roll a segment sequence. */
export function rolloutSequence(
  cfg: PublicConfig,
  start: SimState,
  params: PlantParams,
  seq: readonly Segment[],
  opts: PlantOptions = {},
): RolloutResult {
  return rollout(cfg, start, params, expandSchedule(seq, cfg.controllerPeriod), opts);
}
