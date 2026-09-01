/**
 * Reduced attitude model used to prune candidate action sequences before the
 * survivors are re-scored on the audited plant.
 *
 * It keeps what matters for pulse-sequence selection - rigid-body Euler
 * dynamics about the instantaneous CM, the true torque columns, the 120 ms
 * command delay, the 40 ms pulse quantum, the two-nozzle cap and the Isp mass
 * flow - and drops the slosh / slider back-reaction, which is treated as an
 * unmodelled disturbance and measured explicitly in
 * `belief-rollout-calibration.json`.
 *
 * Nothing here is allowed to be the final word on a metric: every surviving
 * candidate is re-rolled on the full plant and every published number comes
 * from the official `Simulator` plus the file scorer.
 */
import {
  attitudeErrorAngle,
  deg,
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
import { massState } from "../dynamics";
import { torqueColumns } from "../allocate";
import type { PublicConfig, SimState } from "../types";
import { segmentTicks, type Segment } from "./action-space";

export interface SurrogateModel {
  /** Torque columns about the CM at full thrust, body frame. */
  cols: Vec3[];
  I: Mat3;
  Iinv: Mat3;
  /** kg of propellant per second per firing nozzle. */
  fuelRate: number;
  etaT: number;
}

export function buildSurrogate(cfg: PublicConfig, st: SimState, etaT: number): SurrogateModel {
  const ms = massState(cfg, st.s, st.th1, st.th2, st.fuel);
  const cols = torqueColumns(cfg, st.s, st.th1, st.th2, st.fuel, etaT);
  return {
    cols,
    I: ms.Icm,
    Iinv: minv3(ms.Icm),
    fuelRate: (etaT * cfg.maxThrust) / (cfg.isp * cfg.g0),
    etaT,
  };
}

/** Angular-velocity increment delivered by one nozzle firing for `w` seconds. */
export function deltaOmega(model: SurrogateModel, id: number, widthS: number): Vec3 {
  return mv(model.Iinv, vscale(model.cols[id]!, widthS));
}

export interface SurrogateState {
  t: number;
  q: Quat;
  w: Vec3;
  fuel: number;
}

export interface SurrogatePulse {
  id: number;
  tOn: number;
  tOff: number;
}

/** Expand a segment sequence into absolute-time pulse windows (delay applied). */
export function pulseWindows(
  seq: readonly Segment[],
  t0: number,
  ctrlDt: number,
  delay: number,
  minPulse: number,
): SurrogatePulse[] {
  const out: SurrogatePulse[] = [];
  let t = t0;
  for (const seg of seq) {
    const w = Math.max(seg.durationS, seg.action.length > 0 ? minPulse : 0);
    for (const id of seg.action) out.push({ id, tOn: t + delay, tOff: t + delay + w });
    t += segmentTicks(seg.durationS, ctrlDt) * ctrlDt;
  }
  return out;
}

export interface SurrogateRollout {
  end: SurrogateState;
  peakOmega: number;
  onTime: number;
  pulseCount: number;
  /** Attitude error in degrees at the horizon end. */
  attitudeErrorDeg: number;
}

/**
 * Integrate the reduced model over [t0, tEnd] with an explicit pulse list.
 * Fixed RK2 (midpoint) at `dt`; `dt` is a fixed search parameter, never a
 * wall-clock budget.
 */
export function propagate(
  cfg: PublicConfig,
  model: SurrogateModel,
  start: SurrogateState,
  pulses: readonly SurrogatePulse[],
  tEnd: number,
  dt: number,
): SurrogateRollout {
  let q: Quat = qnormalize([start.q[0], start.q[1], start.q[2], start.q[3]]);
  let w: Vec3 = [start.w[0], start.w[1], start.w[2]];
  let fuel = start.fuel;
  let t = start.t;
  let peak = vnorm(w);
  let onTime = 0;

  const torqueAt = (time: number): { tau: Vec3; n: number } => {
    let tau: Vec3 = [0, 0, 0];
    let n = 0;
    for (const p of pulses) {
      if (time >= p.tOn && time < p.tOff) {
        tau = vadd(tau, model.cols[p.id]!);
        n += 1;
      }
    }
    return { tau, n };
  };

  const wdot = (wv: Vec3, tau: Vec3): Vec3 =>
    mv(model.Iinv, vsub(tau, vcross(wv, mv(model.I, wv))));

  while (t < tEnd - 1e-12) {
    const h = Math.min(dt, tEnd - t);
    const a = torqueAt(t + 0.25 * h);
    const b = torqueAt(t + 0.75 * h);
    const tauMid = vscale(vadd(a.tau, b.tau), 0.5);
    const nMid = 0.5 * (a.n + b.n);
    const k1 = wdot(w, tauMid);
    const wMid = vadd(w, vscale(k1, 0.5 * h));
    const qd1 = qdot(q, w);
    const qMid: Quat = qnormalize([
      q[0] + 0.5 * h * qd1[0],
      q[1] + 0.5 * h * qd1[1],
      q[2] + 0.5 * h * qd1[2],
      q[3] + 0.5 * h * qd1[3],
    ]);
    const k2 = wdot(wMid, tauMid);
    const qd2 = qdot(qMid, wMid);
    w = vadd(w, vscale(k2, h));
    q = qnormalize([q[0] + h * qd2[0], q[1] + h * qd2[1], q[2] + h * qd2[2], q[3] + h * qd2[3]]);
    fuel -= model.fuelRate * nMid * h;
    onTime += nMid * h;
    t += h;
    const wm = vnorm(w);
    if (wm > peak) peak = wm;
  }

  return {
    end: { t, q, w, fuel },
    peakOmega: peak,
    onTime,
    pulseCount: pulses.length,
    attitudeErrorDeg: deg(attitudeErrorAngle(q, cfg.qTarget)),
  };
}

/**
 * Free-drift attitude error at `tFinal` starting from (q, w) at `t`.
 * Torque-free rigid-body motion, so it accounts for the polhode drift that a
 * naive `|2 e + w dt|` extrapolation misses.
 */
export function driftAttitudeErrorDeg(
  cfg: PublicConfig,
  model: SurrogateModel,
  from: SurrogateState,
  tFinal: number,
  dt = 0.25,
): { attitudeErrorDeg: number; omega: number; q: Quat; w: Vec3 } {
  const r = propagate(cfg, model, from, [], Math.max(tFinal, from.t), dt);
  return {
    attitudeErrorDeg: r.attitudeErrorDeg,
    omega: vnorm(r.end.w),
    q: r.end.q,
    w: r.end.w,
  };
}

/**
 * Minimum thruster on-time needed to null `w` with the best available column,
 * i.e. an optimistic (admissible) braking cost used as the branch-and-bound
 * lower bound and as the viability / braking-margin test.
 */
export function brakingOnTimeLowerBound(model: SurrogateModel, w: Vec3, live: readonly number[]): number {
  const H = mv(model.I, w);
  const hMag = vnorm(H);
  if (hMag < 1e-12) return 0;
  let best = 0;
  for (const id of live) {
    const c = model.cols[id]!;
    const proj = Math.abs((c[0] * H[0] + c[1] * H[1] + c[2] * H[2]) / hMag);
    if (proj > best) best = proj;
  }
  if (best < 1e-9) return Infinity;
  return hMag / best;
}
