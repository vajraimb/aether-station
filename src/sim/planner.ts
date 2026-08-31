/**
 * Receding-horizon mixed pulse planner. Rigid-body predictor over 5–15 s.
 *
 * Large-error law is bang-coast-bang on the short-way eigenaxis. The only
 * continuous dual-fire is a brief detumble. Fuel floor is a constraint.
 */
import {
  attitudeErrorAngle,
  attitudeErrorVector,
  clamp,
  deg,
  minv3,
  mv,
  qdot,
  qnormalize,
  vadd,
  vcross,
  vdot,
  vnorm,
  vscale,
  vsub,
  type Quat,
  type Vec3,
} from "./math3d";
import { massState } from "./dynamics";
import { allocateTorque, pulseImpulse, torqueColumns } from "./allocate";
import { CTRL_DT, MIN_PULSE } from "./constants";
import { ATT_GATE_DEG, FUEL_FLOOR, FUEL_STOP, HORIZON_S, RATE_GATE } from "./evalset";
import type { PublicConfig } from "./types";
import type { Estimate } from "./estimator";

export interface PlanState {
  q: Quat;
  w: Vec3;
  fuel: number;
  s: number;
  th1: number;
  th2: number;
}

export interface PlannerOpts {
  horizon?: number;
  wCap?: number;
  alphaScale?: number;
  lastFireT?: number;
  ignoreDelay?: boolean;
}

export interface PulseOption {
  pulse: [number, number, number, number, number, number];
  label: string;
}

export interface PlanScore {
  lex: number;
  fuel: number;
  omega: number;
  attDeg: number;
  used: number;
}

export interface SwitchCmd {
  want: Vec3;
  wDesPar: number;
  eN: Vec3;
  theta: number;
  attDeg: number;
  wPerpN: number;
  wPar: number;
  phase: "hold" | "settle" | "perp" | "accel" | "brake" | "coast" | "detumble";
  width: number;
  maxJets: 1 | 2;
  mustFire: boolean;
}

function copyPlan(s: PlanState): PlanState {
  return {
    q: [s.q[0], s.q[1], s.q[2], s.q[3]],
    w: [s.w[0], s.w[1], s.w[2]],
    fuel: s.fuel,
    s: s.s,
    th1: s.th1,
    th2: s.th2,
  };
}

function emptyPulse(): [number, number, number, number, number, number] {
  return [0, 0, 0, 0, 0, 0];
}

function torqueFromPulse(cols: Vec3[], pulse: number[], period: number): Vec3 {
  let tau: Vec3 = [0, 0, 0];
  for (let i = 0; i < 6; i++) {
    const w = pulse[i] ?? 0;
    if (w <= 0) continue;
    tau = vadd(tau, vscale(cols[i]!, w / period));
  }
  return tau;
}

function mdotOf(pulse: number[], eta: number, Fmax: number, ispG0: number, period: number): number {
  const duty = pulse.reduce((s, w) => s + w, 0) / period;
  if (duty <= 0) return 0;
  return (duty * eta * Fmax) / ispG0;
}

function nOn(pulse: number[]): number {
  let n = 0;
  for (const w of pulse) if (w > 0) n += 1;
  return n;
}

export function estimateAlphaMax(
  cols: Vec3[],
  isolated: Set<number>,
  Iinv: ReturnType<typeof minv3>,
): number {
  let best = 0.018;
  const live: number[] = [];
  for (let i = 0; i < 6; i++) if (!isolated.has(i)) live.push(i);
  for (const i of live) {
    const a = vnorm(mv(Iinv, cols[i]!));
    if (a > best) best = a;
  }
  return best;
}

export function switchCommand(
  cfg: PublicConfig,
  q: Quat,
  w: Vec3,
  alphaMax: number,
  opts?: PlannerOpts,
): SwitchCmd {
  const attErr = attitudeErrorVector(q, cfg.qTarget);
  const theta = attitudeErrorAngle(q, cfg.qTarget);
  const attDeg = deg(theta);
  const wmag = vnorm(w);
  const wCap = opts?.wCap ?? 0.04;
  void alphaMax;
  const kRate = 0.16 + 0.08 * (opts?.alphaScale ?? 0.5);
  const zero: Vec3 = [0, 0, 0];
  const eNmag = vnorm(attErr);
  const eN: Vec3 = eNmag > 1e-9 ? vscale(attErr, 1 / eNmag) : [1, 0, 0];
  const wPar = vdot(w, eN);
  const wPerp = vsub(w, vscale(eN, wPar));
  const wPerpN = vnorm(wPerp);

  if (attDeg < 0.82 && wmag < 0.0058) {
    return {
      want: zero, wDesPar: 0, eN, theta, attDeg, wPerpN, wPar,
      phase: "hold", width: 0, maxJets: 1, mustFire: false,
    };
  }

  let wDes: Vec3 = [-kRate * 2 * attErr[0], -kRate * 2 * attErr[1], -kRate * 2 * attErr[2]];
  const wDesN = vnorm(wDes);
  if (wDesN > wCap && wDesN > 1e-9) wDes = vscale(wDes, wCap / wDesN);
  const wErr = vsub(w, wDes);
  const wErrN = vnorm(wErr);
  const want = vscale(wErr, -1);

  if (attDeg > 1.6 || wmag > 0.045) {
    const saturated = wDesN >= wCap * 0.92;
    const tracking = wErrN < 0.012 && wPar < -0.018;
    if (saturated && tracking && wPerpN < 0.045) {
      return {
        want: zero, wDesPar: -wCap, eN, theta, attDeg, wPerpN, wPar,
        phase: "coast", width: 0, maxJets: 1, mustFire: false,
      };
    }
    const reverse = wPar > 0.008;
    const jets: 1 | 2 = (reverse && wPar > 0.02) || wErrN > 0.08 ? 2 : 1;
    return {
      want: reverse ? vscale(eN, -1) : want, wDesPar: -wCap, eN, theta, attDeg, wPerpN, wPar,
      phase: reverse ? "accel" : wErrN > 0.03 && wPar < -wCap * 0.9 ? "brake" : "accel",
      width: wErrN > 0.014 || reverse ? CTRL_DT : 2 * MIN_PULSE,
      maxJets: jets,
      mustFire: wErrN > 0.004 || (wPar > 0.0015 && attDeg > 0.7),
    };
  }

  if (wPerpN > 0.025 && attDeg > 1.4 && wPar < 0.004) {
    return {
      want: vscale(wPerp, -1), wDesPar: 0, eN, theta, attDeg, wPerpN, wPar,
      phase: "perp", width: wPerpN > 0.035 ? CTRL_DT : 2 * MIN_PULSE,
      maxJets: 1, mustFire: true,
    };
  }

  const Kp = attDeg < 1.4 ? 16 : 22;
  const Kd = attDeg < 1.4 ? 170 : 140;
  const tauDes: Vec3 = [
    -Kp * 2 * attErr[0] - Kd * w[0],
    -Kp * 2 * attErr[1] - Kd * w[1],
    -Kp * 2 * attErr[2] - Kd * w[2],
  ];
  const need = attDeg > 0.7 || wmag > 0.005 || attDeg + deg(wmag * 20) > 0.95;
  return {
    want: tauDes, wDesPar: 0, eN, theta, attDeg, wPerpN, wPar,
    phase: "settle",
    width: wmag > 0.018 || attDeg > 3.2 ? 2 * MIN_PULSE : MIN_PULSE,
    maxJets: 1,
    mustFire: need,
  };
}

export function pulseAlongWant(
  want: Vec3,
  cols: Vec3[],
  isolated: Set<number>,
  width: number,
  maxJets: 1 | 2,
): [number, number, number, number, number, number] {
  const pulse = emptyPulse();
  const wantN = vnorm(want);
  if (wantN < 1e-9 || width <= 0) return pulse;

  type Cand = { ids: number[]; tau: Vec3; align: number };
  const cands: Cand[] = [];
  const eWant = vscale(want, 1 / wantN);
  const push = (ids: number[], tau: Vec3) => {
    const n = vnorm(tau);
    if (n < 1e-6) return;
    cands.push({ ids, tau, align: vdot(tau, eWant) / n });
  };
  for (let i = 0; i < 6; i++) {
    if (isolated.has(i)) continue;
    push([i], cols[i]!);
    if (maxJets < 2) continue;
    for (let j = i + 1; j < 6; j++) {
      if (isolated.has(j)) continue;
      push([i, j], vadd(cols[i]!, cols[j]!));
    }
  }
  const aligned = cands.filter((c) => c.align >= 0.5);
  const pool = aligned.length ? aligned : cands.filter((c) => c.align >= 0.2);
  pool.sort((a, b) => {
    if (Math.abs(b.align - a.align) > 0.08) return b.align - a.align;
    return vdot(b.tau, eWant) - vdot(a.tau, eWant);
  });
  const best = pool[0] ?? cands[0];
  if (!best || best.align < 0.04) {
    const alloc = allocateTorque(vscale(eWant, 40), cols, isolated, { wantNGate: 0.5 });
    if (alloc.ids.length) {
      for (const id of alloc.ids.slice(0, maxJets)) pulse[id] = width;
    }
    return pulse;
  }
  for (const id of best.ids.slice(0, maxJets)) pulse[id] = width;
  return pulse;
}

function rigidStep(
  s: PlanState,
  Icm: ReturnType<typeof massState>["Icm"],
  Iinv: ReturnType<typeof minv3>,
  tau: Vec3,
  mdot: number,
  dt: number,
) {
  const Iw = mv(Icm, s.w);
  const alpha = mv(Iinv, vsub(tau, vcross(s.w, Iw)));
  const qd = qdot(s.q, s.w);
  s.q = qnormalize([
    s.q[0] + dt * qd[0],
    s.q[1] + dt * qd[1],
    s.q[2] + dt * qd[2],
    s.q[3] + dt * qd[3],
  ]);
  s.w = [s.w[0] + dt * alpha[0], s.w[1] + dt * alpha[1], s.w[2] + dt * alpha[2]];
  if (mdot > 0) s.fuel = Math.max(0, s.fuel - mdot * dt);
}

export function propagateHorizon(
  cfg: PublicConfig,
  st0: PlanState,
  cols: Vec3[],
  isolated: Set<number>,
  Icm: ReturnType<typeof massState>["Icm"],
  Iinv: ReturnType<typeof minv3>,
  first: number[],
  eta: number,
  alphaMax: number,
  horizon: number,
  opts?: PlannerOpts,
): PlanState {
  const s = copyPlan(st0);
  const dt = 0.1;
  const delay = opts?.ignoreDelay ? 0 : cfg.commandDelay;
  const ispG0 = cfg.isp * cfg.g0;
  const n = Math.max(1, Math.round(horizon / dt));
  const firstTau = torqueFromPulse(cols, first, CTRL_DT);
  const firstMdot = mdotOf(first, eta, cfg.maxThrust, ispG0, CTRL_DT);
  const firstEnd = delay + CTRL_DT;
  for (let k = 0; k < n; k++) {
    const t = k * dt;
    let tau: Vec3 = [0, 0, 0];
    let mdot = 0;
    if (firstMdot > 0 && t + 1e-9 >= delay && t < firstEnd) {
      tau = firstTau;
      mdot = firstMdot;
    } else if (t + 1e-9 >= firstEnd) {
      const cmd = switchCommand(cfg, s.q, s.w, alphaMax, opts);
      if (cmd.mustFire && s.fuel > FUEL_STOP) {
        const p = pulseAlongWant(cmd.want, cols, isolated, cmd.width, cmd.maxJets);
        tau = torqueFromPulse(cols, p, CTRL_DT);
        mdot = mdotOf(p, eta, cfg.maxThrust, ispG0, CTRL_DT);
      }
    }
    rigidStep(s, Icm, Iinv, tau, mdot, dt);
  }
  return s;
}

function scorePredicted(
  cfg: PublicConfig,
  pred: PlanState,
  usedNow: number,
  _cmd: SwitchCmd,
): PlanScore {
  const attDeg = deg(attitudeErrorAngle(pred.q, cfg.qTarget));
  const omega = vnorm(pred.w);
  const fuelPen = pred.fuel < FUEL_FLOOR ? (FUEL_FLOOR - pred.fuel) * 1e8 : 0;
  const inside = attDeg < ATT_GATE_DEG && omega < RATE_GATE && pred.fuel >= FUEL_FLOOR;
  if (inside) {
    return { lex: usedNow, fuel: pred.fuel, omega, attDeg, used: usedNow };
  }
  const ratePen = Math.max(0, omega - RATE_GATE) * 8e4 + 40 * omega;
  const attPen = Math.max(0, attDeg - ATT_GATE_DEG) * 2e3 + attDeg;
  const lex = fuelPen + ratePen + attPen + 4 * usedNow;
  return { lex, fuel: pred.fuel, omega, attDeg, used: usedNow };
}

export function candidatePulses(
  cfg: PublicConfig,
  cols: Vec3[],
  isolated: Set<number>,
  cmd: SwitchCmd,
  fuel: number,
): PulseOption[] {
  const out: PulseOption[] = [];
  out.push({ pulse: emptyPulse(), label: "coast" });
  if (cmd.phase === "hold" || cmd.phase === "coast") return out;
  if (fuel < FUEL_STOP) return out;

  const widths = new Set<number>([cmd.width]);
  if (cmd.phase === "settle") {
    widths.add(MIN_PULSE);
    widths.add(2 * MIN_PULSE);
  } else {
    widths.add(2 * MIN_PULSE);
    widths.add(CTRL_DT);
  }
  for (const width of widths) {
    out.push({
      pulse: pulseAlongWant(cmd.want, cols, isolated, width, cmd.maxJets),
      label: `${cmd.phase}_${width}_j${cmd.maxJets}`,
    });
    if (cmd.maxJets === 2) {
      out.push({
        pulse: pulseAlongWant(cmd.want, cols, isolated, width, 1),
        label: `${cmd.phase}_${width}_j1`,
      });
    }
  }
  const seen = new Set<string>();
  const filtered = out.filter((o) => {
    if (nOn(o.pulse) > 2) return false;
    const k = o.pulse.join(",");
    if (seen.has(k)) return false;
    seen.add(k);
    return true;
  });
  if (filtered.length === 0) filtered.push({ pulse: emptyPulse(), label: "coast" });
  return filtered;
}

export function saturatedPulse(
  want: Vec3,
  cols: Vec3[],
  isolated: Set<number>,
  maxJets: 1 | 2,
  width = CTRL_DT,
): [number, number, number, number, number, number] {
  return pulseAlongWant(want, cols, isolated, width, maxJets);
}

export function choosePulse(
  cfg: PublicConfig,
  est: Estimate,
  isolated: Set<number>,
  t: number,
  horizonOrOpts: number | PlannerOpts = HORIZON_S,
): [number, number, number, number, number, number] {
  const opts: PlannerOpts = typeof horizonOrOpts === "number" ? { horizon: horizonOrOpts } : horizonOrOpts;
  const cols = torqueColumns(cfg, est.s, est.th1, est.th2, est.fuel, est.etaT);
  const ms = massState(cfg, est.s, est.th1, est.th2, est.fuel);
  const Iinv = minv3(ms.Icm);
  const alphaMax = estimateAlphaMax(cols, isolated, Iinv);
  const cmd = switchCommand(cfg, est.q, est.w, alphaMax, opts);
  const aUse = Math.max(0.01, alphaMax * (opts.alphaScale ?? 0.5));
  const Hdef = clamp(2.0 * Math.sqrt(Math.max(cmd.theta, 0.02) / aUse), 5, 12);
  const H = clamp(opts.horizon ?? Hdef, 5, 15);

  if (cmd.phase === "hold" || cmd.phase === "coast") return emptyPulse();
  if (est.fuel < FUEL_STOP) return emptyPulse();

  const reverse = cmd.phase === "accel" && cmd.wPar > 0.008;
  const lean = est.fuel < FUEL_FLOOR + 0.45 && !reverse;
  const jets: 1 | 2 = lean ? 1 : cmd.maxJets;

  if (cmd.mustFire && (cmd.phase === "detumble" || cmd.phase === "accel" || cmd.phase === "brake" || cmd.phase === "perp")) {
    return saturatedPulse(cmd.want, cols, isolated, jets, cmd.width);
  }

  // Do not let the 8 s predictor override a good eigenaxis coast.
  if (cmd.attDeg > 12 && cmd.wPar < -0.02 && cmd.wPerpN < 0.05 && vnorm(est.w) < (opts.wCap ?? 0.04) * 1.35) {
    return emptyPulse();
  }

  const sinceFire = t - (opts.lastFireT ?? -1e9);
  if (sinceFire < cfg.commandDelay + 0.5 * MIN_PULSE && cmd.phase === "settle" && vnorm(est.w) < 0.02) {
    return emptyPulse();
  }

  const st: PlanState = {
    q: [...est.q],
    w: [...est.w],
    fuel: est.fuel,
    s: est.s,
    th1: est.th1,
    th2: est.th2,
  };

  const ispG0 = cfg.isp * cfg.g0;
  const catalogue = candidatePulses(cfg, cols, isolated, cmd, est.fuel);

  let best = cmd.mustFire
    ? pulseAlongWant(cmd.want, cols, isolated, cmd.width, jets)
    : emptyPulse();
  let bestScore = Infinity;
  for (const o of catalogue) {
    const onTime = o.pulse.reduce((s, w) => s + w, 0);
    const usedNow = (onTime * est.etaT * cfg.maxThrust) / ispG0;
    if (est.fuel - usedNow < FUEL_STOP && onTime > 0) continue;
    const pred = propagateHorizon(
      cfg, st, cols, isolated, ms.Icm, Iinv, o.pulse, est.etaT, alphaMax, H, opts,
    );
    const sc = scorePredicted(cfg, pred, usedNow, cmd);
    if (sc.lex < bestScore) {
      bestScore = sc.lex;
      best = o.pulse;
    }
  }
  if (cmd.mustFire && nOn(best) === 0 && cmd.attDeg > 1.2) {
    best = saturatedPulse(cmd.want, cols, isolated, jets, cmd.width);
  }
  return best;
}

export function sliderHold(est: Estimate, cfg: PublicConfig): number {
  const wn = 0.9;
  const z = 1.2;
  const Kp = cfg.sliderMass * wn * wn;
  const Kd = 2 * z * wn * cfg.sliderMass;
  let Fs = -Kp * (est.s - cfg.sTarget) - Kd * est.sd;
  if (Math.abs(est.s) > 1.35 && est.s * est.sd > 0) Fs = -Math.sign(est.sd) * cfg.sliderForceMax;
  return clamp(Fs, -cfg.sliderForceMax, cfg.sliderForceMax);
}

export { pulseImpulse };
void vdot;
void HORIZON_S;
void FUEL_FLOOR;
