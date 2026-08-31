/**
 * Torque allocation under the RCS constraints:
 *   at most two jets, pulse width 0 or >= minPulse, duty in [0, 1] of the
 *   controller period. Independent widths for a pair (least-squares).
 *
 * Used by the observation-only controller and the oracle. Does not read
 * scenario, wall-clock fault time, or thruster identity constants.
 */
import {
  clamp,
  vadd,
  vdot,
  vnorm,
  vscale,
  vsub,
  type Vec3,
} from "./math3d";
import { massState } from "./dynamics";
import { THRUSTERS } from "./thrusters";
import { CTRL_DT, MIN_PULSE } from "./constants";
import type { PublicConfig } from "./types";

export function torqueColumns(
  cfg: PublicConfig,
  s: number,
  th1: number,
  th2: number,
  fuel: number,
  eta: number,
): Vec3[] {
  const ms = massState(cfg, s, th1, th2, fuel);
  return THRUSTERS.map((g) => {
    const Fi = vscale(g.dir, eta * cfg.maxThrust);
    const r: Vec3 = [g.pos[0] - ms.rCmB[0], g.pos[1] - ms.rCmB[1], g.pos[2] - ms.rCmB[2]];
    return [
      r[1] * Fi[2] - r[2] * Fi[1],
      r[2] * Fi[0] - r[0] * Fi[2],
      r[0] * Fi[1] - r[1] * Fi[0],
    ];
  });
}

export function pulseTorque(cols: Vec3[], pulse: number[], etaScale = 1): Vec3 {
  let tau: Vec3 = [0, 0, 0];
  for (let i = 0; i < cols.length; i++) {
    const w = pulse[i] ?? 0;
    if (w <= 0) continue;
    tau = vadd(tau, vscale(cols[i]!, etaScale * (w / CTRL_DT)));
  }
  return tau;
}

export function pulseImpulse(cols: Vec3[], pulse: number[]): Vec3 {
  let H: Vec3 = [0, 0, 0];
  for (let i = 0; i < cols.length; i++) {
    const w = pulse[i] ?? 0;
    if (w <= 0) continue;
    H = vadd(H, vscale(cols[i]!, w));
  }
  return H;
}

function fitOne(c: Vec3, b: Vec3): number {
  const n2 = vdot(c, c);
  if (n2 < 1e-12) return 0;
  return vdot(c, b) / n2;
}

function fitTwo(c0: Vec3, c1: Vec3, b: Vec3): [number, number] {
  const a00 = vdot(c0, c0);
  const a01 = vdot(c0, c1);
  const a11 = vdot(c1, c1);
  const det = a00 * a11 - a01 * a01;
  if (Math.abs(det) < 1e-10) return [fitOne(c0, b), 0];
  const r0 = vdot(c0, b);
  const r1 = vdot(c1, b);
  return [(a11 * r0 - a01 * r1) / det, (a00 * r1 - a01 * r0) / det];
}

function snapDuty(d: number, minDuty: number): number {
  if (d <= 1e-9) return 0;
  if (d >= 1) return 1;
  if (d < minDuty) return d >= 0.5 * minDuty ? minDuty : 0;
  return d;
}

function residual(cols: Vec3[], ids: number[], duties: number[], tauDes: Vec3): number {
  let tau: Vec3 = [0, 0, 0];
  for (let k = 0; k < ids.length; k++) {
    tau = vadd(tau, vscale(cols[ids[k]!]!, duties[k]!));
  }
  return vnorm(vsub(tau, tauDes));
}

export interface AllocResult {
  pulse: [number, number, number, number, number, number];
  ids: number[];
  delivered: Vec3;
  residual: number;
  align: number;
}

/**
 * Enumerate 1- and 2-jet subsets (skipping isolated ids). Fit independent
 * duties by least squares, snap to {0} ∪ [minPulse, CTRL_DT], pick the
 * combination with smallest residual then smallest fuel.
 */
export function allocateTorque(
  tauDes: Vec3,
  cols: Vec3[],
  isolated: Set<number>,
  opts?: { minPulse?: number; period?: number; wantNGate?: number },
): AllocResult {
  const empty: AllocResult = {
    pulse: [0, 0, 0, 0, 0, 0],
    ids: [],
    delivered: [0, 0, 0],
    residual: vnorm(tauDes),
    align: 0,
  };
  const period = opts?.period ?? CTRL_DT;
  const minPulse = opts?.minPulse ?? MIN_PULSE;
  const minDuty = minPulse / period;
  const wantN = vnorm(tauDes);
  if (wantN < (opts?.wantNGate ?? 0.05)) return empty;

  const live: number[] = [];
  for (let i = 0; i < 6; i++) if (!isolated.has(i) && cols[i]) live.push(i);

  let bestPulse: [number, number, number, number, number, number] = [0, 0, 0, 0, 0, 0];
  let bestIds: number[] = [];
  let bestCost = Infinity;
  let bestAlign = 0;
  let bestFuel = Infinity;

  const consider = (ids: number[], duties: number[]) => {
    const snapped = duties.map((d) => snapDuty(clamp(d, 0, 1), minDuty));
    let nOn = 0;
    for (const d of snapped) if (d > 0) nOn += 1;
    if (nOn === 0 || nOn > 2) return;
    const res = residual(cols, ids, snapped, tauDes);
    let tau: Vec3 = [0, 0, 0];
    let fuel = 0;
    const pulse: [number, number, number, number, number, number] = [0, 0, 0, 0, 0, 0];
    const onIds: number[] = [];
    for (let k = 0; k < ids.length; k++) {
      const d = snapped[k]!;
      if (d <= 0) continue;
      pulse[ids[k]!] = d * period;
      tau = vadd(tau, vscale(cols[ids[k]!]!, d));
      fuel += d;
      onIds.push(ids[k]!);
    }
    const n = vnorm(tau);
    const align = n > 1e-9 ? vdot(tau, tauDes) / (n * wantN) : 0;
    if (align < 0.02) return;
    // Prefer residual; break ties on fuel then alignment.
    const cost = res + 0.15 * fuel * wantN;
    if (cost < bestCost - 1e-9 || (Math.abs(cost - bestCost) < 1e-9 && fuel < bestFuel)) {
      bestCost = cost;
      bestFuel = fuel;
      bestPulse = pulse;
      bestIds = onIds;
      bestAlign = align;
    }
  };

  for (let a = 0; a < live.length; a++) {
    const i = live[a]!;
    const d = fitOne(cols[i]!, tauDes);
    consider([i], [d]);
    for (let b = a + 1; b < live.length; b++) {
      const j = live[b]!;
      let [d0, d1] = fitTwo(cols[i]!, cols[j]!, tauDes);
      if (d0 < 0 && d1 < 0) {
        d0 = 0;
        d1 = 0;
      } else if (d0 < 0) {
        d0 = 0;
        d1 = fitOne(cols[j]!, tauDes);
      } else if (d1 < 0) {
        d1 = 0;
        d0 = fitOne(cols[i]!, tauDes);
      }
      consider([i, j], [d0, d1]);
    }
  }

  if (bestIds.length === 0) return empty;
  return {
    pulse: bestPulse,
    ids: bestIds,
    delivered: pulseTorque(cols, bestPulse),
    residual: bestCost,
    align: bestAlign,
  };
}

export function sliderForceCommand(
  s: number,
  sd: number,
  cfg: PublicConfig,
): number {
  const wn = 1.05;
  const z = 1.15;
  const Kp = cfg.sliderMass * wn * wn;
  const Kd = 2 * z * wn * cfg.sliderMass;
  let Fs = -Kp * (s - cfg.sTarget) - Kd * sd;
  const margin = cfg.sliderMax - Math.abs(s);
  if (margin < 0.35 && s * sd > 0) Fs = -Math.sign(sd) * cfg.sliderForceMax;
  if (Math.abs(s) > 1.55 && Math.abs(sd) > 0.08) Fs = -Math.sign(sd) * cfg.sliderForceMax;
  return clamp(Fs, -cfg.sliderForceMax, cfg.sliderForceMax);
}
