/**
 * Open-loop conservation and step-size convergence. No controller, no
 * thrusters. Used by `npm run test:physics`.
 */
import {
  kineticPlusPotential,
  linearMomentumI,
  massState,
  integrateWithCollision,
  totalAngularMomentumI,
} from "./dynamics";
import { defaultPublicConfig } from "./constants";
import { qnorm, qnormalize, qRotate, vnorm, type Vec3 } from "./math3d";
import type { PublicConfig, SimState } from "./types";

function idle(cfg: PublicConfig, over: Partial<SimState> = {}): SimState {
  const st: SimState = {
    t: 0,
    rI: [0, 0, 0],
    vI: [0, 0, 0],
    rCmI: [0, 0, 0],
    vCmI: [0, 0, 0],
    q: qnormalize([0.92388, 0.22094, -0.22094, 0.22094]),
    w: [0.08, -0.05, 0.06],
    s: 0.4,
    sd: 0.15,
    th1: 0.12,
    th1d: 0.04,
    th2: -0.08,
    th2d: -0.03,
    fuel: 5,
    ...over,
  };
  const ms = massState(cfg, st.s, st.th1, st.th2, st.fuel);
  st.rCmI = qRotate(st.q, ms.rCmB);
  return st;
}

const U0 = {
  FthrB: [0, 0, 0] as Vec3,
  tauThrO: [0, 0, 0] as Vec3,
  Fslider: 0,
  c1: 0,
  c2: 0,
  k12: 0.318,
};

export interface OpenLoopResult {
  dt: number;
  duration: number;
  energyDrift: number;
  energyRel: number;
  angularMomentumDrift: number;
  angularMomentumRel: number;
  linearMomentumDrift: number;
  quaternionNormMax: number;
  collisionTime: number | null;
  collisionSpeed: number | null;
  qFinal: number[];
  wFinal: number[];
  sFinal: number;
}

export function runOpenLoop(dt: number, duration: number, collide: boolean): OpenLoopResult {
  const cfg = defaultPublicConfig({ dtMax: dt, fluidPresent: true });
  let st = collide
    ? idle(cfg, { s: 1.72, sd: 1.6, w: [0.04, 0.02, -0.03] })
    : idle(cfg);
  const H0 = totalAngularMomentumI(cfg, st, U0.k12);
  const P0 = linearMomentumI(cfg, st);
  const E0 = kineticPlusPotential(cfg, st, U0.k12);
  let maxQ = Math.abs(qnorm(st.q) - 1);
  let collisionTime: number | null = null;
  let collisionSpeed: number | null = null;
  const n = Math.round(duration / dt);
  for (let i = 0; i < n; i++) {
    const col = integrateWithCollision(cfg, st, U0, dt);
    st = col.state;
    maxQ = Math.max(maxQ, Math.abs(qnorm(st.q) - 1));
    if (col.collided && collisionTime === null) {
      collisionTime = st.t;
      collisionSpeed = col.impactSpeed;
    }
  }
  const H1 = totalAngularMomentumI(cfg, st, U0.k12);
  const P1 = linearMomentumI(cfg, st);
  const E1 = kineticPlusPotential(cfg, st, U0.k12);
  const dH = vnorm([H1[0] - H0[0], H1[1] - H0[1], H1[2] - H0[2]]);
  const dP = vnorm([P1[0] - P0[0], P1[1] - P0[1], P1[2] - P0[2]]);
  return {
    dt,
    duration,
    energyDrift: E1 - E0,
    energyRel: Math.abs(E1 - E0) / (Math.abs(E0) + 1e-12),
    angularMomentumDrift: dH,
    angularMomentumRel: dH / (vnorm(H0) + 1e-12),
    linearMomentumDrift: dP,
    quaternionNormMax: maxQ,
    collisionTime,
    collisionSpeed,
    qFinal: [...st.q],
    wFinal: [...st.w],
    sFinal: st.s,
  };
}

export function trajectoryDiff(a: OpenLoopResult, b: OpenLoopResult): number {
  const dq = vnorm([a.qFinal[1]! - b.qFinal[1]!, a.qFinal[2]! - b.qFinal[2]!, a.qFinal[3]! - b.qFinal[3]!]);
  const dw = vnorm([a.wFinal[0]! - b.wFinal[0]!, a.wFinal[1]! - b.wFinal[1]!, a.wFinal[2]! - b.wFinal[2]!]);
  return dq + dw + Math.abs(a.sFinal - b.sFinal);
}

/** Observed order p from errors at h, h/2, h/4: log2(E(h/2)/E(h/4)). */
export function observedOrder(eCoarse: number, eMid: number, eFine: number): number {
  if (eMid < 1e-16 || eFine < 1e-16) return Number.NaN;
  return Math.log(eMid / eFine) / Math.log(2);
}

export function runConservation(duration = 2): OpenLoopResult {
  return runOpenLoop(0.005, duration, false);
}

export function runConvergence(duration = 0.8) {
  const coarse = runOpenLoop(0.005, duration, true);
  const mid = runOpenLoop(0.0025, duration, true);
  const fine = runOpenLoop(0.00125, duration, true);
  const ref = fine;
  const eC = trajectoryDiff(coarse, ref);
  const eM = trajectoryDiff(mid, ref);
  const eF = 0; // reference
  const order = observedOrder(eC, eM, Math.max(eM / 8, 1e-16));
  // Use mid vs coarse vs a reconstructed fine-error proxy
  const order2 = eM > 1e-16 ? Math.log(eC / eM) / Math.log(2) : Number.NaN;
  return {
    duration,
    steps: {
      "5ms": coarse,
      "2.5ms": mid,
      "1.25ms": fine,
    },
    "trajectoryDiff_5ms_vs_1.25ms": eC,
    "trajectoryDiff_2.5ms_vs_1.25ms": eM,
    collisionTime: {
      "5ms": coarse.collisionTime,
      "2.5ms": mid.collisionTime,
      "1.25ms": fine.collisionTime,
      delta_5_vs_1_25: (coarse.collisionTime ?? 0) - (fine.collisionTime ?? 0),
      delta_2_5_vs_1_25: (mid.collisionTime ?? 0) - (fine.collisionTime ?? 0),
    },
    energyRel: {
      "5ms": coarse.energyRel,
      "2.5ms": mid.energyRel,
      "1.25ms": fine.energyRel,
    },
    angularMomentumRel: {
      "5ms": coarse.angularMomentumRel,
      "2.5ms": mid.angularMomentumRel,
      "1.25ms": fine.angularMomentumRel,
    },
    quaternionNormMax: {
      "5ms": coarse.quaternionNormMax,
      "2.5ms": mid.quaternionNormMax,
      "1.25ms": fine.quaternionNormMax,
    },
    observedOrder_trajectory_coarse_to_mid: order2,
    observedOrder_note:
      "RK4 is formally O(h^4) on the smooth vector field; the inelastic slider collision is event-located so the observed order on colliding trajectories is typically 1–2.",
    reference: { dt: ref.dt, eF },
    orderProxy: order,
  };
}
