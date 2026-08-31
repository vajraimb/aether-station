/**
 * Conservation, action/reaction, and collision audits of the physics kernel.
 * No controller, no sensors, no thrusters unless explicitly enabled.
 */
import {
  actuatorPower,
  angularMomentumCmB,
  applyCollision,
  dampingPower,
  energyParts,
  integrateWithCollision,
  kineticPlusPotential,
  lastQuaternionNormRaw,
  linearMomentumI,
  massState,
  rk4Step,
  totalAngularMomentumI,
  type ForceInput,
} from "./dynamics";
import { defaultPublicConfig } from "./constants";
import { qnormalize, qRotate, vnorm, vsub, type Vec3 } from "./math3d";
import type { PublicConfig, SimState } from "./types";
import { writeJson, writeText } from "./io";

const U_CONSERVATIVE: ForceInput = {
  FthrB: [0, 0, 0],
  tauThrO: [0, 0, 0],
  Fslider: 0,
  c1: 0,
  c2: 0,
  k12: 0.318,
};

/** Smooth IC: modest spin, slider starts at 0 so 180 s stays off the stops. */
export function smoothState(cfg: PublicConfig): SimState {
  const st: SimState = {
    t: 0,
    rI: [0, 0, 0],
    vI: [0, 0, 0],
    rCmI: [0, 0, 0],
    vCmI: [0, 0, 0],
    q: qnormalize([0.92388, 0.22094, -0.22094, 0.22094]),
    w: [0.045, -0.0018, 0.002],
    s: 0.0,
    sd: 0.0,
    th1: 0.10,
    th1d: 0.03,
    th2: -0.07,
    th2d: -0.02,
    fuel: 5,
  };
  const ms = massState(cfg, st.s, st.th1, st.th2, st.fuel);
  st.rCmI = qRotate(st.q, ms.rCmB);
  return st;
}

/** Richer IC for short-horizon RK4 order tests (4 s, no wall contact). */
export function smoothStateOrder(cfg: PublicConfig): SimState {
  const st: SimState = {
    t: 0,
    rI: [0, 0, 0],
    vI: [0, 0, 0],
    rCmI: [0, 0, 0],
    vCmI: [0, 0, 0],
    q: qnormalize([0.92388, 0.22094, -0.22094, 0.22094]),
    w: [0.07, -0.012, 0.015],
    s: 0.05,
    sd: 0.0,
    th1: 0.22,
    th1d: 0.08,
    th2: -0.16,
    th2d: -0.05,
    fuel: 5,
  };
  const ms = massState(cfg, st.s, st.th1, st.th2, st.fuel);
  st.rCmI = qRotate(st.q, ms.rCmB);
  return st;
}

export function collisionState(cfg: PublicConfig): SimState {
  const st: SimState = {
    t: 0,
    rI: [0, 0, 0],
    vI: [0, 0, 0],
    rCmI: [0, 0, 0],
    vCmI: [0, 0, 0],
    q: qnormalize([0.98, 0.1, 0.12, -0.08]),
    w: [0.04, 0.02, -0.03],
    s: 1.72,
    sd: 1.55,
    th1: 0.1,
    th1d: 0.02,
    th2: -0.08,
    th2d: -0.01,
    fuel: 5,
  };
  const ms = massState(cfg, st.s, st.th1, st.th2, st.fuel);
  st.rCmI = qRotate(st.q, ms.rCmB);
  return st;
}

export interface SmoothRun {
  dt: number;
  duration: number;
  steps: number;
  energy0: ReturnType<typeof energyParts>;
  energy1: ReturnType<typeof energyParts>;
  energyRel: number;
  energyAbs: number;
  H0: Vec3;
  H1: Vec3;
  angularMomentumRel: number;
  angularMomentumAbs: number;
  linearMomentumAbs: number;
  maxSlider: number;
  minSlider: number;
  collided: boolean;
  qNormRawMax: number;
  qFinal: number[];
  wFinal: number[];
  sFinal: number;
}

export function runSmooth(dt: number, duration: number, rich = false): SmoothRun {
  const cfg = defaultPublicConfig({ dtMax: dt, fluidPresent: true });
  let st = rich ? smoothStateOrder(cfg) : smoothState(cfg);
  const E0 = energyParts(cfg, st, U_CONSERVATIVE.k12);
  const H0 = totalAngularMomentumI(cfg, st, U_CONSERVATIVE.k12);
  const P0 = linearMomentumI(cfg, st);
  let maxS = Math.abs(st.s);
  let minS = st.s;
  let maxQ = 0;
  let collided = false;
  const n = Math.round(duration / dt);
  for (let i = 0; i < n; i++) {
    const col = integrateWithCollision(cfg, st, U_CONSERVATIVE, dt);
    st = col.state;
    if (col.collided) collided = true;
    maxS = Math.max(maxS, Math.abs(st.s));
    minS = Math.min(minS, st.s);
    maxQ = Math.max(maxQ, Math.abs(lastQuaternionNormRaw(st) - 1));
  }
  const E1 = energyParts(cfg, st, U_CONSERVATIVE.k12);
  const H1 = totalAngularMomentumI(cfg, st, U_CONSERVATIVE.k12);
  const P1 = linearMomentumI(cfg, st);
  const dH = vnorm(vsub(H1, H0));
  return {
    dt,
    duration,
    steps: n,
    energy0: E0,
    energy1: E1,
    energyRel: Math.abs(E1.total - E0.total) / (Math.abs(E0.total) + 1e-12),
    energyAbs: E1.total - E0.total,
    H0,
    H1,
    angularMomentumRel: dH / (vnorm(H0) + 1e-12),
    angularMomentumAbs: dH,
    linearMomentumAbs: vnorm(vsub(P1, P0)),
    maxSlider: maxS,
    minSlider: minS,
    collided,
    qNormRawMax: maxQ,
    qFinal: [...st.q],
    wFinal: [...st.w],
    sFinal: st.s,
  };
}

function stateDistance(a: SmoothRun, b: SmoothRun): number {
  const dq = vnorm([
    a.qFinal[1]! - b.qFinal[1]!,
    a.qFinal[2]! - b.qFinal[2]!,
    a.qFinal[3]! - b.qFinal[3]!,
  ]);
  const dw = vnorm([
    a.wFinal[0]! - b.wFinal[0]!,
    a.wFinal[1]! - b.wFinal[1]!,
    a.wFinal[2]! - b.wFinal[2]!,
  ]);
  return dq + dw + Math.abs(a.sFinal - b.sFinal);
}

export function observedOrder(eCoarse: number, eFine: number, ratio = 2): number {
  if (eFine < 1e-18 || eCoarse < 1e-18) return Number.NaN;
  return Math.log(eCoarse / eFine) / Math.log(ratio);
}

export interface CollisionAudit {
  dt: number;
  tHit: number | null;
  impactSpeed: number | null;
  penetrated: boolean;
  sMin: number;
  sMax: number;
  HrelJump: number;
  HinertialJump: number;
  energyLoss: number;
  naiveLoss: number;
  H0: Vec3;
  H1: Vec3;
}

export function runCollision(dt: number, duration = 0.4): CollisionAudit {
  const cfg = defaultPublicConfig({ dtMax: dt, fluidPresent: true });
  let st = collisionState(cfg);
  const H0 = totalAngularMomentumI(cfg, st, U_CONSERVATIVE.k12);
  let tHit: number | null = null;
  let impact: number | null = null;
  let HrelJump = 0;
  let HinertialJump = 0;
  let energyLoss = 0;
  let sMin = st.s;
  let sMax = st.s;
  let penetrated = false;
  const n = Math.round(duration / dt);
  for (let i = 0; i < n; i++) {
    const col = integrateWithCollision(cfg, st, U_CONSERVATIVE, dt);
    st = col.state;
    sMin = Math.min(sMin, st.s);
    sMax = Math.max(sMax, st.s);
    if (st.s > cfg.sliderMax + 1e-9 || st.s < cfg.sliderMin - 1e-9) penetrated = true;
    if (col.collided && tHit === null) {
      tHit = col.tHit ?? st.t;
      impact = col.impactSpeed;
      HrelJump = col.angularMomentumJump ?? 0;
      energyLoss = col.energyLoss ?? 0;
      const Hhit = totalAngularMomentumI(cfg, st, U_CONSERVATIVE.k12);
      HinertialJump = vnorm(vsub(Hhit, H0));
    }
  }
  const H1 = totalAngularMomentumI(cfg, st, U_CONSERVATIVE.k12);
  const ms = massState(cfg, cfg.sliderMax, 0, 0, 5);
  const naiveLoss = 0.5 * ms.ms * (1 - cfg.restitution * cfg.restitution) * (impact ?? 0) ** 2;
  return {
    dt,
    tHit,
    impactSpeed: impact,
    penetrated,
    sMin,
    sMax,
    HrelJump,
    HinertialJump,
    energyLoss,
    naiveLoss,
    H0,
    H1,
  };
}

export interface LedgerRow {
  t: number;
  E: number;
  Ttrans: number;
  Trot: number;
  Tgyro: number;
  Trel: number;
  Vslosh: number;
  Vcoup: number;
  Hx: number;
  Hy: number;
  Hz: number;
  Hn: number;
  Px: number;
  Py: number;
  Pz: number;
  s: number;
  sd: number;
  th1: number;
  th2: number;
}

function ledgerRow(cfg: PublicConfig, st: SimState, k12: number): LedgerRow {
  const E = energyParts(cfg, st, k12);
  const H = totalAngularMomentumI(cfg, st, k12);
  const P = linearMomentumI(cfg, st);
  return {
    t: st.t,
    E: E.total,
    Ttrans: E.translational,
    Trot: E.rotational,
    Tgyro: E.gyrostat,
    Trel: E.relative,
    Vslosh: E.sloshPotential,
    Vcoup: E.couplingPotential,
    Hx: H[0],
    Hy: H[1],
    Hz: H[2],
    Hn: vnorm(H),
    Px: P[0],
    Py: P[1],
    Pz: P[2],
    s: st.s,
    sd: st.sd,
    th1: st.th1,
    th2: st.th2,
  };
}

function csv(rows: LedgerRow[]): string {
  if (rows.length === 0) return "";
  const keys = Object.keys(rows[0]!) as (keyof LedgerRow)[];
  const head = keys.join(",");
  const body = rows.map((r) => keys.map((k) => Number(r[k]).toExponential(12)).join(",")).join("\n");
  return `${head}\n${body}\n`;
}

export function writeLedgers(duration = 8, dt = 0.005) {
  const cfg = defaultPublicConfig({ dtMax: dt, fluidPresent: true });
  let st = smoothState(cfg);
  const energyRows: LedgerRow[] = [];
  const momRows: LedgerRow[] = [];
  const n = Math.round(duration / dt);
  const stride = Math.max(1, Math.round(0.02 / dt));
  energyRows.push(ledgerRow(cfg, st, U_CONSERVATIVE.k12));
  momRows.push(ledgerRow(cfg, st, U_CONSERVATIVE.k12));
  for (let i = 0; i < n; i++) {
    st = integrateWithCollision(cfg, st, U_CONSERVATIVE, dt).state;
    if ((i + 1) % stride === 0) {
      const row = ledgerRow(cfg, st, U_CONSERVATIVE.k12);
      energyRows.push(row);
      momRows.push(row);
    }
  }
  writeText("outputs/energy-ledger.csv", csv(energyRows));
  writeText("outputs/momentum-ledger.csv", csv(momRows));
  return { energyRows: energyRows.length, momRows: momRows.length };
}

export interface ReactionAudit {
  sliderNetForce: number;
  sliderNetTorqueCm: number;
  slosh1NetForce: number;
  slosh1NetTorqueCm: number;
  slosh2NetForce: number;
  slosh2NetTorqueCm: number;
  dHdtResidual: number;
  dEdtResidualConservative: number;
  dEdtResidualDamped: number;
  dEdtResidualActuated: number;
}

function finiteDiffH(cfg: PublicConfig, st: SimState, u: ForceInput, h = 1e-6): Vec3 {
  const a = rk4Step(st, cfg, u, h);
  const H0 = totalAngularMomentumI(cfg, st, u.k12);
  const H1 = totalAngularMomentumI(cfg, a, u.k12);
  return [(H1[0] - H0[0]) / h, (H1[1] - H0[1]) / h, (H1[2] - H0[2]) / h];
}

function finiteDiffE(cfg: PublicConfig, st: SimState, u: ForceInput, h = 1e-6): number {
  const a = rk4Step(st, cfg, u, h);
  const E0 = kineticPlusPotential(cfg, st, u.k12);
  const E1 = kineticPlusPotential(cfg, a, u.k12);
  return (E1 - E0) / h;
}

export function runReactionAudit(): ReactionAudit {
  const cfg = defaultPublicConfig({ fluidPresent: true });
  const st = smoothState(cfg);
  st.sd = 0.2;
  st.th1d = 0.15;
  st.th2d = -0.11;

  const u0: ForceInput = { ...U_CONSERVATIVE, Fslider: 0, c1: 0, c2: 0 };
  const us: ForceInput = { ...u0, Fslider: 180 };
  const P0 = linearMomentumI(cfg, st);
  const P1 = linearMomentumI(cfg, rk4Step(st, cfg, us, 1e-5));
  const sliderNetForce = vnorm(vsub(P1, P0)) / 1e-5;
  const dHs = finiteDiffH(cfg, st, us);
  const sliderNetTorqueCm = vnorm(dHs);

  const u1: ForceInput = { ...u0, k12: 0.318 };
  const st1: SimState = { ...st, th1: 0.4, th2: 0, th1d: 0, th2d: 0, sd: 0, w: [0, 0, 0] };
  const ms1 = massState(cfg, st1.s, st1.th1, st1.th2, st1.fuel);
  st1.rCmI = qRotate(st1.q, ms1.rCmB);
  const Pth0 = linearMomentumI(cfg, st1);
  const Pth1 = linearMomentumI(cfg, rk4Step(st1, cfg, u1, 1e-5));
  const slosh1NetForce = vnorm(vsub(Pth1, Pth0)) / 1e-5;
  const slosh1NetTorqueCm = vnorm(finiteDiffH(cfg, st1, u1));

  const st2: SimState = { ...st, th1: 0, th2: 0.35, th1d: 0, th2d: 0, sd: 0, w: [0, 0, 0] };
  const ms2 = massState(cfg, st2.s, st2.th1, st2.th2, st2.fuel);
  st2.rCmI = qRotate(st2.q, ms2.rCmB);
  const P2a = linearMomentumI(cfg, st2);
  const P2b = linearMomentumI(cfg, rk4Step(st2, cfg, u1, 1e-5));
  const slosh2NetForce = vnorm(vsub(P2b, P2a)) / 1e-5;
  const slosh2NetTorqueCm = vnorm(finiteDiffH(cfg, st2, u1));

  const dH0 = finiteDiffH(cfg, st, u0);
  const dHdtResidual = vnorm(dH0);

  const dE0 = finiteDiffE(cfg, st, u0);
  const dEdtResidualConservative = Math.abs(dE0);

  const ud: ForceInput = { ...u0, c1: 0.137, c2: 0.091 };
  const dEd = finiteDiffE(cfg, st, ud);
  const Pd = dampingPower(cfg, st, ud.c1, ud.c2);
  const dEdtResidualDamped = Math.abs(dEd - Pd);

  const ua: ForceInput = { ...u0, Fslider: 120 };
  const dEa = finiteDiffE(cfg, st, ua);
  const Pa = actuatorPower(st, ua, cfg);
  const dEdtResidualActuated = Math.abs(dEa - Pa);

  void angularMomentumCmB;
  return {
    sliderNetForce,
    sliderNetTorqueCm,
    slosh1NetForce,
    slosh1NetTorqueCm,
    slosh2NetForce,
    slosh2NetTorqueCm,
    dHdtResidual,
    dEdtResidualConservative,
    dEdtResidualDamped,
    dEdtResidualActuated,
  };
}

export function runFullPhysicsAudit() {
  const dts = [0.01, 0.005, 0.0025, 0.00125, 0.000625];
  const duration = 180;
  const orderDuration = 4;
  const smooth180 = dts.map((dt) => runSmooth(dt, duration));
  const smoothOrder = dts.map((dt) => runSmooth(dt, orderDuration, true));
  const ref = smoothOrder[smoothOrder.length - 1]!;
  const errors = smoothOrder.map((r) => (r.dt === ref.dt ? 0 : stateDistance(r, ref)));
  const orders: number[] = [];
  for (let i = 0; i < errors.length - 2; i++) {
    orders.push(observedOrder(errors[i]!, errors[i + 1]!));
  }
  const collisions = dts.map((dt) => runCollision(dt, 0.5));
  const reaction = runReactionAudit();
  writeLedgers(8, 0.005);

  const worstE = Math.max(...smooth180.map((r) => r.energyRel));
  const worstH = Math.max(...smooth180.map((r) => r.angularMomentumRel));
  const minOrder = Math.min(...orders.filter((x) => Number.isFinite(x)));
  const noHit = smooth180.every((r) => !r.collided && r.maxSlider < 1.75);
  const noPen = collisions.every((c) => !c.penetrated);
  const tHits = collisions.map((c) => c.tHit ?? NaN);
  const tConv = Math.abs((tHits[0] ?? 0) - (tHits[tHits.length - 1] ?? 0));
  const Hjump = Math.max(...collisions.map((c) => c.HrelJump));

  const report = {
    smooth180,
    smoothOrder,
    trajectoryErrorsVsFinest: errors,
    observedOrders_coarse_to_next: orders,
    minObservedOrder: minOrder,
    collisions,
    collisionEventTimeSpan_s: tConv,
    reaction,
    gates: {
      observedOrder: { value: minOrder, pass: minOrder >= 3.7, target: ">= 3.7" },
      angularMomentumRel_180s: { value: worstH, pass: worstH < 1e-4, target: "< 1e-4" },
      energyRel_180s: { value: worstE, pass: worstE < 1e-4, target: "< 1e-4" },
      sliderOffStops: { value: noHit, pass: noHit, target: "no collision" },
      collisionNoPenetration: { value: noPen, pass: noPen, target: "true" },
      collisionHjump: { value: Hjump, pass: Hjump < 1e-6, target: "< 1e-6" },
      reactionForce: {
        value: Math.max(reaction.sliderNetForce, reaction.slosh1NetForce, reaction.slosh2NetForce),
        pass:
          reaction.sliderNetForce < 1e-4 &&
          reaction.slosh1NetForce < 1e-4 &&
          reaction.slosh2NetForce < 1e-4,
        target: "internal F_ext ≈ 0",
      },
      reactionTorque: {
        value: Math.max(reaction.sliderNetTorqueCm, reaction.dHdtResidual),
        pass: reaction.dHdtResidual < 5e-4 && reaction.sliderNetTorqueCm < 5e-3,
        target: "dH/dt ≈ 0 with τ_ext = 0",
      },
      energyBalance: {
        value: Math.max(
          reaction.dEdtResidualConservative,
          reaction.dEdtResidualDamped,
          reaction.dEdtResidualActuated,
        ),
        pass:
          reaction.dEdtResidualConservative < 5e-3 &&
          reaction.dEdtResidualDamped < 5e-2 &&
          reaction.dEdtResidualActuated < 5e-2,
        target: "dE/dt matches power",
      },
    },
  };
  writeJson("outputs/conservation.json", {
    dt: 0.005,
    duration,
    energyRel: smooth180.find((r) => r.dt === 0.005)?.energyRel,
    angularMomentumRel: smooth180.find((r) => r.dt === 0.005)?.angularMomentumRel,
    linearMomentumDrift: smooth180.find((r) => r.dt === 0.005)?.linearMomentumAbs,
    energyParts0: smooth180.find((r) => r.dt === 0.005)?.energy0,
    energyParts1: smooth180.find((r) => r.dt === 0.005)?.energy1,
    qNormRawMax: smooth180.find((r) => r.dt === 0.005)?.qNormRawMax,
  });
  writeJson("outputs/convergence.json", {
    duration: orderDuration,
    errors,
    orders,
    minObservedOrder: minOrder,
    collisions,
  });
  writeJson("outputs/reaction-audit.json", reaction);
  return report;
}
