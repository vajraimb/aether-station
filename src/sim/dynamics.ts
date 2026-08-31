import {
  clamp,
  diag,
  madd,
  minv3,
  mscale,
  mv,
  pointInertia,
  qRotate,
  qRotateInv,
  qdot,
  qnormalize,
  vadd,
  vcross,
  vdot,
  vnorm,
  vscale,
  vsub,
  type Mat3,
  type Quat,
  type Vec3,
} from "./math3d";
import {
  DRY_INERTIA,
  DRY_MASS,
  M1_FRAC,
  M2_FRAC,
  OMEGA1,
  OMEGA2,
  SLIDER_MASS,
  TANK_R,
} from "./constants";
import type { PublicConfig, SimState } from "./types";

/**
 * Modelling notes
 * ----------------
 * Body origin O is the geometric centre of the cylinder, coinciding with the
 * dry-station CM. Slider runs on the body x-axis. Two equivalent pendulums
 * represent n=1 annular-tank slosh:
 *
 *   r1_B = L [sin θ1,  cos θ1, 0]     (swings in the XY plane, AM about −Z)
 *   r2_B = L [sin θ2,  0,  cos θ2]    (swings in the XZ plane, AM about +Y)
 *
 * Restoring ω_i² sin θ_i is a *tank-wall potential*, not gravity (there is
 * none). Coupling k12 shares a single potential V = I_eq k12 (1 − cos(θ1−θ2)).
 * Equal modal masses keep that potential energy-consistent.
 *
 * Approximations (internal forcing uses origin acceleration ≈ a_cm; CM offset
 * is retained in I_cm and τ_cm = τ_O − r_cm × F):
 *   - static fluid inertia is lumped into dry_inertia_B
 *   - fuel is a point mass at O (5 kg / 1050 kg)
 *   - products of inertia from the pendulums are kept
 *   - CM-recoil (m_int/M ~ 5 %) is dropped in the slider/slosh scalar EOM
 *     but retained in Euler through I(s,θ) and h_rel(θ̇)
 *
 * Action/reaction: pendulum restoring and damping act through the hinge at O;
 * they never appear as external torque. External torque is thrusters only.
 * Slider actuator is an internal pair along x (zero net force and, because
 * the line of action is the x-axis, zero torque about O).
 */

export function modalMasses(fluidMass: number): { m1: number; m2: number; mStatic: number } {
  if (fluidMass <= 0) return { m1: 0, m2: 0, mStatic: 0 };
  const m1 = M1_FRAC * fluidMass;
  const m2 = M2_FRAC * fluidMass;
  return { m1, m2, mStatic: fluidMass - m1 - m2 };
}

export function pendulumPos(theta: number, axis: 1 | 2, L: number): Vec3 {
  const s = Math.sin(theta);
  const c = Math.cos(theta);
  if (axis === 1) return [L * s, L * c, 0];
  return [L * s, 0, L * c];
}

export function pendulumETheta(theta: number, axis: 1 | 2): Vec3 {
  const s = Math.sin(theta);
  const c = Math.cos(theta);
  if (axis === 1) return [c, -s, 0];
  return [c, 0, -s];
}

export interface MassState {
  M: number;
  mRigid: number;
  m1: number;
  m2: number;
  ms: number;
  L: number;
  r1: Vec3;
  r2: Vec3;
  rs: Vec3;
  rCmB: Vec3;
  Iorigin: Mat3;
  Icm: Mat3;
  Ieq1: number;
  Ieq2: number;
}

export function massState(cfg: PublicConfig, s: number, th1: number, th2: number, fuel: number): MassState {
  const { m1, m2, mStatic } = modalMasses(cfg.fluidMass);
  const ms = cfg.sliderMass;
  const mRigid = cfg.dryMass + mStatic + Math.max(0, fuel);
  const M = mRigid + ms + m1 + m2;
  const L = cfg.tankMeanRadius;
  const r1 = pendulumPos(th1, 1, L);
  const r2 = pendulumPos(th2, 2, L);
  const rs: Vec3 = [s, 0, 0];
  const rCmB: Vec3 = [
    (ms * rs[0] + m1 * r1[0] + m2 * r2[0]) / M,
    (ms * rs[1] + m1 * r1[1] + m2 * r2[1]) / M,
    (ms * rs[2] + m1 * r1[2] + m2 * r2[2]) / M,
  ];
  const Iorigin = madd(
    madd(diag(cfg.dryInertiaB[0], cfg.dryInertiaB[1], cfg.dryInertiaB[2]), pointInertia(ms, rs)),
    madd(pointInertia(m1, r1), pointInertia(m2, r2)),
  );
  const Icm = madd(Iorigin, mscale(pointInertia(M, rCmB), -1));
  return {
    M,
    mRigid,
    m1,
    m2,
    ms,
    L,
    r1,
    r2,
    rs,
    rCmB,
    Iorigin,
    Icm,
    Ieq1: m1 * L * L,
    Ieq2: m2 * L * L,
  };
}

export function sloshEnergy(
  th1: number,
  th1d: number,
  th2: number,
  th2d: number,
  m1: number,
  m2: number,
  L: number,
  k12: number,
): number {
  const I1 = m1 * L * L;
  const I2 = m2 * L * L;
  const Ieq = 0.5 * (I1 + I2);
  const T = 0.5 * I1 * th1d * th1d + 0.5 * I2 * th2d * th2d;
  const V =
    I1 * OMEGA1 * OMEGA1 * (1 - Math.cos(th1)) +
    I2 * OMEGA2 * OMEGA2 * (1 - Math.cos(th2)) +
    Ieq * k12 * (1 - Math.cos(th1 - th2));
  return T + V;
}

export function relativeAM(ms: MassState, sd: number, th1d: number, th2d: number): Vec3 {
  // h_rel about origin from coordinate rates (slider contributes 0 about O).
  return [0, ms.Ieq2 * th2d, -ms.Ieq1 * th1d];
}

export function totalAngularMomentumI(
  cfg: PublicConfig,
  st: SimState,
  _k12: number,
): Vec3 {
  const ms = massState(cfg, st.s, st.th1, st.th2, st.fuel);
  const hB = vadd(mv(ms.Iorigin, st.w), relativeAM(ms, st.sd, st.th1d, st.th2d));
  // H about origin in inertial, plus CM orbital term about inertial origin
  const H_o_I = qRotate(st.q, hB);
  const H_cm_orb = vcross(st.rCmI, vscale(st.vCmI, ms.M));
  return vadd(H_o_I, H_cm_orb);
}

export function linearMomentumI(cfg: PublicConfig, st: SimState): Vec3 {
  const ms = massState(cfg, st.s, st.th1, st.th2, st.fuel);
  return vscale(st.vCmI, ms.M);
}

export function kineticPlusPotential(
  cfg: PublicConfig,
  st: SimState,
  k12: number,
): number {
  const ms = massState(cfg, st.s, st.th1, st.th2, st.fuel);
  const Ttrans = 0.5 * ms.M * vdot(st.vCmI, st.vCmI);
  const Trot = 0.5 * vdot(st.w, mv(ms.Icm, st.w));
  const Trel = 0.5 * ms.ms * st.sd * st.sd;
  const Es = sloshEnergy(st.th1, st.th1d, st.th2, st.th2d, ms.m1, ms.m2, ms.L, k12);
  return Ttrans + Trot + Trel + Es;
}

export interface ForceInput {
  FthrB: Vec3;
  tauThrO: Vec3; // thruster torque about origin
  Fslider: number;
  c1: number;
  c2: number;
  k12: number;
}

export interface AccelOut {
  aCmI: Vec3;
  alpha: Vec3;
  sdd: number;
  th1dd: number;
  th2dd: number;
}

function IdotOmega(ms: MassState, sd: number, th1d: number, th2d: number, w: Vec3): Vec3 {
  const acc = (m: number, r: Vec3, v: Vec3): Vec3 => {
    const rv = vdot(r, v);
    const rw = vdot(r, w);
    const vw = vdot(v, w);
    return vscale(vsub(vsub(vscale(w, 2 * rv), vscale(v, rw)), vscale(r, vw)), m);
  };
  const vs: Vec3 = [sd, 0, 0];
  const th1 = Math.atan2(ms.r1[0], ms.r1[1]);
  const th2 = Math.atan2(ms.r2[0], ms.r2[2]);
  const v1 = vscale(pendulumETheta(th1, 1), ms.L * th1d);
  const v2 = vscale(pendulumETheta(th2, 2), ms.L * th2d);
  return vadd(acc(ms.ms, ms.rs, vs), vadd(acc(ms.m1, ms.r1, v1), acc(ms.m2, ms.r2, v2)));
}

export function accelerations(cfg: PublicConfig, st: SimState, u: ForceInput): AccelOut {
  const ms = massState(cfg, st.s, st.th1, st.th2, st.fuel);
  const w = st.w;
  const FthrI = qRotate(st.q, u.FthrB);
  const aCmI = vscale(FthrI, 1 / ms.M);
  const aCmB = qRotateInv(st.q, aCmI);

  // Origin acceleration ≈ a_cm (documented 5% approximation).
  const aO = aCmB;

  // Slider EOM along body x. α × r_s has zero x-component.
  const centrif = st.s * (w[1] * w[1] + w[2] * w[2]);
  const sdd = u.Fslider / ms.ms + centrif - aO[0];

  // Kinematic forcing for pendulums, excluding α (handled in 3×3) and θ̈.
  const e1 = pendulumETheta(st.th1, 1);
  const e2 = pendulumETheta(st.th2, 2);
  const v1 = vscale(e1, ms.L * st.th1d);
  const v2 = vscale(e2, ms.L * st.th2d);
  const frameAcc = (r: Vec3, vrel: Vec3): Vec3 =>
    vadd(aO, vadd(vcross(w, vcross(w, r)), vscale(vcross(w, vrel), 2)));
  const a1kin = -vdot(frameAcc(ms.r1, v1), e1) / ms.L;
  const a2kin = -vdot(frameAcc(ms.r2, v2), e2) / ms.L;

  const t1dd0 =
    -u.c1 * st.th1d -
    OMEGA1 * OMEGA1 * Math.sin(st.th1) -
    u.k12 * Math.sin(st.th1 - st.th2) +
    a1kin;
  const t2dd0 =
    -u.c2 * st.th2d -
    OMEGA2 * OMEGA2 * Math.sin(st.th2) -
    u.k12 * Math.sin(st.th2 - st.th1) +
    a2kin;

  // α coupling: θ1̈ += α_z , θ2̈ += −α_y  (see module note).
  // h_rel_dot = [0, I2 θ2̈, −I1 θ1̈]
  // Euler about origin (internal forces drop):
  //   I_o α + İω + ḣ_rel + ω×(I_o ω + h_rel) = τ_thr_O
  const hrel = relativeAM(ms, st.sd, st.th1d, st.th2d);
  const Iw = mv(ms.Iorigin, w);
  const rhsKnown = vsub(
    u.tauThrO,
    vadd(IdotOmega(ms, st.sd, st.th1d, st.th2d, w), vcross(w, vadd(Iw, hrel))),
  );
  // I α + [0, I2 (t2dd0 − αy), −I1 (t1dd0 + αz)] = rhsKnown
  // (I + diag(0,I2,I1)) α = rhsKnown − [0, I2 t2dd0, −I1 t1dd0]
  const rhs: Vec3 = [
    rhsKnown[0],
    rhsKnown[1] - ms.Ieq2 * t2dd0,
    rhsKnown[2] + ms.Ieq1 * t1dd0,
  ];
  const A: Mat3 = [
    [ms.Iorigin[0][0], ms.Iorigin[0][1], ms.Iorigin[0][2]],
    [ms.Iorigin[1][0], ms.Iorigin[1][1] + ms.Ieq2, ms.Iorigin[1][2]],
    [ms.Iorigin[2][0], ms.Iorigin[2][1], ms.Iorigin[2][2] + ms.Ieq1],
  ];
  const alpha = mv(minv3(A), rhs);
  const th1dd = t1dd0 + alpha[2];
  const th2dd = t2dd0 - alpha[1];

  return { aCmI, alpha, sdd, th1dd, th2dd };
}

export function originKinematics(st: SimState, ms: MassState): { rI: Vec3; vI: Vec3 } {
  const rCmB_I = qRotate(st.q, ms.rCmB);
  const rI = vsub(st.rCmI, rCmB_I);
  const e1 = pendulumETheta(st.th1, 1);
  const e2 = pendulumETheta(st.th2, 2);
  const rcmd: Vec3 = [
    (ms.ms * st.sd + ms.m1 * ms.L * st.th1d * e1[0] + ms.m2 * ms.L * st.th2d * e2[0]) / ms.M,
    (ms.ms * 0 + ms.m1 * ms.L * st.th1d * e1[1] + ms.m2 * ms.L * st.th2d * e2[1]) / ms.M,
    (ms.ms * 0 + ms.m1 * ms.L * st.th1d * e1[2] + ms.m2 * ms.L * st.th2d * e2[2]) / ms.M,
  ];
  const vrelI = qRotate(st.q, vadd(vcross(st.w, ms.rCmB), rcmd));
  const vI = vsub(st.vCmI, vrelI);
  return { rI, vI };
}

const N = 19; // rcm(3) vcm(3) q(4) w(3) s sd th1 th1d th2 th2d

export function pack(st: SimState): Float64Array {
  const y = new Float64Array(N);
  y[0] = st.rCmI[0];
  y[1] = st.rCmI[1];
  y[2] = st.rCmI[2];
  y[3] = st.vCmI[0];
  y[4] = st.vCmI[1];
  y[5] = st.vCmI[2];
  y[6] = st.q[0];
  y[7] = st.q[1];
  y[8] = st.q[2];
  y[9] = st.q[3];
  y[10] = st.w[0];
  y[11] = st.w[1];
  y[12] = st.w[2];
  y[13] = st.s;
  y[14] = st.sd;
  y[15] = st.th1;
  y[16] = st.th1d;
  y[17] = st.th2;
  y[18] = st.th2d;
  return y;
}

export function unpack(y: Float64Array, t: number, fuel: number, cfg: PublicConfig): SimState {
  const q = qnormalize([y[6], y[7], y[8], y[9]]);
  const st: SimState = {
    t,
    rCmI: [y[0], y[1], y[2]],
    vCmI: [y[3], y[4], y[5]],
    q,
    w: [y[10], y[11], y[12]],
    s: y[13],
    sd: y[14],
    th1: y[15],
    th1d: y[16],
    th2: y[17],
    th2d: y[18],
    fuel,
    rI: [0, 0, 0],
    vI: [0, 0, 0],
  };
  const ms = massState(cfg, st.s, st.th1, st.th2, st.fuel);
  const kin = originKinematics(st, ms);
  st.rI = kin.rI;
  st.vI = kin.vI;
  return st;
}

function deriv(y: Float64Array, cfg: PublicConfig, u: ForceInput, fuel: number): Float64Array {
  const st = unpack(y, 0, fuel, cfg);
  const a = accelerations(cfg, st, u);
  const qd = qdot(st.q, st.w);
  const yp = new Float64Array(N);
  yp[0] = st.vCmI[0];
  yp[1] = st.vCmI[1];
  yp[2] = st.vCmI[2];
  yp[3] = a.aCmI[0];
  yp[4] = a.aCmI[1];
  yp[5] = a.aCmI[2];
  yp[6] = qd[0];
  yp[7] = qd[1];
  yp[8] = qd[2];
  yp[9] = qd[3];
  yp[10] = a.alpha[0];
  yp[11] = a.alpha[1];
  yp[12] = a.alpha[2];
  yp[13] = st.sd;
  yp[14] = a.sdd;
  yp[15] = st.th1d;
  yp[16] = a.th1dd;
  yp[17] = st.th2d;
  yp[18] = a.th2dd;
  return yp;
}

function axpy(y: Float64Array, k: Float64Array, s: number): Float64Array {
  const z = new Float64Array(N);
  for (let i = 0; i < N; i++) z[i] = y[i] + s * k[i];
  return z;
}

function combine(y: Float64Array, k1: Float64Array, k2: Float64Array, k3: Float64Array, k4: Float64Array, dt: number): Float64Array {
  const z = new Float64Array(N);
  const s = dt / 6;
  for (let i = 0; i < N; i++) z[i] = y[i] + s * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
  return z;
}

export function rk4Step(st: SimState, cfg: PublicConfig, u: ForceInput, dt: number): SimState {
  const y = pack(st);
  const k1 = deriv(y, cfg, u, st.fuel);
  const k2 = deriv(axpy(y, k1, dt * 0.5), cfg, u, st.fuel);
  const k3 = deriv(axpy(y, k2, dt * 0.5), cfg, u, st.fuel);
  const k4 = deriv(axpy(y, k3, dt), cfg, u, st.fuel);
  const yn = combine(y, k1, k2, k3, k4, dt);
  const qn = qnormalize([yn[6], yn[7], yn[8], yn[9]]);
  yn[6] = qn[0];
  yn[7] = qn[1];
  yn[8] = qn[2];
  yn[9] = qn[3];
  return unpack(yn, st.t + dt, st.fuel, cfg);
}

export interface CollisionResult {
  state: SimState;
  collided: boolean;
  impactSpeed: number;
  impulse: number;
  bound: number;
}

/**
 * Inelastic end-stop. Impulse is internal and collocated, so P and H about
 * the inertial origin are conserved by adjusting v_cm (unchanged) and ω so
 * that H_cm is unchanged after ṡ flips. v_cm is invariant because the pair
 * is internal. ω is adjusted to keep H about CM.
 */
export function applyCollision(
  cfg: PublicConfig,
  st: SimState,
  bound: number,
  e: number,
): CollisionResult {
  const ms = massState(cfg, bound, st.th1, st.th2, st.fuel);
  const sdIn = st.sd;
  const sdOut = -e * sdIn;
  const hBefore = vadd(mv(ms.Icm, st.w), relativeAM(ms, sdIn, st.th1d, st.th2d));
  // After: s changes (already at bound), sd flips. h_rel slider about CM:
  // (r_s − r_cm) × m ṡ e_x . About CM this is m ( [s,0,0]−r_cm ) × [sd,0,0]
  // which is [0, rcm_z * m * (−sd change), −rcm_y * m * (−sd change)].
  // Slider relative AM about O is 0; about CM it is not if r_cm has y/z.
  const rs: Vec3 = [bound, 0, 0];
  const rrel = vsub(rs, ms.rCmB);
  const hRelS = (sd: number) => vscale(vcross(rrel, [sd, 0, 0]), ms.ms);
  const hAfterTarget = hBefore; // conserve H_cm_B (no external impulse torque about CM if we account for ω)
  // I ω' + h_rel_other + hRelS(sdOut) = I ω + h_rel_other + hRelS(sdIn)
  // I (ω' − ω) = hRelS(sdIn) − hRelS(sdOut)
  const dh = vsub(hRelS(sdIn), hRelS(sdOut));
  const dw = mv(minv3(ms.Icm), dh);
  const next: SimState = {
    ...st,
    s: bound,
    sd: sdOut,
    w: vadd(st.w, dw),
  };
  const kin = originKinematics(next, massState(cfg, next.s, next.th1, next.th2, next.fuel));
  next.rI = kin.rI;
  next.vI = kin.vI;
  const J = ms.ms * (sdOut - sdIn);
  return { state: next, collided: true, impactSpeed: Math.abs(sdIn), impulse: J, bound };
}

export function integrateWithCollision(
  cfg: PublicConfig,
  st: SimState,
  u: ForceInput,
  dt: number,
): CollisionResult {
  const s0 = st.s;
  const next = rk4Step(st, cfg, u, dt);
  const min = cfg.sliderMin;
  const max = cfg.sliderMax;
  let bound: number | null = null;
  if (next.s > max && next.s > s0) bound = max;
  else if (next.s < min && next.s < s0) bound = min;
  if (bound === null) {
    // Still clip numerically if we sit on the wall pushing out.
    if (next.s > max) {
      return applyCollision(cfg, { ...next, s: max }, max, cfg.restitution);
    }
    if (next.s < min) {
      return applyCollision(cfg, { ...next, s: min }, min, cfg.restitution);
    }
    return { state: next, collided: false, impactSpeed: 0, impulse: 0, bound: 0 };
  }
  const ds = next.s - s0;
  const frac = clamp((bound - s0) / (ds === 0 ? 1e-12 : ds), 0, 1);
  const tHit = Math.max(1e-6, frac * dt);
  const atHit = rk4Step(st, cfg, u, tHit);
  const col = applyCollision(cfg, atHit, bound, cfg.restitution);
  const remain = dt - tHit;
  if (remain > 1e-6) {
    const after = rk4Step(col.state, cfg, u, remain);
    // Prevent re-penetration in the same step.
    if (after.s > max) after.s = max;
    if (after.s < min) after.s = min;
    return { ...col, state: after };
  }
  return col;
}

export function pressureFromSlosh(th1: number, th2: number): [number, number] {
  // Gauge-like wall pressures (Pa). Invertible 2×2 in sin θ.
  const p0 = 2500;
  const a = 1800;
  const b = 220;
  const s1 = Math.sin(th1);
  const s2 = Math.sin(th2);
  return [p0 + a * s1 + b * s2, p0 + a * s2 + b * s1];
}

export function invertPressure(p1: number, p2: number): [number, number] {
  const p0 = 2500;
  const a = 1800;
  const b = 220;
  const u = p1 - p0;
  const v = p2 - p0;
  const det = a * a - b * b;
  const s1 = (a * u - b * v) / det;
  const s2 = (a * v - b * u) / det;
  return [
    Math.asin(clamp(s1, -1, 1)),
    Math.asin(clamp(s2, -1, 1)),
  ];
}

export { DRY_MASS, DRY_INERTIA, SLIDER_MASS, TANK_R };
