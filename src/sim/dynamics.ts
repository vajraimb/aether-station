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
  qnorm,
  qnormalize,
  solveLinear,
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
 * Newton–Euler is written about the system CM with the slider and both
 * pendulums fully coupled (recoil retained). Internal forces therefore drop
 * from F_ext and τ_cm; they reappear only as generalised forces on (s, θ).
 * External torque is thrusters only. Slider actuator is an internal pair
 * along x (zero net force, zero net torque about O; about CM the pair is
 * still internal so τ_cm_ext = 0).
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

export function pendulumERadial(theta: number, axis: 1 | 2): Vec3 {
  const s = Math.sin(theta);
  const c = Math.cos(theta);
  if (axis === 1) return [s, c, 0];
  return [s, 0, c];
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

/** Relative AM about the body origin from coordinate rates. Slider term is 0. */
export function relativeAM(ms: MassState, _sd: number, th1d: number, th2d: number): Vec3 {
  return [0, ms.Ieq2 * th2d, -ms.Ieq1 * th1d];
}

export function rcmDot(ms: MassState, sd: number, th1d: number, th2d: number, th1: number, th2: number): Vec3 {
  const e1 = pendulumETheta(th1, 1);
  const e2 = pendulumETheta(th2, 2);
  return [
    (ms.ms * sd + ms.m1 * ms.L * th1d * e1[0] + ms.m2 * ms.L * th2d * e2[0]) / ms.M,
    (ms.m1 * ms.L * th1d * e1[1] + ms.m2 * ms.L * th2d * e2[1]) / ms.M,
    (ms.m1 * ms.L * th1d * e1[2] + ms.m2 * ms.L * th2d * e2[2]) / ms.M,
  ];
}

/** h_rel about CM: h_O − M r_cm × ṙ_cm. */
export function hRelCm(ms: MassState, sd: number, th1d: number, th2d: number, th1: number, th2: number): Vec3 {
  const hO = relativeAM(ms, sd, th1d, th2d);
  const rd = rcmDot(ms, sd, th1d, th2d, th1, th2);
  return vsub(hO, vscale(vcross(ms.rCmB, rd), ms.M));
}

export function angularMomentumCmB(cfg: PublicConfig, st: SimState): Vec3 {
  const ms = massState(cfg, st.s, st.th1, st.th2, st.fuel);
  return vadd(mv(ms.Icm, st.w), hRelCm(ms, st.sd, st.th1d, st.th2d, st.th1, st.th2));
}

/** Conserved H about the system CM, inertial components. */
export function totalAngularMomentumI(
  cfg: PublicConfig,
  st: SimState,
  _k12: number,
): Vec3 {
  return qRotate(st.q, angularMomentumCmB(cfg, st));
}

export function totalAngularMomentumAbout0(
  cfg: PublicConfig,
  st: SimState,
  k12: number,
): Vec3 {
  const ms = massState(cfg, st.s, st.th1, st.th2, st.fuel);
  return vadd(totalAngularMomentumI(cfg, st, k12), vcross(st.rCmI, vscale(st.vCmI, ms.M)));
}

export function linearMomentumI(cfg: PublicConfig, st: SimState): Vec3 {
  const ms = massState(cfg, st.s, st.th1, st.th2, st.fuel);
  return vscale(st.vCmI, ms.M);
}

export interface EnergyParts {
  translational: number;
  rotational: number;
  gyrostat: number;
  relative: number;
  sloshPotential: number;
  couplingPotential: number;
  total: number;
}

/**
 * System energy about the CM (Koenig + relative):
 *   T = ½ M |v_cm|² + ½ ω·I_cm·ω + ω·h_rel_cm + T⋆ + V
 * T⋆ = ½ m_s ṡ² + ½ I1 θ̇1² + ½ I2 θ̇2² − ½ M |ṙ_cm|²
 */
export function energyParts(cfg: PublicConfig, st: SimState, k12: number): EnergyParts {
  const ms = massState(cfg, st.s, st.th1, st.th2, st.fuel);
  const rd = rcmDot(ms, st.sd, st.th1d, st.th2d, st.th1, st.th2);
  const hcm = hRelCm(ms, st.sd, st.th1d, st.th2d, st.th1, st.th2);
  const translational = 0.5 * ms.M * vdot(st.vCmI, st.vCmI);
  const rotational = 0.5 * vdot(st.w, mv(ms.Icm, st.w));
  const gyrostat = vdot(st.w, hcm);
  const relative =
    0.5 * ms.ms * st.sd * st.sd +
    0.5 * ms.Ieq1 * st.th1d * st.th1d +
    0.5 * ms.Ieq2 * st.th2d * st.th2d -
    0.5 * ms.M * vdot(rd, rd);
  const Ieq = 0.5 * (ms.Ieq1 + ms.Ieq2);
  const sloshPotential =
    ms.Ieq1 * OMEGA1 * OMEGA1 * (1 - Math.cos(st.th1)) +
    ms.Ieq2 * OMEGA2 * OMEGA2 * (1 - Math.cos(st.th2));
  const couplingPotential = Ieq * k12 * (1 - Math.cos(st.th1 - st.th2));
  return {
    translational,
    rotational,
    gyrostat,
    relative,
    sloshPotential,
    couplingPotential,
    total: translational + rotational + gyrostat + relative + sloshPotential + couplingPotential,
  };
}

export function kineticPlusPotential(
  cfg: PublicConfig,
  st: SimState,
  k12: number,
): number {
  return energyParts(cfg, st, k12).total;
}

export function dampingPower(cfg: PublicConfig, st: SimState, c1: number, c2: number): number {
  const ms = massState(cfg, st.s, st.th1, st.th2, st.fuel);
  return -c1 * ms.Ieq1 * st.th1d * st.th1d - c2 * ms.Ieq2 * st.th2d * st.th2d;
}

export function actuatorPower(st: SimState, u: ForceInput, cfg: PublicConfig): number {
  const ms = massState(cfg, st.s, st.th1, st.th2, st.fuel);
  const tauCm = vsub(u.tauThrO, vcross(ms.rCmB, u.FthrB));
  const FthrI = qRotate(st.q, u.FthrB);
  return u.Fslider * st.sd + vdot(tauCm, st.w) + vdot(FthrI, st.vCmI);
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

/** d/dt [ m (|r|² I − r rᵀ) ] ω */
function inertiaRateOmega(m: number, r: Vec3, v: Vec3, w: Vec3): Vec3 {
  const rv = vdot(r, v);
  const rw = vdot(r, w);
  const vw = vdot(v, w);
  return vscale(vsub(vsub(vscale(w, 2 * rv), vscale(v, rw)), vscale(r, vw)), m);
}

function zeros6(): number[][] {
  return [
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
  ];
}

export function accelerations(cfg: PublicConfig, st: SimState, u: ForceInput): AccelOut {
  const ms = massState(cfg, st.s, st.th1, st.th2, st.fuel);
  const w = st.w;
  const FthrI = qRotate(st.q, u.FthrB);
  const aCmI = vscale(FthrI, 1 / ms.M);
  const aCmB = qRotateInv(st.q, aCmI);

  const e1 = pendulumETheta(st.th1, 1);
  const e2 = pendulumETheta(st.th2, 2);
  const er1 = pendulumERadial(st.th1, 1);
  const er2 = pendulumERadial(st.th2, 2);
  const vs: Vec3 = [st.sd, 0, 0];
  const v1 = vscale(e1, ms.L * st.th1d);
  const v2 = vscale(e2, ms.L * st.th2d);
  const rd = rcmDot(ms, st.sd, st.th1d, st.th2d, st.th1, st.th2);
  const hcm = hRelCm(ms, st.sd, st.th1d, st.th2d, st.th1, st.th2);
  const Hcm = vadd(mv(ms.Icm, w), hcm);

  const IdotO = vadd(
    inertiaRateOmega(ms.ms, ms.rs, vs, w),
    vadd(inertiaRateOmega(ms.m1, ms.r1, v1, w), inertiaRateOmega(ms.m2, ms.r2, v2, w)),
  );
  const IdotCm = vsub(IdotO, inertiaRateOmega(ms.M, ms.rCmB, rd, w));
  const tauCm = vsub(u.tauThrO, vcross(ms.rCmB, u.FthrB));

  const rddQuad: Vec3 = vscale(
    vadd(vscale(er1, ms.m1 * ms.L * st.th1d * st.th1d), vscale(er2, ms.m2 * ms.L * st.th2d * st.th2d)),
    -1 / ms.M,
  );

  const wcwcr = vcross(w, vcross(w, ms.rCmB));
  const twoWrd = vscale(vcross(w, rd), 2);

  const A = zeros6();
  const b = [0, 0, 0, 0, 0, 0];

  // Euler about CM:
  //   I_cm α + ḣ_O − M r_cm × r̈_cm = τ_cm − İ_cm ω − ω×H_cm
  // ḣ_O = [0, I2 θ̈2, −I1 θ̈1]
  // r̈_cm = (m_s/M) s̈ e_x + (m1 L/M) θ̈1 e1 + (m2 L/M) θ̈2 e2 + rddQuad
  for (let i = 0; i < 3; i++) {
    A[i]![0] = ms.Icm[i]![0]!;
    A[i]![1] = ms.Icm[i]![1]!;
    A[i]![2] = ms.Icm[i]![2]!;
  }
  A[0]![3] = 0;
  A[1]![3] = -ms.ms * ms.rCmB[2];
  A[2]![3] = ms.ms * ms.rCmB[1];
  const rcx_e1 = vcross(ms.rCmB, e1);
  A[0]![4] = -ms.m1 * ms.L * rcx_e1[0];
  A[1]![4] = -ms.m1 * ms.L * rcx_e1[1];
  A[2]![4] = -ms.Ieq1 - ms.m1 * ms.L * rcx_e1[2];
  const rcx_e2 = vcross(ms.rCmB, e2);
  A[0]![5] = -ms.m2 * ms.L * rcx_e2[0];
  A[1]![5] = ms.Ieq2 - ms.m2 * ms.L * rcx_e2[1];
  A[2]![5] = -ms.m2 * ms.L * rcx_e2[2];

  const eulerRhs = vadd(
    vsub(tauCm, IdotCm),
    vsub(vscale(vcross(ms.rCmB, rddQuad), ms.M), vcross(w, Hcm)),
  );
  b[0] = eulerRhs[0];
  b[1] = eulerRhs[1];
  b[2] = eulerRhs[2];

  // Slider along body x, with exact a_O (recoil retained).
  A[3]![0] = 0;
  A[3]![1] = -ms.rCmB[2];
  A[3]![2] = ms.rCmB[1];
  A[3]![3] = 1 - ms.ms / ms.M;
  A[3]![4] = -((ms.m1 * ms.L) / ms.M) * e1[0];
  A[3]![5] = -((ms.m2 * ms.L) / ms.M) * e2[0];
  const centrif = st.s * (w[1] * w[1] + w[2] * w[2]);
  b[3] =
    u.Fslider / Math.max(ms.ms, 1e-12) -
    aCmB[0] +
    wcwcr[0] +
    twoWrd[0] +
    centrif +
    rddQuad[0];

  const L = ms.L;
  const invL = L > 1e-12 ? 1 / L : 0;
  const frameCm = vadd(wcwcr, twoWrd);

  const Q1 =
    -u.c1 * st.th1d -
    OMEGA1 * OMEGA1 * Math.sin(st.th1) -
    u.k12 * Math.sin(st.th1 - st.th2);
  if (ms.m1 < 1e-12 || L < 1e-12) {
    A[4]![4] = 1;
    b[4] = 0;
  } else {
    const rcmxe1 = vcross(ms.rCmB, e1);
    A[4]![0] = -rcmxe1[0] * invL;
    A[4]![1] = -rcmxe1[1] * invL;
    A[4]![2] = -rcmxe1[2] * invL - 1;
    A[4]![3] = -((ms.ms / ms.M) * e1[0]) * invL;
    A[4]![4] = 1 - ms.m1 / ms.M;
    A[4]![5] = -(ms.m2 / ms.M) * vdot(e2, e1);
    const frame1 = vadd(vcross(w, vcross(w, ms.r1)), vscale(vcross(w, v1), 2));
    b[4] =
      Q1 -
      vdot(aCmB, e1) * invL +
      vdot(frameCm, e1) * invL -
      vdot(frame1, e1) * invL +
      vdot(rddQuad, e1) * invL;
  }

  const Q2 =
    -u.c2 * st.th2d -
    OMEGA2 * OMEGA2 * Math.sin(st.th2) -
    u.k12 * Math.sin(st.th2 - st.th1);
  if (ms.m2 < 1e-12 || L < 1e-12) {
    A[5]![5] = 1;
    b[5] = 0;
  } else {
    const rcmxe2 = vcross(ms.rCmB, e2);
    A[5]![0] = -rcmxe2[0] * invL;
    A[5]![1] = -rcmxe2[1] * invL + 1;
    A[5]![2] = -rcmxe2[2] * invL;
    A[5]![3] = -((ms.ms / ms.M) * e2[0]) * invL;
    A[5]![4] = -(ms.m1 / ms.M) * vdot(e1, e2);
    A[5]![5] = 1 - ms.m2 / ms.M;
    const frame2 = vadd(vcross(w, vcross(w, ms.r2)), vscale(vcross(w, v2), 2));
    b[5] =
      Q2 -
      vdot(aCmB, e2) * invL +
      vdot(frameCm, e2) * invL -
      vdot(frame2, e2) * invL +
      vdot(rddQuad, e2) * invL;
  }

  const x = solveLinear(A, b);
  return {
    aCmI,
    alpha: [x[0]!, x[1]!, x[2]!],
    sdd: x[3]!,
    th1dd: x[4]!,
    th2dd: x[5]!,
  };
}

export function originKinematics(st: SimState, ms: MassState): { rI: Vec3; vI: Vec3 } {
  const rCmB_I = qRotate(st.q, ms.rCmB);
  const rI = vsub(st.rCmI, rCmB_I);
  const rd = rcmDot(ms, st.sd, st.th1d, st.th2d, st.th1, st.th2);
  const vrelI = qRotate(st.q, vadd(vcross(st.w, ms.rCmB), rd));
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
  const q = qnormalize([y[6]!, y[7]!, y[8]!, y[9]!]);
  const st: SimState = {
    t,
    rCmI: [y[0]!, y[1]!, y[2]!],
    vCmI: [y[3]!, y[4]!, y[5]!],
    q,
    w: [y[10]!, y[11]!, y[12]!],
    s: y[13]!,
    sd: y[14]!,
    th1: y[15]!,
    th1d: y[16]!,
    th2: y[17]!,
    th2d: y[18]!,
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
  for (let i = 0; i < N; i++) z[i] = y[i]! + s * k[i]!;
  return z;
}

function combine(
  y: Float64Array,
  k1: Float64Array,
  k2: Float64Array,
  k3: Float64Array,
  k4: Float64Array,
  dt: number,
): Float64Array {
  const z = new Float64Array(N);
  const s = dt / 6;
  for (let i = 0; i < N; i++) z[i] = y[i]! + s * (k1[i]! + 2 * k2[i]! + 2 * k3[i]! + k4[i]!);
  return z;
}

export function rk4Step(st: SimState, cfg: PublicConfig, u: ForceInput, dt: number): SimState {
  const y = pack(st);
  const k1 = deriv(y, cfg, u, st.fuel);
  const k2 = deriv(axpy(y, k1, dt * 0.5), cfg, u, st.fuel);
  const k3 = deriv(axpy(y, k2, dt * 0.5), cfg, u, st.fuel);
  const k4 = deriv(axpy(y, k3, dt), cfg, u, st.fuel);
  const yn = combine(y, k1, k2, k3, k4, dt);
  const qRaw: Quat = [yn[6]!, yn[7]!, yn[8]!, yn[9]!];
  const qn = qnorm(qRaw);
  const q = qnormalize(qRaw);
  yn[6] = q[0];
  yn[7] = q[1];
  yn[8] = q[2];
  yn[9] = q[3];
  const next = unpack(yn, st.t + dt, st.fuel, cfg);
  (next as SimState & { qNormRaw?: number }).qNormRaw = qn;
  return next;
}

export function lastQuaternionNormRaw(st: SimState): number {
  const extra = st as SimState & { qNormRaw?: number };
  return extra.qNormRaw ?? qnorm(st.q);
}

export interface CollisionResult {
  state: SimState;
  collided: boolean;
  impactSpeed: number;
  impulse: number;
  bound: number;
  tHit?: number;
  energyLoss?: number;
  angularMomentumJump?: number;
}

/**
 * Inelastic end-stop. Impulse pair along the rail, collocated, so P and H_cm
 * are conserved. ṡ′ = −e ṡ; ω is adjusted so H_cm is unchanged.
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
  const Hbefore = vadd(mv(ms.Icm, st.w), hRelCm(ms, sdIn, st.th1d, st.th2d, st.th1, st.th2));
  const Ebefore = kineticPlusPotential(cfg, { ...st, s: bound }, 0);
  void Ebefore;
  const rs: Vec3 = [bound, 0, 0];
  const rrel = vsub(rs, ms.rCmB);
  const hRelS = (sd: number) => vscale(vcross(rrel, [sd, 0, 0]), ms.ms);
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
  const Hafter = vadd(
    mv(ms.Icm, next.w),
    hRelCm(ms, next.sd, next.th1d, next.th2d, next.th1, next.th2),
  );
  const dH = vnorm(vsub(Hafter, Hbefore));
  const J = ms.ms * (sdOut - sdIn);
  return {
    state: next,
    collided: true,
    impactSpeed: Math.abs(sdIn),
    impulse: J,
    bound,
    angularMomentumJump: dH,
  };
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
  const tHit = Math.max(1e-9, frac * dt);
  const atHit = rk4Step(st, cfg, u, tHit);
  const k12 = u.k12;
  const Epre = kineticPlusPotential(cfg, { ...atHit, s: bound }, k12);
  const col = applyCollision(cfg, atHit, bound, cfg.restitution);
  const Epost = kineticPlusPotential(cfg, col.state, k12);
  col.energyLoss = Epre - Epost;
  col.tHit = st.t + tHit;
  const remain = dt - tHit;
  if (remain > 1e-9) {
    const after = rk4Step(col.state, cfg, u, remain);
    if (after.s > max) after.s = max;
    if (after.s < min) after.s = min;
    if (after.s === max && after.sd > 0) after.sd = 0;
    if (after.s === min && after.sd < 0) after.sd = 0;
    return { ...col, state: after };
  }
  return col;
}

export function pressureFromSlosh(th1: number, th2: number): [number, number] {
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
  return [Math.asin(clamp(s1, -1, 1)), Math.asin(clamp(s2, -1, 1))];
}

export { DRY_MASS, DRY_INERTIA, SLIDER_MASS, TANK_R };
