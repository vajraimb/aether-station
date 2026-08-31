/** Right-handed 3D math. Quaternions are [w, x, y, z].
 *  q_BI rotates a body-frame vector into inertial: v_I = R(q_BI) v_B
 *  (Hamilton product, active rotation, aerospace convention). */

export type Vec3 = [number, number, number];
export type Quat = [number, number, number, number];
export type Mat3 = [Vec3, Vec3, Vec3];

export const EPS = 1e-15;

export function v3(x = 0, y = 0, z = 0): Vec3 {
  return [x, y, z];
}

export function vcopy(a: Vec3): Vec3 {
  return [a[0], a[1], a[2]];
}

export function vadd(a: Vec3, b: Vec3): Vec3 {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

export function vsub(a: Vec3, b: Vec3): Vec3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

export function vscale(a: Vec3, s: number): Vec3 {
  return [a[0] * s, a[1] * s, a[2] * s];
}

export function vdot(a: Vec3, b: Vec3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

export function vcross(a: Vec3, b: Vec3): Vec3 {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

export function vnorm(a: Vec3): number {
  return Math.hypot(a[0], a[1], a[2]);
}

export function vnormalize(a: Vec3): Vec3 {
  const n = vnorm(a);
  return n > EPS ? vscale(a, 1 / n) : [0, 0, 0];
}

export function vlerp(a: Vec3, b: Vec3, t: number): Vec3 {
  return [
    a[0] + (b[0] - a[0]) * t,
    a[1] + (b[1] - a[1]) * t,
    a[2] + (b[2] - a[2]) * t,
  ];
}

export function qid(): Quat {
  return [1, 0, 0, 0];
}

export function qcopy(q: Quat): Quat {
  return [q[0], q[1], q[2], q[3]];
}

export function qnorm(q: Quat): number {
  return Math.hypot(q[0], q[1], q[2], q[3]);
}

export function qnormalize(q: Quat): Quat {
  const n = qnorm(q);
  if (n < EPS) return [1, 0, 0, 0];
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}

/** Hamilton product q ⊗ p. */
export function qmul(q: Quat, p: Quat): Quat {
  const [qw, qx, qy, qz] = q;
  const [pw, px, py, pz] = p;
  return [
    qw * pw - qx * px - qy * py - qz * pz,
    qw * px + qx * pw + qy * pz - qz * py,
    qw * py - qx * pz + qy * pw + qz * px,
    qw * pz + qx * py - qy * px + qz * pw,
  ];
}

export function qconj(q: Quat): Quat {
  return [q[0], -q[1], -q[2], -q[3]];
}

/** Rotate body vector to inertial: v_I = q ⊗ [0,v] ⊗ q*. */
export function qRotate(q: Quat, v: Vec3): Vec3 {
  const qv: Quat = [0, v[0], v[1], v[2]];
  const r = qmul(qmul(q, qv), qconj(q));
  return [r[1], r[2], r[3]];
}

/** Rotate inertial vector to body. */
export function qRotateInv(q: Quat, v: Vec3): Vec3 {
  return qRotate(qconj(q), v);
}

/** DCM R such that v_I = R v_B. Row-major as 3 Vec3 rows. */
export function quatToR(q: Quat): Mat3 {
  const [w, x, y, z] = q;
  const xx = x * x, yy = y * y, zz = z * z;
  const xy = x * y, xz = x * z, yz = y * z;
  const wx = w * x, wy = w * y, wz = w * z;
  return [
    [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
    [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
    [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
  ];
}

export function mv(m: Mat3, v: Vec3): Vec3 {
  return [
    m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
    m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
    m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
  ];
}

export function mT(m: Mat3): Mat3 {
  return [
    [m[0][0], m[1][0], m[2][0]],
    [m[0][1], m[1][1], m[2][1]],
    [m[0][2], m[1][2], m[2][2]],
  ];
}

export function mvmulT(m: Mat3, v: Vec3): Vec3 {
  return [
    m[0][0] * v[0] + m[1][0] * v[1] + m[2][0] * v[2],
    m[0][1] * v[0] + m[1][1] * v[1] + m[2][1] * v[2],
    m[0][2] * v[0] + m[1][2] * v[1] + m[2][2] * v[2],
  ];
}

export function madd(a: Mat3, b: Mat3): Mat3 {
  return [
    [a[0][0] + b[0][0], a[0][1] + b[0][1], a[0][2] + b[0][2]],
    [a[1][0] + b[1][0], a[1][1] + b[1][1], a[1][2] + b[1][2]],
    [a[2][0] + b[2][0], a[2][1] + b[2][1], a[2][2] + b[2][2]],
  ];
}

export function mscale(a: Mat3, s: number): Mat3 {
  return [
    [a[0][0] * s, a[0][1] * s, a[0][2] * s],
    [a[1][0] * s, a[1][1] * s, a[1][2] * s],
    [a[2][0] * s, a[2][1] * s, a[2][2] * s],
  ];
}

export function mvec(m: Mat3, v: Vec3): Vec3 {
  return mv(m, v);
}

export function diag(a: number, b: number, c: number): Mat3 {
  return [
    [a, 0, 0],
    [0, b, 0],
    [0, 0, c],
  ];
}

export function mI(): Mat3 {
  return diag(1, 1, 1);
}

/** Inertia of a point mass about the origin: m (|r|^2 I - r r^T). */
export function pointInertia(m: number, r: Vec3): Mat3 {
  const r2 = vdot(r, r);
  return [
    [m * (r2 - r[0] * r[0]), -m * r[0] * r[1], -m * r[0] * r[2]],
    [-m * r[1] * r[0], m * (r2 - r[1] * r[1]), -m * r[1] * r[2]],
    [-m * r[2] * r[0], -m * r[2] * r[1], m * (r2 - r[2] * r[2])],
  ];
}

export function minv3(m: Mat3): Mat3 {
  const a = m[0][0], b = m[0][1], c = m[0][2];
  const d = m[1][0], e = m[1][1], f = m[1][2];
  const g = m[2][0], h = m[2][1], i = m[2][2];
  const A = e * i - f * h;
  const B = f * g - d * i;
  const C = d * h - e * g;
  const det = a * A + b * B + c * C;
  if (Math.abs(det) < 1e-18) {
    return diag(1e-9, 1e-9, 1e-9);
  }
  const inv = 1 / det;
  return [
    [A * inv, (c * h - b * i) * inv, (b * f - c * e) * inv],
    [B * inv, (a * i - c * g) * inv, (c * d - a * f) * inv],
    [C * inv, (b * g - a * h) * inv, (a * e - b * d) * inv],
  ];
}

export function solve3(A: Mat3, b: Vec3): Vec3 {
  return mv(minv3(A), b);
}

/** q_dot = 0.5 q ⊗ [0, ω]. */
export function qdot(q: Quat, w: Vec3): Quat {
  return qmul(q, [0, 0.5 * w[0], 0.5 * w[1], 0.5 * w[2]]);
}

/** Shortest-path attitude error: 2 * vec(q_err) with q_err = q_target* ⊗ q, flipped if w<0. */
export function attitudeErrorVector(q: Quat, qTarget: Quat = [1, 0, 0, 0]): Vec3 {
  const qe = qmul(qconj(qTarget), q);
  const s = qe[0] < 0 ? -1 : 1;
  return [s * qe[1], s * qe[2], s * qe[3]];
}

/** Principal rotation angle in radians (0..π), q vs identity. */
export function attitudeErrorAngle(q: Quat, qTarget: Quat = [1, 0, 0, 0]): number {
  const qe = qmul(qconj(qTarget), q);
  const w = Math.abs(qe[0]);
  const clamped = Math.min(1, Math.max(-1, w));
  return 2 * Math.acos(clamped);
}

export function deg(rad: number): number {
  return (rad * 180) / Math.PI;
}

export function clamp(x: number, lo: number, hi: number): number {
  return Math.min(hi, Math.max(lo, x));
}

export function skew(v: Vec3): Mat3 {
  return [
    [0, -v[2], v[1]],
    [v[2], 0, -v[0]],
    [-v[1], v[0], 0],
  ];
}

/** Mulberry32 + gaussian via Box-Muller. Deterministic given seed. */
export function makeRng(seed: number) {
  let a = seed >>> 0;
  if (a === 0) a = 0x9e3779b9;
  const u01 = () => {
    a += 0x6d2b79f5;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
  let spare: number | null = null;
  const gauss = () => {
    if (spare !== null) {
      const v = spare;
      spare = null;
      return v;
    }
    let u = 0, v = 0, s = 0;
    do {
      u = 2 * u01() - 1;
      v = 2 * u01() - 1;
      s = u * u + v * v;
    } while (s === 0 || s >= 1);
    const m = Math.sqrt((-2 * Math.log(s)) / s);
    spare = v * m;
    return u * m;
  };
  return { u01, gauss, seed };
}

export type Rng = ReturnType<typeof makeRng>;

export function almostEqual(a: number, b: number, tol = 1e-9): boolean {
  return Math.abs(a - b) <= tol;
}

export function vmaxabs(a: Vec3): number {
  return Math.max(Math.abs(a[0]), Math.abs(a[1]), Math.abs(a[2]));
}

/** Solve A x = b with partial-pivot Gaussian elimination. A is n×n. */
export function solveLinear(Ain: number[][], bin: number[]): number[] {
  const n = bin.length;
  const A: number[][] = new Array(n);
  const b = bin.slice();
  for (let i = 0; i < n; i++) A[i] = Ain[i]!.slice();
  for (let k = 0; k < n; k++) {
    let piv = k;
    let best = Math.abs(A[k]![k]!);
    for (let i = k + 1; i < n; i++) {
      const v = Math.abs(A[i]![k]!);
      if (v > best) {
        best = v;
        piv = i;
      }
    }
    if (best < 1e-18) {
      A[k]![k] = 1e-18;
    } else if (piv !== k) {
      const tmp = A[k]!;
      A[k] = A[piv]!;
      A[piv] = tmp;
      const tb = b[k]!;
      b[k] = b[piv]!;
      b[piv] = tb;
    }
    const akk = A[k]![k]!;
    for (let i = k + 1; i < n; i++) {
      const f = A[i]![k]! / akk;
      A[i]![k] = 0;
      for (let j = k + 1; j < n; j++) A[i]![j]! -= f * A[k]![j]!;
      b[i]! -= f * b[k]!;
    }
  }
  const x = new Array<number>(n).fill(0);
  for (let i = n - 1; i >= 0; i--) {
    let s = b[i]!;
    for (let j = i + 1; j < n; j++) s -= A[i]![j]! * x[j]!;
    x[i] = s / A[i]![i]!;
  }
  return x;
}
