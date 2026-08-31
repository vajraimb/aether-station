import { attitudeErrorAngle, deg, qmul, qnormalize, type Quat, type Vec3 } from "../math3d";
import {
  classifyPhase,
  mismatchAt,
  quantiles,
  summarizePhase,
  type BeliefSnapshot,
  type TruthSnapshot,
} from "./belief-mismatch";

interface T {
  name: string;
  pass: boolean;
  detail: string;
}

function check(name: string, pass: boolean, detail: string, out: T[]): void {
  out.push({ name, pass, detail });
}

function qAxis(axis: Vec3, degA: number): Quat {
  const n = Math.hypot(axis[0], axis[1], axis[2]) || 1;
  const h = (0.5 * degA * Math.PI) / 180;
  const s = Math.sin(h);
  return qnormalize([Math.cos(h), (axis[0] / n) * s, (axis[1] / n) * s, (axis[2] / n) * s]);
}

const ident: BeliefSnapshot = {
  q: [1, 0, 0, 0],
  w: [0.05, -0.02, 0.01],
  s: 0.2,
  sd: 0.01,
  th1: 0.05,
  th1d: 0,
  th2: -0.04,
  th2d: 0.01,
  fuel: 4.2,
  bias: [0.001, 0, 0],
  c1: 0.13,
  c2: 0.09,
  k12: 0.3,
  etaT: 0.87,
  isolated: [],
};

function truthFrom(b: BeliefSnapshot, over: Partial<TruthSnapshot> = {}): TruthSnapshot {
  return {
    q: b.q,
    w: b.w,
    s: b.s,
    sd: b.sd,
    th1: b.th1,
    th1d: b.th1d,
    th2: b.th2,
    th2d: b.th2d,
    fuel: b.fuel,
    gyroBias: b.bias,
    c1: b.c1,
    c2: b.c2,
    k12: b.k12,
    etaT: b.etaT,
    failedThruster: -1,
    faultHasOccurred: false,
    ...over,
  };
}

export function runBeliefMismatchTests(): T[] {
  const out: T[] = [];

  check("phase_nominal", classifyPhase({ t: 10, faultTime: 70, attDegTruth: 40 }) === "nominal", "nominal", out);
  check("phase_pre_fault", classifyPhase({ t: 65, faultTime: 70, attDegTruth: 30 }) === "pre_fault", "pre_fault", out);
  check("phase_post_fault", classifyPhase({ t: 80, faultTime: 70, attDegTruth: 20 }) === "post_fault", "post_fault", out);
  check("phase_terminal_overrides", classifyPhase({ t: 90, faultTime: 70, attDegTruth: 8 }) === "terminal", "terminal", out);

  const zero = mismatchAt(ident, truthFrom(ident), [], [0, 0, 0, 0, 0, 0], 1);
  check("identical_att_zero", zero.attGeodesicDeg < 1e-9, `att=${zero.attGeodesicDeg}`, out);
  check("identical_w_zero", zero.wErr < 1e-12, `w=${zero.wErr}`, out);
  check("identical_fuel_zero", Math.abs(zero.fuelErr) < 1e-12, `fuel=${zero.fuelErr}`, out);
  check("identical_fdir_ok", zero.fdirMaskMismatch === false, "fdir", out);

  const qErr = qmul(qAxis([0, 0, 1], 5), ident.q as Quat);
  const five = mismatchAt({ ...ident, q: qErr }, truthFrom(ident), [], [0, 0, 0, 0, 0, 0], 1);
  check(
    "geodesic_five_deg",
    Math.abs(five.attGeodesicDeg - 5) < 0.05,
    `got ${five.attGeodesicDeg.toFixed(3)} (truth angle ${deg(attitudeErrorAngle(qErr)).toFixed(3)})`,
    out,
  );

  const fdir = mismatchAt(
    { ...ident, isolated: [4] },
    truthFrom(ident, { failedThruster: 2, faultHasOccurred: true }),
    [],
    [0, 0, 0, 0, 0, 0],
    10,
  );
  check("fdir_mismatch_detected", fdir.fdirMaskMismatch === true, "mask 4 vs 2", out);

  const pending = mismatchAt(
    ident,
    truthFrom(ident),
    [{ id: 0, tOn: 0.1, tOff: 0.3 }],
    [1, 0, 0, 0, 0, 0],
    0.2,
  );
  check("pending_match_when_on", pending.pendingMismatchCount === 0, `mismatch=${pending.pendingMismatchCount}`, out);
  const pendingMiss = mismatchAt(ident, truthFrom(ident), [{ id: 0, tOn: 0.1, tOff: 0.3 }], [0, 0, 0, 0, 0, 0], 0.2);
  check("pending_mismatch_when_off", pendingMiss.pendingMismatchCount === 1, `mismatch=${pendingMiss.pendingMismatchCount}`, out);

  const q = quantiles([1, 2, 3, 4, 10]);
  check("quantiles_n", q.n === 5 && q.p50 === 3, JSON.stringify(q), out);

  const summary = summarizePhase("nominal", [zero, five]);
  check("summary_n", summary.n === 2 && summary.attGeodesicDeg.mean > 0, `mean=${summary.attGeodesicDeg.mean}`, out);

  return out;
}
