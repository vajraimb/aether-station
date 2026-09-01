import {
  A_MOON,
  MU_EARTH,
  MU_MOON,
  PHASE_LABEL,
  R_EARTH,
  R_LEO,
  R_LLO,
  R_MOON,
  type Phase,
} from "./constants";
import { circularState, ellipseState, hypot3 } from "./kepler";

export interface CislunarSample {
  t: number;
  r: [number, number, number];
  v: [number, number, number];
  moon: [number, number, number];
  phase: Phase;
  altEarth: number;
  altMoon: number;
  speed: number;
}

export interface CislunarMission {
  samples: CislunarSample[];
  duration: number;
  tofCoast: number;
  dvTli: number;
  dvLoi: number;
  phases: { phase: Phase; t0: number; t1: number; label: string }[];
}

function moonOnRail(t: number, angle0: number): [number, number, number] {
  const n = Math.sqrt(MU_EARTH / (A_MOON * A_MOON * A_MOON));
  const th = angle0 + n * t;
  return [A_MOON * Math.cos(th), 0, A_MOON * Math.sin(th)];
}

function pack(
  t: number,
  r: [number, number, number],
  v: [number, number, number],
  moon: [number, number, number],
  phase: Phase,
): CislunarSample {
  const speed = hypot3(v[0], v[1], v[2]);
  return {
    t,
    r,
    v,
    moon,
    phase,
    altEarth: hypot3(r[0], r[1], r[2]) - R_EARTH,
    altMoon: hypot3(r[0] - moon[0], r[1] - moon[1], r[2] - moon[2]) - R_MOON,
    speed,
  };
}

/** Hohmann-like Earth–Moon transfer then circular LLO. Deterministic. */
export function buildCislunarMission(): CislunarMission {
  const rp = R_LEO;
  const ra = A_MOON;
  const a = (rp + ra) / 2;
  const e = (ra - rp) / (ra + rp);
  const tofCoast = Math.PI * Math.sqrt((a * a * a) / MU_EARTH);
  const nMoon = Math.sqrt(MU_EARTH / (A_MOON * A_MOON * A_MOON));
  const angle0 = Math.PI - nMoon * tofCoast;

  const vLeo = Math.sqrt(MU_EARTH / R_LEO);
  const peri = ellipseState(MU_EARTH, a, e, 0);
  const dvTli = hypot3(peri.v[0], peri.v[1], peri.v[2]) - vLeo;
  const dvLoi = 0.85;

  const samples: CislunarSample[] = [];
  const tLeo = 0.45 * 2 * Math.PI * Math.sqrt((R_LEO * R_LEO * R_LEO) / MU_EARTH);
  const nLeo = 40;
  for (let i = 0; i < nLeo; i++) {
    const t = (i / (nLeo - 1)) * tLeo - tLeo;
    const th = (i / (nLeo - 1)) * 0.9 * Math.PI - 0.9 * Math.PI;
    const s = circularState(MU_EARTH, R_LEO, th, 0, 0, 0);
    samples.push(pack(t, s.r, s.v, moonOnRail(0, angle0), "leo"));
  }

  const nCoast = 220;
  for (let i = 0; i < nCoast; i++) {
    const t = (i / (nCoast - 1)) * tofCoast;
    const s = ellipseState(MU_EARTH, a, e, t);
    const phase: Phase = i === 0 ? "tli" : i === nCoast - 1 ? "loi" : "coast";
    samples.push(pack(t, s.r, s.v, moonOnRail(t, angle0), phase));
  }

  const tLoi = tofCoast;
  const moonCap = moonOnRail(tLoi, angle0);
  const tLlo = 3 * 2 * Math.PI * Math.sqrt((R_LLO * R_LLO * R_LLO) / MU_MOON);
  const nLlo = 180;
  for (let i = 0; i < nLlo; i++) {
    const tau = (i / (nLlo - 1)) * tLlo;
    const th = (i / (nLlo - 1)) * 6 * Math.PI;
    const s = circularState(MU_MOON, R_LLO, th, moonCap[0], moonCap[1], moonCap[2]);
    const moon = moonOnRail(tLoi + tau, angle0);
    const r: [number, number, number] = [
      s.r[0] - moonCap[0] + moon[0],
      s.r[1] - moonCap[1] + moon[1],
      s.r[2] - moonCap[2] + moon[2],
    ];
    samples.push(pack(tLoi + tau, r, s.v, moon, i === 0 ? "loi" : "llo"));
  }

  const t0 = samples[0]!.t;
  for (const s of samples) s.t -= t0;
  const duration = samples[samples.length - 1]!.t;

  const phases: CislunarMission["phases"] = [];
  let cur = samples[0]!.phase;
  let start = samples[0]!.t;
  for (const s of samples) {
    if (s.phase !== cur) {
      phases.push({ phase: cur, t0: start, t1: s.t, label: PHASE_LABEL[cur] });
      cur = s.phase;
      start = s.t;
    }
  }
  phases.push({ phase: cur, t0: start, t1: duration, label: PHASE_LABEL[cur] });

  return { samples, duration, tofCoast, dvTli, dvLoi, phases };
}

export function sampleAt(mission: CislunarMission, t: number): CislunarSample {
  const xs = mission.samples;
  const tt = Math.min(Math.max(t, xs[0]!.t), xs[xs.length - 1]!.t);
  let lo = 0;
  let hi = xs.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (xs[mid]!.t <= tt) lo = mid;
    else hi = mid;
  }
  const a = xs[lo]!;
  const b = xs[hi]!;
  const u = b.t === a.t ? 0 : (tt - a.t) / (b.t - a.t);
  const lerp = (p: [number, number, number], q: [number, number, number]): [number, number, number] => [
    p[0] + (q[0] - p[0]) * u,
    p[1] + (q[1] - p[1]) * u,
    p[2] + (q[2] - p[2]) * u,
  ];
  const r = lerp(a.r, b.r);
  const v = lerp(a.v, b.v);
  const moon = lerp(a.moon, b.moon);
  return pack(tt, r, v, moon, u < 0.5 ? a.phase : b.phase);
}
