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
  tTli: number;
  tCapture: number;
  periodLlo: number;
  periodMoon: number;
  angle0: number;
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

function lloAroundMoon(
  phys: number,
  angle0: number,
  tLoiPhys: number,
): { r: [number, number, number]; v: [number, number, number]; moon: [number, number, number] } {
  const moon = moonOnRail(phys, angle0);
  const n = Math.sqrt(MU_MOON / (R_LLO * R_LLO * R_LLO));
  const th = n * (phys - tLoiPhys);
  const s = circularState(MU_MOON, R_LLO, th, moon[0], moon[1], moon[2]);
  return { r: s.r, v: s.v, moon };
}

function phaseOfLlo(tau: number, periodLlo: number): Phase {
  if (tau < 45) return "loi";
  if (tau < 4 * periodLlo) return "llo";
  return "revolution";
}

/** Hohmann-like Earth–Moon transfer, circular LLO, then one lunar sidereal month. */
export function buildCislunarMission(): CislunarMission {
  const rp = R_LEO;
  const ra = A_MOON;
  const a = (rp + ra) / 2;
  const e = (ra - rp) / (ra + rp);
  const tofCoast = Math.PI * Math.sqrt((a * a * a) / MU_EARTH);
  const periodMoon = 2 * Math.PI * Math.sqrt((A_MOON * A_MOON * A_MOON) / MU_EARTH);
  const periodLlo = 2 * Math.PI * Math.sqrt((R_LLO * R_LLO * R_LLO) / MU_MOON);
  const nMoon = 2 * Math.PI / periodMoon;
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

  const tLoiPhys = tofCoast;
  const nLloShow = 4;
  const nDense = 96;
  for (let i = 0; i < nDense; i++) {
    const tau = (i / (nDense - 1)) * nLloShow * periodLlo;
    const phys = tLoiPhys + tau;
    const s = lloAroundMoon(phys, angle0, tLoiPhys);
    samples.push(pack(phys, s.r, s.v, s.moon, phaseOfLlo(tau, periodLlo)));
  }

  const tRev0 = nLloShow * periodLlo;
  const nRev = 240;
  for (let i = 1; i < nRev; i++) {
    const tau = tRev0 + (i / (nRev - 1)) * (periodMoon - tRev0);
    const phys = tLoiPhys + tau;
    const s = lloAroundMoon(phys, angle0, tLoiPhys);
    samples.push(pack(phys, s.r, s.v, s.moon, "revolution"));
  }

  const t0 = samples[0]!.t;
  for (const s of samples) s.t -= t0;
  const duration = samples[samples.length - 1]!.t;
  const tTli = -t0;
  const tCapture = tLoiPhys - t0;

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

  return {
    samples,
    duration,
    tofCoast,
    dvTli,
    dvLoi,
    tTli,
    tCapture,
    periodLlo,
    periodMoon,
    angle0,
    phases,
  };
}

export function sampleAt(mission: CislunarMission, t: number): CislunarSample {
  const tt = Math.min(Math.max(t, 0), mission.duration);
  if (tt + 1e-6 >= mission.tCapture) {
    const phys = tt - mission.tTli;
    const s = lloAroundMoon(phys, mission.angle0, mission.tCapture - mission.tTli);
    const tau = tt - mission.tCapture;
    return pack(tt, s.r, s.v, s.moon, phaseOfLlo(tau, mission.periodLlo));
  }
  const xs = mission.samples;
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
  return pack(tt, lerp(a.r, b.r), lerp(a.v, b.v), lerp(a.moon, b.moon), u < 0.5 ? a.phase : b.phase);
}
