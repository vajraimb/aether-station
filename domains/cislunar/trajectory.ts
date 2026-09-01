import {
  A_EARTH,
  A_MARS,
  A_MOON,
  MU_EARTH,
  MU_MARS,
  MU_MOON,
  MU_SUN,
  PHASE_LABEL,
  R_EARTH,
  R_LEO,
  R_LLO,
  R_LMO,
  R_MARS,
  R_MOON,
  isHelioPhase,
  type Phase,
} from "./constants";
import { circularState, ellipseState, hypot3, rotateXZ } from "./kepler";

export interface CislunarSample {
  t: number;
  r: [number, number, number];
  v: [number, number, number];
  moon: [number, number, number];
  earthH: [number, number, number];
  mars: [number, number, number];
  phase: Phase;
  altEarth: number;
  altMoon: number;
  altMars: number;
  speed: number;
}

export interface CislunarMission {
  samples: CislunarSample[];
  duration: number;
  tofCoast: number;
  tofHelio: number;
  dvTli: number;
  dvLoi: number;
  dvTmi: number;
  dvMoi: number;
  tTli: number;
  tCapture: number;
  tTmi: number;
  tMoi: number;
  periodLlo: number;
  periodMoon: number;
  periodLmo: number;
  periodMars: number;
  angle0: number;
  thPeri: number;
  aTransfer: number;
  eTransfer: number;
  thetaE0: number;
  thetaM0: number;
  nEarth: number;
  nMars: number;
  phases: { phase: Phase; t0: number; t1: number; label: string }[];
}

const SIDEREAL_YEAR = 365.256363 * 86400;

function bodyOnRail(
  radius: number,
  n: number,
  theta0: number,
  t: number,
): [number, number, number] {
  const th = theta0 + n * t;
  return [radius * Math.cos(th), 0, radius * Math.sin(th)];
}

function moonOnRail(t: number, angle0: number): [number, number, number] {
  const n = Math.sqrt(MU_EARTH / (A_MOON * A_MOON * A_MOON));
  return bodyOnRail(A_MOON, n, angle0, t);
}

function pack(
  t: number,
  r: [number, number, number],
  v: [number, number, number],
  moon: [number, number, number],
  earthH: [number, number, number],
  mars: [number, number, number],
  phase: Phase,
): CislunarSample {
  const speed = hypot3(v[0], v[1], v[2]);
  if (isHelioPhase(phase)) {
    return {
      t,
      r,
      v,
      moon,
      earthH,
      mars,
      phase,
      altEarth: hypot3(r[0] - earthH[0], r[1] - earthH[1], r[2] - earthH[2]) - R_EARTH,
      altMoon: hypot3(r[0] - earthH[0] - moon[0], r[1] - earthH[1] - moon[1], r[2] - earthH[2] - moon[2]) - R_MOON,
      altMars: hypot3(r[0] - mars[0], r[1] - mars[1], r[2] - mars[2]) - R_MARS,
      speed,
    };
  }
  return {
    t,
    r,
    v,
    moon,
    earthH,
    mars,
    phase,
    altEarth: hypot3(r[0], r[1], r[2]) - R_EARTH,
    altMoon: hypot3(r[0] - moon[0], r[1] - moon[1], r[2] - moon[2]) - R_MOON,
    altMars: hypot3(earthH[0] - mars[0], earthH[1] - mars[1], earthH[2] - mars[2]) - R_MARS,
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

function phaseOfLmo(tau: number, periodLmo: number): Phase {
  if (tau < 60) return "moi";
  if (tau < 4 * periodLmo) return "lmo";
  return "marsrev";
}

function helioTransfer(
  tau: number,
  a: number,
  e: number,
  thPeri: number,
): { r: [number, number, number]; v: [number, number, number] } {
  const s = ellipseState(MU_SUN, a, e, tau);
  return rotateXZ(s.r, s.v, thPeri);
}

function lmoAroundMars(
  t: number,
  tMoi: number,
  thetaM0: number,
  nMars: number,
): { r: [number, number, number]; v: [number, number, number]; mars: [number, number, number] } {
  const mars = bodyOnRail(A_MARS, nMars, thetaM0, t);
  const n = Math.sqrt(MU_MARS / (R_LMO * R_LMO * R_LMO));
  const th = n * (t - tMoi);
  const s = circularState(MU_MARS, R_LMO, th, mars[0], mars[1], mars[2]);
  return { r: s.r, v: s.v, mars };
}

function collectPhases(samples: CislunarSample[]): CislunarMission["phases"] {
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
  phases.push({
    phase: cur,
    t0: start,
    t1: samples[samples.length - 1]!.t,
    label: PHASE_LABEL[cur],
  });
  return phases;
}

/** Earth → Moon patched-conic, then Hohmann Earth → Mars and one Mars year. */
export function buildCislunarMission(): CislunarMission {
  const rp = R_LEO;
  const ra = A_MOON;
  const aMoon = (rp + ra) / 2;
  const eMoon = (ra - rp) / (ra + rp);
  const tofCoast = Math.PI * Math.sqrt((aMoon * aMoon * aMoon) / MU_EARTH);
  const periodMoon = 2 * Math.PI * Math.sqrt((A_MOON * A_MOON * A_MOON) / MU_EARTH);
  const periodLlo = 2 * Math.PI * Math.sqrt((R_LLO * R_LLO * R_LLO) / MU_MOON);
  const nMoon = (2 * Math.PI) / periodMoon;
  const angle0 = Math.PI - nMoon * tofCoast;

  const vLeo = Math.sqrt(MU_EARTH / R_LEO);
  const peri = ellipseState(MU_EARTH, aMoon, eMoon, 0);
  const dvTli = hypot3(peri.v[0], peri.v[1], peri.v[2]) - vLeo;
  const dvLoi = 0.85;

  const aTransfer = (A_EARTH + A_MARS) / 2;
  const eTransfer = (A_MARS - A_EARTH) / (A_MARS + A_EARTH);
  const tofHelio = Math.PI * Math.sqrt((aTransfer * aTransfer * aTransfer) / MU_SUN);
  const periodMars = 2 * Math.PI * Math.sqrt((A_MARS * A_MARS * A_MARS) / MU_SUN);
  const periodLmo = 2 * Math.PI * Math.sqrt((R_LMO * R_LMO * R_LMO) / MU_MARS);
  const nEarth = (2 * Math.PI) / SIDEREAL_YEAR;
  const nMars = (2 * Math.PI) / periodMars;
  const thetaE0 = 0.62 + Math.PI;

  const tLeo = 0.45 * 2 * Math.PI * Math.sqrt((R_LEO * R_LEO * R_LEO) / MU_EARTH);
  const tTmiPred = tLeo + tofCoast + periodMoon;
  const thPeri = thetaE0 + nEarth * tTmiPred;
  const thetaM0 = thPeri + Math.PI - nMars * tofHelio - nMars * tTmiPred;

  const vEarth = Math.sqrt(MU_SUN / A_EARTH);
  const transP = ellipseState(MU_SUN, aTransfer, eTransfer, 0);
  const dvTmi = hypot3(transP.v[0], transP.v[1], transP.v[2]) - vEarth;
  const transA = ellipseState(MU_SUN, aTransfer, eTransfer, tofHelio);
  const vMars = Math.sqrt(MU_SUN / A_MARS);
  const dvMoi = vMars - hypot3(transA.v[0], transA.v[1], transA.v[2]);

  const rails = (t: number) => ({
    earthH: bodyOnRail(A_EARTH, nEarth, thetaE0, t),
    mars: bodyOnRail(A_MARS, nMars, thetaM0, t),
  });

  const samples: CislunarSample[] = [];
  const nLeo = 40;
  for (let i = 0; i < nLeo; i++) {
    const tUn = (i / (nLeo - 1)) * tLeo - tLeo;
    const t = tUn + tLeo;
    const th = (i / (nLeo - 1)) * 0.9 * Math.PI - 0.9 * Math.PI;
    const s = circularState(MU_EARTH, R_LEO, th, 0, 0, 0);
    const h = rails(t);
    samples.push(pack(t, s.r, s.v, moonOnRail(0, angle0), h.earthH, h.mars, "leo"));
  }

  const nCoast = 220;
  for (let i = 0; i < nCoast; i++) {
    const tUn = (i / (nCoast - 1)) * tofCoast;
    const t = tUn + tLeo;
    const s = ellipseState(MU_EARTH, aMoon, eMoon, tUn);
    const phase: Phase = i === 0 ? "tli" : i === nCoast - 1 ? "loi" : "coast";
    const h = rails(t);
    samples.push(pack(t, s.r, s.v, moonOnRail(tUn, angle0), h.earthH, h.mars, phase));
  }

  const tLoiPhys = tofCoast;
  const nLloShow = 4;
  const nDense = 96;
  for (let i = 0; i < nDense; i++) {
    const tau = (i / (nDense - 1)) * nLloShow * periodLlo;
    const phys = tLoiPhys + tau;
    const t = phys + tLeo;
    const s = lloAroundMoon(phys, angle0, tLoiPhys);
    const h = rails(t);
    samples.push(pack(t, s.r, s.v, s.moon, h.earthH, h.mars, phaseOfLlo(tau, periodLlo)));
  }

  const tRev0 = nLloShow * periodLlo;
  const nRev = 240;
  for (let i = 1; i < nRev; i++) {
    const tau = tRev0 + (i / (nRev - 1)) * (periodMoon - tRev0);
    const phys = tLoiPhys + tau;
    const t = phys + tLeo;
    const s = lloAroundMoon(phys, angle0, tLoiPhys);
    const h = rails(t);
    samples.push(pack(t, s.r, s.v, s.moon, h.earthH, h.mars, "revolution"));
  }

  const tTli = tLeo;
  const tCapture = tLeo + tofCoast;
  const tTmi = samples[samples.length - 1]!.t;
  const tMoi = tTmi + tofHelio;

  const nHelio = 220;
  for (let i = 0; i < nHelio; i++) {
    const tau = (i / (nHelio - 1)) * tofHelio;
    const t = tTmi + tau;
    const s = helioTransfer(tau, aTransfer, eTransfer, thPeri);
    const phase: Phase = i === 0 ? "tmi" : i === nHelio - 1 ? "moi" : "heliocoast";
    const h = rails(t);
    const moon = moonOnRail(t - tTli, angle0);
    samples.push(pack(t, s.r, s.v, moon, h.earthH, h.mars, phase));
  }

  const nLmoShow = 4;
  const nLmo = 80;
  for (let i = 1; i < nLmo; i++) {
    const tau = (i / (nLmo - 1)) * nLmoShow * periodLmo;
    const t = tMoi + tau;
    const s = lmoAroundMars(t, tMoi, thetaM0, nMars);
    const h = rails(t);
    const moon = moonOnRail(t - tTli, angle0);
    samples.push(pack(t, s.r, s.v, moon, h.earthH, s.mars, phaseOfLmo(tau, periodLmo)));
  }

  const tMarsRev0 = nLmoShow * periodLmo;
  const nMarsRev = 240;
  for (let i = 1; i < nMarsRev; i++) {
    const tau = tMarsRev0 + (i / (nMarsRev - 1)) * (periodMars - tMarsRev0);
    const t = tMoi + tau;
    const s = lmoAroundMars(t, tMoi, thetaM0, nMars);
    const h = rails(t);
    const moon = moonOnRail(t - tTli, angle0);
    samples.push(pack(t, s.r, s.v, moon, h.earthH, s.mars, "marsrev"));
  }

  const duration = samples[samples.length - 1]!.t;

  return {
    samples,
    duration,
    tofCoast,
    tofHelio,
    dvTli,
    dvLoi,
    dvTmi,
    dvMoi,
    tTli,
    tCapture,
    tTmi,
    tMoi,
    periodLlo,
    periodMoon,
    periodLmo,
    periodMars,
    angle0,
    thPeri,
    aTransfer,
    eTransfer,
    thetaE0,
    thetaM0,
    nEarth,
    nMars,
    phases: collectPhases(samples),
  };
}

export function sampleAt(mission: CislunarMission, t: number): CislunarSample {
  const tt = Math.min(Math.max(t, 0), mission.duration);
  const earthH = bodyOnRail(A_EARTH, mission.nEarth, mission.thetaE0, tt);
  const marsH = bodyOnRail(A_MARS, mission.nMars, mission.thetaM0, tt);
  const moonPhys = tt - mission.tTli;

  if (tt + 1e-6 >= mission.tMoi) {
    const s = lmoAroundMars(tt, mission.tMoi, mission.thetaM0, mission.nMars);
    const tau = tt - mission.tMoi;
    const moon = moonOnRail(moonPhys, mission.angle0);
    return pack(tt, s.r, s.v, moon, earthH, s.mars, phaseOfLmo(tau, mission.periodLmo));
  }
  if (tt + 1e-6 >= mission.tTmi) {
    const tau = tt - mission.tTmi;
    const s = helioTransfer(tau, mission.aTransfer, mission.eTransfer, mission.thPeri);
    const moon = moonOnRail(moonPhys, mission.angle0);
    const phase: Phase = tau < 45 ? "tmi" : tau > mission.tofHelio - 45 ? "moi" : "heliocoast";
    return pack(tt, s.r, s.v, moon, earthH, marsH, phase);
  }
  if (tt + 1e-6 >= mission.tCapture) {
    const phys = tt - mission.tTli;
    const s = lloAroundMoon(phys, mission.angle0, mission.tCapture - mission.tTli);
    const tau = tt - mission.tCapture;
    return pack(tt, s.r, s.v, s.moon, earthH, marsH, phaseOfLlo(tau, mission.periodLlo));
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
  const phase = isHelioPhase(a.phase) ? a.phase : u < 0.5 ? a.phase : b.phase;
  return pack(tt, lerp(a.r, b.r), lerp(a.v, b.v), lerp(a.moon, b.moon), earthH, marsH, phase);
}
