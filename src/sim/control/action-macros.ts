/**
 * Offline 2–4 segment pulse/coast macros. Diagnostic library only —
 * not imported by the online planner.
 *
 * Each pulse still obeys 40 ms min-width, 120 ms delay, max two jets,
 * and the fuel floor. Macros are evaluated on public representative
 * states; they are not a controller.
 */
import { CMD_DELAY, FMAX, MAX_ACTIVE, MIN_PULSE, THRUSTERS } from "../constants";
import { angularMomentumCmB, massState } from "../dynamics";
import {
  attitudeErrorAngle,
  deg,
  makeRng,
  qmul,
  qnormalize,
  vdot,
  vnorm,
  vscale,
  vsub,
  type Quat,
  type Vec3,
} from "../math3d";
import type { PublicConfig, SimState } from "../types";
import {
  generatePulsePrimitives,
  isLegalPulsePrimitive,
  leavesFuelFloor,
  propellantFor,
  type PulseDurationS,
  type PulsePrimitive,
} from "./discrete-actions";
import {
  applyPrimitiveUntilComplete,
  DEFAULT_ROLLOUT_CONFIG,
  rolloutAdvance,
  rolloutFromSimLike,
  type RolloutParameters,
  type RolloutState,
} from "./rollout-model";
import { makePublicState, publicBelief, publicStateSpecs } from "./rollout-error";
import { eigenComponents } from "./terminal-reachable";

export const MACRO_PULSE_DURATIONS = [0.04, 0.08, 0.16] as const satisfies readonly PulseDurationS[];
export const MACRO_COAST_DURATIONS = [0.08, 0.16, 0.24] as const satisfies readonly PulseDurationS[];
export const MACRO_FUEL_FLOOR = 2.8;
export const OPPOSITE_JET: readonly number[] = [1, 0, 3, 2, 5, 4];

export type MacroTemplate = "pulse-coast" | "pulse-pulse" | "pulse-coast-pulse" | "pulse-coast-pulse-coast";

export interface ActionMacro {
  readonly id: string;
  readonly template: MacroTemplate;
  readonly segments: readonly PulsePrimitive[];
  readonly totalDurationS: number;
  readonly propellantKg: number;
  readonly jetSet: readonly number[];
}

export interface MacroEval {
  readonly macroId: string;
  readonly template: MacroTemplate;
  readonly stateId: string;
  readonly isolated: readonly number[];
  readonly legal: boolean;
  readonly reason: string;
  readonly dtHPar: number;
  readonly dtHPerp: number;
  readonly hParReduction: number;
  readonly attDriftDeg: number;
  readonly dOmega: number;
  readonly dWPar: number;
  readonly dWPerp: number;
  readonly fuelKg: number;
  readonly durationS: number;
  readonly robustness: {
    readonly n: number;
    readonly hParReductionMean: number;
    readonly hParReductionStd: number;
    readonly signAgree: number;
    readonly hPerpMean: number;
  } | null;
}

function coastPrimitive(durationS: PulseDurationS): PulsePrimitive {
  return {
    id: `coast:${durationS.toFixed(3)}`,
    thrusterIds: [],
    durationS,
    commandedThrustN: 0,
    propellantKg: 0,
  };
}

function pulsePrimitive(ids: readonly number[], durationS: PulseDurationS): PulsePrimitive {
  const sorted = [...ids].sort((a, b) => a - b);
  return {
    id: `pulse:${sorted.join("+")}:${durationS.toFixed(3)}`,
    thrusterIds: sorted,
    durationS,
    commandedThrustN: FMAX,
    propellantKg: propellantFor(sorted.length, FMAX, durationS),
  };
}

function spanOf(segments: readonly PulsePrimitive[]): number {
  let t = 0;
  for (const seg of segments) {
    t += seg.thrusterIds.length === 0 ? seg.durationS : CMD_DELAY + seg.durationS;
  }
  return t;
}

function jetsOf(segments: readonly PulsePrimitive[]): number[] {
  const s = new Set<number>();
  for (const seg of segments) for (const id of seg.thrusterIds) s.add(id);
  return [...s].sort((a, b) => a - b);
}

function makeMacro(template: MacroTemplate, segments: readonly PulsePrimitive[]): ActionMacro {
  const propellantKg = segments.reduce((a, s) => a + s.propellantKg, 0);
  return {
    id: `${template}:${segments.map((s) => s.id).join("|")}`,
    template,
    segments,
    totalDurationS: spanOf(segments),
    propellantKg,
    jetSet: jetsOf(segments),
  };
}

export function generateActionMacros(): ActionMacro[] {
  const singles: PulsePrimitive[] = [];
  for (const d of MACRO_PULSE_DURATIONS) {
    for (let id = 0; id < 6; id += 1) singles.push(pulsePrimitive([id], d));
  }
  const coasts = MACRO_COAST_DURATIONS.map((d) => coastPrimitive(d));
  const out: ActionMacro[] = [];

  for (const p of singles) {
    for (const c of coasts) out.push(makeMacro("pulse-coast", [p, c]));
  }

  for (const a of singles) {
    const aId = a.thrusterIds[0]!;
    for (const d of MACRO_PULSE_DURATIONS) {
      const opp = pulsePrimitive([OPPOSITE_JET[aId]!], d);
      out.push(makeMacro("pulse-pulse", [a, opp]));
      const next = (aId + 2) % 6;
      if (next !== OPPOSITE_JET[aId]) out.push(makeMacro("pulse-pulse", [a, pulsePrimitive([next], d)]));
    }
  }

  const short = MACRO_PULSE_DURATIONS.slice(0, 2) as PulseDurationS[];
  const midCoast = coastPrimitive(0.16);
  for (const dA of short) {
    for (let a = 0; a < 6; a += 1) {
      const pA = pulsePrimitive([a], dA);
      for (const dB of short) {
        out.push(makeMacro("pulse-coast-pulse", [pA, midCoast, pulsePrimitive([OPPOSITE_JET[a]!], dB)]));
        out.push(makeMacro("pulse-coast-pulse", [pA, midCoast, pulsePrimitive([(a + 2) % 6], dB)]));
      }
    }
  }

  const c80 = coastPrimitive(0.08);
  for (let a = 0; a < 6; a += 1) {
    const pA = pulsePrimitive([a], 0.04);
    const pB = pulsePrimitive([OPPOSITE_JET[a]!], 0.04);
    const pC = pulsePrimitive([(a + 2) % 6], 0.04);
    out.push(makeMacro("pulse-coast-pulse-coast", [pA, c80, pB, c80]));
    out.push(makeMacro("pulse-coast-pulse-coast", [pA, c80, pC, c80]));
  }

  const seen = new Set<string>();
  return out.filter((m) => {
    if (seen.has(m.id)) return false;
    seen.add(m.id);
    return true;
  });
}

export function macroIsLegal(
  macro: ActionMacro,
  isolated: ReadonlySet<number>,
  estimatedFuelKg: number,
  fuelFloorKg = MACRO_FUEL_FLOOR,
): { legal: boolean; reason: string } {
  if (macro.segments.length < 2 || macro.segments.length > 4) {
    return { legal: false, reason: "segment-count" };
  }
  let fuel = estimatedFuelKg;
  for (const seg of macro.segments) {
    if (!isLegalPulsePrimitive(seg, THRUSTERS, isolated)) return { legal: false, reason: `illegal:${seg.id}` };
    if (seg.thrusterIds.length > MAX_ACTIVE) return { legal: false, reason: "max-active" };
    if (seg.thrusterIds.length > 0 && seg.durationS + 1e-12 < MIN_PULSE) {
      return { legal: false, reason: "min-pulse" };
    }
    if (seg.thrusterIds.some((id) => isolated.has(id))) return { legal: false, reason: "isolated-jet" };
    if (!leavesFuelFloor(seg, fuel, fuelFloorKg, 0)) return { legal: false, reason: "fuel-floor" };
    fuel -= seg.propellantKg;
  }
  return { legal: true, reason: "ok" };
}

function toSim(state: RolloutState): SimState {
  return {
    t: state.time,
    rI: [0, 0, 0],
    vI: [0, 0, 0],
    rCmI: [state.rCmI[0], state.rCmI[1], state.rCmI[2]],
    vCmI: [state.vCmI[0], state.vCmI[1], state.vCmI[2]],
    q: [state.qBI[0], state.qBI[1], state.qBI[2], state.qBI[3]],
    w: [state.omegaB[0], state.omegaB[1], state.omegaB[2]],
    s: state.sliderS,
    sd: state.sliderV,
    th1: state.theta1,
    th1d: state.theta1Dot,
    th2: state.theta2,
    th2d: state.theta2Dot,
    fuel: state.fuelMass,
  };
}

function bodyH(plant: PublicConfig, state: RolloutState): Vec3 {
  return angularMomentumCmB(plant, toSim(state));
}

export function applyMacro(
  state: RolloutState,
  params: RolloutParameters,
  plant: PublicConfig,
  macro: ActionMacro,
): RolloutState {
  let s = state;
  const rcfg = DEFAULT_ROLLOUT_CONFIG;
  for (const seg of macro.segments) {
    if (seg.thrusterIds.length === 0) {
      s = rolloutAdvance(s, params, plant, seg.durationS, rcfg);
    } else {
      s = applyPrimitiveUntilComplete(s, params, plant, seg, rcfg);
    }
  }
  return s;
}

function evalPair(
  before: RolloutState,
  after: RolloutState,
  plant: PublicConfig,
): {
  dtHPar: number;
  dtHPerp: number;
  hParReduction: number;
  attDriftDeg: number;
  dOmega: number;
  dWPar: number;
  dWPerp: number;
  fuelKg: number;
} {
  const eigen0 = eigenComponents(before, plant);
  const eigen1 = eigenComponents(after, plant);
  const H0 = bodyH(plant, before);
  const H1 = bodyH(plant, after);
  const dH = vsub(H1, H0);
  const dtHPar = vdot(dH, eigen0.eN);
  const dtHPerp = vnorm(vsub(dH, vscale(eigen0.eN, dtHPar)));
  const hPar0 = vdot(H0, eigen0.eN);
  const hPar1 = vdot(H1, eigen0.eN);
  const att0 = deg(attitudeErrorAngle(before.qBI, plant.qTarget));
  const att1 = deg(attitudeErrorAngle(after.qBI, plant.qTarget));
  return {
    dtHPar,
    dtHPerp,
    hParReduction: Math.abs(hPar0) - Math.abs(hPar1),
    attDriftDeg: att1 - att0,
    dOmega: vnorm(after.omegaB) - vnorm(before.omegaB),
    dWPar: eigen1.wPar - eigen0.wPar,
    dWPerp: eigen1.wPerp - eigen0.wPerp,
    fuelKg: Math.max(0, before.fuelMass - after.fuelMass),
  };
}

function qAxisAngle(axis: Vec3, angle: number): Quat {
  const n = vnorm(axis) || 1;
  const a: Vec3 = [axis[0] / n, axis[1] / n, axis[2] / n];
  const h = 0.5 * angle;
  const s = Math.sin(h);
  return qnormalize([Math.cos(h), a[0] * s, a[1] * s, a[2] * s]);
}

function perturbState(state: RolloutState, seed: number, k: number): RolloutState {
  const rng = makeRng((seed + 17 * (k + 1)) >>> 0);
  const axis: Vec3 = [rng.gauss(), rng.gauss(), rng.gauss()];
  const dq = qAxisAngle(axis, ((2 * rng.u01() - 1) * 2 * Math.PI) / 180);
  const scale = 1 + 0.1 * rng.gauss();
  return {
    ...state,
    qBI: qnormalize(qmul(dq, state.qBI)),
    omegaB: [state.omegaB[0] * scale, state.omegaB[1] * scale, state.omegaB[2] * scale],
  };
}

export function evaluateMacroOnState(
  macro: ActionMacro,
  state: RolloutState,
  params: RolloutParameters,
  plant: PublicConfig,
  stateId: string,
  robustSeed = 1,
): MacroEval {
  const isolated = new Set(params.failedThrusterBeliefs);
  const gate = macroIsLegal(macro, isolated, state.fuelMass);
  if (!gate.legal) {
    return {
      macroId: macro.id,
      template: macro.template,
      stateId,
      isolated: [...isolated],
      legal: false,
      reason: gate.reason,
      dtHPar: 0,
      dtHPerp: 0,
      hParReduction: 0,
      attDriftDeg: 0,
      dOmega: 0,
      dWPar: 0,
      dWPerp: 0,
      fuelKg: 0,
      durationS: macro.totalDurationS,
      robustness: null,
    };
  }
  const after = applyMacro(state, params, plant, macro);
  const core = evalPair(state, after, plant);
  const reductions: number[] = [];
  const perps: number[] = [];
  for (let k = 0; k < 4; k += 1) {
    const p = perturbState(state, robustSeed, k);
    const a = applyMacro(p, params, plant, macro);
    const e = evalPair(p, a, plant);
    reductions.push(e.hParReduction);
    perps.push(e.dtHPerp);
  }
  const mean = reductions.reduce((a, b) => a + b, 0) / reductions.length;
  const var_ = reductions.reduce((a, b) => a + (b - mean) ** 2, 0) / reductions.length;
  const sign0 = core.hParReduction >= 0 ? 1 : -1;
  const signAgree = reductions.filter((v) => (v >= 0 ? 1 : -1) === sign0).length / reductions.length;
  return {
    macroId: macro.id,
    template: macro.template,
    stateId,
    isolated: [...isolated],
    legal: true,
    reason: "ok",
    ...core,
    durationS: after.time - state.time,
    robustness: {
      n: reductions.length,
      hParReductionMean: mean,
      hParReductionStd: Math.sqrt(var_),
      signAgree,
      hPerpMean: perps.reduce((a, b) => a + b, 0) / perps.length,
    },
  };
}

export function maxProjectionSingle(
  state: RolloutState,
  params: RolloutParameters,
  plant: PublicConfig,
): PulsePrimitive {
  const isolated = new Set(params.failedThrusterBeliefs);
  const all = generatePulsePrimitives(THRUSTERS, {
    isolatedThrusters: isolated,
    durationsS: MACRO_PULSE_DURATIONS,
    includeCoast: false,
  });
  const eigen = eigenComponents(state, plant);
  const ms = massState(plant, state.sliderS, state.theta1, state.theta2, state.fuelMass);
  let best: PulsePrimitive | null = null;
  let bestDot = -Infinity;
  for (const p of all) {
    if (p.thrusterIds.length !== 1) continue;
    const geom = THRUSTERS[p.thrusterIds[0]!];
    if (!geom) continue;
    const r: Vec3 = [geom.pos[0] - ms.rCmB[0], geom.pos[1] - ms.rCmB[1], geom.pos[2] - ms.rCmB[2]];
    const F = vscale(geom.dir, params.etaTEstimate * plant.maxThrust * p.durationS);
    const L: Vec3 = [
      r[1] * F[2] - r[2] * F[1],
      r[2] * F[0] - r[0] * F[2],
      r[0] * F[1] - r[1] * F[0],
    ];
    const want = -Math.sign(eigen.wPar || 1);
    const proj = want * (L[0] * eigen.eN[0] + L[1] * eigen.eN[1] + L[2] * eigen.eN[2]);
    if (proj > bestDot) {
      bestDot = proj;
      best = p;
    }
  }
  return (
    best ?? {
      id: "coast:0.040",
      thrusterIds: [],
      durationS: MIN_PULSE,
      commandedThrustN: 0,
      propellantKg: 0,
    }
  );
}

function dominates(a: MacroEval, b: MacroEval): boolean {
  const ge =
    a.hParReduction >= b.hParReduction - 1e-12 &&
    a.dtHPerp <= b.dtHPerp + 1e-12 &&
    a.fuelKg <= b.fuelKg + 1e-12 &&
    a.attDriftDeg <= b.attDriftDeg + 1e-12;
  const gt =
    a.hParReduction > b.hParReduction + 1e-9 ||
    a.dtHPerp < b.dtHPerp - 1e-9 ||
    a.fuelKg < b.fuelKg - 1e-9 ||
    a.attDriftDeg < b.attDriftDeg - 1e-9;
  return ge && gt;
}

export function paretoFront(evals: readonly MacroEval[]): MacroEval[] {
  const legal = evals.filter((e) => e.legal);
  return legal.filter((a) => !legal.some((b) => b.macroId !== a.macroId && dominates(b, a)));
}

export interface MacroStudyState {
  readonly id: string;
  readonly state: RolloutState;
  readonly params: RolloutParameters;
}

export function representativeMacroStates(plant: PublicConfig): MacroStudyState[] {
  const specs = publicStateSpecs().filter((s) => s.pending === "empty");
  const out: MacroStudyState[] = [];
  for (const spec of specs) {
    out.push({
      id: spec.id,
      state: makePublicState(spec, plant),
      params: publicBelief(plant, spec.isolatedIds),
    });
  }
  const extra: Array<{ id: string; attDeg: number; wmag: number; axis: Vec3; isolated: number[] }> = [
    { id: "closing|healthy|empty", attDeg: 25, wmag: 0.08, axis: [0.4, 1, -0.2], isolated: [] },
    { id: "opening|healthy|empty", attDeg: 25, wmag: 0.08, axis: [0.4, 1, -0.2], isolated: [] },
    { id: "closing|one-isolated|empty", attDeg: 18, wmag: 0.06, axis: [1, 0.2, 0.5], isolated: [2] },
    { id: "detumble|healthy|empty", attDeg: 8, wmag: 0.11, axis: [0.1, 0.2, 1], isolated: [] },
  ];
  for (const e of extra) {
    const attRad = (e.attDeg * Math.PI) / 180;
    const n = vnorm(e.axis) || 1;
    const ax: Vec3 = [e.axis[0] / n, e.axis[1] / n, e.axis[2] / n];
    const qErr = qAxisAngle(ax, attRad);
    const q = qnormalize(qmul(plant.qTarget, qErr));
    const sign = e.id.startsWith("opening") ? 1 : -1;
    const w: Vec3 = [ax[0] * e.wmag * sign, ax[1] * e.wmag * sign, ax[2] * e.wmag * sign];
    out.push({
      id: e.id,
      state: rolloutFromSimLike({
        time: 0,
        q,
        w,
        s: 0.2,
        sd: 0,
        th1: 0.06,
        th1d: 0,
        th2: -0.04,
        th2d: 0,
        fuel: 4.1,
      }),
      params: publicBelief(plant, e.isolated),
    });
  }
  return out;
}

export function runMacroStudy(plant: PublicConfig): {
  macros: number;
  states: number;
  evaluations: MacroEval[];
  pareto: MacroEval[];
  baseline: MacroEval[];
  fractionStatesWithDominatingMacro: number;
  omegaParReduced: number;
  omegaParReducedNoPerpGrowth: number;
  omegaParReducedModestPerp: number;
  perState: Array<{
    stateId: string;
    baselineDWPar: number;
    baselineDWPerp: number;
    bestDWPar: number;
    bestDWParId: string | null;
    bestDWParPerp: number;
    nOmegaParReduced: number;
    nNoPerpGrowth: number;
  }>;
} {
  const macros = generateActionMacros();
  const states = representativeMacroStates(plant);
  const evaluations: MacroEval[] = [];
  const baseline: MacroEval[] = [];
  let dominated = 0;
  let omegaParReduced = 0;
  let omegaParReducedNoPerpGrowth = 0;
  let omegaParReducedModestPerp = 0;
  const perState: Array<{
    stateId: string;
    baselineDWPar: number;
    baselineDWPerp: number;
    bestDWPar: number;
    bestDWParId: string | null;
    bestDWParPerp: number;
    nOmegaParReduced: number;
    nNoPerpGrowth: number;
  }> = [];
  for (const st of states) {
    const rows: MacroEval[] = [];
    for (const m of macros) {
      const row = evaluateMacroOnState(m, st.state, st.params, plant, st.id);
      rows.push(row);
      evaluations.push(row);
      if (row.legal && row.dWPar < -1e-6) {
        omegaParReduced += 1;
        if (row.dWPerp <= 1e-6) omegaParReducedNoPerpGrowth += 1;
        if (row.dWPerp < 0.001) omegaParReducedModestPerp += 1;
      }
    }
    const single = maxProjectionSingle(st.state, st.params, plant);
    const asMacro = makeMacro("pulse-coast", [single, coastPrimitive(0.08)]);
    const base = evaluateMacroOnState(asMacro, st.state, st.params, plant, st.id, 99);
    baseline.push(base);
    const front = paretoFront(rows);
    if (front.some((e) => dominates(e, base))) dominated += 1;
    const reducers = rows.filter((e) => e.legal && e.dWPar < -1e-6);
    const best = reducers.slice().sort((a, b) => a.dWPar - b.dWPar)[0];
    perState.push({
      stateId: st.id,
      baselineDWPar: base.dWPar,
      baselineDWPerp: base.dWPerp,
      bestDWPar: best?.dWPar ?? NaN,
      bestDWParId: best?.macroId ?? null,
      bestDWParPerp: best?.dWPerp ?? NaN,
      nOmegaParReduced: reducers.length,
      nNoPerpGrowth: reducers.filter((e) => e.dWPerp <= 1e-6).length,
    });
  }
  return {
    macros: macros.length,
    states: states.length,
    evaluations,
    pareto: paretoFront(evaluations),
    baseline,
    fractionStatesWithDominatingMacro: states.length === 0 ? 0 : dominated / states.length,
    omegaParReduced,
    omegaParReducedNoPerpGrowth,
    omegaParReducedModestPerp,
    perState,
  };
}
