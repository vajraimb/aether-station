/**
 * Offline low-rate, post-failure, η_T-robust terminal cancellation.
 * Not a controller. Not imported by DiscretePulseV2.
 *
 * States: ‖ω‖ ≤ 0.015 rad/s, e_q ≤ 15°. All single-jet masks.
 * Sequences: 8–32 segments, intermediate ω_⊥ allowed with a peak cap.
 * Robustness: worst-case over the estimator 2σ η_T interval, clipped
 * to the published etaRange — not a hand-set ±20% on (η,c,k12).
 */
import { ETA_RANGE, MAX_ACTIVE, MIN_PULSE, THRUSTERS } from "../constants";
import { angularMomentumCmB } from "../dynamics";
import {
  clamp,
  deg,
  qmul,
  qnormalize,
  vcross,
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
  type PulseDurationS,
  type PulsePrimitive,
} from "./discrete-actions";
import { maxProjectionSingle } from "./action-macros";
import { greedyThenCancel, searchActionSet } from "./nullspace-search";
import {
  applyPrimitiveUntilComplete,
  DEFAULT_ROLLOUT_CONFIG,
  geodesicAttitudeError,
  rolloutAdvance,
  rolloutFromSimLike,
  type RolloutParameters,
  type RolloutState,
} from "./rollout-model";
import { publicBelief } from "./rollout-error";
import {
  TERMINAL_ATT_GATE_RAD,
  TERMINAL_FUEL_GATE,
  TERMINAL_RATE_GATE,
  searchTerminal,
} from "./terminal-planner";
import { eigenComponents } from "./terminal-reachable";

export const RATE_CAP = 0.015;
export const ATT_CAP_DEG = 15;
export const PEAK_WPERP_LIMIT = 0.03;
export const FUEL_FLOOR = TERMINAL_FUEL_GATE;
export const DEFAULT_MAX_SEG = 24;
export const DEFAULT_BEAM = 10;
export const DEFAULT_ETA_HAT = 0.873;
export const DEFAULT_ETA_P = 0.01;
export const DEFAULT_ETA_Z = 2;
export const W_PAR = 1;
export const W_PERP = 2;
export const W_ATT = 50;
export const W_FUEL = 10;

export interface EtaInterval {
  readonly hat: number;
  readonly P: number;
  readonly sigma: number;
  readonly z: number;
  readonly min: number;
  readonly max: number;
}

export function etaInterval(hat = DEFAULT_ETA_HAT, P = DEFAULT_ETA_P, z = DEFAULT_ETA_Z): EtaInterval {
  const sigma = Math.sqrt(Math.max(0, P));
  return {
    hat,
    P,
    sigma,
    z,
    min: clamp(hat - z * sigma, ETA_RANGE[0], ETA_RANGE[1]),
    max: clamp(hat + z * sigma, ETA_RANGE[0], ETA_RANGE[1]),
  };
}

export function etaGrid(interval: EtaInterval, n = 5): number[] {
  const count = Math.max(3, n);
  const leftN = Math.floor((count - 3) / 2);
  const rightN = count - 3 - leftN;
  const out: number[] = [];
  const push = (x: number) => {
    if (!out.some((y) => Math.abs(y - x) < 1e-12)) out.push(x);
  };
  push(interval.min);
  for (let i = 1; i <= leftN; i += 1) {
    push(interval.min + ((interval.hat - interval.min) * i) / (leftN + 1));
  }
  push(interval.hat);
  for (let i = 1; i <= rightN; i += 1) {
    push(interval.hat + ((interval.max - interval.hat) * i) / (rightN + 1));
  }
  push(interval.max);
  return out;
}

export type RateKind = "close" | "open" | "rest";

export interface TerminalFamily {
  readonly name: string;
  readonly attDeg: number;
  readonly wmag: number;
  readonly kind: RateKind;
  readonly axis: Vec3;
}

export const TERMINAL_FAMILIES: readonly TerminalFamily[] = [
  { name: "near-close", attDeg: 2.5, wmag: 0.006, kind: "close", axis: [0.1, 0.2, 1] },
  { name: "mid-close", attDeg: 8, wmag: 0.01, kind: "close", axis: [0.2, 1, -0.3] },
  { name: "entry-close", attDeg: 14, wmag: 0.012, kind: "close", axis: [0.4, 0.7, 0.5] },
  { name: "mid-rest", attDeg: 8, wmag: 0.008, kind: "rest", axis: [0.2, 1, -0.3] },
  { name: "mid-open", attDeg: 8, wmag: 0.006, kind: "open", axis: [0.3, -0.5, 0.8] },
  { name: "near-rest", attDeg: 3, wmag: 0.004, kind: "rest", axis: [1, 0.2, 0.1] },
];

export const ALL_MASKS: readonly (readonly number[])[] = [[], [0], [1], [2], [3], [4], [5]];

function qAxisAngle(axis: Vec3, angle: number): Quat {
  const n = vnorm(axis) || 1;
  const a: Vec3 = [axis[0] / n, axis[1] / n, axis[2] / n];
  const h = 0.5 * angle;
  const s = Math.sin(h);
  return qnormalize([Math.cos(h), a[0] * s, a[1] * s, a[2] * s]);
}

function unit(v: Vec3): Vec3 {
  const n = vnorm(v);
  return n > 1e-12 ? vscale(v, 1 / n) : [1, 0, 0];
}

function perpOf(e: Vec3): Vec3 {
  const a: Vec3 = Math.abs(e[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
  const p = vcross(e, a);
  return unit(p);
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

export function bodyH(plant: PublicConfig, state: RolloutState): Vec3 {
  return angularMomentumCmB(plant, toSim(state));
}

export function terminalCostTerms(
  state: RolloutState,
  plant: PublicConfig,
  startFuel: number,
): { attRad: number; omega: number; hPar: number; hPerp: number; fuelUsed: number; wPerp: number } {
  const eigen = eigenComponents(state, plant);
  const H = bodyH(plant, state);
  const hPar = vdot(H, eigen.eN);
  const hPerp = vnorm(vsub(H, vscale(eigen.eN, hPar)));
  return {
    attRad: geodesicAttitudeError(state.qBI, plant.qTarget),
    omega: vnorm(state.omegaB),
    hPar,
    hPerp,
    fuelUsed: Math.max(0, startFuel - state.fuelMass),
    wPerp: eigen.wPerp,
  };
}

export function terminalCost(terms: { attRad: number; hPar: number; hPerp: number; fuelUsed: number }): number {
  return W_PAR * Math.abs(terms.hPar) + W_PERP * terms.hPerp + W_ATT * terms.attRad + W_FUEL * terms.fuelUsed;
}

export interface TerminalCase {
  readonly id: string;
  readonly family: string;
  readonly isolated: readonly number[];
  readonly state: RolloutState;
  readonly params: RolloutParameters;
}

export function makeTerminalCase(plant: PublicConfig, family: TerminalFamily, isolated: readonly number[]): TerminalCase {
  const attRad = (family.attDeg * Math.PI) / 180;
  const e = unit(family.axis);
  const qErr = qAxisAngle(e, attRad);
  const q = qnormalize(qmul(plant.qTarget, qErr));
  let w: Vec3;
  if (family.kind === "close") w = vscale(e, -family.wmag);
  else if (family.kind === "open") w = vscale(e, family.wmag);
  else w = vscale(perpOf(e), family.wmag);
  const mask = isolated.length === 0 ? "healthy" : `iso${isolated.join("+")}`;
  return {
    id: `${family.name}|${mask}`,
    family: family.name,
    isolated: [...isolated],
    state: rolloutFromSimLike({
      time: 0,
      q,
      w,
      s: 0.2,
      sd: 0,
      th1: 0.05,
      th1d: 0,
      th2: -0.03,
      th2d: 0,
      fuel: 3.2,
    }),
    params: publicBelief(plant, isolated),
  };
}

export function lowRateCases(plant: PublicConfig, opts: { quick?: boolean } = {}): TerminalCase[] {
  const families = opts.quick ? TERMINAL_FAMILIES.slice(0, 2) : TERMINAL_FAMILIES;
  const masks = opts.quick ? ([[], [0], [2]] as const) : ALL_MASKS;
  const out: TerminalCase[] = [];
  for (const f of families) {
    for (const m of masks) out.push(makeTerminalCase(plant, f, m));
  }
  return out;
}

export function caseWithinCaps(c: TerminalCase, plant: PublicConfig): boolean {
  const att = deg(geodesicAttitudeError(c.state.qBI, plant.qTarget));
  const w = vnorm(c.state.omegaB);
  return att <= ATT_CAP_DEG + 1e-6 && w <= RATE_CAP + 1e-9 && c.state.fuelMass > FUEL_FLOOR;
}

function withEta(params: RolloutParameters, eta: number): RolloutParameters {
  return { ...params, etaTEstimate: eta };
}

function applySeg(
  state: RolloutState,
  params: RolloutParameters,
  plant: PublicConfig,
  prim: PulsePrimitive,
): RolloutState {
  if (prim.thrusterIds.length === 0) {
    return rolloutAdvance(state, params, plant, prim.durationS, DEFAULT_ROLLOUT_CONFIG);
  }
  return applyPrimitiveUntilComplete(state, params, plant, prim, DEFAULT_ROLLOUT_CONFIG);
}

function legalNext(prim: PulsePrimitive, isolated: ReadonlySet<number>, fuel: number): boolean {
  if (!isLegalPulsePrimitive(prim, THRUSTERS, isolated)) return false;
  if (prim.thrusterIds.length > MAX_ACTIVE) return false;
  if (prim.thrusterIds.length > 0 && prim.durationS + 1e-12 < MIN_PULSE) return false;
  if (!leavesFuelFloor(prim, fuel, FUEL_FLOOR, 0)) return false;
  return true;
}

const SEARCH_PULSE_S = [0.04, 0.08, 0.16] as const satisfies readonly PulseDurationS[];

export function terminalActionSet(isolated: ReadonlySet<number>): PulsePrimitive[] {
  const pulses = generatePulsePrimitives(THRUSTERS, {
    isolatedThrusters: isolated,
    durationsS: SEARCH_PULSE_S,
    includeCoast: false,
  });
  const singles = pulses.filter((p) => p.thrusterIds.length === 1);
  const pairs = pulses.filter((p) => p.thrusterIds.length === 2 && p.durationS === 0.04);
  const coast: PulsePrimitive = {
    id: "coast:0.080",
    thrusterIds: [],
    durationS: 0.08,
    commandedThrustN: 0,
    propellantKg: 0,
  };
  return [...singles, ...pairs, coast].filter((p) => isLegalPulsePrimitive(p, THRUSTERS, isolated));
}

export interface SampleMetrics {
  readonly eta: number;
  readonly attDeg: number;
  readonly omega: number;
  readonly hPar: number;
  readonly hPerp: number;
  readonly fuel: number;
  readonly fuelUsed: number;
  readonly peakWPerp: number;
  readonly J: number;
  readonly captured: boolean;
  readonly fuelOk: boolean;
}

function metricsAt(
  state: RolloutState,
  plant: PublicConfig,
  startFuel: number,
  eta: number,
  peakWPerp: number,
): SampleMetrics {
  const t = terminalCostTerms(state, plant, startFuel);
  const attDeg = deg(t.attRad);
  const fuelOk = state.fuelMass > FUEL_FLOOR;
  const captured =
    t.attRad < TERMINAL_ATT_GATE_RAD && t.omega < TERMINAL_RATE_GATE && fuelOk;
  return {
    eta,
    attDeg,
    omega: t.omega,
    hPar: t.hPar,
    hPerp: t.hPerp,
    fuel: state.fuelMass,
    fuelUsed: t.fuelUsed,
    peakWPerp,
    J: terminalCost(t),
    captured,
    fuelOk,
  };
}

export function replayOnEta(
  root: RolloutState,
  params: RolloutParameters,
  plant: PublicConfig,
  seq: readonly PulsePrimitive[],
  eta: number,
): SampleMetrics {
  const p = withEta(params, eta);
  let s = root;
  let peak = eigenComponents(root, plant).wPerp;
  for (const prim of seq) {
    s = applySeg(s, p, plant, prim);
    peak = Math.max(peak, eigenComponents(s, plant).wPerp);
  }
  return metricsAt(s, plant, root.fuelMass, eta, peak);
}

export interface SequenceEval {
  readonly method: string;
  readonly nSeg: number;
  readonly ids: readonly string[];
  readonly nominal: SampleMetrics;
  readonly p10: SampleMetrics;
  readonly worst: SampleMetrics;
  readonly grid: readonly SampleMetrics[];
}

export function evaluateSequence(
  method: string,
  root: RolloutState,
  params: RolloutParameters,
  plant: PublicConfig,
  seq: readonly PulsePrimitive[],
  grid: readonly number[],
  hat: number,
): SequenceEval {
  const samples = grid.map((eta) => replayOnEta(root, params, plant, seq, eta));
  const byJ = [...samples].sort((a, b) => a.J - b.J);
  const p10 = byJ[Math.min(byJ.length - 1, Math.max(0, Math.ceil(0.1 * byJ.length) - 1))]!;
  const worst = byJ[byJ.length - 1]!;
  const nominal = samples.find((s) => Math.abs(s.eta - hat) < 1e-9) ?? samples[Math.floor(samples.length / 2)]!;
  return {
    method,
    nSeg: seq.length,
    ids: seq.map((p) => p.id),
    nominal,
    p10,
    worst,
    grid: samples,
  };
}

interface RobustNode {
  seq: PulsePrimitive[];
  nom: RolloutState;
  lo: RolloutState;
  hi: RolloutState;
  peakWPerp: number;
  Jnom: number;
  Jlo: number;
  Jhi: number;
  Jworst: number;
  attWorst: number;
  omegaWorst: number;
  attResid: number;
  omegaResid: number;
  nCaptured: number;
  illegal: boolean;
  fuelUsedNom: number;
}

function scoreNode(
  seq: PulsePrimitive[],
  nom: RolloutState,
  lo: RolloutState,
  hi: RolloutState,
  plant: PublicConfig,
  startFuel: number,
  peakWPerp: number,
): RobustNode {
  const mn = metricsAt(nom, plant, startFuel, DEFAULT_ETA_HAT, peakWPerp);
  const ml = metricsAt(lo, plant, startFuel, 0, peakWPerp);
  const mh = metricsAt(hi, plant, startFuel, 0, peakWPerp);
  const fuelFail = !mn.fuelOk || !ml.fuelOk || !mh.fuelOk;
  const peakFail = peakWPerp > PEAK_WPERP_LIMIT + 1e-9;
  return {
    seq,
    nom,
    lo,
    hi,
    peakWPerp,
    Jnom: mn.J,
    Jlo: ml.J,
    Jhi: mh.J,
    Jworst: Math.max(mn.J, ml.J, mh.J),
    attWorst: Math.max(mn.attDeg, ml.attDeg, mh.attDeg),
    omegaWorst: Math.max(mn.omega, ml.omega, mh.omega),
    attResid: Math.max(0, Math.max(mn.attDeg, ml.attDeg, mh.attDeg) - deg(TERMINAL_ATT_GATE_RAD)),
    omegaResid: Math.max(0, Math.max(mn.omega, ml.omega, mh.omega) - TERMINAL_RATE_GATE),
    nCaptured: (mn.captured ? 1 : 0) + (ml.captured ? 1 : 0) + (mh.captured ? 1 : 0),
    illegal: fuelFail || peakFail,
    fuelUsedNom: mn.fuelUsed,
  };
}

function cmpNode(a: RobustNode, b: RobustNode): number {
  const ka = [
    a.illegal ? 1 : 0,
    -a.nCaptured,
    a.attResid,
    a.omegaResid,
    a.Jworst,
    a.fuelUsedNom,
    a.seq.length,
  ];
  const kb = [
    b.illegal ? 1 : 0,
    -b.nCaptured,
    b.attResid,
    b.omegaResid,
    b.Jworst,
    b.fuelUsedNom,
    b.seq.length,
  ];
  for (let i = 0; i < ka.length; i += 1) {
    if (ka[i]! < kb[i]! - 1e-12) return -1;
    if (ka[i]! > kb[i]! + 1e-12) return 1;
  }
  return 0;
}

export function robustCancelSearch(
  root: RolloutState,
  params: RolloutParameters,
  plant: PublicConfig,
  interval: EtaInterval,
  maxSeg = DEFAULT_MAX_SEG,
  beamWidth = DEFAULT_BEAM,
): PulsePrimitive[] {
  const isolated = new Set(params.failedThrusterBeliefs);
  const actions = terminalActionSet(isolated);
  const pNom = withEta(params, interval.hat);
  const pLo = withEta(params, interval.min);
  const pHi = withEta(params, interval.max);
  const peak0 = eigenComponents(root, plant).wPerp;
  const startFuel = root.fuelMass;
  let beam: RobustNode[] = [scoreNode([], root, root, root, plant, startFuel, peak0)];
  let best: RobustNode | null = null;
  for (let depth = 0; depth < maxSeg; depth += 1) {
    const kids: RobustNode[] = [];
    for (const n of beam) {
      if (n.illegal) continue;
      for (const a of actions) {
        if (!legalNext(a, isolated, n.nom.fuelMass)) continue;
        const nom = applySeg(n.nom, pNom, plant, a);
        const lo = applySeg(n.lo, pLo, plant, a);
        const hi = applySeg(n.hi, pHi, plant, a);
        const peak = Math.max(
          n.peakWPerp,
          eigenComponents(nom, plant).wPerp,
          eigenComponents(lo, plant).wPerp,
          eigenComponents(hi, plant).wPerp,
        );
        kids.push(scoreNode([...n.seq, a], nom, lo, hi, plant, startFuel, peak));
      }
    }
    if (kids.length === 0) break;
    kids.sort(cmpNode);
    beam = kids.slice(0, beamWidth);
    for (const n of beam) {
      if (n.seq.length === 0 || n.illegal) continue;
      if (!best || cmpNode(n, best) < 0) best = n;
    }
  }
  return best?.seq ?? [];
}

function resolveIds(ids: readonly string[], isolated: readonly number[]): PulsePrimitive[] {
  const catalog = searchActionSet(new Set(isolated));
  const out: PulsePrimitive[] = [];
  for (const id of ids) {
    const p = catalog.find((x) => x.id === id);
    if (p) out.push(p);
  }
  return out;
}

export interface MethodBundle {
  readonly singlePulse: SequenceEval;
  readonly greedyCancel: SequenceEval;
  readonly terminalSearch: SequenceEval;
  readonly robustCancel: SequenceEval;
}

export function runCase(
  c: TerminalCase,
  plant: PublicConfig,
  interval: EtaInterval,
  grid: readonly number[],
  maxSeg: number,
  beamWidth: number,
): MethodBundle {
  const single = maxProjectionSingle(c.state, c.params, plant);
  const greedy = greedyThenCancel(c.state, c.params, plant, c.id, maxSeg);
  const term = searchTerminal(c.state, c.params, plant);
  const robust = robustCancelSearch(c.state, c.params, plant, interval, maxSeg, beamWidth);
  return {
    singlePulse: evaluateSequence("single-pulse", c.state, c.params, plant, [single], grid, interval.hat),
    greedyCancel: evaluateSequence(
      "greedy-then-cancel",
      c.state,
      c.params,
      plant,
      resolveIds(greedy.ids, c.isolated),
      grid,
      interval.hat,
    ),
    terminalSearch: evaluateSequence("terminal-search", c.state, c.params, plant, [...term.plan], grid, interval.hat),
    robustCancel: evaluateSequence("robust-cancel", c.state, c.params, plant, robust, grid, interval.hat),
  };
}

export interface MethodSummary {
  readonly method: string;
  readonly n: number;
  readonly nominalCaptureRate: number;
  readonly worstCaptureRate: number;
  readonly fuelHoldWorstRate: number;
  readonly medianJnom: number;
  readonly medianJworst: number;
  readonly perMaskCapture: Record<string, number>;
}

function median(xs: number[]): number {
  if (xs.length === 0) return NaN;
  const s = [...xs].sort((a, b) => a - b);
  return s[Math.floor(s.length / 2)]!;
}

function summarizeMethod(
  method: string,
  rows: Array<{ isolated: readonly number[]; eval: SequenceEval }>,
): MethodSummary {
  const n = rows.length;
  const perMask: Record<string, { hit: number; n: number }> = {};
  for (const r of rows) {
    const k = r.isolated.length === 0 ? "healthy" : `isolated-${r.isolated.join("+")}`;
    const cur = perMask[k] ?? { hit: 0, n: 0 };
    cur.n += 1;
    if (r.eval.nominal.captured) cur.hit += 1;
    perMask[k] = cur;
  }
  const perMaskCapture: Record<string, number> = {};
  for (const [k, v] of Object.entries(perMask)) perMaskCapture[k] = v.n === 0 ? 0 : v.hit / v.n;
  return {
    method,
    n,
    nominalCaptureRate: n === 0 ? 0 : rows.filter((r) => r.eval.nominal.captured).length / n,
    worstCaptureRate: n === 0 ? 0 : rows.filter((r) => r.eval.worst.captured).length / n,
    fuelHoldWorstRate: n === 0 ? 0 : rows.filter((r) => r.eval.grid.every((s) => s.fuelOk)).length / n,
    medianJnom: median(rows.map((r) => r.eval.nominal.J)),
    medianJworst: median(rows.map((r) => r.eval.worst.J)),
    perMaskCapture,
  };
}

export interface GateReport {
  readonly overall80: boolean;
  readonly perMask70: boolean;
  readonly worstCaseFuel: boolean;
  readonly beatsTerminal: boolean;
  readonly readyToWire: boolean;
}

export interface RobustTerminalStudy {
  readonly interval: EtaInterval;
  readonly grid: readonly number[];
  readonly maxSeg: number;
  readonly beamWidth: number;
  readonly nStates: number;
  readonly weights: { wPar: number; wPerp: number; wAtt: number; wFuel: number };
  readonly byMethod: Record<string, MethodSummary>;
  readonly gates: GateReport;
  readonly results: Array<{
    stateId: string;
    family: string;
    isolated: readonly number[];
    att0: number;
    omega0: number;
    methods: Record<string, SequenceEval>;
  }>;
}

export function runRobustTerminalStudy(
  plant: PublicConfig,
  opts: { quick?: boolean; maxSeg?: number; beamWidth?: number } = {},
): RobustTerminalStudy {
  const interval = etaInterval();
  const grid = etaGrid(interval, opts.quick ? 3 : 5);
  const maxSeg = opts.maxSeg ?? (opts.quick ? 8 : DEFAULT_MAX_SEG);
  const beamWidth = opts.beamWidth ?? (opts.quick ? 6 : DEFAULT_BEAM);
  const cases = lowRateCases(plant, { quick: opts.quick });
  const results: RobustTerminalStudy["results"] = [];
  const buckets: Record<string, Array<{ isolated: readonly number[]; eval: SequenceEval }>> = {
    "single-pulse": [],
    "greedy-then-cancel": [],
    "terminal-search": [],
    "robust-cancel": [],
  };
  for (const c of cases) {
    const bundle = runCase(c, plant, interval, grid, maxSeg, beamWidth);
    const methods = {
      "single-pulse": bundle.singlePulse,
      "greedy-then-cancel": bundle.greedyCancel,
      "terminal-search": bundle.terminalSearch,
      "robust-cancel": bundle.robustCancel,
    };
    results.push({
      stateId: c.id,
      family: c.family,
      isolated: c.isolated,
      att0: deg(geodesicAttitudeError(c.state.qBI, plant.qTarget)),
      omega0: vnorm(c.state.omegaB),
      methods,
    });
    for (const [k, ev] of Object.entries(methods)) {
      buckets[k]!.push({ isolated: c.isolated, eval: ev });
    }
  }
  const byMethod: Record<string, MethodSummary> = {};
  for (const [k, rows] of Object.entries(buckets)) byMethod[k] = summarizeMethod(k, rows);
  const robust = byMethod["robust-cancel"]!;
  const terminal = byMethod["terminal-search"]!;
  const maskOk = Object.values(robust.perMaskCapture).every((r) => r + 1e-12 >= 0.7);
  const beats =
    robust.nominalCaptureRate + 1e-12 >= terminal.nominalCaptureRate + 0.15 && robust.medianJworst < terminal.medianJworst - 1e-6;
  const overall80 = robust.nominalCaptureRate + 1e-12 >= 0.8;
  const worstFuel = robust.fuelHoldWorstRate + 1e-12 >= 1;
  const gates: GateReport = {
    overall80,
    perMask70: maskOk,
    worstCaseFuel: worstFuel,
    beatsTerminal: beats,
    readyToWire: false,
  };
  return {
    interval,
    grid,
    maxSeg,
    beamWidth,
    nStates: cases.length,
    weights: { wPar: W_PAR, wPerp: W_PERP, wAtt: W_ATT, wFuel: W_FUEL },
    byMethod,
    gates,
    results,
  };
}

