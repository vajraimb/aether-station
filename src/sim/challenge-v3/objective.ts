/**
 * Lexicographic objective for the truth-state action-sequence optimizer (spec
 * section 7) and its scalarised search surrogate.
 *
 * Level 1 - hard constraints: fuel floor, at most two concurrent nozzles,
 *           pulses no shorter than 40 ms, never command a failed nozzle,
 *           no slider penetration, no numeric anomaly.
 * Level 2 - joint terminal gates: attitude, rate, slosh ratio, slider impact.
 * Level 3 - performance: remaining fuel, terminal error, pulse count, peak
 *           transverse rate and slosh excitation.
 *
 * The comparator below is the single source of truth for candidate ranking;
 * the scalar `searchCost` is only a smooth guide used inside the optimizers.
 * All published metrics come from the official simulator plus the file scorer.
 */
import type { RolloutResult } from "./plant";

export const GATES = {
  attitudeDeg: 1.0,
  omega: 0.008,
  impactSpeed: 0.25,
  sloshRatio: 0.08,
  fuelHard: 2.8,
  /** Working floor, kept above the hard limit so plant/model mismatch cannot eat it. */
  fuelFloor: 2.82,
  quatNormErr: 1e-6,
} as const;

export interface TerminalEval {
  /** Number of level-1 hard-constraint violations. */
  hardViolations: number;
  /** Number of level-2 terminal gates not satisfied. */
  gatesFailed: number;
  /** Worst normalised gate ratio (value / threshold); < 1 means all gates pass. */
  worstRatio: number;
  /** Sum of gate excess, a smooth measure of how far outside the box we are. */
  gateExcess: number;
  attitudeDeg: number;
  omega: number;
  /** Worst attitude error across the persistent-capture window. */
  dwellAttitudeDeg: number;
  /** Worst rate across the persistent-capture window. */
  dwellOmega: number;
  /** True if the joint terminal set held across the whole capture window. */
  dwellHeld: boolean;
  /** True if a command can still be burning when the mission ends. */
  pendingAtEnd: boolean;
  sloshRatio: number;
  impactSpeed: number;
  fuel: number;
  pulses: number;
  peakOmega: number;
  allGatesPass: boolean;
}

export function evaluateTerminal(r: RolloutResult, pulses: number): TerminalEval {
  let hard = 0;
  if (r.fuel < GATES.fuelHard) hard += 1;
  if (r.maxConstraintViolation > 1e-9) hard += 1;
  if (r.numericAnomaly) hard += 1;
  if (r.maxQuatNormError > GATES.quatNormErr) hard += 1;
  // A command still in flight at the mission end means the terminal state was
  // never actually settled; treat it as a hard violation of the capture.
  if (r.pendingAtEnd) hard += 1;

  // Persistent capture: the attitude and rate ratios are taken over the whole
  // capture window rather than at the final instant, so a free-drift fly-by of
  // the target cannot be scored as a capture (spec follow-up item A).
  const ratios = [
    r.dwellAttitudeMaxDeg / GATES.attitudeDeg,
    r.dwellOmegaMax / GATES.omega,
    r.sloshRatio / GATES.sloshRatio,
    r.maxImpactSpeed / GATES.impactSpeed,
  ];
  let gatesFailed = 0;
  let excess = 0;
  let worst = 0;
  for (const x of ratios) {
    if (x >= 1) gatesFailed += 1;
    excess += Math.max(0, x - 1);
    if (x > worst) worst = x;
  }
  return {
    hardViolations: hard,
    gatesFailed,
    worstRatio: worst,
    gateExcess: excess,
    attitudeDeg: r.attitudeErrorDeg,
    omega: r.omega,
    dwellAttitudeDeg: r.dwellAttitudeMaxDeg,
    dwellOmega: r.dwellOmegaMax,
    dwellHeld:
      r.dwellSamples > 0 &&
      r.dwellAttitudeMaxDeg < GATES.attitudeDeg &&
      r.dwellOmegaMax < GATES.omega &&
      !r.pendingAtEnd,
    pendingAtEnd: r.pendingAtEnd,
    sloshRatio: r.sloshRatio,
    impactSpeed: r.maxImpactSpeed,
    fuel: r.fuel,
    pulses,
    peakOmega: r.peakOmega,
    allGatesPass: hard === 0 && gatesFailed === 0,
  };
}

/** Lexicographic key, ascending-better on every component. */
export function lexKey(e: TerminalEval): number[] {
  return [
    e.hardViolations,
    e.gatesFailed,
    e.worstRatio,
    -e.fuel,
    e.attitudeDeg,
    e.omega,
    e.pulses,
    e.peakOmega,
  ];
}

export function lexCompare(a: number[], b: number[]): number {
  const n = Math.min(a.length, b.length);
  for (let i = 0; i < n; i++) {
    const d = a[i]! - b[i]!;
    if (Math.abs(d) > 1e-12) return d < 0 ? -1 : 1;
  }
  return 0;
}

/**
 * Smooth scalar guide used inside the optimizers. Dominated by the hard
 * constraints, then by how far the predicted t=180 state sits outside the
 * terminal box, then by fuel and pulse economy.
 */
export function searchCost(e: TerminalEval): number {
  return (
    1e6 * e.hardViolations +
    1e3 * Math.log1p(e.attitudeDeg / GATES.attitudeDeg) +
    3e3 * Math.log1p(e.omega / GATES.omega) +
    2e2 * Math.max(0, e.sloshRatio / GATES.sloshRatio - 0.5) +
    2e3 * Math.max(0, e.impactSpeed / GATES.impactSpeed - 0.5) +
    40 * Math.max(0, GATES.fuelFloor + 0.1 - e.fuel) +
    0.02 * e.pulses
  );
}
