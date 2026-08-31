/**
 * Capture labels for the offline dataset. Search failure is not a proof.
 *
 *   captured           — a committed high-fid trajectory met the conjunction
 *   search_unreached   — budgeted search did not find a sequence
 *   proven_infeasible  — a conservative certificate (no torque and no rate)
 */
import { attitudeErrorAngle, attitudeErrorVector, qnormalize, vdot, vnorm, vscale, vsub, type Vec3 } from "../math3d";
import type { PublicConfig } from "../types";
import { capturedGates } from "./capture-reachability";
import type { RolloutState } from "./rollout-model";
import { TERMINAL_FUEL_GATE } from "./terminal-planner";

export type CaptureLabel = "captured" | "search_unreached" | "proven_infeasible";

export const FEATURE_NAMES = [
  "attRad",
  "wPar",
  "wPerp",
  "wMag",
  "fuelMargin",
  "nIsolated",
  "pendingBusy",
  "sliderS",
] as const;

export type FeatureVector = number[];

export function captureFeatures(state: RolloutState, isolated: readonly number[], plant: PublicConfig): FeatureVector {
  const q = qnormalize(state.qBI);
  const attRad = attitudeErrorAngle(q, plant.qTarget);
  const err = attitudeErrorVector(q, plant.qTarget);
  const n = vnorm(err);
  const eN: Vec3 = n > 1e-9 ? vscale(err, 1 / n) : [1, 0, 0];
  const wPar = vdot(state.omegaB, eN);
  const wPerp = vnorm(vsub(state.omegaB, vscale(eN, wPar)));
  return [
    attRad,
    wPar,
    wPerp,
    vnorm(state.omegaB),
    state.fuelMass - TERMINAL_FUEL_GATE,
    isolated.length,
    state.pendingPulses.length > 0 ? 1 : 0,
    state.sliderS,
  ];
}

/**
 * Conservative certificates only. Coasting with leftover rate can still
 * walk into the 1° ball, so a fuel floor without near-zero rate is not a proof.
 */
export function proveInfeasible(state: RolloutState, plant: PublicConfig, isolated: readonly number[]): boolean {
  const g = capturedGates(state, plant.qTarget);
  if (g.captured) return false;
  const noTorque = isolated.length >= 6 || g.fuelKg <= TERMINAL_FUEL_GATE + 1e-12;
  const noRate = g.omega < 1e-4;
  return noTorque && noRate && g.attDeg >= 1 - 1e-9;
}

export function captureCostFromLabel(label: CaptureLabel, captureTimeS: number | null, fuelUsedKg: number, minAttDeg: number, omega: number): number {
  if (label === "captured") return Math.max(0, captureTimeS ?? 0) + 5 * Math.max(0, fuelUsedKg);
  if (label === "proven_infeasible") return 200;
  return 80 + minAttDeg + 12 * Math.max(0, omega - 0.008);
}
