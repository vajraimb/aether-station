import { makeRng, type Vec3 } from "./math3d";
import {
  C1_RANGE,
  C2_RANGE,
  DEMO_C1,
  DEMO_C2,
  DEMO_ETA,
  DEMO_K12,
  DEMO_SEED,
  ETA_RANGE,
  FAULT_THRUSTER,
  FAULT_TIME,
  K12_RANGE,
} from "./constants";
import type { PrivateScenario } from "./types";

function inRange(u: number, lo: number, hi: number) {
  return lo + u * (hi - lo);
}

export function generateScenario(seed: number, demo = false): PrivateScenario {
  if (demo || seed === DEMO_SEED) {
    return {
      seed: DEMO_SEED,
      c1: DEMO_C1,
      c2: DEMO_C2,
      k12: DEMO_K12,
      etaT: DEMO_ETA,
      faultTime: FAULT_TIME,
      faultThruster: FAULT_THRUSTER,
      gyroBias0: [0.0012, -0.0007, 0.0009],
      demo: true,
    };
  }
  const rng = makeRng(seed);
  const gyroBias0: Vec3 = [0.003 * rng.gauss(), 0.003 * rng.gauss(), 0.003 * rng.gauss()];
  return {
    seed,
    c1: inRange(rng.u01(), C1_RANGE[0], C1_RANGE[1]),
    c2: inRange(rng.u01(), C2_RANGE[0], C2_RANGE[1]),
    k12: inRange(rng.u01(), K12_RANGE[0], K12_RANGE[1]),
    etaT: inRange(rng.u01(), ETA_RANGE[0], ETA_RANGE[1]),
    faultTime: FAULT_TIME,
    faultThruster: FAULT_THRUSTER,
    gyroBias0,
    demo: false,
  };
}

export function scenarioToJson(s: PrivateScenario): string {
  return JSON.stringify(s, null, 2);
}
