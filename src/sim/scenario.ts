import { clamp, makeRng, qmul, qnormalize, vnormalize, type Quat, type Vec3 } from "./math3d";
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
  Q0,
  S0,
  SD0,
  W0,
} from "./constants";
import type { PrivateScenario, ThrusterIndex } from "./types";

function inRange(u: number, lo: number, hi: number) {
  return lo + u * (hi - lo);
}

function randomAttitude(rng: ReturnType<typeof makeRng>): Quat {
  const axis = vnormalize([rng.gauss(), rng.gauss(), rng.gauss()]);
  const ang = ((18 + 28 * rng.u01()) * Math.PI) / 180;
  const half = ang / 2;
  const dq: Quat = [Math.cos(half), axis[0] * Math.sin(half), axis[1] * Math.sin(half), axis[2] * Math.sin(half)];
  return qnormalize(qmul(Q0, dq));
}

export function generateScenario(seed: number, demo = false): PrivateScenario {
  if (demo) {
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
      q0: [Q0[0], Q0[1], Q0[2], Q0[3]],
      w0: [W0[0], W0[1], W0[2]],
      s0: S0,
      sd0: SD0,
    };
  }
  const rng = makeRng(seed);
  const gyroBias0: Vec3 = [0.003 * rng.gauss(), 0.003 * rng.gauss(), 0.003 * rng.gauss()];
  const faultThruster = Math.floor(rng.u01() * 6) as ThrusterIndex;
  const faultTime = 55 + rng.u01() * 55;
  const wScale = 0.85 + 0.3 * rng.u01();
  const w0: Vec3 = [
    (W0[0] + 0.035 * rng.gauss()) * wScale,
    (W0[1] + 0.035 * rng.gauss()) * wScale,
    (W0[2] + 0.035 * rng.gauss()) * wScale,
  ];
  return {
    seed,
    c1: inRange(rng.u01(), C1_RANGE[0], C1_RANGE[1]),
    c2: inRange(rng.u01(), C2_RANGE[0], C2_RANGE[1]),
    k12: inRange(rng.u01(), K12_RANGE[0], K12_RANGE[1]),
    etaT: inRange(rng.u01(), ETA_RANGE[0], ETA_RANGE[1]),
    faultTime,
    faultThruster,
    gyroBias0,
    demo: false,
    q0: randomAttitude(rng),
    w0,
    s0: clamp(S0 + 0.15 * rng.gauss(), -1.2, 1.2),
    sd0: clamp(SD0 + 0.08 * rng.gauss(), -0.5, 0.5),
  };
}

export function scenarioToJson(s: PrivateScenario): string {
  return JSON.stringify(s, null, 2);
}
