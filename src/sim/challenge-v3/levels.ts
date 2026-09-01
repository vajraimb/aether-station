/**
 * Capability-level harnesses (spec 5).
 *
 * The levels strip the mission down until only "basic control" is left, so a
 * failure can be localised instead of being blamed on the estimator. Nothing
 * here modifies the audited physics: L1 integrates the same
 * `integrateWithCollision` used by the plant and the official simulator, it
 * simply feeds an ideal continuous body torque through `u.tauThrO` instead of
 * routing a pulse train through the thruster system.
 *
 * L1: known parameters, ideal continuous three-axis torque, no fault, no
 *     slosh, no sensor noise, no command delay, no minimum pulse.
 *     A failure here means the basic pointing problem itself is not solved and
 *     no amount of estimation or search work will help.
 */
import { defaultPublicConfig } from "../constants";
import { integrateWithCollision } from "../dynamics";
import {
  attitudeErrorAngle,
  attitudeErrorVector,
  deg,
  mv,
  vnorm,
  type Vec3,
} from "../math3d";
import { generateScenario } from "../scenario";
import { massState } from "../dynamics";
import type { SimState } from "../types";
import { initialState } from "./plant";

/** Result of one capability-level run. */
export interface LevelResult {
  level: string;
  seed: number;
  attitudeDeg: number;
  omega: number;
  peakTorque: number;
  passed: boolean;
}

/** Peak per-axis torque available from a single nozzle, N*m. Conservative. */
const TAU_MAX = 5.8;

/**
 * L1: ideal continuous torque, saturating eigen-axis proportional-derivative
 * law. This is the *definition* of level 1 in the specification (an ideal
 * continuous three-axis torque authority), not a candidate flight controller:
 * the flight-side planner never uses a feedback law, it optimizes action
 * sequences. L1 exists only to answer "is the basic pointing problem solvable
 * at all in this inertia and torque envelope".
 */
export function runLevel1(seed: number): LevelResult {
  const cfg = { ...defaultPublicConfig({ seed }), fluidPresent: false };
  const sc = generateScenario(seed, false);
  let st: SimState = initialState(cfg, sc.q0, sc.w0, sc.s0, sc.sd0);
  const I = massState(cfg, st.s, st.th1, st.th2, st.fuel).Icm;
  const dt = cfg.dtMax;
  // Critically damped eigen-axis gains for the mission duration.
  const wn = 0.12;
  const kp = wn * wn;
  const kd = 2 * wn;
  let peak = 0;
  while (st.t < cfg.duration - 1e-12) {
    const e = attitudeErrorVector(st.q, cfg.qTarget);
    // Body error-angle vector is approximately twice the quaternion vector part.
    const a: Vec3 = [
      -kp * 2 * e[0] - kd * st.w[0],
      -kp * 2 * e[1] - kd * st.w[1],
      -kp * 2 * e[2] - kd * st.w[2],
    ];
    const tau = mv(I, a) as Vec3;
    for (let i = 0; i < 3; i++) {
      if (tau[i]! > TAU_MAX) tau[i] = TAU_MAX;
      if (tau[i]! < -TAU_MAX) tau[i] = -TAU_MAX;
    }
    peak = Math.max(peak, vnorm(tau));
    const u = {
      FthrB: [0, 0, 0] as Vec3,
      tauThrO: tau,
      Fslider: 0,
      c1: 0,
      c2: 0,
      k12: 0,
    };
    st = integrateWithCollision(cfg, st, u, dt).state;
  }
  const attitudeDeg = deg(attitudeErrorAngle(st.q, cfg.qTarget));
  const omega = vnorm(st.w);
  return {
    level: "L1",
    seed,
    attitudeDeg,
    omega,
    peakTorque: peak,
    passed: attitudeDeg < 1.0 && omega < 0.008,
  };
}
