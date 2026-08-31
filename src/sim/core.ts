/**
 * SimCore — frozen plant surface.
 *
 * Physics kernel is `math3d.ts` / `dynamics.ts` / `audit.ts` at
 * bdfff5b4c62733d7156d2f9cdeeaa75661d6c9f4. Do not edit those files.
 * This module only re-exports. It is not a controller.
 */
export {
  CMD_DELAY,
  DT,
  ETA_RANGE,
  C1_RANGE,
  C2_RANGE,
  K12_RANGE,
  FMAX,
  MAX_ACTIVE,
  MIN_PULSE,
  SCORE_TARGETS,
  THRUSTERS,
  defaultPublicConfig,
} from "./constants";
export type { Observation, PublicConfig, SimState, ThrusterIndex } from "./types";
export { OBSERVATION_KEYS } from "./types";
export {
  attitudeErrorAngle,
  clamp,
  deg,
  qRotate,
  vdot,
  vnorm,
  vsub,
  type Quat,
  type Vec3,
} from "./math3d";
export { angularMomentumCmB, massState, sloshEnergy, totalAngularMomentumI } from "./dynamics";
