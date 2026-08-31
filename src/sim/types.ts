import type { Quat, Vec3 } from "./math3d";

export type ThrusterIndex = 0 | 1 | 2 | 3 | 4 | 5;

export interface PublicConfig {
  dryMass: number;
  dryInertiaB: Vec3;
  sliderMass: number;
  sliderMin: number;
  sliderMax: number;
  sliderForceMax: number;
  fluidMass: number;
  tankMeanRadius: number;
  initialFuelMass: number;
  duration: number;
  controllerPeriod: number;
  sensorPeriod: number;
  dtMax: number;
  restitution: number;
  maxThrust: number;
  minPulse: number;
  commandDelay: number;
  maxActiveThrusters: number;
  isp: number;
  g0: number;
  omega1: number;
  omega2: number;
  c1Range: [number, number];
  c2Range: [number, number];
  k12Range: [number, number];
  etaRange: [number, number];
  qTarget: Quat;
  omegaTarget: Vec3;
  sTarget: number;
  seed: number;
  /** Counterfactual: controller may know there is no fluid. */
  fluidPresent: boolean;
  debugTruth: boolean;
}

export interface PrivateScenario {
  seed: number;
  c1: number;
  c2: number;
  k12: number;
  etaT: number;
  faultTime: number;
  faultThruster: ThrusterIndex;
  gyroBias0: Vec3;
  demo: boolean;
  q0: Quat;
  w0: Vec3;
  s0: number;
  sd0: number;
}

export interface SimState {
  t: number;
  /** Inertial position of the body-frame origin (station reference point). */
  rI: Vec3;
  /** Inertial velocity of the body-frame origin. */
  vI: Vec3;
  /** Inertial position of the system CM. */
  rCmI: Vec3;
  /** Inertial velocity of the system CM. */
  vCmI: Vec3;
  q: Quat;
  w: Vec3;
  s: number;
  sd: number;
  th1: number;
  th1d: number;
  th2: number;
  th2d: number;
  fuel: number;
}

/** Serialised observation. This is the ONLY input the flight controller may read. */
export interface Observation {
  timestamp: number;
  quaternionMeasured: Quat;
  gyroMeasured: Vec3;
  sliderPosition: number;
  sliderVelocity: number;
  tankWallPressure1: number;
  tankWallPressure2: number;
  remainingFuelEstimate: number;
  thrusterCurrentFeedback: [number, number, number, number, number, number];
  actuatorResponseAbnormal: boolean;
}

export const OBSERVATION_KEYS = [
  "timestamp",
  "quaternionMeasured",
  "gyroMeasured",
  "sliderPosition",
  "sliderVelocity",
  "tankWallPressure1",
  "tankWallPressure2",
  "remainingFuelEstimate",
  "thrusterCurrentFeedback",
  "actuatorResponseAbnormal",
] as const;

export interface Command {
  sliderForce: number;
  /** Requested on-time in seconds for this 0.1 s cycle, 0 or >= minPulse. */
  pulseWidth: [number, number, number, number, number, number];
}

export interface SimEvent {
  t: number;
  type:
    | "scenario"
    | "collision"
    | "fault_injected"
    | "abnormal_flag"
    | "fault_detected"
    | "fault_isolated"
    | "settled"
    | "fuel_empty"
    | "thruster_on"
    | "thruster_off";
  data?: Record<string, number | string | boolean | null>;
}

export interface FdirReport {
  faultInjectionTime: number | null;
  abnormalFlagTime: number | null;
  detectionTime: number | null;
  isolationTime: number | null;
  detectionDelay: number | null;
  isolationDelay: number | null;
  isolatedThrusterId: number;
  confidence: number;
}

export interface Sample {
  t: number;
  r: Vec3;
  v: Vec3;
  q: Quat;
  w: Vec3;
  qEst: Quat;
  wEst: Vec3;
  gyroBiasEst: Vec3;
  s: number;
  sd: number;
  sEst: number;
  sdEst: number;
  th1: number;
  th1d: number;
  th2: number;
  th2d: number;
  th1Est: number;
  th2Est: number;
  sloshEnergy: number;
  fuelTrue: number;
  fuelEst: number;
  c1Est: number;
  c2Est: number;
  k12Est: number;
  etaTEst: number;
  c1P: number;
  c2P: number;
  k12P: number;
  etaP: number;
  attitudeErrorDeg: number;
  thrusterCmd: [number, number, number, number, number, number];
  thrusterActual: [number, number, number, number, number, number];
  faultConfidence: [number, number, number, number, number, number];
  detectedFailedThruster: number;
  quaternionNormError: number;
  totalAngularMomentumError: number;
  nis: number;
  sliderForce: number;
  hI: Vec3;
}

export interface Metrics {
  final_attitude_error_deg: number;
  final_angular_speed_rad_s: number;
  max_slider_impact_speed_m_s: number;
  initial_slosh_energy: number;
  final_slosh_energy: number;
  final_slosh_energy_ratio: number;
  remaining_fuel_kg: number;
  parameter_relative_error: number;
  fault_detection_delay_s: number | null;
  fault_isolation_accuracy: number;
  quaternion_norm_max_error: number;
  maximum_constraint_violation: number;
  total_thruster_on_time: number;
  pulse_count: number;
  run_is_deterministic: boolean;
  detection_time: number | null;
  isolation_time: number | null;
  isolated_thruster: number;
  settled_time: number | null;
  faultInjectionTime: number | null;
  abnormalFlagTime: number | null;
  detectionTime: number | null;
  isolationTime: number | null;
  detectionDelay: number | null;
  isolationDelay: number | null;
  isolatedThrusterId: number;
  confidence: number;
  scorecard: Record<string, { value: number | boolean | null; pass: boolean; target: string }>;
}

export interface ThrusterGeom {
  id: ThrusterIndex;
  name: string;
  dir: Vec3;
  pos: Vec3;
}

export interface MassProps {
  mTotal: number;
  mRigid: number;
  rCmB: Vec3;
  Icm: import("./math3d").Mat3;
  Iorigin: import("./math3d").Mat3;
}

export const THRUSTER_NAMES = ["+X", "-X", "+Y", "-Y", "+Z", "-Z"] as const;

export interface HiddenParams {
  c1: number;
  c2: number;
  k12: number;
  etaT: number;
  faultTime: number;
  faultThruster: number;
  seed: number;
}
