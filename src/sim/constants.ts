import type { PublicConfig, ThrusterGeom } from "./types";
import type { Quat, Vec3 } from "./math3d";

export const DT = 0.005;
export const DURATION = 180;
export const CTRL_DT = 0.1;
export const SENS_DT = 0.05;

export const DRY_MASS = 850;
export const DRY_INERTIA: Vec3 = [620, 710, 540];
export const SLIDER_MASS = 55;
export const SLIDER_MIN = -1.8;
export const SLIDER_MAX = 1.8;
export const SLIDER_FMAX = 220;
export const FLUID_MASS = 140;
export const TANK_R = 1.25;
export const FUEL0 = 5.0;
export const RESTITUTION = 0.15;
export const FMAX = 18;
export const MIN_PULSE = 0.04;
export const CMD_DELAY = 0.12;
export const MAX_ACTIVE = 2;
export const ISP = 68;
export const G0 = 9.80665;
export const OMEGA1 = 1.15;
export const OMEGA2 = 1.73;

export const C1_RANGE: [number, number] = [0.08, 0.18];
export const C2_RANGE: [number, number] = [0.04, 0.14];
export const K12_RANGE: [number, number] = [0.15, 0.45];
export const ETA_RANGE: [number, number] = [0.82, 0.96];

export const Q0: Quat = [0.92388, 0.22094, -0.22094, 0.22094];
export const W0: Vec3 = [0.12, -0.08, 0.16];
export const S0 = 0.6;
export const SD0 = -0.3;
export const TH1_0 = 0.18;
export const TH1D_0 = 0;
export const TH2_0 = -0.11;
export const TH2D_0 = 0.06;

export const Q_TARGET: Quat = [1, 0, 0, 0];

/** Equivalent modal masses. Remainder of fluid is static with the tank.
 *  Equal modal masses keep the k12 coupling energy-consistent. */
export const M1_FRAC = 0.4;
export const M2_FRAC = 0.4;

export const DEMO_C1 = 0.137;
export const DEMO_C2 = 0.091;
export const DEMO_K12 = 0.318;
export const DEMO_ETA = 0.873;
export const DEMO_SEED = 20260831;
export const FAULT_TIME = 73.4;
export const FAULT_THRUSTER = 2 as const; // +Y

/**
 * Thruster layout: cylinder along body x. Each nozzle is offset on its face
 * so the six torque columns span R³ and remain rank-3 after losing +Y.
 *
 *   +X / −X  →  ±τz and a τy component
 *   +Y / −Y  →  ±τx and a τz component
 *   +Z / −Z  →  ±τy and a τx component
 */
export const THRUSTERS: ThrusterGeom[] = [
  { id: 0, name: "+X", dir: [1, 0, 0], pos: [2.2, 0.42, 0.18] },
  { id: 1, name: "-X", dir: [-1, 0, 0], pos: [-2.2, 0.42, 0.18] },
  { id: 2, name: "+Y", dir: [0, 1, 0], pos: [0.18, 1.18, 0.42] },
  { id: 3, name: "-Y", dir: [0, -1, 0], pos: [0.18, -1.18, 0.42] },
  { id: 4, name: "+Z", dir: [0, 0, 1], pos: [0.42, 0.18, 1.18] },
  { id: 5, name: "-Z", dir: [0, 0, -1], pos: [0.42, 0.18, -1.18] },
];

export const STATION_LENGTH = 4.4;
export const STATION_RADIUS = 1.08;

export function defaultPublicConfig(overrides: Partial<PublicConfig> = {}): PublicConfig {
  return {
    dryMass: DRY_MASS,
    dryInertiaB: DRY_INERTIA,
    sliderMass: SLIDER_MASS,
    sliderMin: SLIDER_MIN,
    sliderMax: SLIDER_MAX,
    sliderForceMax: SLIDER_FMAX,
    fluidMass: FLUID_MASS,
    tankMeanRadius: TANK_R,
    initialFuelMass: FUEL0,
    duration: DURATION,
    controllerPeriod: CTRL_DT,
    sensorPeriod: SENS_DT,
    dtMax: DT,
    restitution: RESTITUTION,
    maxThrust: FMAX,
    minPulse: MIN_PULSE,
    commandDelay: CMD_DELAY,
    maxActiveThrusters: MAX_ACTIVE,
    isp: ISP,
    g0: G0,
    omega1: OMEGA1,
    omega2: OMEGA2,
    c1Range: C1_RANGE,
    c2Range: C2_RANGE,
    k12Range: K12_RANGE,
    etaRange: ETA_RANGE,
    qTarget: Q_TARGET,
    omegaTarget: [0, 0, 0],
    sTarget: 0,
    seed: DEMO_SEED,
    fluidPresent: true,
    debugTruth: false,
    ...overrides,
  };
}

export const SCORE_TARGETS = {
  final_attitude_error_deg: 1.0,
  final_angular_speed_rad_s: 0.008,
  max_slider_impact_speed_m_s: 0.25,
  final_slosh_energy_ratio: 0.08,
  remaining_fuel_kg: 2.8,
  parameter_relative_error: 0.15,
  fault_detection_delay_s: 3.0,
  quaternion_norm_max_error: 1e-6,
};
