import { FMAX, G0, ISP, MIN_PULSE, THRUSTERS } from "../constants";
import { vadd, vcross, vscale, type Vec3 } from "../math3d";
import type { ThrusterGeom } from "../types";
import type { ControlCommand } from "./interface";
import { pulseWidthFromPrimitive } from "./convert";

export const PULSE_DURATIONS_S = [
  MIN_PULSE,
  0.08,
  0.12,
  0.16,
  0.24,
  0.32,
] as const;

export type PulseDurationS = (typeof PULSE_DURATIONS_S)[number];

export interface PulsePrimitive {
  readonly id: string;
  readonly thrusterIds: readonly number[];
  readonly durationS: PulseDurationS;
  readonly commandedThrustN: number;
  readonly propellantKg: number;
}

export interface PrimitiveGenerationOptions {
  readonly isolatedThrusters?: ReadonlySet<number>;
  readonly durationsS?: readonly PulseDurationS[];
  readonly commandedThrustN?: number;
  readonly includeCoast?: boolean;
}

export interface PendingPulse {
  readonly id: number;
  readonly tOn: number;
  readonly tOff: number;
}

export interface PrimitiveWrench {
  readonly forceB: Vec3;
  readonly torqueB: Vec3;
  readonly linearImpulse: Vec3;
  readonly angularImpulse: Vec3;
  readonly propellantKg: number;
}

export interface ExecutableFilterOptions {
  readonly nowS: number;
  readonly commandDelayS: number;
  readonly pendingPulses: readonly PendingPulse[];
  readonly isolatedThrusters: ReadonlySet<number>;
  readonly estimatedFuelKg: number;
  readonly fuelFloorKg: number;
  readonly reserveKg: number;
  readonly maxActive: number;
  readonly minPulseS?: number;
}

const primitiveId = (ids: readonly number[], durationS: number): string =>
  ids.length === 0
    ? `coast:${durationS.toFixed(3)}`
    : `pulse:${ids.join("+")}:${durationS.toFixed(3)}`;

export function propellantFor(thrusterCount: number, thrustN: number, durationS: number): number {
  return (thrusterCount * thrustN * durationS) / (ISP * G0);
}

export function generatePulsePrimitives(
  thrusters: readonly ThrusterGeom[],
  options: PrimitiveGenerationOptions = {},
): readonly PulsePrimitive[] {
  const isolated = options.isolatedThrusters ?? new Set<number>();
  const durations = options.durationsS ?? PULSE_DURATIONS_S;
  const thrustN = Math.min(Math.max(options.commandedThrustN ?? FMAX, 0), FMAX);
  const healthyIds = thrusters
    .map((thruster) => thruster.id)
    .filter((id) => !isolated.has(id))
    .sort((a, b) => a - b);

  const result: PulsePrimitive[] = [];
  if (options.includeCoast !== false) {
    for (const durationS of durations) {
      if (durationS + Number.EPSILON < MIN_PULSE) continue;
      result.push({
        id: primitiveId([], durationS),
        thrusterIds: [],
        durationS,
        commandedThrustN: 0,
        propellantKg: 0,
      });
    }
    if (durations.length === 0) {
      result.push({
        id: primitiveId([], MIN_PULSE),
        thrusterIds: [],
        durationS: MIN_PULSE,
        commandedThrustN: 0,
        propellantKg: 0,
      });
    }
  }

  for (const durationS of durations) {
    if (durationS + Number.EPSILON < MIN_PULSE) continue;
    for (const first of healthyIds) {
      result.push({
        id: primitiveId([first], durationS),
        thrusterIds: [first],
        durationS,
        commandedThrustN: thrustN,
        propellantKg: propellantFor(1, thrustN, durationS),
      });
    }
    for (let i = 0; i < healthyIds.length; i += 1) {
      for (let j = i + 1; j < healthyIds.length; j += 1) {
        const ids = [healthyIds[i]!, healthyIds[j]!];
        result.push({
          id: primitiveId(ids, durationS),
          thrusterIds: ids,
          durationS,
          commandedThrustN: thrustN,
          propellantKg: propellantFor(2, thrustN, durationS),
        });
      }
    }
  }

  result.sort((a, b) => (a.id < b.id ? -1 : a.id > b.id ? 1 : 0));
  return result;
}

export function isLegalPulsePrimitive(
  primitive: Readonly<PulsePrimitive>,
  thrusters: readonly ThrusterGeom[],
  isolatedThrusters: ReadonlySet<number> = new Set<number>(),
): boolean {
  if (primitive.thrusterIds.length > 2) return false;
  if (primitive.thrusterIds.length === 0) return primitive.commandedThrustN === 0;
  if (primitive.durationS + Number.EPSILON < MIN_PULSE) return false;
  if (primitive.commandedThrustN < 0 || primitive.commandedThrustN > FMAX) return false;
  if (new Set(primitive.thrusterIds).size !== primitive.thrusterIds.length) return false;
  const defined = new Set<number>(thrusters.map((thruster) => thruster.id));
  return primitive.thrusterIds.every((id) => defined.has(id) && !isolatedThrusters.has(id));
}

export function leavesFuelFloor(
  primitive: Readonly<PulsePrimitive>,
  estimatedFuelKg: number,
  fuelFloorKg: number,
  reserveKg = 0,
): boolean {
  return estimatedFuelKg - primitive.propellantKg >= fuelFloorKg + reserveKg;
}

/**
 * Body-frame wrench about the supplied CM: τ = (r_nozzle − r_cm) × F.
 * Force and torque are NOT assumed parallel.
 */
export function netWrenchForPrimitive(
  primitive: Readonly<PulsePrimitive>,
  thrusters: readonly ThrusterGeom[] = THRUSTERS,
  etaT: number,
  rCmB: Vec3,
  Fmax = FMAX,
): PrimitiveWrench {
  let forceB: Vec3 = [0, 0, 0];
  let torqueB: Vec3 = [0, 0, 0];
  const Fmag = etaT * Math.min(primitive.commandedThrustN, Fmax);
  for (const id of primitive.thrusterIds) {
    const geom = thrusters.find((item) => item.id === id);
    if (!geom) continue;
    const F = vscale(geom.dir, Fmag);
    forceB = vadd(forceB, F);
    const r: Vec3 = [geom.pos[0] - rCmB[0], geom.pos[1] - rCmB[1], geom.pos[2] - rCmB[2]];
    torqueB = vadd(torqueB, vcross(r, F));
  }
  const dt = primitive.durationS;
  return {
    forceB,
    torqueB,
    linearImpulse: vscale(forceB, dt),
    angularImpulse: vscale(torqueB, dt),
    propellantKg: primitive.propellantKg,
  };
}

export function commandFromPrimitive(
  primitive: Readonly<PulsePrimitive>,
  sliderForceN = 0,
): ControlCommand {
  if (primitive.thrusterIds.length === 0) {
    return { thrusters: [], sliderForceN };
  }
  return {
    thrusters: primitive.thrusterIds.map((id) => ({ id, pulseWidthS: primitive.durationS })),
    sliderForceN,
  };
}

export function occupancyAt(
  pending: readonly PendingPulse[],
  t: number,
  failed: ReadonlySet<number>,
  maxActive: number,
): number[] {
  const live = pending
    .filter((p) => t >= p.tOn && t < p.tOff && !failed.has(p.id))
    .sort((a, b) => b.tOn - a.tOn || a.id - b.id);
  const chosen: number[] = [];
  const seen = new Set<number>();
  for (const p of live) {
    if (seen.has(p.id)) continue;
    if (chosen.length >= maxActive) break;
    chosen.push(p.id);
    seen.add(p.id);
  }
  return chosen.sort((a, b) => a - b);
}

export function filterExecutablePrimitives(
  primitives: readonly PulsePrimitive[],
  options: ExecutableFilterOptions,
): PulsePrimitive[] {
  const minPulse = options.minPulseS ?? MIN_PULSE;
  const fireAt = options.nowS + options.commandDelayS;
  const occupied = new Set(
    occupancyAt(options.pendingPulses, fireAt, options.isolatedThrusters, options.maxActive),
  );
  const out: PulsePrimitive[] = [];
  for (const primitive of primitives) {
    if (primitive.thrusterIds.length === 0) {
      out.push(primitive);
      continue;
    }
    if (primitive.durationS + Number.EPSILON < minPulse) continue;
    if (primitive.thrusterIds.some((id) => options.isolatedThrusters.has(id))) continue;
    if (primitive.thrusterIds.some((id) => occupied.has(id))) continue;
    const merged = new Set<number>([...occupied, ...primitive.thrusterIds]);
    if (merged.size > options.maxActive) continue;
    if (!leavesFuelFloor(primitive, options.estimatedFuelKg, options.fuelFloorKg, options.reserveKg)) continue;
    out.push(primitive);
  }
  if (!out.some((p) => p.thrusterIds.length === 0)) {
    out.unshift({
      id: primitiveId([], MIN_PULSE),
      thrusterIds: [],
      durationS: MIN_PULSE,
      commandedThrustN: 0,
      propellantKg: 0,
    });
  }
  out.sort((a, b) => (a.id < b.id ? -1 : a.id > b.id ? 1 : 0));
  return out;
}

export function enqueuePrimitive(
  pending: readonly PendingPulse[],
  primitive: Readonly<PulsePrimitive>,
  nowS: number,
  delayS: number,
): PendingPulse[] {
  const next = pending.filter((p) => p.tOff > nowS).map((p) => ({ ...p }));
  const tOn = nowS + delayS;
  const tOff = tOn + primitive.durationS;
  for (const id of primitive.thrusterIds) {
    next.push({ id, tOn, tOff });
  }
  next.sort((a, b) => a.tOn - b.tOn || a.id - b.id);
  return next;
}

void pulseWidthFromPrimitive;
