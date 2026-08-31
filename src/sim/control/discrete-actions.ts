import { FMAX, G0, ISP, MIN_PULSE } from '../constants'
import type { ThrusterGeom } from '../types'

export const PULSE_DURATIONS_S = [
  MIN_PULSE,
  0.08,
  0.12,
  0.16,
  0.24,
  0.32,
] as const

export type PulseDurationS = (typeof PULSE_DURATIONS_S)[number]

export interface PulsePrimitive {
  readonly id: string
  readonly thrusterIds: readonly number[]
  readonly durationS: PulseDurationS
  readonly commandedThrustN: number
  readonly propellantKg: number
}

export interface PrimitiveGenerationOptions {
  readonly isolatedThrusters?: ReadonlySet<number>
  readonly durationsS?: readonly PulseDurationS[]
  readonly commandedThrustN?: number
  readonly includeCoast?: boolean
}

const primitiveId = (ids: readonly number[], durationS: number): string =>
  ids.length === 0
    ? `coast:${durationS.toFixed(3)}`
    : `pulse:${ids.join('+')}:${durationS.toFixed(3)}`

const propellantFor = (thrusterCount: number, thrustN: number, durationS: number): number =>
  (thrusterCount * thrustN * durationS) / (ISP * G0)

export function generatePulsePrimitives(
  thrusters: readonly ThrusterGeom[],
  options: PrimitiveGenerationOptions = {},
): readonly PulsePrimitive[] {
  const isolated = options.isolatedThrusters ?? new Set<number>()
  const durations = options.durationsS ?? PULSE_DURATIONS_S
  const thrustN = Math.min(Math.max(options.commandedThrustN ?? FMAX, 0), FMAX)
  const healthyIds = thrusters
    .map((thruster) => thruster.id)
    .filter((id) => !isolated.has(id))
    .sort((a, b) => a - b)

  const result: PulsePrimitive[] = []
  if (options.includeCoast !== false) {
    result.push({
      id: primitiveId([], MIN_PULSE),
      thrusterIds: [],
      durationS: MIN_PULSE,
      commandedThrustN: 0,
      propellantKg: 0,
    })
  }

  for (const durationS of durations) {
    if (durationS + Number.EPSILON < MIN_PULSE) continue
    for (const first of healthyIds) {
      result.push({
        id: primitiveId([first], durationS),
        thrusterIds: [first],
        durationS,
        commandedThrustN: thrustN,
        propellantKg: propellantFor(1, thrustN, durationS),
      })
    }
    for (let i = 0; i < healthyIds.length; i += 1) {
      for (let j = i + 1; j < healthyIds.length; j += 1) {
        const ids = [healthyIds[i], healthyIds[j]] as const
        result.push({
          id: primitiveId(ids, durationS),
          thrusterIds: ids,
          durationS,
          commandedThrustN: thrustN,
          propellantKg: propellantFor(2, thrustN, durationS),
        })
      }
    }
  }

  return result
}

export function isLegalPulsePrimitive(
  primitive: Readonly<PulsePrimitive>,
  thrusters: readonly ThrusterGeom[],
  isolatedThrusters: ReadonlySet<number> = new Set<number>(),
): boolean {
  if (primitive.thrusterIds.length > 2) return false
  if (primitive.thrusterIds.length === 0) return primitive.commandedThrustN === 0
  if (primitive.durationS + Number.EPSILON < MIN_PULSE) return false
  if (primitive.commandedThrustN < 0 || primitive.commandedThrustN > FMAX) return false
  if (new Set(primitive.thrusterIds).size !== primitive.thrusterIds.length) return false
  const defined = new Set<number>(thrusters.map((thruster) => thruster.id))
  return primitive.thrusterIds.every((id) => defined.has(id) && !isolatedThrusters.has(id))
}

export function leavesFuelFloor(
  primitive: Readonly<PulsePrimitive>,
  estimatedFuelKg: number,
  fuelFloorKg: number,
  reserveKg = 0,
): boolean {
  return estimatedFuelKg - primitive.propellantKg >= fuelFloorKg + reserveKg
}
