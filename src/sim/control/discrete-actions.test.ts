import { THRUSTERS } from '../constants'
import {
  PULSE_DURATIONS_S,
  generatePulsePrimitives,
  isLegalPulsePrimitive,
  leavesFuelFloor,
} from './discrete-actions'

const assert = (condition: unknown, message: string): void => {
  if (!condition) throw new Error(message)
}

export function runDiscreteActionTests(): readonly string[] {
  const passed: string[] = []
  const all = generatePulsePrimitives(THRUSTERS)
  const expectedPerDuration = 6 + (6 * 5) / 2
  assert(all.length === 1 + PULSE_DURATIONS_S.length * expectedPerDuration, 'unexpected primitive count')
  passed.push('generates coast, singles, and unordered pairs')

  assert(new Set(all.map((primitive) => primitive.id)).size === all.length, 'primitive ids are not unique')
  passed.push('primitive ids are unique')

  assert(all.every((primitive) => isLegalPulsePrimitive(primitive, THRUSTERS)), 'illegal primitive generated')
  passed.push('all generated primitives are legal')

  const isolated = new Set([2, 5])
  const degraded = generatePulsePrimitives(THRUSTERS, { isolatedThrusters: isolated })
  assert(
    degraded.every((primitive) => primitive.thrusterIds.every((id) => !isolated.has(id))),
    'isolated thruster remained in action set',
  )
  passed.push('isolated thrusters are excluded')

  const pairs = degraded.filter((primitive) => primitive.thrusterIds.length === 2)
  assert(pairs.every((primitive) => primitive.thrusterIds[0] < primitive.thrusterIds[1]), 'pairs are not canonical')
  passed.push('two-thruster pairs are canonical')

  const shortestSingle = all.find(
    (primitive) => primitive.thrusterIds.length === 1 && primitive.durationS === PULSE_DURATIONS_S[0],
  )
  assert(shortestSingle, 'shortest single pulse missing')
  assert(!leavesFuelFloor(shortestSingle!, 2.8, 2.8), 'fuel-floor violation was accepted')
  assert(leavesFuelFloor(shortestSingle!, 3.0, 2.8), 'feasible pulse was rejected')
  passed.push('fuel floor is a hard feasibility check')

  return passed
}
