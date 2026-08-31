import type { Observation, ThrusterCommand } from '../types'

export type ControllerMode = 'baseline' | 'discrete-pulse-v2'

export interface PublicControllerConfig {
  readonly mode: ControllerMode
  readonly fuelFloorKg: number
  readonly planningHorizonS: number
  readonly replanPeriodS: number
  readonly beamWidth: number
}

export interface ControllerDiagnostics {
  readonly mode: ControllerMode
  readonly observationTime: number
  readonly estimatedFuelKg: number
  readonly isolatedThrusters: readonly number[]
  readonly candidateCount: number
  readonly expandedNodeCount: number
  readonly selectedPrimitiveId: string | null
  readonly predictedTerminalAttitudeErrorDeg: number | null
  readonly predictedTerminalAngularSpeedRadS: number | null
  readonly predictedTerminalFuelKg: number | null
}

export interface ControlCommand {
  readonly thrusters: readonly ThrusterCommand[]
  readonly sliderForceN: number
}

export interface FlightController {
  reset(config: Readonly<PublicControllerConfig>): void
  step(observation: Readonly<Observation>): ControlCommand
  diagnostics(): Readonly<ControllerDiagnostics>
}
