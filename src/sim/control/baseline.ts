/**
 * Observation-only baseline adapter. Wraps AgentController without changing
 * its law. Simulator and eval talk to this through FlightController.
 */
import { AgentController } from "../controller";
import type { Estimate } from "../estimator";
import type { FdirSnapshot } from "../fdir";
import type { PublicConfig, Observation, Command } from "../types";
import type { AnyController } from "../oracle";
import { controlFromCommand } from "./convert";
import type {
  ControlCommand,
  ControllerDiagnostics,
  FlightController,
  PublicControllerConfig,
} from "./interface";

export const DEFAULT_FLIGHT_CONFIG: PublicControllerConfig = {
  mode: "baseline",
  fuelFloorKg: 2.8,
  planningHorizonS: 10,
  replanPeriodS: 0.5,
  beamWidth: 48,
};

export function mergeFlightConfig(
  over: Partial<PublicControllerConfig> = {},
): PublicControllerConfig {
  return { ...DEFAULT_FLIGHT_CONFIG, ...over };
}

export class BaselineController implements FlightController {
  readonly inner: AgentController;
  private flightCfg: PublicControllerConfig;
  private lastObsTime = 0;

  constructor(plant: PublicConfig, config: Readonly<PublicControllerConfig> = DEFAULT_FLIGHT_CONFIG) {
    this.inner = new AgentController(plant);
    this.flightCfg = { ...config, mode: "baseline" };
  }

  reset(config: Readonly<PublicControllerConfig>): void {
    this.flightCfg = { ...config, mode: "baseline" };
  }

  step(observation: Readonly<Observation>): ControlCommand {
    const cmd = this.inner.step(observation);
    this.lastObsTime = observation.timestamp;
    return controlFromCommand(cmd);
  }

  /** Plant-facing step used by Simulator (lossless round-trip of inner Command). */
  stepPlant(observation: Observation): Command {
    return this.inner.step(observation);
  }

  diagnostics(): Readonly<ControllerDiagnostics> {
    const est = this.inner.getEstimate();
    const fdir = this.inner.getFdir();
    const isolated = [...this.inner.isolated].sort((a, b) => a - b);
    return {
      mode: "baseline",
      observationTime: this.lastObsTime,
      estimatedFuelKg: est.fuel,
      isolatedThrusters: isolated,
      candidateCount: 0,
      expandedNodeCount: 0,
      selectedPrimitiveId: null,
      predictedTerminalAttitudeErrorDeg: null,
      predictedTerminalAngularSpeedRadS: null,
      predictedTerminalFuelKg: est.fuel,
    };
  }

  getEstimate(): Estimate {
    return this.inner.getEstimate();
  }

  getFdir(): FdirSnapshot {
    return this.inner.getFdir();
  }

  asPlant(): AnyController {
    return this.inner;
  }
}
