import type { Estimate } from "../estimator";
import type { FdirSnapshot } from "../fdir";
import type { AnyController } from "../oracle";
import type { Observation, PublicConfig } from "../types";
import { BaselineController, mergeFlightConfig } from "./baseline";
import { commandFromControl } from "./convert";
import type { FlightController, PublicControllerConfig } from "./interface";

export interface PlantFlightController extends FlightController {
  asPlant(): AnyController;
  getEstimate(): Estimate;
  getFdir(): FdirSnapshot;
}

export type DiscreteV2Ctor = new (
  plant: PublicConfig,
  config: Readonly<PublicControllerConfig>,
) => PlantFlightController;

let discreteV2Ctor: DiscreteV2Ctor | null = null;

/** Registered by controller-v2 so the factory does not import the planner eagerly. */
export function registerDiscreteV2Controller(ctor: DiscreteV2Ctor): void {
  discreteV2Ctor = ctor;
}

export function createFlightController(
  plant: PublicConfig,
  config: Partial<PublicControllerConfig> = {},
): FlightController {
  const cfg = mergeFlightConfig(config);
  if (cfg.mode === "baseline") return new BaselineController(plant, cfg);
  if (cfg.mode === "discrete-pulse-v2") {
    if (!discreteV2Ctor) {
      throw new Error("discrete-pulse-v2 controller is not registered");
    }
    return new discreteV2Ctor(plant, cfg);
  }
  throw new Error(`unknown controller mode: ${String((cfg as { mode: string }).mode)}`);
}

export function createPlantController(
  plant: PublicConfig,
  config: Partial<PublicControllerConfig> = {},
): AnyController {
  const cfg = mergeFlightConfig(config);
  const flight = createFlightController(plant, cfg);
  if (isPlantFlight(flight)) return flight.asPlant();
  const hooked = flight as FlightController & {
    getEstimate(): Estimate;
    getFdir(): FdirSnapshot;
  };
  return adaptFlightToPlant(hooked, cfg.mode === "discrete-pulse-v2" ? "discrete-pulse-v2" : "observation");
}

export function adaptFlightToPlant(
  flight: FlightController & { getEstimate(): Estimate; getFdir(): FdirSnapshot },
  name: string,
): AnyController {
  const getFdir = () => flight.getFdir();
  return {
    name,
    step: (obs: Observation) => commandFromControl(flight.step(obs)),
    getEstimate: () => flight.getEstimate(),
    getFdir,
    get faultConfidence() {
      return getFdir().faultConfidence;
    },
    get detectedFailedThruster() {
      return getFdir().detectedFailedThruster;
    },
    get detectionTime() {
      return getFdir().detectionTime;
    },
    get isolationTime() {
      return getFdir().isolationTime;
    },
    get isolationConfidence() {
      return getFdir().isolationConfidence;
    },
  };
}

function isPlantFlight(flight: FlightController): flight is PlantFlightController {
  return typeof (flight as PlantFlightController).asPlant === "function";
}
