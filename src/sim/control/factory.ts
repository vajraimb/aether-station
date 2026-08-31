import type { Estimate } from "../estimator";
import type { FdirSnapshot } from "../fdir";
import type { AnyController } from "../oracle";
import type { Observation, PublicConfig } from "../types";
import { BaselineController, mergeFlightConfig } from "./baseline";
import { DiscretePulseV2Controller } from "./controller-v2";
import { commandFromControl } from "./convert";
import type { FlightController, PublicControllerConfig } from "./interface";

export interface PlantFlightController extends FlightController {
  asPlant(): AnyController;
  getEstimate(): Estimate;
  getFdir(): FdirSnapshot;
}

export function createFlightController(
  plant: PublicConfig,
  config: Partial<PublicControllerConfig> = {},
): FlightController {
  const cfg = mergeFlightConfig(config);
  if (cfg.mode === "baseline") return new BaselineController(plant, cfg);
  if (cfg.mode === "discrete-pulse-v2") return new DiscretePulseV2Controller(plant, cfg);
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
