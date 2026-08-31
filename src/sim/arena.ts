/**
 * AgentArena — observation-only agent protocol and independent scoring.
 *
 * An agent receives `Observation` and returns `ControlCommand`. It never
 * reads Simulator, PrivateScenario, or truth. File scoring never accepts
 * a live Simulator. Failed planners stay available as named baselines.
 *
 * Not a new controller. Research-phase controllers are not re-exported
 * here (kNN-value, macros, null-space, robust-terminal).
 */
export type {
  ControlCommand,
  ControllerDiagnostics,
  ControllerMode,
  FlightController,
  PublicControllerConfig,
} from "./control/interface";
export { createFlightController } from "./control/factory";
export { BaselineController } from "./control/baseline";
export {
  ATT_GATE_DEG,
  FUEL_HARD,
  HIDDEN_SEEDS,
  PARAM_GATE,
  RATE_GATE,
  TRAIN_SEEDS,
} from "./evalset";
export { fdirFromEvents, fillScorecard, scoreFromFiles, scoreFromLog } from "./scoring";
