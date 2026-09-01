/**
 * AETHER domain adapter. AgentArena stays generic.
 */
import { readFileSync } from "node:fs";
import type { Agent } from "../../../packages/agent-arena/src/agent.ts";
import { fileRecorder } from "../../../packages/agent-arena/src/recorder.ts";
import type { EpisodeArtifacts, EpisodeDriver } from "../../../packages/agent-arena/src/runner.ts";
import type { ArtifactScorer, ScoreReport } from "../../../packages/agent-arena/src/scorer.ts";
import { createFlightController, createPlantController } from "../control/factory.ts";
import type { ControlCommand, PublicControllerConfig } from "../control/interface.ts";
import { generateScenario } from "../scenario.ts";
import { eventsJsonl, scoreFromFiles, trajectoryCsv } from "../scoring.ts";
import { Simulator } from "../simulator.ts";
import type { Observation, PrivateScenario, PublicConfig } from "../types.ts";

export type SpaceStationObservation = Observation;
export type SpaceStationAction = ControlCommand;

export class SpaceStationAgent implements Agent<SpaceStationObservation, SpaceStationAction> {
  private flight;

  constructor(plant: PublicConfig, config: Partial<PublicControllerConfig> = {}) {
    this.flight = createFlightController(plant, config);
  }

  reset(publicConfig: unknown): void {
    this.flight.reset(publicConfig as PublicControllerConfig);
  }

  step(observation: Readonly<SpaceStationObservation>): SpaceStationAction {
    return this.flight.step(observation);
  }
}

export class SpaceStationScorer implements ArtifactScorer {
  score(trajectoryPath: string, eventsPath: string): ScoreReport {
    const m = scoreFromFiles(readFileSync(trajectoryPath, "utf8"), readFileSync(eventsPath, "utf8"));
    const gates: Record<string, { pass: boolean; value: number | null }> = {};
    for (const [k, g] of Object.entries(m.scorecard)) {
      gates[k] = { pass: g.pass, value: typeof g.value === "number" ? g.value : null };
    }
    return { pass: Object.values(gates).every((g) => g.pass), gates };
  }
}

export class SpaceStationDriver implements EpisodeDriver<SpaceStationObservation, SpaceStationAction> {
  constructor(
    readonly plant: PublicConfig,
    readonly outDir: string,
  ) {}

  run(
    _agent: Agent<SpaceStationObservation, SpaceStationAction>,
    publicConfig: unknown,
    privateScenario: unknown,
  ): EpisodeArtifacts {
    const cfg = publicConfig as Partial<PublicControllerConfig>;
    const sc = privateScenario as PrivateScenario;
    const ctrl = createPlantController(this.plant, cfg);
    const sim = new Simulator(this.plant, sc, ctrl);
    sim.runAll();
    const trajectoryPath = `${this.outDir}/trajectory.csv`;
    const eventsPath = `${this.outDir}/events.jsonl`;
    fileRecorder.write(trajectoryPath, trajectoryCsv(sim.log));
    fileRecorder.write(eventsPath, eventsJsonl(sim.events));
    return { trajectoryPath, eventsPath };
  }
}

export function stationScenario(seed: number): PrivateScenario {
  return generateScenario(seed, false);
}
