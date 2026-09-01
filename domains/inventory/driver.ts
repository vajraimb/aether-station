import type { Agent } from "../../packages/agent-arena/src/agent.ts";
import type { EpisodeArtifacts, EpisodeDriver } from "../../packages/agent-arena/src/runner.ts";
import { ReorderPointAgent } from "./agent";
import { InventoryEnvironment } from "./environment";
import { writeInventoryArtifacts } from "./recorder";
import type { InventoryAction, InventoryObservation, InventoryPrivateScenario, InventoryPublicConfig } from "./types";

export class InventoryDriver implements EpisodeDriver<InventoryObservation, InventoryAction> {
  constructor(
    readonly cfg: InventoryPublicConfig,
    readonly outDir: string,
  ) {}

  run(
    agent: Agent<InventoryObservation, InventoryAction>,
    publicConfig: unknown,
    privateScenario: unknown,
  ): EpisodeArtifacts {
    const sc = privateScenario as InventoryPrivateScenario;
    agent.reset(publicConfig);
    const env = new InventoryEnvironment(this.cfg);
    let obs = env.reset(sc);
    for (;;) {
      const r = env.step(agent.step(obs));
      obs = r.observation;
      if (r.terminated) break;
    }
    if (!env.sc) throw new Error("missing scenario");
    return writeInventoryArtifacts(this.outDir, env.log, env.events, env.sc);
  }
}

export function runInventoryEpisode(cfg: InventoryPublicConfig, sc: InventoryPrivateScenario, dir: string): EpisodeArtifacts {
  return new InventoryDriver(cfg, dir).run(new ReorderPointAgent(), cfg, sc);
}
