import type { Agent } from "./agent";
import type { ArtifactScorer, ScoreReport } from "./scorer";

export interface EpisodeArtifacts {
  readonly trajectoryPath: string;
  readonly eventsPath: string;
}

/** Domain-owned drive. Arena does not step a physics kernel. */
export interface EpisodeDriver<O, A> {
  run(agent: Agent<O, A>, publicConfig: unknown, privateScenario: unknown): EpisodeArtifacts;
}

export function scoreEpisode(scorer: ArtifactScorer, art: EpisodeArtifacts): ScoreReport {
  return scorer.score(art.trajectoryPath, art.eventsPath);
}
