export interface GateResult {
  readonly pass: boolean;
  readonly value: number | null;
}

export interface ScoreReport {
  readonly pass: boolean;
  readonly gates: Readonly<Record<string, GateResult>>;
}

/** File-only scorer. Implementations must not accept a live plant object. */
export interface ArtifactScorer {
  score(trajectoryPath: string, eventsPath: string, scenarioPath?: string): ScoreReport;
}
