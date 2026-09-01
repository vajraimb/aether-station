export interface ScenarioSet<S> {
  readonly id: string;
  readonly items: readonly S[];
}
