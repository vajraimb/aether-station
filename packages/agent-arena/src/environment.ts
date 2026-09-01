export interface StepResult<O> {
  readonly observation: O;
  readonly terminated: boolean;
  readonly truncated: boolean;
}

export interface Environment<O, A> {
  reset(privateScenario: unknown): O;
  step(action: A): StepResult<O>;
}
