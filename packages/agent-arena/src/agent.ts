export interface Agent<O, A> {
  reset(publicConfig: unknown): void;
  step(observation: Readonly<O>): A;
}
