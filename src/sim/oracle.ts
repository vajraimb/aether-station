/**
 * Oracle controller. Reads the true SimState (attitude, rate, slider, slosh)
 * but still obeys every actuator constraint. Same allocation / FDIR as the
 * observation-only agent; only the state used for the control law is truth.
 */
import { AgentController } from "./controller";
import type { Estimate } from "./estimator";
import type { PublicConfig, SimState } from "./types";

export class OracleController extends AgentController {
  override readonly name = "oracle";
  private truth: SimState | null = null;

  constructor(cfg: PublicConfig) {
    super(cfg);
  }

  ingestTruth(state: SimState) {
    this.truth = {
      ...state,
      q: [...state.q],
      w: [...state.w],
      rI: [...state.rI],
      vI: [...state.vI],
      rCmI: [...state.rCmI],
      vCmI: [...state.vCmI],
    };
  }

  override getEstimate(): Estimate {
    const base = super.getEstimate();
    const t = this.truth;
    if (!t) return base;
    return {
      ...base,
      q: [...t.q],
      w: [...t.w],
      s: t.s,
      sd: t.sd,
      th1: t.th1,
      th1d: t.th1d,
      th2: t.th2,
      th2d: t.th2d,
      fuel: t.fuel,
    };
  }

  override get estimate(): Estimate {
    return this.getEstimate();
  }
}

export type AnyController = {
  name: string;
  step: AgentController["step"];
  getEstimate(): Estimate;
  getFdir: AgentController["getFdir"];
  ingestTruth?(state: SimState): void;
  readonly faultConfidence: [number, number, number, number, number, number];
  readonly detectedFailedThruster: number;
  readonly detectionTime: number | null;
  readonly isolationTime: number | null;
  readonly isolationConfidence: number;
};
