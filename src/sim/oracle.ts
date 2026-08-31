/**
 * Truth-feedback baseline. Reads the true SimState (attitude, rate, slider,
 * slosh) but still obeys every actuator constraint. Same receding-horizon
 * allocator as the observation-only agent; only the state fed to the law is
 * truth. This is NOT an offline pulse-sequence optimizer.
 */
import { AgentController } from "./controller";
import type { Estimate } from "./estimator";
import type { PlannerOpts } from "./planner";
import type { PublicConfig, SimState } from "./types";

export class TruthFeedbackBaseline extends AgentController {
  override readonly name = "truthFeedbackBaseline";
  private truth: SimState | null = null;

  constructor(cfg: PublicConfig, plannerOpts?: PlannerOpts) {
    super(cfg);
    if (plannerOpts) this.plannerOpts = { ...this.plannerOpts, ...plannerOpts };
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

/** @deprecated Use TruthFeedbackBaseline. Kept as an alias. */
export const OracleController = TruthFeedbackBaseline;

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
