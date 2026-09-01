/**
 * Replays an optimised open-loop action sequence inside the official
 * `Simulator`, so that every claimed number comes from the audited plant and
 * the file scorer rather than from the search-time roll-out model.
 *
 * `ScriptedTruthController` is a truth-state artefact (stage 1 only): it reads
 * the true slider state for the slider law exactly like the roll-out model, so
 * plant parity is exact. It is never used for observation-only results.
 */
import { AgentController } from "../controller";
import { TruthFeedbackBaseline } from "../oracle";
import { sliderForceCommand } from "../allocate";
import type { Estimate } from "../estimator";
import type { Command, PublicConfig } from "../types";
import { expandSchedule } from "./plant";
import type { Segment } from "./action-space";

export type TickSchedule = ReadonlyArray<readonly number[]>;

export class ScriptedTruthController extends TruthFeedbackBaseline {
  override readonly name: string = "challenge-v3-truth-script";
  private tick = 0;
  readonly schedule: TickSchedule;

  constructor(cfg: PublicConfig, schedule: TickSchedule) {
    super(cfg);
    this.schedule = schedule;
  }

  static fromSequence(cfg: PublicConfig, seq: readonly Segment[]) {
    return new ScriptedTruthController(cfg, expandSchedule(seq, cfg.controllerPeriod));
  }

  protected override control(_t: number, est: Estimate): Command {
    const widths = this.schedule[this.tick];
    this.tick += 1;
    const pulseWidth: [number, number, number, number, number, number] = [0, 0, 0, 0, 0, 0];
    if (widths) for (let i = 0; i < 6; i++) pulseWidth[i] = widths[i] ?? 0;
    return { sliderForce: sliderForceCommand(est.s, est.sd, this.cfg), pulseWidth };
  }
}

/** Observation-only scripted replay (no truth access). Used by ablations. */
export class ScriptedObservationController extends AgentController {
  override readonly name: string = "challenge-v3-obs-script";
  private tick = 0;
  readonly schedule: TickSchedule;

  constructor(cfg: PublicConfig, schedule: TickSchedule) {
    super(cfg);
    this.schedule = schedule;
  }

  protected override control(_t: number, est: Estimate): Command {
    const widths = this.schedule[this.tick];
    this.tick += 1;
    const pulseWidth: [number, number, number, number, number, number] = [0, 0, 0, 0, 0, 0];
    if (widths) for (let i = 0; i < 6; i++) pulseWidth[i] = widths[i] ?? 0;
    return { sliderForce: sliderForceCommand(est.s, est.sd, this.cfg), pulseWidth };
  }
}
