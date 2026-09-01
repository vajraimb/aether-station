/**
 * Stage-1 truth-state receding-horizon action-sequence optimizer, wrapped as a
 * controller so that the official `Simulator` produces every reported number.
 *
 * This is deliberately NOT a flight controller: it reads the true state and the
 * true parameter vector, which the spec allows (and requires) for the level-2
 * capability check and the truth-state feasibility gate. The observation-only
 * controller lives in `robust-controller.ts` and shares the same optimizer.
 *
 * There is no PD law, no bang-coast-bang state machine and no attitude
 * threshold in here. Every command byte comes out of `planHorizon`.
 */
import { TruthFeedbackBaseline } from "../oracle";
import { sliderForceCommand } from "../allocate";
import { sloshEnergy, modalMasses } from "../dynamics";
import type { Estimate } from "../estimator";
import type { Command, PublicConfig, SimState } from "../types";
import { expandSchedule, type PlantParams } from "./plant";
import { sequenceDurationS, type Segment } from "./action-space";
import { DEFAULT_BUDGET, liveThrusters, planHorizon, type PlannerBudget } from "./planner";

export interface TruthHorizonOptions {
  budget?: Partial<PlannerBudget>;
  /** Never replan more often than this (seconds). */
  minReplanGap?: number;
  /** Force a replan at least this often (seconds). */
  maxReplanGap?: number;
  /** Stop planning burns this long before the end (kept > command delay). */
  endGuard?: number;
  /** Optional synchronous progress hook (diagnostics only). */
  onReplan?: (r: ReplanRecord) => void;
}

export interface ReplanRecord {
  t: number;
  winner: string;
  rollouts: number;
  iterations: number;
  segments: number;
  planDurationS: number;
  predictedAttitudeDeg: number;
  predictedOmega: number;
  predictedFuel: number;
  perOptimizer: Record<string, { rollouts: number; iterations: number; cost: number }>;
}

export class TruthHorizonController extends TruthFeedbackBaseline {
  override readonly name: string = "challenge-v3-truth-horizon";
  private truthState: SimState | null = null;
  private plan: Array<readonly number[]> = [];
  private planIdx = 0;
  private nextReplanT = -1e9;
  private sloshRef: number | null = null;
  readonly replans: ReplanRecord[] = [];
  private readonly budget: PlannerBudget;
  private readonly minGap: number;
  private readonly maxGap: number;
  private readonly endGuard: number;
  private readonly onReplan?: (r: ReplanRecord) => void;

  constructor(
    cfg: PublicConfig,
    private readonly truthParams: PlantParams,
    opts: TruthHorizonOptions = {},
  ) {
    super(cfg);
    this.budget = { ...DEFAULT_BUDGET, ...(opts.budget ?? {}) };
    this.minGap = opts.minReplanGap ?? 2.0;
    this.maxGap = opts.maxReplanGap ?? 12.0;
    this.endGuard = opts.endGuard ?? 0.5;
    this.onReplan = opts.onReplan;
  }

  override ingestTruth(state: SimState): void {
    super.ingestTruth(state);
    this.truthState = {
      ...state,
      q: [...state.q],
      w: [...state.w],
      rI: [...state.rI],
      vI: [...state.vI],
      rCmI: [...state.rCmI],
      vCmI: [...state.vCmI],
    };
    if (this.sloshRef === null) {
      const mm = modalMasses(this.cfg.fluidMass);
      this.sloshRef = sloshEnergy(
        state.th1,
        state.th1d,
        state.th2,
        state.th2d,
        mm.m1,
        mm.m2,
        this.cfg.tankMeanRadius,
        this.truthParams.k12,
      );
    }
  }

  private replan(t: number, st: SimState): void {
    const failed = new Set<number>();
    if (t >= this.truthParams.faultTime) failed.add(this.truthParams.faultThruster);
    for (const id of this.truthParams.preFailed ?? []) failed.add(id);
    const live = liveThrusters(failed);
    const res = planHorizon({
      cfg: this.cfg,
      start: st,
      params: this.truthParams,
      sloshRef: this.sloshRef ?? 1,
      tFinal: this.cfg.duration,
      live,
      budget: this.budget,
    });

    const seq: Segment[] = res.seq;
    this.plan = expandSchedule(seq, this.cfg.controllerPeriod);
    this.planIdx = 0;
    const planDur = sequenceDurationS(seq, this.cfg.controllerPeriod);
    const gap = Math.min(this.maxGap, Math.max(this.minGap, planDur));
    this.nextReplanT = t + gap;
    const perOpt: ReplanRecord["perOptimizer"] = {};
    for (const [k, v] of Object.entries(res.diag.perOptimizer)) {
      perOpt[k] = { rollouts: v.rollouts, iterations: v.iterations, cost: v.cost };
    }
    const rec: ReplanRecord = {
      t,
      winner: res.diag.winner,
      rollouts: res.diag.rollouts,
      iterations: res.diag.iterations,
      segments: seq.length,
      planDurationS: planDur,
      predictedAttitudeDeg: res.term.attitudeDeg,
      predictedOmega: res.term.omega,
      predictedFuel: res.term.fuel,
      perOptimizer: perOpt,
    };
    this.replans.push(rec);
    this.onReplan?.(rec);
  }

  protected override control(t: number, est: Estimate): Command {
    const Fs = sliderForceCommand(est.s, est.sd, this.cfg);
    const pulseWidth: [number, number, number, number, number, number] = [0, 0, 0, 0, 0, 0];
    const st = this.truthState;
    if (!st) return { sliderForce: Fs, pulseWidth };

    const tGo = this.cfg.duration - t;
    if (tGo <= this.endGuard) return { sliderForce: Fs, pulseWidth };

    if (t >= this.nextReplanT || this.planIdx >= this.plan.length) {
      if (t >= this.nextReplanT) this.replan(t, st);
    }
    const widths = this.plan[this.planIdx];
    this.planIdx += 1;
    if (widths) for (let i = 0; i < 6; i++) pulseWidth[i] = widths[i] ?? 0;
    return { sliderForce: Fs, pulseWidth };
  }
}
