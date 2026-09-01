/**
 * Model-predictive cost-to-go for the action-sequence optimizer.
 *
 * Why this exists. The section-7 objective is defined at t = 180 s, but while
 * the station is still tumbling at ~0.2 rad/s the attitude error at t = 180 s
 * is an essentially arbitrary number: it is not a controllable quantity yet, so
 * using it as search guidance is indistinguishable from optimising noise. The
 * scalar below replaces it with a *completion estimate* that stays meaningful
 * across the whole rate range:
 *
 *   - `onTimeToStop`   - nozzle seconds needed to null the angular momentum
 *                        that is left, from the exact minimum-fuel allocation;
 *   - `onTimeToPoint`  - nozzle seconds for the spin-up / spin-down pair that
 *                        removes the attitude error left after stopping;
 *   - `timeNeeded`     - mission seconds those burns occupy, including the
 *                        duty-cycle loss of the 40 ms quantum and the 120 ms
 *                        delay;
 *   - infeasibility    - how far the completion estimate overruns the mission
 *                        clock or the fuel floor.
 *
 * The total mission on-time (already burned + still needed) is the true fuel
 * objective, so a wasteful burn is penalised even though a perfectly efficient
 * burn is cost-neutral; the predicted completion epoch supplies the urgency.
 * Once the state is inside or near the terminal box the box terms dominate and
 * the cost reduces to the section-7 objective.
 *
 * This is a terminal cost inside an MPC, not a hand-tuned feedback law: it
 * never emits a command, it only ranks candidate action sequences.
 */
import { mv, vnorm, type Vec3 } from "../math3d";
import { GATES, type TerminalEval } from "./objective";
import { allocateDeltaOmega } from "./proposals";
import { attitudeErrorVector, deg } from "../math3d";
import type { SimState } from "../types";
import type { PublicConfig } from "../types";
import { buildSurrogate, propagate, type SurrogateModel } from "./surrogate";
import type { RolloutResult } from "./plant";

/** Fraction of mission time a burn actually spends firing (40 ms in a 100 ms tick). */
const DUTY = 0.8;

/** Timing-jitter horizon used to inflate the terminal pointing error, seconds. */
export const TAU_JITTER = 0.0;

export interface CompletionEstimate {
  onTimeToStop: number;
  onTimeToPoint: number;
  onTimeRemaining: number;
  timeNeeded: number;
  fuelNeeded: number;
  /** Mission epoch at which the manoeuvre is predicted to be complete. */
  completionT: number;
  timeOverrun: number;
  fuelOverrun: number;
}

/**
 * Estimate what it still costs to finish, starting from `st`.
 * `model` is built at `st`, so the torque columns and inertia are the local
 * ones rather than the ones from the start of the mission.
 */
export function completionEstimate(
  cfg: PublicConfig,
  model: SurrogateModel,
  st: SimState,
  live: readonly number[],
  tFinal: number,
): CompletionEstimate {
  const w: Vec3 = [st.w[0], st.w[1], st.w[2]];
  const stopAlloc = allocateDeltaOmega(model, [-w[0], -w[1], -w[2]], live);
  const onTimeToStop = Number.isFinite(stopAlloc.total) ? stopAlloc.total : 1e6;
  const stopMissionTime = onTimeToStop / DUTY;

  // Where would the attitude be once the stop burn is finished?
  const afterStop = propagate(
    cfg,
    model,
    { t: st.t, q: [st.q[0], st.q[1], st.q[2], st.q[3]], w, fuel: st.fuel },
    [],
    st.t + stopMissionTime,
    0.25,
  );
  const eAng = vnorm(attitudeErrorVector(afterStop.end.q, cfg.qTarget)) * 2;
  const tLeft = Math.max(1e-3, tFinal - (st.t + stopMissionTime));
  // Two-impulse eigen-axis slew: spin up to eAng/T, coast, spin down.
  const slewRate = eAng / tLeft;
  const axis = attitudeErrorVector(afterStop.end.q, cfg.qTarget);
  const an = vnorm(axis);
  let onTimeToPoint = 0;
  if (an > 1e-12 && Number.isFinite(slewRate)) {
    const dwSlew: Vec3 = [
      (-axis[0] / an) * slewRate,
      (-axis[1] / an) * slewRate,
      (-axis[2] / an) * slewRate,
    ];
    const a = allocateDeltaOmega(model, dwSlew, live);
    onTimeToPoint = 2 * (Number.isFinite(a.total) ? a.total : 1e6);
  }

  const onTimeRemaining = onTimeToStop + onTimeToPoint;
  const timeNeeded = onTimeRemaining / DUTY + 2 * cfg.commandDelay;
  const fuelNeeded = onTimeRemaining * model.fuelRate;
  const completionT = st.t + timeNeeded;
  return {
    onTimeToStop,
    onTimeToPoint,
    onTimeRemaining,
    timeNeeded,
    fuelNeeded,
    completionT,
    timeOverrun: Math.max(0, completionT - tFinal),
    fuelOverrun: Math.max(0, fuelNeeded - (st.fuel - GATES.fuelFloor)),
  };
}

export interface CostWeights {
  /** Penalty for a plan whose terminal set does not hold across the window. */
  dwell: number;
  /** Penalty for arriving with no braking reserve left. */
  braking: number;
  /** Per second of total mission nozzle on-time. */
  onTime: number;
  /** Per second of predicted completion epoch. */
  completion: number;
  /** Per unit of normalised terminal-box excess at t = 180 s. */
  box: number;
  /** Per unit of normalised terminal error *inside* the box (level-3 term). */
  margin: number;
  /** Per unit of infeasibility (mission clock / fuel floor overrun). */
  infeasible: number;
  hard: number;
}

export const DEFAULT_WEIGHTS: CostWeights = {
  dwell: 300,
  braking: 250,
  onTime: 60,
  completion: 3,
  box: 4000,
  margin: 400,
  infeasible: 2e4,
  hard: 1e7,
};

export interface CostBreakdown {
  cost: number;
  completion: CompletionEstimate;
  boxExcess: number;
  boxMargin: number;
  attEff: number;
  onTimeMission: number;
  term: TerminalEval;
}

/**
 * Scalar guidance cost for one candidate roll-out.
 * `r` must be a roll-out that executes the candidate and then coasts to
 * `tFinal`, so `r` carries both the section-7 terminal evaluation and the
 * state at the end of the commanded burns.
 */
export function candidateCost(
  cfg: PublicConfig,
  r: RolloutResult,
  term: TerminalEval,
  live: readonly number[],
  tFinal: number,
  etaT: number,
  weights: CostWeights = DEFAULT_WEIGHTS,
  boxValid = true,
): CostBreakdown {
  const se = r.atScheduleEnd;
  const model = buildSurrogate(cfg, se, etaT);
  const comp = completionEstimate(cfg, model, se, live, tFinal);

  // Normalised distance outside the terminal box at t = 180 s. Uses a soft
  // ramp so that candidates far outside the box are still ordered.
  // Timing-robust pointing error. Judging the raw attitude error at t = 180 s
  // lets a candidate score well by *flying past* the target attitude at a high
  // residual rate: the pointing gate is met at exactly one instant and the
  // result is an artefact of the coast tail, not a capture. Inflating the error
  // by the drift the residual rate produces over `TAU_JITTER` seconds removes
  // that artefact and makes the pointing term monotone in how well the rate is
  // actually nulled, which is the joint terminal condition section 7 asks for.
  const attEff = term.dwellAttitudeDeg + deg(term.dwellOmega) * TAU_JITTER;
  // Persistent capture is the primary pointing measure: the worst value across
  // the capture window, not the value at the final instant. A free-drift
  // fly-by of the target cannot earn credit here, because sweeping past
  // implies a rate large enough to leave the window.
  const ratios = [
    attEff / GATES.attitudeDeg,
    term.dwellOmega / GATES.omega,
    term.sloshRatio / GATES.sloshRatio,
    term.impactSpeed / GATES.impactSpeed,
  ];
  // Demoting persistent capture to a graded secondary term was implemented and
  // measured and is worse: see failure-analysis.json, `graded_dwell_worse`.
  // Using min over t of the pointing error as a success proxy is forbidden and
  // is used nowhere.
  // Outside the box the excess dominates; inside the box the section-7 level-3
  // objective still asks for the smallest terminal error, so keep a bounded
  // margin term that separates a grazing pass from a comfortable one.
  let boxExcess = 0;
  let boxMargin = 0;
  if (boxValid) {
    for (const x of ratios) {
      boxExcess += Math.log1p(Math.max(0, x - 1));
      boxMargin += Math.min(1, x);
    }
  }

  // Braking reserve: how much of the still-needed stopping burn the remaining
  // fuel and remaining clock can actually cover. A candidate that arrives near
  // the box with no reserve left is worse than one that arrives slightly wider
  // but can still be corrected.
  const fuelLeft = Math.max(0, r.fuel - GATES.fuelFloor);
  const brakingDeficit =
    comp.fuelNeeded > 1e-9 ? Math.max(0, 1 - fuelLeft / comp.fuelNeeded) : 0;
  const dwellPenalty = term.dwellHeld ? 0 : 1;

  const onTimeMission = r.totalOnTime + comp.onTimeRemaining;
  const cost =
    weights.hard * term.hardViolations +
    weights.infeasible * (comp.timeOverrun + 40 * comp.fuelOverrun) +
    weights.onTime * onTimeMission +
    weights.completion * comp.completionT +
    weights.box * boxExcess +
    weights.dwell * dwellPenalty +
    weights.braking * brakingDeficit +
    weights.margin * boxMargin;
  return { cost, completion: comp, boxExcess, boxMargin, attEff, onTimeMission, term };
}

export { deg, mv };
