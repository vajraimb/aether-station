/**
 * Receding-horizon direct action-sequence optimizer.
 *
 * At every replanning epoch the decision variable is the action sequence
 * itself - `pi = [(action, duration), ...]` over the legal action set and the
 * 40..320 ms duration grid - and the objective is the section-7 lexicographic
 * terminal objective *evaluated by rolling the candidate out on the audited
 * plant all the way to the end of the mission*. There is no PD law, no gain
 * schedule and no attitude-threshold state machine in the loop: the executed
 * command is whatever the optimizer returns.
 *
 * The same routine serves stage 1 (single truth parameter vector) and stage 3
 * (a parameter/state ensemble ranked by the section-9 robust order), which is
 * what makes the two comparable.
 */
import { vnorm, type Vec3 } from "../math3d";
import type { PublicConfig, SimState } from "../types";
import {
  ALL_ACTIONS,
  sequenceDurationS,
  sequencePulseCount,
  type Action,
  type Segment,
} from "./action-space";
import { expandSchedule, rollout, type PlantParams, type RolloutResult } from "./plant";
import { buildSurrogate, brakingOnTimeLowerBound } from "./surrogate";
import { evaluateTerminal, lexCompare, GATES, type TerminalEval } from "./objective";
import { candidateCost, DEFAULT_WEIGHTS, type CostWeights } from "./cost";
import {
  allocateDeltaOmega,
  generateProposals,
  restToRestProposals,
  scheduleAllocation,
} from "./proposals";
import { branchAndBound, cem, type ExpansionSet, type SeqScore } from "./optimizers";

export interface PlannerBudget {
  /** Coarse search fidelity. */
  searchDt: number;
  searchCoastDt: number;
  /** Fine fidelity used to re-rank the shortlist. */
  refineDt: number;
  refineCoastDt: number;
  refineTopK: number;
  maxProposalSegments: number;
  bnbNodeExpansions: number;
  bnbBeamWidth: number;
  bnbMaxDepth: number;
  bnbDurations: number[];
  cemSlots: number;
  cemPopulation: number;
  cemGenerations: number;
  cemElite: number;
  seed: number;
  optimizers: Array<"proposals" | "bnb" | "cem">;
  /** Ensemble members re-scored in the shortlist stage (stage 3). */
  ensembleTopK: number;
  /** Score candidates with an analytic completion attached (diagnostic). */
  autoComplete: boolean;
  /** Shooting-correction rounds applied to the best coarse candidates. */
  polishRounds: number;
  /** How many candidates enter each polish round. */
  polishSeeds: number;
}

export const DEFAULT_BUDGET: PlannerBudget = {
  searchDt: 0.02,
  searchCoastDt: 0.4,
  refineDt: 0.005,
  refineCoastDt: 0.05,
  refineTopK: 16,
  maxProposalSegments: 140,
  bnbNodeExpansions: 8,
  bnbBeamWidth: 8,
  bnbMaxDepth: 4,
  bnbDurations: [0.04, 0.16, 0.32],
  cemSlots: 6,
  cemPopulation: 24,
  cemGenerations: 4,
  cemElite: 5,
  seed: 1,
  optimizers: ["proposals", "bnb", "cem"],
  ensembleTopK: 6,
  autoComplete: false,
  polishRounds: 2,
  polishSeeds: 3,
};

export interface PlanRequest {
  cfg: PublicConfig;
  /** Nominal / truth state the plan starts from. */
  start: SimState;
  /** Nominal parameters (ensemble member 0). */
  params: PlantParams;
  /** Parameter/state ensemble for robust ranking. A single member = nominal MPC. */
  ensemble?: Array<{ state: SimState; params: PlantParams; weight: number }>;
  /** Mission-initial slosh energy (denominator of the reported ratio). */
  sloshRef: number;
  /** End of mission. */
  tFinal: number;
  /** Nozzles believed alive right now. */
  live: number[];
  budget: PlannerBudget;
  weights?: CostWeights;
}

export interface PlanDiagnostics {
  rollouts: number;
  iterations: number;
  perOptimizer: Record<string, { rollouts: number; iterations: number; cost: number; key: number[] }>;
  winner: string;
  refinedFrom: string;
  candidatesRefined: number;
}

export interface PlanResult {
  seq: Segment[];
  term: TerminalEval;
  key: number[];
  cost: number;
  diag: PlanDiagnostics;
}

/**
 * Aggregate per-member candidate scores with the section-9 robust order:
 *   1. probability of a hard-constraint violation,
 *   2. worst-case fuel *feasibility* (clipped at the floor plus a reserve, so
 *      fuel is a constraint and not an objective - once every member clears the
 *      floor with reserve the members tie and ranking moves on),
 *   3. terminal-gate satisfaction probability,
 *   4. CVaR of the terminal cost-to-go over the worst half of the ensemble,
 *   5. expected terminal cost-to-go,
 *   6. pulse count.
 */
export const FUEL_RESERVE = 0.1;

export function aggregateEnsemble(
  evals: TerminalEval[],
  costs: number[],
  weights: number[],
  cvarAlpha = 0.5,
): { key: number[]; cost: number; rep: TerminalEval } {
  const n = evals.length;
  const wsum = weights.reduce((a, b) => a + b, 0) || 1;
  const pHard = evals.reduce((a, e, i) => a + (e.hardViolations > 0 ? weights[i]! : 0), 0) / wsum;
  const worstFuel = Math.min(...evals.map((e) => e.fuel));
  const fuelFeas = Math.min(worstFuel, GATES.fuelFloor + FUEL_RESERVE);
  const pGates = evals.reduce((a, e, i) => a + (e.allGatesPass ? weights[i]! : 0), 0) / wsum;
  const sorted = [...costs].sort((a, b) => b - a);
  const kTail = Math.max(1, Math.ceil(cvarAlpha * n));
  const cvar = sorted.slice(0, kTail).reduce((a, b) => a + b, 0) / kTail;
  const mean = costs.reduce((a, c, i) => a + c * weights[i]!, 0) / wsum;
  const pulses = Math.max(...evals.map((e) => e.pulses));
  const key = [pHard, -fuelFeas, -pGates, cvar, mean, pulses];
  let rep = evals[0]!;
  for (const e of evals) if (e.worstRatio > rep.worstRatio) rep = e;
  return { key, cost: mean, rep };
}

function legalOnly(seq: readonly Segment[], live: readonly number[]): boolean {
  const alive = new Set(live);
  for (const s of seq) {
    if (s.action.length > 2) return false;
    for (const id of s.action) if (!alive.has(id)) return false;
    if (s.action.length > 0 && s.durationS < 0.04 - 1e-12) return false;
  }
  return true;
}

/**
 * Nozzles that are still available for the *whole* remaining horizon. A nozzle
 * that is known to fail before the end must not be planned with beyond that
 * point, so the completion is always built on the post-fault actuator set.
 */
function liveAt(live: readonly number[], params: PlantParams, tFinal: number): number[] {
  if (params.faultThruster < 0 || params.faultTime > tFinal) return [...live];
  return live.filter((id) => id !== params.faultThruster);
}

/**
 * Rate below which a rest-to-rest capture solve is trustworthy, rad/s.
 * Above it the mission is still in the detumble regime.
 */
export const CAPTURE_RATE = 0.02;

/** Solve one replanning epoch. */
export function planHorizon(req: PlanRequest): PlanResult {
  const { cfg, start, params, sloshRef, tFinal, live, budget } = req;
  const ensemble = req.ensemble ?? [{ state: start, params, weight: 1 }];
  const weights = req.weights ?? DEFAULT_WEIGHTS;
  const model = buildSurrogate(cfg, start, params.etaT);

  let rolloutCount = 0;
  const scoreOn = (
    seq: readonly Segment[],
    member: { state: SimState; params: PlantParams },
    dt: number,
    coastDt: number,
  ): { term: TerminalEval; r: RolloutResult } => {
    rolloutCount += 1;
    const r = rollout(cfg, member.state, member.params, expandSchedule(seq, cfg.controllerPeriod), {
      dt,
      coastDt,
      until: tFinal,
      sloshRef,
      audit: false,
    });
    return { term: evaluateTerminal(r, sequencePulseCount(seq)), r };
  };

  /**
   * Auto-completion. Judging a candidate by "execute it, then coast to
   * t = 180 s" is meaningless in mid-mission: the coast tail dominates the
   * terminal state and a prefix that achieves nothing can score well simply
   * because the free drift happens to sweep past the target attitude. Instead
   * every candidate prefix is completed by the analytic rest-to-rest capture
   * solved from the state at which the prefix stops burning, so a candidate is
   * always judged as "prefix + best known way of finishing the mission from
   * there". The optimizers therefore search over prefixes while the ranking
   * always sees a full mission plan.
   */
  const completionFor = (
    member: { state: SimState; params: PlantParams },
    seq: readonly Segment[],
  ): { full: Segment[]; se: SimState; boxValid: boolean } => {
    const planned = sequenceDurationS(seq, cfg.controllerPeriod);
    const until = Math.min(tFinal, member.state.t + planned + cfg.commandDelay + 0.4);
    rolloutCount += 1;
    const pre = rollout(
      cfg,
      member.state,
      member.params,
      expandSchedule(seq, cfg.controllerPeriod),
      { dt: budget.searchDt, coastDt: budget.searchCoastDt, until, sloshRef, audit: false },
    );
    const se = pre.atScheduleEnd;
    if (se.t > tFinal - 2 || se.fuel <= GATES.fuelFloor) {
      return { full: [...seq], se, boxValid: vnorm(se.w) <= CAPTURE_RATE };
    }
    const m2 = buildSurrogate(cfg, se, member.params.etaT);
    const liveEnd = liveAt(live, member.params, tFinal);
    if (vnorm(se.w) > CAPTURE_RATE) {
      // Still tumbling. A rest-to-rest solve is a small-angle construction and
      // is meaningless here, so complete with a plain full detumble: the
      // terminal attitude of the resulting plan carries no information and the
      // box terms are switched off, leaving the ranking to the fuel and
      // completion-time terms, which are exactly the right guidance while the
      // objective is still "stop the tumble efficiently".
      const alloc = allocateDeltaOmega(m2, [-se.w[0], -se.w[1], -se.w[2]], liveEnd);
      if (!Number.isFinite(alloc.total) || alloc.total <= 0) {
        return { full: [...seq], se, boxValid: false };
      }
      const stop = scheduleAllocation(alloc, "nearest", true, budget.maxProposalSegments);
      return { full: [...seq, ...stop], se, boxValid: false };
    }
    const corr = restToRestProposals(
      m2,
      se.q,
      se.w,
      cfg.qTarget,
      se.t,
      tFinal,
      liveEnd,
      cfg.controllerPeriod,
      budget.maxProposalSegments,
    );
    // Deterministic pick: the latest feasible arrival, i.e. the longest lever
    // arm and the smallest slew rate, which is also the cheapest completion.
    let pick: Segment[] | null = null;
    for (const c of corr) if (!c.label.endsWith("/pair")) pick = c.seq;
    if (!pick && corr.length > 0) pick = corr[corr.length - 1]!.seq;
    return { full: pick ? [...seq, ...pick] : [...seq], se, boxValid: true };
  };

  /** Coarse single-member evaluator used inside the optimizers. */
  const coarse = (seq: readonly Segment[]): SeqScore => {
    if (!legalOnly(seq, live)) {
      const bad: TerminalEval = {
        hardViolations: 99,
        gatesFailed: 4,
        worstRatio: 1e9,
        gateExcess: 1e9,
        attitudeDeg: 1e9,
        omega: 1e9,
        sloshRatio: 1e9,
        impactSpeed: 1e9,
        fuel: 0,
        pulses: 0,
        peakOmega: 1e9,
        allGatesPass: false,
      };
      return { cost: 1e12, key: [1e9], term: bad };
    }
    // Receding-horizon evaluation: execute the candidate, then let the mission
    // coast to t = 180 s. Auto-completing the candidate with an analytic
    // capture before scoring was tried and measured worse (see
    // failure-analysis.json, `deferral_pathology`): because the tail plan is
    // free in the score, the optimizer defers all work to a completion that is
    // never actually committed.
    const useCompletion = budget.autoComplete;
    const c2 = useCompletion ? completionFor(ensemble[0]!, seq) : null;
    const scored = c2 ? c2.full : seq;
    const { term, r } = scoreOn(scored, ensemble[0]!, budget.searchDt, budget.searchCoastDt);
    const cb = candidateCost(
      cfg,
      r,
      term,
      live,
      tFinal,
      params.etaT,
      weights,
      c2 ? c2.boxValid : true,
    );
    return { cost: cb.cost, key: [cb.cost], term, full: c2 ? c2.full : undefined };
  };

  const proposals = generateProposals(
    model,
    start.q,
    start.w,
    cfg.qTarget,
    start.t,
    tFinal,
    live,
    budget.maxProposalSegments,
    cfg.controllerPeriod,
  );

  const shortlist: Array<{ label: string; seq: Segment[]; coarseCost: number }> = [];
  const perOptimizer: PlanDiagnostics["perOptimizer"] = {};
  let iterations = 0;

  if (budget.optimizers.includes("proposals")) {
    const before = rolloutCount;
    let best = Infinity;
    for (const p of proposals) {
      const s = coarse(p.seq);
      shortlist.push({ label: `proposal:${p.label}`, seq: [...p.seq], coarseCost: s.cost });
      if (s.cost < best) best = s.cost;
      iterations += 1;
    }
    perOptimizer.proposals = {
      rollouts: rolloutCount - before,
      iterations: proposals.length,
      cost: best,
      key: [best],
    };
  }

  // Expansion alphabet: coast plus every live single nozzle. Nozzle pairs are
  // reachable through the proposal seeds and the CEM alphabet; keeping the
  // branching factor at 1 + |live| is what makes a fixed node budget useful.
  const expansionActions: Action[] = ALL_ACTIONS.filter(
    (a) => a.length <= 1 && a.every((id) => live.includes(id)),
  );
  const expansion: ExpansionSet = {
    actions: expansionActions,
    durations: budget.bnbDurations,
  };

  if (budget.optimizers.includes("bnb")) {
    const before = rolloutCount;
    const seeds = proposals.slice(0, 8).map((p) => p.seq);
    const fuelRate = model.fuelRate;
    const res = branchAndBound(coarse, seeds, expansion, {
      maxDepth: budget.bnbMaxDepth,
      nodeExpansions: budget.bnbNodeExpansions,
      beamWidth: budget.bnbBeamWidth,
      // Sound prune. Extending a node only ever burns more propellant, so a
      // node whose own completion is already below the fuel floor is dead.
      // Additionally, satisfying the rate gate requires at least
      // `brakingOnTimeLowerBound` more seconds of nozzle on-time - an
      // optimistic bound that assumes the single best-aligned column - so a
      // node that cannot pay for that burn can never satisfy the rate gate and
      // the fuel floor simultaneously.
      prune: (_seq, score) => {
        if (score.term.fuel < GATES.fuelHard) return true;
        const excessRate = Math.max(0, score.term.omega - GATES.omega);
        if (excessRate <= 0) return false;
        const lb = brakingOnTimeLowerBound(
          model,
          [excessRate, 0, 0] as Vec3,
          live,
        );
        if (!Number.isFinite(lb)) return true;
        return score.term.fuel - fuelRate * lb < GATES.fuelHard;
      },
    });
    shortlist.push({ label: "bnb", seq: res.seq, coarseCost: res.score.cost });
    perOptimizer.bnb = {
      rollouts: rolloutCount - before,
      iterations: res.iterations,
      cost: res.score.cost,
      key: res.score.key,
    };
    iterations += res.iterations;
  }

  if (budget.optimizers.includes("cem")) {
    const before = rolloutCount;
    const res = cem(coarse, expansion, {
      slots: budget.cemSlots,
      population: budget.cemPopulation,
      generations: budget.cemGenerations,
      elite: budget.cemElite,
      seed: budget.seed + Math.round(start.t * 1000),
      warmStart: proposals.slice(0, 6).map((p) => p.seq),
    });
    shortlist.push({ label: "cem", seq: res.seq, coarseCost: res.score.cost });
    perOptimizer.cem = {
      rollouts: rolloutCount - before,
      iterations: res.iterations,
      cost: res.score.cost,
      key: res.score.key,
    };
    iterations += res.iterations;
  }

  // ---------------------------------------------------------------------
  // Capture polish. A rest-to-rest manoeuvre is built from a small-angle
  // fixed point, so it lands the *rate* accurately but leaves a degree-level
  // pointing residual. Rather than widen the search, close the loop inside the
  // horizon: roll the candidate out, read the residual at the instant its
  // burns finish, and append a corrective rest-to-rest solved from that state.
  // This is a shooting correction on the same action-sequence grammar - it
  // adds segments, never a feedback law - and two rounds are enough to bring
  // the residual under the pointing gate.
  // ---------------------------------------------------------------------
  if (budget.polishRounds > 0 && shortlist.length > 0) {
    const before = rolloutCount;
    let polishAdded = 0;
    const ranked = [...shortlist].sort(
      (a, b) => a.coarseCost - b.coarseCost || a.label.localeCompare(b.label),
    );
    let frontier = ranked.slice(0, budget.polishSeeds).map((c) => ({ label: c.label, seq: c.seq }));
    for (let round = 0; round < budget.polishRounds; round++) {
      const next: Array<{ label: string; seq: Segment[] }> = [];
      for (const c of frontier) {
        const { r } = scoreOn(c.seq, ensemble[0]!, budget.searchDt, budget.searchCoastDt);
        const se = r.atScheduleEnd;
        if (se.t > tFinal - 2 || se.fuel <= GATES.fuelFloor) continue;
        const m2 = buildSurrogate(cfg, se, params.etaT);
        const corr = restToRestProposals(
          m2,
          se.q,
          se.w,
          cfg.qTarget,
          se.t,
          tFinal,
          live,
          cfg.controllerPeriod,
          budget.maxProposalSegments,
        );
        // Keep a handful of arrival times: an early one leaves room for a
        // further correction, a late one maximises the drift lever arm.
        const picks = [corr[0], corr[Math.floor(corr.length / 2)], corr[corr.length - 1]].filter(
          (x): x is { label: string; seq: Segment[] } => x !== undefined,
        );
        for (const pk of picks) {
          const seq = [...c.seq, ...pk.seq];
          const label = `${c.label}+polish${round + 1}`;
          const sc = coarse(seq);
          shortlist.push({ label, seq, coarseCost: sc.cost });
          next.push({ label, seq });
          polishAdded += 1;
        }
      }
      frontier = next.slice(0, budget.polishSeeds);
      if (frontier.length === 0) break;
    }
    perOptimizer.polish = {
      rollouts: rolloutCount - before,
      iterations: polishAdded,
      cost: 0,
      key: [0],
    };
  }

  // Shortlist -> fine fidelity, ensemble-robust ranking (section 9 order).
  shortlist.sort((a, b) => a.coarseCost - b.coarseCost || a.label.localeCompare(b.label));
  const uniq: typeof shortlist = [];
  const seen = new Set<string>();
  for (const c of shortlist) {
    const k = c.seq.map((s) => `${s.action.join("+")}@${s.durationS}`).join(",");
    if (seen.has(k)) continue;
    seen.add(k);
    uniq.push(c);
    if (uniq.length >= budget.refineTopK) break;
  }

  const members = ensemble.slice(0, Math.max(1, budget.ensembleTopK));
  let best: { label: string; seq: Segment[]; key: number[]; cost: number; rep: TerminalEval } | null =
    null;
  for (const c of uniq) {
    const evals: TerminalEval[] = [];
    const costs: number[] = [];
    for (const m of members) {
      // Receding horizon: the candidate is *scored* with its auto-completion
      // attached, but only the prefix is ever committed - the completion is
      // re-solved from the measured state at the next replan.
      const cm = budget.autoComplete ? completionFor(m, c.seq) : null;
      const { term, r } = scoreOn(cm ? cm.full : c.seq, m, budget.refineDt, budget.refineCoastDt);
      evals.push(term);
      costs.push(
        candidateCost(
          cfg,
          r,
          term,
          live,
          tFinal,
          m.params.etaT,
          weights,
          cm ? cm.boxValid : true,
        ).cost,
      );
    }
    const agg = aggregateEnsemble(
      evals,
      costs,
      members.map((m) => m.weight),
    );
    if (!best || lexCompare(agg.key, best.key) < 0) {
      best = { label: c.label, seq: c.seq, key: agg.key, cost: agg.cost, rep: agg.rep };
    }
  }

  const chosen = best ?? { label: "coast", seq: [] as Segment[], key: [0], cost: 0, rep: coarse([]).term };
  return {
    seq: chosen.seq,
    term: chosen.rep,
    key: chosen.key,
    cost: chosen.cost,
    diag: {
      rollouts: rolloutCount,
      iterations,
      perOptimizer,
      winner: chosen.label,
      refinedFrom: uniq[0]?.label ?? "none",
      candidatesRefined: uniq.length,
    },
  };
}

/** Nozzles not known to be dead. */
export function liveThrusters(failed: ReadonlySet<number>): number[] {
  const out: number[] = [];
  for (let i = 0; i < 6; i++) if (!failed.has(i)) out.push(i);
  return out;
}
