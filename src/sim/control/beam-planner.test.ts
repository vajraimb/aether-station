import { defaultPublicConfig, MIN_PULSE, THRUSTERS } from "../constants";
import { massState } from "../dynamics";
import { qnormalize } from "../math3d";
import { compareLexicographic, quaternionSignInvariantAtt, scoreRollout, type LexicographicScore } from "./lexicographic-cost";
import { DEFAULT_BEAM_CONFIG, planBeam, quantizeStateKey, selectCandidatePrimitives } from "./beam-planner";
import { generatePulsePrimitives } from "./discrete-actions";
import { rolloutFromSimLike, type RolloutParameters } from "./rollout-model";

interface T {
  name: string;
  pass: boolean;
  detail: string;
}

function check(name: string, pass: boolean, detail: string, out: T[]): void {
  out.push({ name, pass, detail });
}

function baseScore(over: Partial<LexicographicScore> = {}): LexicographicScore {
  return {
    hardViolationCount: 0,
    predictedFuelBelowFloor: 0,
    terminalRateGateFailure: 0,
    terminalAttitudeGateFailure: 0,
    sliderRisk: 0,
    sloshRisk: 0,
    captureCost: 0,
    attRad: 0.2,
    omega: 0.02,
    perpMomentum: 0.01,
    sloshEnergy: 0.1,
    fuelConsumedKg: 0.2,
    pulseTransitions: 3,
    constraintMargin: 1,
    ...over,
  };
}

export function runBeamPlannerTests(): T[] {
  const out: T[] = [];

  {
    const feasible = baseScore({ attRad: 0.5 });
    const infeasible = baseScore({ predictedFuelBelowFloor: 1, attRad: 0.001 });
    check(
      "test_lexicographic_feasibility",
      compareLexicographic(feasible, infeasible) === -1,
      "fuel-infeasible cannot beat a feasible candidate",
      out,
    );
    const lowFuel = baseScore({ predictedFuelBelowFloor: 1, attRad: 0.001, fuelConsumedKg: 0 });
    const okFuel = baseScore({ predictedFuelBelowFloor: 0, attRad: 0.2, fuelConsumedKg: 1 });
    check(
      "test_beam_fuel_hard_constraint",
      compareLexicographic(okFuel, lowFuel) === -1,
      "2.8 kg floor is not a weighted penalty",
      out,
    );
  }

  {
    const cfg = defaultPublicConfig();
    const q = qnormalize([0.9, 0.2, -0.2, 0.2]);
    const d = quaternionSignInvariantAtt(q, cfg.qTarget);
    check("test_quaternion_sign_invariance", d < 1e-12, `Δatt=${d}`, out);
  }

  {
    const cfg = defaultPublicConfig();
    const st = rolloutFromSimLike({
      time: 0,
      q: [0.92, 0.22, -0.22, 0.22],
      w: [0.08, -0.04, 0.05],
      s: 0.2,
      sd: 0,
      th1: 0.1,
      th1d: 0,
      th2: -0.05,
      th2d: 0,
      fuel: 4.5,
    });
    const params: RolloutParameters = {
      inertiaEstimate: massState(cfg, st.sliderS, st.theta1, st.theta2, st.fuelMass).Icm,
      etaTEstimate: 0.87,
      c1Estimate: 0.13,
      c2Estimate: 0.09,
      k12Estimate: 0.3,
      failedThrusterBeliefs: [],
    };
    const cands = selectCandidatePrimitives(st, params, cfg, 2.8, 0.08);
    check("test_beam_keeps_coast", cands.some((p) => p.thrusterIds.length === 0), `n=${cands.length} ids=${cands.map((c) => c.id).join(",")}`, out);

    const a = planBeam(st, params, cfg, { ...DEFAULT_BEAM_CONFIG, expansionBudget: 400, beamWidth: 16, horizonS: 3 });
    const b = planBeam(st, params, cfg, { ...DEFAULT_BEAM_CONFIG, expansionBudget: 400, beamWidth: 16, horizonS: 3 });
    check(
      "test_beam_determinism",
      a.diagnostics.selectedPrimitiveId === b.diagnostics.selectedPrimitiveId &&
        a.plan.map((p) => p.id).join() === b.plan.map((p) => p.id).join(),
      `id=${a.diagnostics.selectedPrimitiveId} expanded=${a.diagnostics.expandedNodes}`,
      out,
    );

    const k1 = quantizeStateKey(st, params, cfg, 2);
    const flipped = { ...st, qBI: [-st.qBI[0], -st.qBI[1], -st.qBI[2], -st.qBI[3]] as typeof st.qBI };
    const k2 = quantizeStateKey(flipped, params, cfg, 2);
    check("test_beam_state_dedup", k1 === k2, `k1=${k1} k2=${k2}`, out);
  }

  {
    const cfg = defaultPublicConfig();
    const st = rolloutFromSimLike({
      time: 0,
      q: [1, 0, 0, 0],
      w: [0, 0, 0],
      s: 0,
      sd: 0,
      th1: 0,
      th1d: 0,
      th2: 0,
      th2d: 0,
      fuel: 2.82,
    });
    const params: RolloutParameters = {
      inertiaEstimate: massState(cfg, 0, 0, 0, 2.82).Icm,
      etaTEstimate: 0.87,
      c1Estimate: 0.13,
      c2Estimate: 0.09,
      k12Estimate: 0.3,
      failedThrusterBeliefs: [],
    };
    const plan = planBeam(st, params, cfg, { ...DEFAULT_BEAM_CONFIG, expansionBudget: 200, beamWidth: 8, horizonS: 2 });
    check(
      "test_controller_safe_fallback",
      plan.primitive.thrusterIds.length === 0 || plan.diagnostics.selectedPlanFuelMargin >= 0,
      `prim=${plan.primitive.id} margin=${plan.diagnostics.selectedPlanFuelMargin}`,
      out,
    );
    void generatePulsePrimitives;
    void MIN_PULSE;
    void THRUSTERS;
    void scoreRollout;
  }

  return out;
}
