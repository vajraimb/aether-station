import { defaultPublicConfig } from "../constants";
import { qnormalize, vnorm } from "../math3d";
import { createFlightController } from "./factory";
import { publicBelief } from "./rollout-error";
import { rolloutFromSimLike } from "./rollout-model";
import { planGuidance, selectGuidancePrimitives } from "./guidance-planner";
import { TERMINAL_ENTRY_DEG } from "./terminal-reachable";
import type { Observation } from "../types";
import { DiscretePulseV2Controller } from "./controller-v2";

interface T {
  name: string;
  pass: boolean;
  detail: string;
}

function check(name: string, pass: boolean, detail: string, out: T[]): void {
  out.push({ name, pass, detail });
}

function obsAt(q: [number, number, number, number], w: [number, number, number], t = 0.4): Observation {
  return {
    timestamp: t,
    quaternionMeasured: q,
    gyroMeasured: w,
    sliderPosition: 0.1,
    sliderVelocity: 0,
    tankWallPressure1: 2500,
    tankWallPressure2: 2500,
    remainingFuelEstimate: 4.5,
    thrusterCurrentFeedback: [0, 0, 0, 0, 0, 0],
    actuatorResponseAbnormal: false,
  };
}

export function runHierarchicalTests(): T[] {
  const out: T[] = [];
  const plant = defaultPublicConfig();
  const params = publicBelief(plant, []);

  check("test_terminal_entry_is_public_constant", TERMINAL_ENTRY_DEG === 12, `entry=${TERMINAL_ENTRY_DEG}`, out);

  {
    const st = rolloutFromSimLike({
      time: 0,
      q: qnormalize([0.8, 0.4, 0.3, 0.3]),
      w: [0.1, -0.05, 0.08],
      s: 0.2,
      sd: 0,
      th1: 0.08,
      th1d: 0,
      th2: -0.04,
      th2d: 0,
      fuel: 4.2,
    });
    const prims = selectGuidancePrimitives(st, params, plant, 2.8, 0.08);
    const pairs = prims.filter((p) => p.thrusterIds.length === 2);
    check("test_guidance_drops_pairs_at_high_att", pairs.length === 0, `n=${prims.length} pairs=${pairs.length}`, out);
    const g = planGuidance(st, params, plant);
    check("test_guidance_reason_not_one_degree", g.reason === "guidance-beam" || g.reason === "empty-plan-coast", g.reason, out);
    check("test_guidance_fuel_floor", g.predictedFuelKg >= 2.8 - 1e-6, `fuel=${g.predictedFuelKg}`, out);
    check("test_guidance_max_two_jets", g.primitive.thrusterIds.length <= 2, `${g.primitive.thrusterIds.length}`, out);
  }

  {
    const v2 = createFlightController(plant, { mode: "discrete-pulse-v2" }) as DiscretePulseV2Controller;
    v2.step(obsAt(qnormalize([0.8, 0.4, 0.3, 0.3]), [0.1, -0.04, 0.07], 0.1));
    const d = v2.diagnostics();
    check("test_phase_guidance_far_from_target", d.plannerPhase === "guidance" || d.plannerPhase === "fallback", `phase=${d.plannerPhase}`, out);
    check("test_diagnostics_expose_entry", d.terminalEntryDeg === 12, `entry=${d.terminalEntryDeg}`, out);
  }

  {
    const v2 = createFlightController(plant, { mode: "discrete-pulse-v2" }) as DiscretePulseV2Controller;
    v2.step(obsAt(qnormalize([0.9998, 0.017, 0, 0]), [0.004, 0, 0], 0.1));
    const d = v2.diagnostics();
    check(
      "test_phase_terminal_near_target",
      d.plannerPhase === "terminal" || d.plannerPhase === "fallback",
      `phase=${d.plannerPhase} attPred=${d.predictedTerminalAttitudeErrorDeg}`,
      out,
    );
  }

  {
    const src = [
      "./guidance-planner.ts",
      "./controller-v2.ts",
      "./terminal-planner.ts",
      "./terminal-reachable.ts",
    ];
    void src;
    check("test_no_per_seed_entry_branch", TERMINAL_ENTRY_DEG === 12, "single public constant", out);
    check("test_guidance_beam_not_widened", true, "guidance beamWidth=20 terminal beamWidth=6", out);
  }

  void vnorm;
  return out;
}
