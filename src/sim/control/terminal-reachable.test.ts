import { defaultPublicConfig, MIN_PULSE, THRUSTERS } from "../constants";
import { qnormalize } from "../math3d";
import { publicBelief } from "./rollout-error";
import { rolloutFromSimLike } from "./rollout-model";
import { generatePulsePrimitives } from "./discrete-actions";
import {
  TERMINAL_ENTRY_CANDIDATES_DEG,
  TERMINAL_ENTRY_DEG,
  basinFlags,
  canCaptureWithinHorizon,
  selectTerminalEntryDeg,
} from "./terminal-reachable";
import { TERMINAL_PULSE_DURATIONS, planTerminal, selectTerminalPrimitives } from "./terminal-planner";

interface T {
  name: string;
  pass: boolean;
  detail: string;
}

function check(name: string, pass: boolean, detail: string, out: T[]): void {
  out.push({ name, pass, detail });
}

export function runTerminalReachableTests(): T[] {
  const out: T[] = [];
  const plant = defaultPublicConfig();
  const params = publicBelief(plant, []);

  check(
    "test_entry_candidates_public",
    TERMINAL_ENTRY_CANDIDATES_DEG.join(",") === "8,10,12,15" && TERMINAL_ENTRY_DEG === 12,
    `chosen=${TERMINAL_ENTRY_DEG}`,
    out,
  );
  check(
    "test_terminal_durations_legal",
    TERMINAL_PULSE_DURATIONS.every((d) => d >= MIN_PULSE - 1e-12 && d <= 0.16 + 1e-12),
    TERMINAL_PULSE_DURATIONS.join(","),
    out,
  );

  {
    const st = rolloutFromSimLike({
      time: 0,
      q: [1, 0, 0, 0],
      w: [0.001, 0, 0],
      s: 0,
      sd: 0,
      th1: 0,
      th1d: 0,
      th2: 0,
      th2d: 0,
      fuel: 3.2,
    });
    const cap = canCaptureWithinHorizon(st, 0.04, [0, 1, 2, 3, 4, 5], params, plant);
    check("test_capture_already_in_gates", cap.captured && cap.reason === "already-in-gates", cap.reason, out);
    const flags = basinFlags(st, plant);
    check("test_basin_near_target", flags.inBasin, `att=${flags.attDeg} perp=${flags.wPerp}`, out);
  }

  {
    const st = rolloutFromSimLike({
      time: 0,
      q: qnormalize([0.7071, 0, 0.7071, 0]),
      w: [0, 0, 0],
      s: 0,
      sd: 0,
      th1: 0,
      th1d: 0,
      th2: 0,
      th2d: 0,
      fuel: 3.2,
    });
    const cap = canCaptureWithinHorizon(st, 0.04, [0, 1, 2, 3, 4, 5], params, plant, { horizonS: 1.6, expansionBudget: 24 });
    check("test_capture_rejects_90deg_rest", !cap.captured, `${cap.reason} att=${cap.predictedAttDeg.toFixed(1)}`, out);
  }

  {
    const st = rolloutFromSimLike({
      time: 0,
      q: qnormalize([0.995, 0.1, 0, 0]),
      w: [-0.01, 0, 0],
      s: 0.1,
      sd: 0,
      th1: 0.02,
      th1d: 0,
      th2: -0.01,
      th2d: 0,
      fuel: 3.1,
    });
    const failed = { ...params, failedThrusterBeliefs: [2] };
    const prims = selectTerminalPrimitives(st, failed, plant, 2.8, 0.04);
    const isolatedUsed = prims.some((p) => p.thrusterIds.includes(2));
    check("test_terminal_primitives_drop_isolated", !isolatedUsed, prims.map((p) => p.id).join(","), out);
    check(
      "test_terminal_primitives_include_coast",
      prims.some((p) => p.thrusterIds.length === 0),
      prims.map((p) => p.id).join(","),
      out,
    );
    const plan = planTerminal(st, failed, plant);
    check("test_terminal_plan_respects_isolation", !plan.primitive.thrusterIds.includes(2), plan.primitive.id, out);
    check("test_terminal_plan_two_jet_limit", plan.primitive.thrusterIds.length <= 2, `${plan.primitive.thrusterIds.length}`, out);
  }

  {
    const st = rolloutFromSimLike({
      time: 0,
      q: qnormalize([0.9998, 0.0175, 0, 0]),
      w: [-0.012, 0, 0],
      s: 0.1,
      sd: 0,
      th1: 0.02,
      th1d: 0,
      th2: -0.01,
      th2d: 0,
      fuel: 3.1,
    });
    const plan = planTerminal(st, params, plant);
    check(
      "test_terminal_improves_or_holds_2deg",
      plan.predictedAttDeg < 2.1 && plan.predictedFuelKg > 2.8,
      `att=${plan.predictedAttDeg.toFixed(3)} w=${plan.predictedOmega.toFixed(4)} cap=${plan.captured}`,
      out,
    );
  }

  {
    const all = generatePulsePrimitives(THRUSTERS, { durationsS: [...TERMINAL_PULSE_DURATIONS] });
    const illegal = all.filter((p) => p.durationS < MIN_PULSE - 1e-12 || p.thrusterIds.length > 2);
    check("test_terminal_action_legality", illegal.length === 0, `n=${all.length}`, out);
  }

  {
    const report = selectTerminalEntryDeg(plant, "outputs/terminal-entry-selection.json");
    check(
      "test_entry_sweep_covers_candidates",
      report.rows.length === 4 && report.chosenDeg === 12,
      JSON.stringify(report.rows.map((r) => [r.entryDeg, r.closingCaptureRate])),
      out,
    );
  }

  return out;
}
