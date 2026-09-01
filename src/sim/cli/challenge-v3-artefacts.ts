/**
 * Challenge V3 stage-1 artefacts: capability levels, failure analysis, and the
 * release manifest.
 *
 * Every claim carries a `claim_type` from the specification vocabulary
 * {measured, search_unreached, proven, hypothesis, deprecated}. Nothing that
 * was merely not found by the search is labelled physically infeasible.
 *
 * Usage: tsx src/sim/cli/challenge-v3-artefacts.ts --truth <train10.jsonl>
 */
import { readFileSync } from "node:fs";
import { runLevel1 } from "../challenge-v3/levels";
import { DEFAULT_BUDGET } from "../challenge-v3/planner";
import { DEFAULT_WEIGHTS } from "../challenge-v3/cost";
import { GATES } from "../challenge-v3/objective";
import { TRAIN_SEEDS } from "../evalset";
import { writeJson } from "../io";
import { provenance, summarize, type RawRow, type ScoredRow } from "./challenge-v3-report";

function scored(rows: RawRow[]): ScoredRow[] {
  return rows.map((r) => {
    const gate_attitude = r.att < GATES.attitudeDeg;
    const gate_rate = r.omega < GATES.omega;
    const gate_fuel = r.fuel > GATES.fuelHard;
    const gate_slosh = r.slosh < GATES.sloshRatio;
    const gate_impact = r.impact < GATES.impactSpeed;
    const gate_quat = r.quat < GATES.quatNormErr;
    return {
      ...r,
      gate_attitude,
      gate_rate,
      gate_fuel,
      gate_slosh,
      gate_impact,
      gate_quat,
      gate_all:
        gate_attitude && gate_rate && gate_fuel && gate_slosh && gate_impact && gate_quat,
    };
  });
}

/** Artefacts the specification requires but which stage 1 did not reach. */
const HALTED = {
  status: "not_produced",
  claim_type: "search_unreached",
  reason:
    "Specification section 7 requires that observation-only development stop when the truth-state Train-10 gate fails. The gate failed (attitude 70% against a 90% requirement), so estimation, active identification, belief-rollout calibration and observation-only planning were not attempted and no numbers are reported for them. This is a halt mandated by the specification, not a claim that these stages are infeasible.",
} as const;

function main(): void {
  const argv = process.argv.slice(2);
  const arg = (k: string): string | undefined => {
    const i = argv.indexOf(k);
    return i >= 0 ? argv[i + 1] : undefined;
  };
  const truthPath = arg("--truth");
  if (!truthPath) throw new Error("--truth <file.jsonl> is required");
  const raw: RawRow[] = readFileSync(truthPath, "utf8")
    .split("\n")
    .filter((l) => l.trim().length > 0)
    .map((l) => JSON.parse(l) as RawRow);
  raw.sort((a, b) => a.seed - b.seed);
  const rows = scored(raw);
  const sum = summarize(rows) as { rates: Record<string, number> };
  const prov = provenance();

  const seeds = TRAIN_SEEDS.slice(0, 10);
  const l1 = seeds.map((s) => runLevel1(s));
  const l1Rate = l1.filter((r) => r.passed).length / l1.length;

  writeJson("outputs/challenge-v3/level-baseline.json", {
    artefact: "level-baseline",
    ...prov,
    dataset: { set: "train", split: "train10", seeds },
    levels: {
      L1: {
        claim_type: "measured",
        definition:
          "known parameters, ideal continuous three-axis body torque saturated at 5.8 N*m per axis, no fault, no slosh, no sensor noise, no command delay, no minimum pulse width",
        harness:
          "src/sim/challenge-v3/levels.ts runLevel1 - integrates the audited integrateWithCollision with the torque injected through u.tauThrO; the audited physics files are untouched",
        pass_rate: l1Rate,
        per_seed: l1,
        verdict:
          l1Rate >= 0.9
            ? "PASS - the basic pointing problem is solved in this inertia and torque envelope, so a stage-1 failure cannot be attributed to basic control"
            : "FAIL - basic control must be fixed before anything else",
      },
      L2: {
        claim_type: "measured",
        definition:
          "known parameters, real 40 ms pulse quantisation, 120 ms command delay, two-thruster concurrency, single thruster failure, fuel constraint, truth state readable",
        note:
          "The measured stage-1 run is L2 with the slosh and slider subsystems left active, i.e. strictly harder than the minimal L2 definition; it is the same run reported in truth-optimizer-train10.json.",
        pass_rate_attitude: sum.rates.attitude,
        pass_rate_rate: sum.rates.rate,
        pass_rate_fuel: sum.rates.fuel,
        pass_rate_all_gates: sum.rates.all_gates,
        verdict:
          "FAIL - the level that fails is the discrete-actuation level, with L1 passing at 100%. Per specification section 5 this failure MUST NOT be attributed to the estimator.",
      },
      L3: { ...HALTED, definition: "real thrusters, single fault, observation-only, eta_T unknown, slosh off or known" },
      L4: { ...HALTED, definition: "full mission" },
    },
    localisation:
      "L1 passes 10/10 with terminal attitude 0.004-0.010 deg and terminal rate below 2e-4 rad/s. The same scenarios fail at L2. The loss is therefore created entirely by the discrete action space (40 ms quantum, two-thruster concurrency, 120 ms delay) and by the finite search over action sequences, not by control-theoretic authority, not by the estimator, and not by fuel.",
  });

  writeJson("outputs/challenge-v3/failure-analysis.json", {
    artefact: "failure-analysis",
    ...prov,
    scope:
      "Action-space, horizon and optimization failure analysis required by specification section 7 when the truth-state Train-10 gate fails.",
    gate_outcome: {
      claim_type: "measured",
      attitude_pass_rate: sum.rates.attitude,
      rate_pass_rate: sum.rates.rate,
      fuel_pass_rate: sum.rates.fuel,
      all_gates_pass_rate: sum.rates.all_gates,
      required: { attitude: 0.9, rate: 0.9, fuel: 0.9, all_gates: 0.8 },
      passed: false,
    },
    failing_seeds: rows
      .filter((r) => !r.gate_all)
      .map((r) => ({
        seed: r.seed,
        attitude_deg: r.att,
        angular_speed_rad_s: r.omega,
        remaining_fuel_kg: r.fuel,
        slosh_ratio: r.slosh,
        fault_thruster: r.faultThruster,
        fault_time_s: r.faultTime,
        binding_gate: "attitude",
        margin_deg: r.att - GATES.attitudeDeg,
      })),
    diagnosis: [
      {
        id: "attitude_only_binding_gate",
        claim_type: "measured",
        finding:
          "Every failure on Train-10 is an attitude failure. Rate, fuel, slosh, slider impact and quaternion-norm gates pass on 10 of 10 seeds. Worst remaining fuel is 3.27 kg against a 2.8 kg floor, so roughly 0.47 kg of unused control authority remains on the worst seed: the failures are not fuel-limited.",
      },
      {
        id: "not_basic_control",
        claim_type: "measured",
        finding:
          "L1 with ideal continuous torque reaches 0.004-0.010 deg on the same 10 scenarios. The attitude requirement is therefore two to three orders of magnitude inside the reach of the plant, and the loss is created by discretisation and search.",
      },
      {
        id: "terminal_timing_sensitivity",
        claim_type: "measured",
        finding:
          "Attitude error drifts as omega times elapsed time, so a plan must arrive at the target attitude exactly at t = 180 s. At the measured terminal rates of 3e-4 to 5e-3 rad/s, a 1 s timing error costs 0.02-0.29 deg. The 40 ms pulse quantum sets a floor on how finely arrival time can be shaped, and the 120 ms command delay shifts every correction. This is the dominant residual error source on the three failing seeds.",
      },
      {
        id: "horizon_versus_budget",
        claim_type: "search_unreached",
        finding:
          "The planner replans 20-65 times per mission and expends 6.2k-23.4k full-fidelity rollouts. The failing seeds are the ones with the largest replan counts (65, 50 and 43), i.e. the ones where the search never settled on a committed capture. Larger node-expansion and population budgets were not explored to convergence inside this run. Whether the required sequences exist inside the given action space is therefore unresolved: no infeasibility is claimed.",
      },
      {
        id: "deferral_pathology",
        claim_type: "measured",
        finding:
          "Scoring each candidate prefix with an analytically auto-completed terminal capture attached was implemented and measured. It is strictly worse: on Train-10 it produced 0 of 10 attitude passes with terminal attitudes of 5-173 deg. The mechanism is that the completion is free in the score but is never committed, so the optimizer defers all work to a tail plan that is re-solved and discarded at every epoch. The change was reverted and the flag `autoComplete` retained as a diagnostic default-off option.",
      },
      {
        id: "coarse_coast_inside_schedule",
        claim_type: "measured",
        finding:
          "Coarsening the integrator step during idle stretches inside a schedule, rather than only after the schedule ends, gave a large speed-up and destroyed accuracy where it matters: the coast leg of a rest-to-rest capture. Train-10 attitude fell to 0 of 10 with terminal attitudes of 0.5-13 deg. Reverted.",
      },
      {
        id: "terminal_rate_penalty",
        claim_type: "measured",
        finding:
          "Adding a terminal-rate term to the effective attitude cost (TAU_JITTER = 2.0 s of residual drift) regressed two seeds that had been inside the gate (800051 0.433 -> 3.73 deg, 800017 2.43 -> 7.46 deg). Reverted to 0.",
      },
      {
        id: "configuration_variance",
        claim_type: "measured",
        finding:
          "Two planner configurations were each measured on the full Train-10 split. The richer rest-to-rest arrival grid without the shooting-polish stage passed 8 of 10 attitude (failing 800017 and 800119); the reduced grid with the polish stage passed 7 of 10 (failing 800017, 800051 and 800085). The union of seeds passed by at least one configuration is 9 of 10, and only 800017 fails under both. This is evidence that the required sequences are close to reachable and that the shortfall is a search and selection problem, not an action-space limit. It is not evidence that any single deterministic configuration reaches 90%.",
      },
      {
        id: "persistent_hard_seed",
        claim_type: "measured",
        finding:
          "Seed 800017 fails under every configuration measured, best 2.43 deg. A reachability probe from its own t = 165 s truth state (attitude 12.479 deg, omega 2.789e-2 rad/s, fuel 3.990 kg, live nozzles 0,1,2,4,5) found a best achievable terminal attitude of 2.908 deg, so at that point the mission is already lost. The error is therefore committed upstream: between t = 100 s and t = 146 s the planner ranked an accidental free-drift fly-by of the target (predicted 1.00 deg, predicted rate 0.0211 rad/s) above a real capture, and only committed to braking too late.",
      },
    ],
    action_space_assessment: {
      claim_type: "search_unreached",
      statement:
        "The action space specified (coast, all legal single nozzles, all legal nozzle pairs, durations 40/80/120/160/240/320 ms) is NOT shown to be infeasible for the terminal gates. The evidence points the other way: 9 of 10 Train-10 seeds were passed by some measured configuration within this action space, and the torque matrix retains rank 3 with condition number about 2.4 after a single failure. What has not been achieved is a single fixed, deterministic search configuration that reaches the required 90%.",
      explicitly_not_claimed:
        "physically infeasible - the specification forbids labelling a search shortfall as infeasibility, and no infeasibility certificate was computed",
    },
    next_actions: [
      "Replace the greedy per-epoch selection with an explicit commit-time decision: rank candidates by terminal-capture probability of the committed prefix alone, and forbid crediting free-drift fly-by attitudes that the plan does not actively brake into.",
      "Raise branch-and-bound node expansions and CEM population until the per-epoch winner stops changing, and report the convergence curve rather than a fixed budget chosen by wall-clock affordability.",
      "Add arrival-time refinement on the terminal capture at the 40 ms quantum, i.e. search the arrival tick explicitly instead of taking whichever rest-to-rest arrival the generator proposes.",
      "Only after the truth-state gate passes, resume the estimation, active identification and observation-only stages.",
    ],
    configuration: { budget: DEFAULT_BUDGET, cost_weights: DEFAULT_WEIGHTS, gates: GATES },
    retained_failed_runs:
      "No failed run was deleted. Every seed measured on Train-10 is reported in truth-optimizer-train10.json with its full metric row, including the three failures.",
  });

  for (const [name, extra] of [
    ["identifiability", { required_by: "specification section 8" }],
    ["dual-control-ablation", { required_by: "specification section 8" }],
    ["belief-rollout-calibration", { required_by: "specification section 12" }],
    ["train10", { required_by: "specification section 4, observation-only public gate" }],
    ["train50", { required_by: "specification section 4, observation-only public gate" }],
    ["ablation", { required_by: "specification section 14" }],
  ] as const) {
    writeJson(`outputs/challenge-v3/${name}.json`, {
      artefact: name,
      ...prov,
      ...HALTED,
      ...extra,
    });
  }

  writeJson("outputs/challenge-v3/release-manifest.json", {
    artefact: "release-manifest",
    ...prov,
    stage: "stage-1-truth-state-feasibility",
    verdict: "CONTROL FAIL / TASK SOLVED NO",
    verdict_reason:
      "The truth-state Train-10 feasibility gate of specification section 7 was not met (attitude 70% against 90%), so observation-only development was halted as the specification requires and no hidden-set evaluation was run.",
    hidden_set: {
      status: "not_run",
      claim_type: "search_unreached",
      reason:
        "Specification section 3 forbids running the hidden set before the public training gates pass. Neither public gate was reached.",
    },
    artefacts: {
      produced: [
        "outputs/challenge-v3/level-baseline.json",
        "outputs/challenge-v3/truth-optimizer-train10.json",
        "outputs/challenge-v3/failure-analysis.json",
        "outputs/challenge-v3/release-manifest.json",
      ],
      halted: [
        "outputs/challenge-v3/identifiability.json",
        "outputs/challenge-v3/dual-control-ablation.json",
        "outputs/challenge-v3/belief-rollout-calibration.json",
        "outputs/challenge-v3/train10.json",
        "outputs/challenge-v3/train50.json",
        "outputs/challenge-v3/ablation.json",
      ],
      not_authorized: ["outputs/challenge-v3/hidden50-summary.json"],
    },
    regeneration: {
      level_and_failure_artefacts:
        "npx tsx src/sim/cli/challenge-v3-artefacts.ts --truth <rows.jsonl>",
      truth_optimizer_report:
        "npx tsx src/sim/cli/challenge-v3-report.ts --rows <rows.jsonl>",
      raw_rows:
        "the raw per-seed rows are regenerated by running TruthHorizonController over TRAIN_SEEDS.slice(0,10) with DEFAULT_BUDGET; the planner is deterministic under a fixed seed so the rows reproduce exactly",
    },
    large_trajectories_committed: false,
    compute: {
      claim_type: "measured",
      rollouts_per_mission: [6233, 23411],
      replans_per_mission: [20, 65],
      planning_wall_s_per_mission: [181, 519],
      note:
        "Wall-clock is reported for information only. No decision anywhere in the planner depends on it; all budgets are fixed iteration, node-expansion and population counts.",
    },
  });

  process.stdout.write(`L1 pass rate ${l1Rate}\n`);
}

main();
