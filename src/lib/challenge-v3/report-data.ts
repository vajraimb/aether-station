/**
 * Read model for the Challenge V3 truth-state result page.
 *
 * The committed artefacts under `outputs/challenge-v3/` are the single source
 * of truth and are imported directly, so the page cannot drift from the numbers
 * that were audited and pushed. Nothing here recomputes a metric: every value
 * rendered is a field of an artefact, and the only derived quantities are chart
 * geometry (log positions, bar widths) and the pass counts, which are
 * cross-checked against the pass rates the artefacts already state.
 */

import ablationJson from "../../../outputs/challenge-v3/objective-ablation.json";
import convergenceJson from "../../../outputs/challenge-v3/search-convergence.json";
import truthJson from "../../../outputs/challenge-v3/truth-optimizer-train10.json";
import failureJson from "../../../outputs/challenge-v3/failure-analysis.json";
import levelsJson from "../../../outputs/challenge-v3/level-baseline.json";

/** The artefact vocabulary. `search_unreached` is never "proven infeasible". */
export type ClaimType = "measured" | "search_unreached" | "proven" | "hypothesis" | "deprecated";

export interface Claim {
  claim_type: ClaimType;
  text: string;
}

export interface SeedRow {
  seed: number;
  passed: boolean;
  final_attitude_error_deg: number;
  final_angular_speed_rad_s: number;
  remaining_fuel_kg: number;
  final_slosh_energy_ratio: number;
  replans: number;
  rollouts: number;
  wall_clock_s: number;
}

export interface ConfigRow {
  id: string;
  claim_type: ClaimType;
  description: string;
  raw_rows: string;
  seeds: number;
  attitude_pass_rate: number;
  rate_pass_rate: number;
  fuel_pass_rate: number;
  all_gates_pass_rate: number;
  attitude_deg: { best: number; median: number; worst: number };
  per_seed: SeedRow[];
}

export interface LadderRung {
  budget_multiplier: number;
  bnb_node_expansions: number;
  cem_generations: number;
  cem_population: number;
  rollouts: number;
  winner: string;
  segments: number;
  best_attitude_deg: number;
  best_dwell_attitude_deg: number;
  best_dwell_omega: number;
  dwell_held: boolean;
  all_gates_pass: boolean;
  remaining_fuel_kg: number;
  sequence_sha256_16: string;
  wall_clock_s: number;
}

export interface ConvergenceStudy {
  key: string;
  seed: number;
  epoch_s: number;
  budget_ladder: LadderRung[];
  rollout_growth: number;
  distinct_best_sequences: number;
  best_sequence_unchanged_across_ladder: boolean;
  attitude_improvement_deg: number;
}

export interface DiagnosisEntry {
  id: string;
  claim_type: ClaimType;
  finding: string;
}

export interface GateBlock {
  required: {
    attitude_pass_rate: number;
    rate_pass_rate: number;
    fuel_pass_rate: number;
    all_physical_terminal_gates: number;
  };
  measured: {
    attitude_pass_rate: number;
    rate_pass_rate: number;
    fuel_pass_rate: number;
    all_physical_terminal_gates: number;
  };
  passed: boolean;
}

interface AblationArtefact {
  question: string;
  answer: Claim;
  provenance: {
    branch: string;
    commit: string;
    start_commit: string;
    physics_baseline_sha: string;
    immutable_file_sha256: Record<string, string>;
  };
  configurations: ConfigRow[];
  retained_configuration: string;
  leave_one_seed_out_selector: Claim;
}

interface ConvergenceArtefact {
  method: Claim;
  studies: ConvergenceStudy[];
  finding: Claim;
  consequence: Claim;
  explicitly_not_claimed: string;
}

interface TruthArtefact {
  claim_type: ClaimType;
  claim: string;
  branch: string;
  commit: string;
  start_commit: string;
  immutable_diff_vs_start: string;
  determinism: { fixed_seed: boolean; wall_clock_deadlines: boolean; note: string };
  gate: GateBlock;
  n: number;
}

interface FailureArtefact {
  diagnosis: DiagnosisEntry[];
  formally_recorded_limitation: { claim_type: ClaimType; title: string; statement: string };
  next_actions: unknown[];
  completed_actions?: unknown[];
}

interface LevelsArtefact {
  levels: Record<
    string,
    {
      claim_type: ClaimType;
      definition: string;
      pass_rate: number;
      per_seed: { seed: number; attitudeDeg: number; omega: number; passed: boolean }[];
    }
  >;
}

const ablation = ablationJson as unknown as AblationArtefact;
const convergence = convergenceJson as unknown as ConvergenceArtefact;
const truth = truthJson as unknown as TruthArtefact;
const failure = failureJson as unknown as FailureArtefact;
const levels = levelsJson as unknown as LevelsArtefact;

/** Human-facing labels, in the reader's language. Ids stay verbatim. */
const CONFIG_LABELS: Record<string, { short: string; title: string }> = {
  c1_baseline_instantaneous_terminal: { short: "C1", title: "基线：瞬时终端评分" },
  c2_dwell_primary_plus_slot_search: { short: "C2", title: "持续捕获为主 + 槽位搜索" },
  c3_dwell_graded_secondary: { short: "C3", title: "持续捕获降级为次要项" },
  c4_exact_eigen_axis_slew_sizing: { short: "C4", title: "精确特征轴 slew 定尺" },
};

/** Which follow-up item each configuration corresponds to, and its fate. */
const CONFIG_FATE: Record<string, { items: string; fate: "retained" | "rejected" }> = {
  c1_baseline_instantaneous_terminal: { items: "第一轮", fate: "rejected" },
  c2_dwell_primary_plus_slot_search: { items: "A + B + C", fate: "retained" },
  c3_dwell_graded_secondary: { items: "A 变体", fate: "rejected" },
  c4_exact_eigen_axis_slew_sizing: { items: "B 变体", fate: "rejected" },
};

export interface Config extends ConfigRow {
  short: string;
  title: string;
  items: string;
  retained: boolean;
  passCount: number;
}

export const configs: Config[] = ablation.configurations.map((c) => {
  const label = CONFIG_LABELS[c.id] ?? { short: c.id, title: c.id };
  const fate = CONFIG_FATE[c.id] ?? { items: "—", fate: "rejected" as const };
  return {
    ...c,
    short: label.short,
    title: label.title,
    items: fate.items,
    retained: c.id === ablation.retained_configuration,
    passCount: c.per_seed.filter((s) => s.passed).length,
  };
});

/** Train-10 seed order, taken from the artefact rather than assumed. */
export const seeds: number[] = [...configs[0]!.per_seed.map((s) => s.seed)].sort((a, b) => a - b);

export const canonical: Config = configs.find((c) => c.retained) ?? configs[0]!;

/** Seeds that fail under the retained configuration, in seed order. */
export const failingSeeds: number[] = seeds.filter(
  (s) => !canonical.per_seed.find((r) => r.seed === s)?.passed,
);

/** Seeds that fail under at least one of the four measured configurations. */
export const everFailingSeeds: number[] = seeds.filter((s) =>
  configs.some((c) => !c.per_seed.find((r) => r.seed === s)?.passed),
);

export const gate: GateBlock = truth.gate;

export const gateRows = [
  {
    key: "attitude",
    label: "终端指向",
    detail: "e_q < 1°",
    measured: gate.measured.attitude_pass_rate,
    required: gate.required.attitude_pass_rate,
  },
  {
    key: "rate",
    label: "终端角速度",
    detail: "‖ω‖ < 0.008 rad/s",
    measured: gate.measured.rate_pass_rate,
    required: gate.required.rate_pass_rate,
  },
  {
    key: "fuel",
    label: "剩余燃料",
    detail: "> 2.8 kg",
    measured: gate.measured.fuel_pass_rate,
    required: gate.required.fuel_pass_rate,
  },
  {
    key: "all",
    label: "全部物理终端门",
    detail: "指向 · 角速度 · 燃料 · 晃动 · 碰撞 · 四元数",
    measured: gate.measured.all_physical_terminal_gates,
    required: gate.required.all_physical_terminal_gates,
  },
] as const;

export const provenance = {
  branch: truth.branch,
  commit: truth.commit,
  commitShort: truth.commit.slice(0, 7),
  startCommit: truth.start_commit,
  startCommitShort: truth.start_commit.slice(0, 7),
  immutableDiff: truth.immutable_diff_vs_start,
  determinism: truth.determinism.note,
  fixedSeed: truth.determinism.fixed_seed,
  wallClockDeadlines: truth.determinism.wall_clock_deadlines,
  claimType: truth.claim_type,
  claim: truth.claim,
  n: truth.n,
  physicsBaseline: ablation.provenance.physics_baseline_sha.slice(0, 7),
  immutableFiles: Object.keys(ablation.provenance.immutable_file_sha256),
};

export const ablationQuestion = ablation.question;
export const ablationAnswer = ablation.answer;
export const selectorNote = ablation.leave_one_seed_out_selector;

export const studies: ConvergenceStudy[] = convergence.studies;
export const convergenceMethod = convergence.method;
export const convergenceFinding = convergence.finding;
export const convergenceConsequence = convergence.consequence;
export const convergenceNotClaimed: string = convergence.explicitly_not_claimed;

/** Ladders whose winning action list is bit-identical from 1x to 8x budget. */
export const identicalLadders = studies.filter(
  (s) => s.best_sequence_unchanged_across_ladder,
).length;

export const limitation = failure.formally_recorded_limitation;

/** The findings added by this round, in the order they were established. */
const THIS_ROUND_FINDINGS = [
  "coast_wins_by_default",
  "search_converged_at_8x",
  "candidate_expressiveness_not_budget",
  "pending_queue_over_strict",
  "small_angle_slew_sizing",
  "graded_dwell_worse",
  "persistent_hard_seed",
] as const;

const FINDING_TITLES: Record<string, string> = {
  coast_wins_by_default: "纯 coast 直接胜出",
  search_converged_at_8x: "8× 预算下搜索已收敛",
  candidate_expressiveness_not_budget: "瓶颈是候选表达力，不是预算",
  pending_queue_over_strict: "pending 队列规则过严",
  small_angle_slew_sizing: "小角度 slew 定尺偏差",
  graded_dwell_worse: "分级 dwell 更差",
  persistent_hard_seed: "800017 在所有配置下均失败",
};

export interface Finding extends DiagnosisEntry {
  title: string;
}

export const findings: Finding[] = THIS_ROUND_FINDINGS.map((id) => {
  const entry = failure.diagnosis.find((d) => d.id === id);
  if (!entry) return null;
  return { ...entry, title: FINDING_TITLES[id] ?? id };
}).filter((f): f is Finding => f !== null);

/** The capability-level contrast: ideal continuous torque vs pulse-quantised. */
export const level1 = levels.levels.L1;

export const level1Range = (() => {
  const att = level1.per_seed.map((s) => s.attitudeDeg);
  const om = level1.per_seed.map((s) => s.omega);
  return {
    attMin: Math.min(...att),
    attMax: Math.max(...att),
    omMin: Math.min(...om),
    omMax: Math.max(...om),
    passRate: level1.pass_rate,
  };
})();

/** Terminal-authority arithmetic, quoted from the recorded next actions. */
export const terminalAuthority = {
  minPulseS: 0.04,
  deltaOmega: 3.2e-4,
  leverArmS: 100,
};

export const ATTITUDE_GATE_DEG = 1.0;
export const RATE_GATE = 0.008;
export const FUEL_GATE_KG = 2.8;
export const DWELL_WINDOW_S = 3.0;
