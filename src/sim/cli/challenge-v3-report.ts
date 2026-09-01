/**
 * Challenge V3 artefact writer.
 *
 * Reads the raw per-seed rows produced by a Challenge V3 evaluation run and
 * emits the artefacts required by the specification under
 * `outputs/challenge-v3/`. All rates and statistics are recomputed here from
 * the raw metrics; no controller ever self-reports a score (spec 18).
 *
 * Usage:
 *   tsx src/sim/cli/challenge-v3-report.ts --rows <file.jsonl> --set train10
 */
import { createHash } from "node:crypto";
import { execFileSync } from "node:child_process";
import { readFileSync } from "node:fs";
import { GATES } from "../challenge-v3/objective";
import { DEFAULT_BUDGET } from "../challenge-v3/planner";
import { writeJson } from "../io";

/** One raw evaluation row, exactly as recorded by the runner. */
export interface RawRow {
  seed: number;
  wall_s: number;
  replans: number;
  rollouts: number;
  att: number;
  omega: number;
  fuel: number;
  slosh: number;
  impact: number;
  quat: number;
  pulses: number;
  faultThruster: number;
  faultTime: number;
  winners?: Record<string, number>;
}

/** Per-gate booleans recomputed by the scorer from the raw metrics. */
export interface ScoredRow extends RawRow {
  gate_attitude: boolean;
  gate_rate: boolean;
  gate_fuel: boolean;
  gate_slosh: boolean;
  gate_impact: boolean;
  gate_quat: boolean;
  gate_all: boolean;
}

function scoreRow(r: RawRow): ScoredRow {
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
}

function quantile(xs: number[], q: number): number {
  if (xs.length === 0) return Number.NaN;
  const s = [...xs].sort((a, b) => a - b);
  const i = (s.length - 1) * q;
  const lo = Math.floor(i);
  const hi = Math.ceil(i);
  return lo === hi ? s[lo]! : s[lo]! + (s[hi]! - s[lo]!) * (i - lo);
}

/** Median / p90 / worst / best of a metric, worst meaning "largest". */
export function stats(xs: number[]): Record<string, number> {
  return {
    median: quantile(xs, 0.5),
    p90: quantile(xs, 0.9),
    worst: Math.max(...xs),
    best: Math.min(...xs),
    mean: xs.reduce((a, b) => a + b, 0) / xs.length,
  };
}

function sha256File(p: string): string {
  return createHash("sha256").update(readFileSync(p)).digest("hex");
}

function git(args: string[]): string {
  return execFileSync("git", args, { encoding: "utf8" }).trim();
}

const PHYSICS_BASELINE = "bdfff5b4c62733d7156d2f9cdeeaa75661d6c9f4";
const START_COMMIT = "86ce40cb98acd32dd7c82fea2a7695e799a144a7";
const IMMUTABLE = ["src/sim/math3d.ts", "src/sim/dynamics.ts", "src/sim/audit.ts"];

/** Provenance block embedded in every artefact (spec 15). */
export function provenance(): Record<string, unknown> {
  const immutable: Record<string, string> = {};
  for (const f of IMMUTABLE) immutable[f] = sha256File(f);
  return {
    branch: git(["rev-parse", "--abbrev-ref", "HEAD"]),
    commit: git(["rev-parse", "HEAD"]),
    start_commit: START_COMMIT,
    physics_baseline_sha: PHYSICS_BASELINE,
    immutable_file_sha256: immutable,
    immutable_diff_vs_start: git(["diff", "--stat", START_COMMIT, "--", ...IMMUTABLE]) || "none",
    determinism: {
      fixed_seed: true,
      wall_clock_deadlines: false,
      note: "All optimizers use fixed iteration/expansion/population counts and a seeded RNG; no wall-clock cutoff is used anywhere in the planner (spec 13).",
    },
  };
}

/** Aggregate a scored row set into the rate/statistic block used by artefacts. */
export function summarize(rows: ScoredRow[]): Record<string, unknown> {
  const n = rows.length;
  const rate = (f: (r: ScoredRow) => boolean) => rows.filter(f).length / n;
  return {
    n,
    rates: {
      attitude: rate((r) => r.gate_attitude),
      rate: rate((r) => r.gate_rate),
      fuel: rate((r) => r.gate_fuel),
      slosh: rate((r) => r.gate_slosh),
      impact: rate((r) => r.gate_impact),
      quaternion: rate((r) => r.gate_quat),
      all_gates: rate((r) => r.gate_all),
    },
    stats: {
      attitude_deg: stats(rows.map((r) => r.att)),
      angular_speed_rad_s: stats(rows.map((r) => r.omega)),
      remaining_fuel_kg: stats(rows.map((r) => r.fuel)),
      slosh_ratio: stats(rows.map((r) => r.slosh)),
      pulse_count: stats(rows.map((r) => r.pulses)),
      rollouts: stats(rows.map((r) => r.rollouts)),
      replans: stats(rows.map((r) => r.replans)),
      planning_wall_s: stats(rows.map((r) => r.wall_s)),
    },
    per_seed: rows,
  };
}

function main(): void {
  const argv = process.argv.slice(2);
  const arg = (k: string): string | undefined => {
    const i = argv.indexOf(k);
    return i >= 0 ? argv[i + 1] : undefined;
  };
  const rowsPath = arg("--rows");
  if (!rowsPath) throw new Error("--rows <file.jsonl> is required");
  const raw: RawRow[] = readFileSync(rowsPath, "utf8")
    .split("\n")
    .filter((l) => l.trim().length > 0)
    .map((l) => JSON.parse(l) as RawRow);
  raw.sort((a, b) => a.seed - b.seed);
  const rows = raw.map(scoreRow);
  const sum = summarize(rows);
  const rates = (sum as { rates: Record<string, number> }).rates;

  const winners: Record<string, number> = {};
  let winnersSeeds = 0;
  for (const r of raw) {
    if (!r.winners) continue;
    winnersSeeds += 1;
    for (const [k, v] of Object.entries(r.winners)) winners[k] = (winners[k] ?? 0) + v;
  }

  // Spec 7 truth-state Train-10 gate.
  const gate = {
    required: {
      attitude_pass_rate: 0.9,
      rate_pass_rate: 0.9,
      fuel_pass_rate: 0.9,
      all_physical_terminal_gates: 0.8,
    },
    measured: {
      attitude_pass_rate: rates.attitude,
      rate_pass_rate: rates.rate,
      fuel_pass_rate: rates.fuel,
      all_physical_terminal_gates: rates.all_gates,
    },
    passed:
      rates.attitude! >= 0.9 &&
      rates.rate! >= 0.9 &&
      rates.fuel! >= 0.9 &&
      rates.all_gates! >= 0.8,
  };

  writeJson("outputs/challenge-v3/truth-optimizer-train10.json", {
    artefact: "truth-optimizer-train10",
    claim_type: gate.passed ? "measured" : "search_unreached",
    claim:
      "Truth-state constrained action-sequence optimization on the public Train-10 split, scored by this file scorer from raw simulator metrics.",
    ...provenance(),
    dataset: { set: "train", split: "train10", seeds: rows.map((r) => r.seed) },
    optimizer: {
      action_space: {
        actions: "coast + all legal single thrusters + all legal legal thruster pairs",
        duration_grid_s: [0.04, 0.08, 0.12, 0.16, 0.24, 0.32],
      },
      deterministic_optimizer: "best-first branch-and-bound with sound fuel-floor and braking-lower-bound pruning",
      stochastic_optimizer: "cross-entropy method over per-slot categorical action distributions, seeded RNG",
      candidate_generators:
        "wrench-space impulse allocation (exact minimum-sum LP over <=3-column bases), rest-to-rest arrival family, coast family; generators only propose, selection is always by full-fidelity rollout of the audited physics",
      budget: DEFAULT_BUDGET,
    },
    gate,
    ...sum,
    optimizer_attribution: {
      seeds_instrumented: winnersSeeds,
      replan_wins_by_optimizer: winners,
      note:
        winnersSeeds === 0
          ? "not captured in this run"
          : "counts the optimizer that produced the committed plan at each replan epoch",
    },
  });

  process.stdout.write(
    `attitude ${(rates.attitude! * 100).toFixed(0)}% rate ${(rates.rate! * 100).toFixed(0)}% fuel ${(rates.fuel! * 100).toFixed(0)}% all ${(rates.all_gates! * 100).toFixed(0)}% -> gate ${gate.passed ? "PASS" : "FAIL"}\n`,
  );
}

// Only run as a CLI; this module is also imported for its helpers.
if (process.argv[1] && process.argv[1].endsWith("challenge-v3-report.ts")) main();
