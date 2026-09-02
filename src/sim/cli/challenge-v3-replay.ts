#!/usr/bin/env npx tsx
/**
 * Challenge V3 replay exporter.
 *
 * Re-runs the retained truth-state L2 configuration over Train-10 seeds
 * through the official `Simulator` and writes a compact trajectory file per
 * seed, so the 3D station view can replay the *measured* mission instead of
 * re-planning in the browser. The planner needs minutes of CPU per mission
 * (see `wall_clock_s` in the raw rows), so a browser-side re-run is not an
 * option; and a re-run would also be a second, unaudited source of truth.
 *
 * The exporter therefore does two things and refuses to hide either:
 *   1. it re-derives the terminal metrics from `Simulator.metrics()`, and
 *   2. it compares them against the committed raw row for the same seed,
 *      recording the absolute difference in the output file.
 * A non-zero difference is a determinism failure and is visible in the file
 * rather than being silently smoothed over.
 *
 * Usage:
 *   npx tsx src/sim/cli/challenge-v3-replay.ts                  # all Train-10
 *   npx tsx src/sim/cli/challenge-v3-replay.ts --seeds 800017   # one seed
 *   npx tsx src/sim/cli/challenge-v3-replay.ts --stride 20      # 0.1 s frames
 */
import { execFileSync } from "node:child_process";
import { readFileSync, existsSync } from "node:fs";
import { defaultPublicConfig } from "../constants.ts";
import { generateScenario } from "../scenario.ts";
import { Simulator } from "../simulator.ts";
import { TRAIN_SEEDS } from "../evalset.ts";
import { writeJson } from "../io.ts";
import { GATES } from "../challenge-v3/objective.ts";
import { TruthHorizonController, type ReplanRecord } from "../challenge-v3/truth-controller.ts";
import type { PlantParams } from "../challenge-v3/plant.ts";
import type { Sample } from "../types.ts";

/** The retained configuration's raw rows, used only as a cross-check. */
const RAW_ROWS = "outputs/challenge-v3/raw/truth-config-2-dwell-slot.jsonl";

interface RawRow {
  seed: number;
  att: number;
  omega: number;
  fuel: number;
  replans: number;
  rollouts: number;
  pulses: number;
  faultThruster: number;
  faultTime: number;
  pass: boolean;
}

function arg(name: string): string | null {
  const i = process.argv.indexOf(name);
  return i >= 0 ? (process.argv[i + 1] ?? null) : null;
}

function gitHead(): string {
  try {
    return execFileSync("git", ["rev-parse", "HEAD"], { encoding: "utf8" }).trim();
  } catch {
    return "unknown";
  }
}

function committedRow(seed: number): RawRow | null {
  if (!existsSync(RAW_ROWS)) return null;
  for (const line of readFileSync(RAW_ROWS, "utf8").split("\n")) {
    if (!line.trim()) continue;
    const r = JSON.parse(line) as RawRow;
    if (r.seed === seed) return r;
  }
  return null;
}

/** Round to `d` decimals. Frames are for drawing, not for re-deriving metrics. */
function r(x: number, d: number): number {
  const f = 10 ** d;
  return Math.round(x * f) / f;
}

/** Six thruster booleans as one integer, so a frame stays small. */
function mask(bits: readonly number[]): number {
  let m = 0;
  for (let i = 0; i < bits.length; i++) if (bits[i] === 1) m |= 1 << i;
  return m;
}

/**
 * One replay frame. Field order is fixed and documented here because the
 * frames ship as arrays, not objects: at 0.1 s over a 180 s mission a seed is
 * 1800 frames, and object keys would be most of the file.
 */
const FRAME_FIELDS = [
  "t",
  "rx",
  "ry",
  "rz",
  "qw",
  "qx",
  "qy",
  "qz",
  "wx",
  "wy",
  "wz",
  "s",
  "th1",
  "th2",
  "attDeg",
  "fuel",
  "thrusterMask",
  "detectedFailed",
] as const;

function frameOf(s: Sample, firing: number): number[] {
  return [
    r(s.t, 3),
    r(s.r[0], 4),
    r(s.r[1], 4),
    r(s.r[2], 4),
    r(s.q[0], 6),
    r(s.q[1], 6),
    r(s.q[2], 6),
    r(s.q[3], 6),
    r(s.w[0], 6),
    r(s.w[1], 6),
    r(s.w[2], 6),
    r(s.s, 5),
    r(s.th1, 5),
    r(s.th2, 5),
    r(s.attitudeErrorDeg, 4),
    r(s.fuelTrue, 4),
    firing,
    s.detectedFailedThruster,
  ];
}

function runSeed(seed: number, stride: number) {
  const cfg = defaultPublicConfig({ seed, fluidPresent: true });
  const sc = generateScenario(seed, false);
  const params: PlantParams = {
    c1: sc.c1,
    c2: sc.c2,
    k12: sc.k12,
    etaT: sc.etaT,
    faultTime: sc.faultTime,
    faultThruster: sc.faultThruster,
  };
  const replans: ReplanRecord[] = [];
  const controller = new TruthHorizonController(cfg, params, {
    onReplan: (rec) => replans.push(rec),
  });
  const sim = new Simulator(cfg, sc, controller);
  const t0 = Date.now();
  sim.runAll();
  const wall = (Date.now() - t0) / 1000;
  const m = sim.metrics();

  // Frames are decimated for transport. A pulse can be as short as 40 ms, so a
  // frame that only sampled the thruster byte at its own instant would drop
  // most firings from the picture. `thrusterMask` is therefore the OR of every
  // logged sample inside the frame's window: "this nozzle fired at some point
  // during these 190 ms", not "this nozzle was on at this instant".
  const frames: number[][] = [];
  for (let i = 0; i < sim.log.length; i += stride) {
    let firing = 0;
    for (let j = i; j < Math.min(i + stride, sim.log.length); j++) {
      firing |= mask(sim.log[j]!.thrusterActual);
    }
    frames.push(frameOf(sim.log[i]!, firing));
  }
  const last = sim.log[sim.log.length - 1];
  if (last && frames[frames.length - 1]?.[0] !== r(last.t, 3)) {
    frames.push(frameOf(last, mask(last.thrusterActual)));
  }
  // Median gap between exported frames. The simulator's log is not perfectly
  // uniform (the first step and the collision handler shorten a few samples),
  // so `stride * dt` would be wrong; the median of the actual frame times is
  // what a player needs.
  const gaps: number[] = [];
  for (let i = 1; i < frames.length; i++) gaps.push(frames[i]![0]! - frames[i - 1]![0]!);
  gaps.sort((a, b) => a - b);
  const frameInterval = gaps.length ? gaps[Math.floor(gaps.length / 2)]! : cfg.dtMax;

  // The six Challenge V3 terminal gates, in exactly the form the audited
  // scorer applies them (`scoreRow` in challenge-v3-report.ts). The
  // simulator's own scorecard is a superset - it also carries the parameter
  // and FDIR gates, which are not part of this verdict - so it must not be
  // used to decide pass/fail here.
  const gate = {
    attitude: m.final_attitude_error_deg < GATES.attitudeDeg,
    rate: m.final_angular_speed_rad_s < GATES.omega,
    fuel: m.remaining_fuel_kg > GATES.fuelHard,
    slosh: m.final_slosh_energy_ratio < GATES.sloshRatio,
    impact: m.max_slider_impact_speed_m_s < GATES.impactSpeed,
    quat: m.quaternion_norm_max_error < GATES.quatNormErr,
  };
  const passed = Object.values(gate).every(Boolean);

  const row = committedRow(seed);
  const check = row
    ? {
        raw_row_file: RAW_ROWS,
        attitude_abs_diff_deg: Math.abs(row.att - m.final_attitude_error_deg),
        omega_abs_diff: Math.abs(row.omega - m.final_angular_speed_rad_s),
        fuel_abs_diff_kg: Math.abs(row.fuel - m.remaining_fuel_kg),
        replans_diff: replans.length - row.replans,
        pass_matches: row.pass === passed,
      }
    : null;

  return {
    seed,
    configuration: "c2_dwell_primary_plus_slot_search",
    frame_fields: FRAME_FIELDS,
    frame_interval_s: r(frameInterval, 4),
    thruster_mask_is_window_or: true,
    duration_s: cfg.duration,
    dwell_window_s: 3,
    gates: {
      attitudeDeg: GATES.attitudeDeg,
      omega: GATES.omega,
      fuelHard: GATES.fuelHard,
      sloshRatio: GATES.sloshRatio,
      impactSpeed: GATES.impactSpeed,
    },
    scenario: {
      faultThruster: sc.faultThruster,
      faultTime: sc.faultTime,
      c1: sc.c1,
      c2: sc.c2,
      k12: sc.k12,
      etaT: sc.etaT,
    },
    metrics: {
      final_attitude_error_deg: m.final_attitude_error_deg,
      final_angular_speed_rad_s: m.final_angular_speed_rad_s,
      remaining_fuel_kg: m.remaining_fuel_kg,
      final_slosh_energy_ratio: m.final_slosh_energy_ratio,
      max_slider_impact_speed_m_s: m.max_slider_impact_speed_m_s,
      quaternion_norm_max_error: m.quaternion_norm_max_error,
      pulse_count: m.pulse_count,
      passed,
      gate,
    },
    replan_count: replans.length,
    replans: replans.map((x) => ({
      t: r(x.t, 3),
      winner: x.winner,
      segments: x.segments,
      rollouts: x.rollouts,
      predicted_attitude_deg: r(x.predictedAttitudeDeg, 4),
      predicted_omega: r(x.predictedOmega, 6),
      plan_duration_s: r(x.planDurationS, 3),
    })),
    determinism_check: check,
    wall_clock_s: r(wall, 1),
    commit: gitHead(),
    frames,
  };
}

const stride = Number(arg("--stride") ?? 4);
const seeds = (arg("--seeds") ?? TRAIN_SEEDS.slice(0, 10).join(","))
  .split(",")
  .map((s) => Number(s.trim()))
  .filter((s) => Number.isFinite(s));

for (const seed of seeds) {
  const out = runSeed(seed, stride);
  const path = `outputs/challenge-v3/replay/seed-${seed}.json`;
  writeJson(path, out);
  const c = out.determinism_check;
  console.log(
    `${out.metrics.passed ? "PASS" : "fail"} seed=${seed} att=${out.metrics.final_attitude_error_deg.toFixed(3)}deg ` +
      `omega=${out.metrics.final_angular_speed_rad_s.toExponential(2)} fuel=${out.metrics.remaining_fuel_kg.toFixed(3)}kg ` +
      `frames=${out.frames.length} replans=${out.replan_count} wall=${out.wall_clock_s}s ` +
      (c ? `attdiff=${c.attitude_abs_diff_deg.toExponential(2)}` : "no-raw-row") +
      ` -> ${path}`,
  );
}
