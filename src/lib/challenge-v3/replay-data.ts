/**
 * Read model for the Challenge V3 3D replay.
 *
 * The trajectories are produced offline by `src/sim/cli/challenge-v3-replay.ts`
 * and committed under `outputs/challenge-v3/replay/`. Nothing here re-runs the
 * planner: one mission costs minutes of CPU (`wall_clock_s` in each file), and
 * a browser-side re-run would be a second, unaudited source of numbers. The
 * viewer replays what was measured, and each file carries the diff against the
 * committed raw row so a determinism failure would be visible rather than
 * quietly absorbed.
 */
import type { SceneSample } from "@/viz/types";

export interface ReplanEvent {
  t: number;
  winner: string;
  segments: number;
  rollouts: number;
  predicted_attitude_deg: number;
  predicted_omega: number;
  plan_duration_s: number;
}

export interface ReplayFile {
  seed: number;
  configuration: string;
  frame_fields: string[];
  frame_interval_s: number;
  thruster_mask_is_window_or: boolean;
  duration_s: number;
  dwell_window_s: number;
  gates: {
    attitudeDeg: number;
    omega: number;
    fuelHard: number;
    sloshRatio: number;
    impactSpeed: number;
  };
  scenario: {
    faultThruster: number;
    faultTime: number;
    c1: number;
    c2: number;
    k12: number;
    etaT: number;
  };
  metrics: {
    final_attitude_error_deg: number;
    final_angular_speed_rad_s: number;
    remaining_fuel_kg: number;
    final_slosh_energy_ratio: number;
    max_slider_impact_speed_m_s: number;
    quaternion_norm_max_error: number;
    pulse_count: number;
    passed: boolean;
    gate: Record<string, boolean>;
  };
  replan_count: number;
  replans: ReplanEvent[];
  determinism_check: {
    raw_row_file: string;
    attitude_abs_diff_deg: number;
    omega_abs_diff: number;
    fuel_abs_diff_kg: number;
    replans_diff: number;
    pass_matches: boolean;
  } | null;
  wall_clock_s: number;
  commit: string;
  frames: number[][];
}

/** Lazy per-seed chunks: a trajectory is only fetched when its seed is opened. */
const loaders = import.meta.glob<{ default: ReplayFile }>(
  "../../../outputs/challenge-v3/replay/seed-*.json",
);

function seedOfPath(path: string): number {
  const m = /seed-(\d+)\.json$/.exec(path);
  return m ? Number(m[1]) : NaN;
}

/** Seeds that actually have a committed trajectory on disk, ascending. */
export const replaySeeds: number[] = Object.keys(loaders)
  .map(seedOfPath)
  .filter((s) => Number.isFinite(s))
  .sort((a, b) => a - b);

export async function loadReplay(seed: number): Promise<ReplayFile> {
  const entry = Object.entries(loaders).find(([p]) => seedOfPath(p) === seed);
  if (!entry) throw new Error(`no committed replay for seed ${seed}`);
  const mod = await entry[1]();
  return mod.default;
}

/** Decoded frame: the scene fields plus the read-outs the HUD shows. */
export interface Frame extends SceneSample {
  attitudeErrorDeg: number;
  fuelTrue: number;
  /** Bitmask; bit i = nozzle i fired somewhere inside this frame's window. */
  thrusterMask: number;
}

export function decodeFrame(f: number[]): Frame {
  const mask = f[16] ?? 0;
  const actual: number[] = [];
  for (let i = 0; i < 6; i++) actual.push((mask >> i) & 1 ? 1 : 0);
  return {
    t: f[0] ?? 0,
    r: [f[1] ?? 0, f[2] ?? 0, f[3] ?? 0],
    q: [f[4] ?? 1, f[5] ?? 0, f[6] ?? 0, f[7] ?? 0],
    w: [f[8] ?? 0, f[9] ?? 0, f[10] ?? 0],
    s: f[11] ?? 0,
    th1: f[12] ?? 0,
    th2: f[13] ?? 0,
    attitudeErrorDeg: f[14] ?? 0,
    fuelTrue: f[15] ?? 0,
    thrusterMask: mask,
    thrusterActual: actual as unknown as SceneSample["thrusterActual"],
    detectedFailedThruster: f[17] ?? -1,
  };
}

export function omegaNorm(w: readonly number[]): number {
  return Math.hypot(w[0] ?? 0, w[1] ?? 0, w[2] ?? 0);
}
