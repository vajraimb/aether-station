#!/usr/bin/env npx tsx
/**
 * Multi-scenario evaluation. Gains are not re-fit per seed.
 *   npm run benchmark -- --count 20
 */
import { defaultPublicConfig } from "../constants.ts";
import { generateScenario } from "../scenario.ts";
import { Simulator } from "../simulator.ts";
import { writeJson } from "../io.ts";
import type { Metrics } from "../types.ts";

function arg(flag: string, fallback?: string): string {
  const i = process.argv.indexOf(flag);
  if (i >= 0 && i + 1 < process.argv.length) return process.argv[i + 1]!;
  return fallback ?? "";
}

const count = Math.max(1, Number(arg("--count", "20")));
const outPath = arg("--out", "outputs/benchmark.json");

const keys = [
  "final_attitude_error_deg",
  "final_angular_speed_rad_s",
  "max_slider_impact_speed_m_s",
  "final_slosh_energy_ratio",
  "remaining_fuel_kg",
  "parameter_relative_error",
  "fault_detection_delay_s",
  "isolationDelay",
  "quaternion_norm_max_error",
] as const;

type Key = (typeof keys)[number];

function num(m: Metrics, k: Key): number | null {
  const v = m[k];
  return typeof v === "number" && Number.isFinite(v) ? v : null;
}

function quantile(xs: number[], p: number): number {
  if (xs.length === 0) return Number.NaN;
  const s = [...xs].sort((a, b) => a - b);
  const i = (s.length - 1) * p;
  const lo = Math.floor(i);
  const hi = Math.ceil(i);
  if (lo === hi) return s[lo]!;
  return s[lo]! * (hi - i) + s[hi]! * (i - lo);
}

const rows: Array<Record<string, unknown>> = [];
const buckets: Record<Key, number[]> = {
  final_attitude_error_deg: [],
  final_angular_speed_rad_s: [],
  max_slider_impact_speed_m_s: [],
  final_slosh_energy_ratio: [],
  remaining_fuel_kg: [],
  parameter_relative_error: [],
  fault_detection_delay_s: [],
  isolationDelay: [],
  quaternion_norm_max_error: [],
};

let attPass = 0;
let allPass = 0;
const t0 = Date.now();

for (let i = 0; i < count; i++) {
  const seed = 710_000 + i * 97;
  const cfg = defaultPublicConfig({ seed, fluidPresent: true });
  const sc = generateScenario(seed, false);
  const sim = new Simulator(cfg, sc);
  sim.runAll();
  const m = sim.metrics();
  const attOk = m.final_attitude_error_deg < 1;
  const gatesOk = Object.values(m.scorecard).every((g) => g.pass);
  if (attOk) attPass += 1;
  if (gatesOk) allPass += 1;
  for (const k of keys) {
    const v = num(m, k);
    if (v !== null) buckets[k].push(v);
  }
  rows.push({
    seed,
    faultTime: sc.faultTime,
    faultThruster: sc.faultThruster,
    c1: sc.c1,
    c2: sc.c2,
    k12: sc.k12,
    etaT: sc.etaT,
    att: m.final_attitude_error_deg,
    w: m.final_angular_speed_rad_s,
    fuel: m.remaining_fuel_kg,
    isolationDelay: m.isolationDelay,
    isolated: m.isolatedThrusterId,
    isolationOk: m.fault_isolation_accuracy,
    scorecardPass: gatesOk,
  });
  console.error(`benchmark ${i + 1}/${count} seed=${seed} att=${m.final_attitude_error_deg.toFixed(3)} isoΔ=${m.isolationDelay?.toFixed(3)} thr=${sc.faultThruster}→${m.isolatedThrusterId}`);
}

const stats: Record<string, unknown> = {};
for (const k of keys) {
  const xs = buckets[k];
  const loBetter = k !== "remaining_fuel_kg";
  stats[k] = {
    n: xs.length,
    median: quantile(xs, 0.5),
    p90: quantile(xs, loBetter ? 0.9 : 0.1),
    worst: xs.length ? (loBetter ? Math.max(...xs) : Math.min(...xs)) : null,
    successRate:
      k === "final_attitude_error_deg"
        ? attPass / count
        : k === "remaining_fuel_kg"
          ? xs.filter((x) => x > 2.8).length / count
          : k === "final_angular_speed_rad_s"
            ? xs.filter((x) => x < 0.008).length / count
            : k === "fault_detection_delay_s" || k === "isolationDelay"
              ? xs.filter((x) => x >= 0 && x < 3).length / count
              : k === "max_slider_impact_speed_m_s"
                ? xs.filter((x) => x < 0.25).length / count
                : k === "final_slosh_energy_ratio"
                  ? xs.filter((x) => x < 0.08).length / count
                  : k === "parameter_relative_error"
                    ? xs.filter((x) => x < 0.15).length / count
                    : k === "quaternion_norm_max_error"
                      ? xs.filter((x) => x < 1e-6).length / count
                      : null,
  };
}

const report = {
  count,
  elapsed_ms: Date.now() - t0,
  attitudeSuccessRate: attPass / count,
  allGatesSuccessRate: allPass / count,
  stats,
  rows,
};
writeJson(outPath, report);
console.log(JSON.stringify({ count, attitudeSuccessRate: attPass / count, allGatesSuccessRate: allPass / count, stats, elapsed_ms: report.elapsed_ms }, null, 2));
