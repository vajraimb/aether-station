import { LEO_ALT, LLO_ALT, R_EARTH, R_MOON } from "./constants";
import { hypot3 } from "./kepler";
import { buildCislunarMission } from "./trajectory";

interface T {
  name: string;
  pass: boolean;
  detail: string;
}

function check(name: string, pass: boolean, detail: string, out: T[]): void {
  out.push({ name, pass, detail });
}

export function runCislunarTests(): T[] {
  const out: T[] = [];
  const m = buildCislunarMission();
  check("cislunar_samples", m.samples.length > 300, `n=${m.samples.length}`, out);
  const leo = m.samples.filter((s) => s.phase === "leo");
  const coast = m.samples.filter((s) => s.phase === "coast");
  const llo = m.samples.filter((s) => s.phase === "llo");
  const leoAlt = leo.reduce((s, x) => s + x.altEarth, 0) / leo.length;
  check("cislunar_leo_alt", Math.abs(leoAlt - LEO_ALT) < 5, `alt=${leoAlt.toFixed(1)}`, out);
  check("cislunar_leo_speed", leo[5]!.speed > 7.4 && leo[5]!.speed < 8.1, `v=${leo[5]!.speed.toFixed(2)}`, out);
  const far = coast.reduce((best, s) => (s.altEarth > best.altEarth ? s : best), coast[0]!);
  check("cislunar_reaches_moon_distance", far.altEarth > 3e5, `altE=${far.altEarth.toFixed(0)}`, out);
  const lloAlt = llo.reduce((s, x) => s + x.altMoon, 0) / llo.length;
  check("cislunar_llo_alt", Math.abs(lloAlt - LLO_ALT) < 40, `altM=${lloAlt.toFixed(1)}`, out);
  check("cislunar_llo_speed", llo[10]!.speed > 1.4 && llo[10]!.speed < 1.9, `v=${llo[10]!.speed.toFixed(2)}`, out);
  check("cislunar_finite", m.samples.every((s) => Number.isFinite(s.r[0]) && Number.isFinite(s.speed)), "finite", out);
  const first = m.samples[0]!;
  const last = m.samples[m.samples.length - 1]!;
  check("cislunar_time_mono", last.t > first.t && last.t > 2 * 86400, `T=${(last.t / 86400).toFixed(2)} d`, out);
  check(
    "cislunar_no_earth_impact",
    m.samples.every((s) => hypot3(s.r[0], s.r[1], s.r[2]) > R_EARTH + 50),
    "clear",
    out,
  );
  check(
    "cislunar_no_moon_impact",
    llo.every((s) => hypot3(s.r[0] - s.moon[0], s.r[1] - s.moon[1], s.r[2] - s.moon[2]) > R_MOON + 20),
    "clear",
    out,
  );
  check("cislunar_dv_tli", m.dvTli > 2.8 && m.dvTli < 3.6, `dv=${m.dvTli.toFixed(2)}`, out);
  return out;
}
