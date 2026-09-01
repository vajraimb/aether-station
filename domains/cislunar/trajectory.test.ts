import { LEO_ALT, LLO_ALT, LMO_ALT, R_EARTH, R_MOON, A_MOON, A_MARS, isHelioPhase } from "./constants";
import { hypot3 } from "./kepler";
import { buildCislunarMission, sampleAt } from "./trajectory";

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
  const rev = m.samples.filter((s) => s.phase === "revolution");
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
  check("cislunar_time_mono", last.t > first.t && last.t > 900 * 86400, `T=${(last.t / 86400).toFixed(2)} d`, out);
  check(
    "cislunar_no_earth_impact",
    m.samples.filter((s) => !isHelioPhase(s.phase)).every((s) => hypot3(s.r[0], s.r[1], s.r[2]) > R_EARTH + 50),
    "clear",
    out,
  );
  check(
    "cislunar_no_moon_impact",
    [...llo, ...rev].every((s) => hypot3(s.r[0] - s.moon[0], s.r[1] - s.moon[1], s.r[2] - s.moon[2]) > R_MOON + 20),
    "clear",
    out,
  );
  check("cislunar_dv_tli", m.dvTli > 2.8 && m.dvTli < 3.6, `dv=${m.dvTli.toFixed(2)}`, out);
  check("cislunar_has_revolution", rev.length > 100, `n=${rev.length}`, out);
  check(
    "cislunar_moon_sidereal",
    Math.abs(m.periodMoon - 27.3 * 86400) < 0.5 * 86400,
    `Tm=${(m.periodMoon / 86400).toFixed(2)} d`,
    out,
  );
  const cap = sampleAt(m, m.tCapture);
  const moonEnd = sampleAt(m, m.tTmi);
  const moonBack = hypot3(moonEnd.moon[0] - cap.moon[0], moonEnd.moon[1] - cap.moon[1], moonEnd.moon[2] - cap.moon[2]);
  check("cislunar_moon_full_lap", moonBack < 2500, `dMoon=${moonBack.toFixed(0)} km`, out);
  const lloRevs = (m.tTmi - m.tCapture) / m.periodLlo;
  check("cislunar_llo_many_revs", lloRevs > 300, `revs=${lloRevs.toFixed(0)}`, out);
  const midRev = sampleAt(m, m.tCapture + 0.5 * m.periodMoon);
  const moonR = hypot3(midRev.moon[0], midRev.moon[1], midRev.moon[2]);
  check("cislunar_moon_rail", Math.abs(moonR - A_MOON) < 50, `r=${moonR.toFixed(0)}`, out);
  const midTh = Math.atan2(midRev.moon[2], midRev.moon[0]);
  const capTh = Math.atan2(cap.moon[2], cap.moon[0]);
  let dth = ((midTh - capTh) * 180) / Math.PI;
  if (dth < 0) dth += 360;
  check("cislunar_moon_halfway", dth > 140 && dth < 220, `dth=${dth.toFixed(1)} deg`, out);
  const helio = m.samples.filter((s) => s.phase === "heliocoast");
  const lmo = m.samples.filter((s) => s.phase === "lmo");
  const marsrev = m.samples.filter((s) => s.phase === "marsrev");
  check("cislunar_has_helio", helio.length > 100, `n=${helio.length}`, out);
  const farH = helio.reduce((best, s) => (s.altMars < best.altMars ? s : best), helio[0]!);
  check("cislunar_reaches_mars", farH.altMars < 4e5, `altMars=${farH.altMars.toFixed(0)}`, out);
  const arr = sampleAt(m, m.tMoi);
  const arrR = hypot3(arr.r[0], arr.r[1], arr.r[2]);
  check("cislunar_arrive_mars_au", Math.abs(arrR - A_MARS) < 5e4, `r=${arrR.toFixed(0)}`, out);
  const lmoAlt = lmo.reduce((s, x) => s + x.altMars, 0) / lmo.length;
  check("cislunar_lmo_alt", Math.abs(lmoAlt - LMO_ALT) < 80, `alt=${lmoAlt.toFixed(1)}`, out);
  check("cislunar_dv_tmi", m.dvTmi > 2.4 && m.dvTmi < 3.6, `dv=${m.dvTmi.toFixed(2)}`, out);
  check("cislunar_has_marsrev", marsrev.length > 100, `n=${marsrev.length}`, out);
  const moi = sampleAt(m, m.tMoi);
  const marsEnd = sampleAt(m, m.duration);
  const marsBack = hypot3(marsEnd.mars[0] - moi.mars[0], marsEnd.mars[1] - moi.mars[1], marsEnd.mars[2] - moi.mars[2]);
  check("cislunar_mars_full_lap", marsBack < 8e5, `dMars=${marsBack.toFixed(0)} km`, out);
  check("cislunar_tof_helio", m.tofHelio > 240 * 86400 && m.tofHelio < 280 * 86400, `tof=${(m.tofHelio / 86400).toFixed(1)} d`, out);
  return out;
}
