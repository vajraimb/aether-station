import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { defaultPublicConfig, ETA_RANGE, MIN_PULSE } from "../constants";
import { deg, vnorm } from "../math3d";
import { geodesicAttitudeError } from "./rollout-model";
import {
  ALL_MASKS,
  ATT_CAP_DEG,
  RATE_CAP,
  caseWithinCaps,
  etaGrid,
  etaInterval,
  evaluateSequence,
  lowRateCases,
  makeTerminalCase,
  robustCancelSearch,
  terminalActionSet,
  TERMINAL_FAMILIES,
} from "./robust-terminal";
import { maxProjectionSingle } from "./action-macros";

interface T {
  name: string;
  pass: boolean;
  detail: string;
}

function check(name: string, pass: boolean, detail: string, out: T[]): void {
  out.push({ name, pass, detail });
}

export function runRobustTerminalTests(): T[] {
  const out: T[] = [];
  const iv = etaInterval();
  check("eta_clipped_lo", iv.min >= ETA_RANGE[0] - 1e-12, `min=${iv.min}`, out);
  check("eta_clipped_hi", iv.max <= ETA_RANGE[1] + 1e-12, `max=${iv.max}`, out);
  check("eta_hat_inside", iv.min - 1e-12 <= iv.hat && iv.hat <= iv.max + 1e-12, `hat=${iv.hat}`, out);
  check("eta_not_fixed_pm20", Math.abs(iv.min - 0.8 * iv.hat) > 1e-6 || Math.abs(iv.max - 1.2 * iv.hat) > 1e-6, ` [${iv.min},${iv.max}]`, out);
  const g = etaGrid(iv, 5);
  check("grid_covers_hat", g.some((x) => Math.abs(x - iv.hat) < 1e-9), g.join(","), out);
  check("grid_endpoints", Math.abs(g[0]! - iv.min) < 1e-12 && Math.abs(g[g.length - 1]! - iv.max) < 1e-12, `${g[0]}..${g[g.length - 1]}`, out);

  const plant = defaultPublicConfig();
  const cases = lowRateCases(plant, { quick: true });
  check("quick_cases_n", cases.length === 6, `n=${cases.length}`, out);
  const allOk = cases.every((c) => caseWithinCaps(c, plant));
  check("quick_within_caps", allOk, allOk ? "ok" : "cap fail", out);

  const fullCaps = TERMINAL_FAMILIES.every((f) => f.attDeg <= ATT_CAP_DEG + 1e-9 && f.wmag <= RATE_CAP + 1e-12);
  check("families_in_spec", fullCaps, `nFam=${TERMINAL_FAMILIES.length}`, out);
  check("seven_masks", ALL_MASKS.length === 7, `n=${ALL_MASKS.length}`, out);

  const actions = terminalActionSet(new Set());
  check("action_set_nonempty", actions.length > 15, `n=${actions.length}`, out);
  check(
    "action_has_160ms",
    actions.some((a) => a.thrusterIds.length === 1 && Math.abs(a.durationS - 0.16) < 1e-12),
    "160ms single",
    out,
  );
  check("action_max_two", actions.every((a) => a.thrusterIds.length <= 2), "max 2", out);

  const c = makeTerminalCase(plant, TERMINAL_FAMILIES[0]!, []);
  const att = deg(geodesicAttitudeError(c.state.qBI, plant.qTarget));
  const w = vnorm(c.state.omegaB);
  check("near_close_att", Math.abs(att - 2.5) < 0.05, `att=${att.toFixed(3)}`, out);
  check("near_close_rate", Math.abs(w - 0.006) < 1e-6, `w=${w}`, out);
  check("fuel_above_floor", c.state.fuelMass > 2.8, `fuel=${c.state.fuelMass}`, out);

  const iso = makeTerminalCase(plant, TERMINAL_FAMILIES[1]!, [2]);
  check("mask_recorded", iso.isolated.length === 1 && iso.isolated[0] === 2, iso.id, out);

  const single = maxProjectionSingle(c.state, c.params, plant);
  check("single_min_pulse", single.durationS + 1e-12 >= MIN_PULSE, `dt=${single.durationS}`, out);
  const ev = evaluateSequence("single-pulse", c.state, c.params, plant, [single], etaGrid(iv, 3), iv.hat);
  check("eval_grid3", ev.grid.length === 3, `n=${ev.grid.length}`, out);
  check("eval_finite_J", Number.isFinite(ev.nominal.J) && Number.isFinite(ev.worst.J), `J=${ev.nominal.J}`, out);
  check("eval_fuel_ok", ev.grid.every((s) => s.fuelOk), "fuel", out);
  check("worst_ge_nom", ev.worst.J + 1e-9 >= ev.nominal.J, `w=${ev.worst.J.toFixed(3)} n=${ev.nominal.J.toFixed(3)}`, out);

  const seq = robustCancelSearch(c.state, c.params, plant, iv, 6, 5);
  check("search_returns_seq", seq.length >= 1 && seq.length <= 6, `n=${seq.length}`, out);
  check(
    "search_legal_jets",
    seq.every((p) => p.thrusterIds.length <= 2 && (p.thrusterIds.length === 0 || p.durationS + 1e-12 >= MIN_PULSE)),
    "jets",
    out,
  );

  const here = dirname(fileURLToPath(import.meta.url));
  const v2 = readFileSync(join(here, "controller-v2.ts"), "utf8");
  const beam = readFileSync(join(here, "beam-planner.ts"), "utf8");
  const guidance = readFileSync(join(here, "guidance-planner.ts"), "utf8");
  const hooked =
    /robust-terminal/.test(v2) || /robust-terminal/.test(beam) || /robust-terminal/.test(guidance);
  check("not_wired_online", !hooked, hooked ? "imported" : "offline", out);
  return out;
}
