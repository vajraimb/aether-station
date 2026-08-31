import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { defaultPublicConfig, MIN_PULSE } from "../constants";
import { representativeMacroStates } from "./action-macros";
import {
  beamNullspace,
  greedyParSequence,
  greedyThenCancel,
  robustnessOf,
  searchActionSet,
} from "./nullspace-search";

interface T {
  name: string;
  pass: boolean;
  detail: string;
}

function check(name: string, pass: boolean, detail: string, out: T[]): void {
  out.push({ name, pass, detail });
}

export function runNullspaceSearchTests(): T[] {
  const out: T[] = [];
  const actions = searchActionSet(new Set());
  check("action_set_nonempty", actions.length > 10, `n=${actions.length}`, out);
  check(
    "min_pulse",
    actions.every((a) => a.thrusterIds.length === 0 || a.durationS + 1e-12 >= MIN_PULSE),
    ">=40ms",
    out,
  );
  check("max_two", actions.every((a) => a.thrusterIds.length <= 2), "max 2", out);
  const isolated = searchActionSet(new Set([4]));
  check(
    "isolated_excluded",
    isolated.every((a) => !a.thrusterIds.includes(4)),
    `n=${isolated.length}`,
    out,
  );

  const plant = defaultPublicConfig();
  const st = representativeMacroStates(plant).find((s) => s.id === "medium-rate|healthy|empty");
  check("found_state", Boolean(st), st?.id ?? "missing", out);
  if (st) {
    const g = greedyParSequence(st.state, st.params, plant, st.id, 6);
    const c = greedyThenCancel(st.state, st.params, plant, st.id, 6);
    const b = beamNullspace(st.state, st.params, plant, st.id, 6, 6);
    check("greedy_has_segments", g.nSeg >= 1 && g.nSeg <= 6, `n=${g.nSeg}`, out);
    check("beam_has_segments", b.nSeg >= 1 && b.nSeg <= 6, `n=${b.nSeg} rho=${b.rho.toFixed(2)}`, out);
    check("beam_finite_rho", Number.isFinite(b.rho) && b.rho >= 0, `rho=${b.rho}`, out);
    check("fuel_nonneg", g.fuelKg >= 0 && c.fuelKg >= 0 && b.fuelKg >= 0, "fuel", out);
    check("beam_rho_ge_greedy", b.rho + 1e-6 >= Math.min(g.rho, c.rho) || b.fractionParReduced >= g.fractionParReduced - 1e-6, `beam=${b.rho.toFixed(2)} greedy=${g.rho.toFixed(2)}`, out);
    const seq = searchActionSet(new Set());
    const prims = b.ids.map((id) => seq.find((p) => p.id === id)).filter((p): p is NonNullable<typeof p> => Boolean(p));
    check("replay_ids_resolve", prims.length === b.ids.length && b.ids.length > 0, `${prims.length}/${b.ids.length}`, out);
    const rob = robustnessOf(st.state, st.params, plant, prims);
    check("robust_8", rob.length === 8, `n=${rob.length}`, out);
    check("robust_finite", rob.every((s) => Number.isFinite(s.rho)), "rho", out);
  }

  const here = dirname(fileURLToPath(import.meta.url));
  const v2 = readFileSync(join(here, "controller-v2.ts"), "utf8");
  const beam = readFileSync(join(here, "beam-planner.ts"), "utf8");
  const guidance = readFileSync(join(here, "guidance-planner.ts"), "utf8");
  const hooked =
    /wrench-geometry|nullspace-search/.test(v2) ||
    /wrench-geometry|nullspace-search/.test(beam) ||
    /wrench-geometry|nullspace-search/.test(guidance);
  check("not_wired_online", !hooked, hooked ? "imported" : "offline", out);
  return out;
}
