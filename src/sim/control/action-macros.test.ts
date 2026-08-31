import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { defaultPublicConfig, MIN_PULSE, THRUSTERS } from "../constants";
import { vnorm } from "../math3d";
import {
  applyMacro,
  generateActionMacros,
  macroIsLegal,
  maxProjectionSingle,
  paretoFront,
  representativeMacroStates,
  evaluateMacroOnState,
  MACRO_FUEL_FLOOR,
} from "./action-macros";
import { publicBelief } from "./rollout-error";
import { isLegalPulsePrimitive } from "./discrete-actions";

interface T {
  name: string;
  pass: boolean;
  detail: string;
}

function check(name: string, pass: boolean, detail: string, out: T[]): void {
  out.push({ name, pass, detail });
}

export function runActionMacroTests(): T[] {
  const out: T[] = [];
  const macros = generateActionMacros();
  check("macro_count_nonzero", macros.length > 50, `n=${macros.length}`, out);
  check(
    "macro_segment_bounds",
    macros.every((m) => m.segments.length >= 2 && m.segments.length <= 4),
    "2-4 segments",
    out,
  );
  check(
    "macro_min_pulse",
    macros.every((m) => m.segments.every((s) => s.thrusterIds.length === 0 || s.durationS + 1e-12 >= MIN_PULSE)),
    ">= 40 ms",
    out,
  );
  check(
    "macro_max_two_jets",
    macros.every((m) => m.segments.every((s) => s.thrusterIds.length <= 2)),
    "max 2 jets / segment",
    out,
  );
  check(
    "macro_primitives_legal",
    macros.every((m) => m.segments.every((s) => isLegalPulsePrimitive(s, THRUSTERS))),
    "legal primitives",
    out,
  );
  const ids = macros.map((m) => m.id);
  check("macro_ids_unique", new Set(ids).size === ids.length, "unique ids", out);

  const isolated = new Set([2]);
  const blocked = macros.filter((m) => m.jetSet.includes(2));
  check(
    "isolated_rejected",
    blocked.every((m) => !macroIsLegal(m, isolated, 4.5).legal),
    `blocked=${blocked.length}`,
    out,
  );

  const plant = defaultPublicConfig();
  const states = representativeMacroStates(plant);
  check("rep_states", states.length >= 8, `n=${states.length}`, out);
  const st = states[0]!;
  const coasty = macros.find((m) => m.template === "pulse-coast" && m.propellantKg > 0);
  check("found_pulse_coast", Boolean(coasty), coasty?.id ?? "missing", out);
  if (coasty) {
    const dry = macroIsLegal(coasty, new Set(), MACRO_FUEL_FLOOR);
    check("fuel_floor_blocks", dry.legal === false, dry.reason, out);
    const after = applyMacro(st.state, st.params, plant, coasty);
    check(
      "apply_consumes_fuel_or_coasts",
      after.fuelMass <= st.state.fuelMass + 1e-9,
      `fuel ${st.state.fuelMass} -> ${after.fuelMass}`,
      out,
    );
    check("apply_advances_time", after.time > st.state.time + 0.1, `dt=${after.time - st.state.time}`, out);
  }

  const single = maxProjectionSingle(st.state, st.params, plant);
  check("max_projection_single", single.thrusterIds.length <= 1, single.id, out);

  const subset = macros.filter((m) => m.template === "pulse-coast").slice(0, 24);
  const evals = subset.map((m) => evaluateMacroOnState(m, st.state, publicBelief(plant), plant, st.id));
  const front = paretoFront(evals);
  check("pareto_nonempty", front.length > 0 && front.every((e) => e.legal), `front=${front.length}`, out);
  check("omega_finite", evals.every((e) => Number.isFinite(e.dOmega) && Number.isFinite(e.dtHPerp)), "finite", out);

  const here = dirname(fileURLToPath(import.meta.url));
  const v2 = readFileSync(join(here, "controller-v2.ts"), "utf8");
  const beam = readFileSync(join(here, "beam-planner.ts"), "utf8");
  const guidance = readFileSync(join(here, "guidance-planner.ts"), "utf8");
  const hooked = /action-macros/.test(v2) || /action-macros/.test(beam) || /action-macros/.test(guidance);
  check("macros_not_wired_online", !hooked, hooked ? "imported by planner" : "offline only", out);

  void vnorm;
  return out;
}
