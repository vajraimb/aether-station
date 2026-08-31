import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { ATT_GATE_DEG, RATE_GATE, createFlightController, scoreFromFiles } from "./arena";
import { ETA_RANGE, OBSERVATION_KEYS, defaultPublicConfig } from "./core";

interface T {
  name: string;
  pass: boolean;
  detail: string;
}

function check(name: string, pass: boolean, detail: string, out: T[]): void {
  out.push({ name, pass, detail });
}

export function runBenchmarkContractTests(): T[] {
  const out: T[] = [];
  const plant = defaultPublicConfig({ duration: 0.4 });
  check("core_eta_range", ETA_RANGE[0] < ETA_RANGE[1], `${ETA_RANGE[0]}..${ETA_RANGE[1]}`, out);
  check("arena_gates", ATT_GATE_DEG === 1 && RATE_GATE === 0.008, `att=${ATT_GATE_DEG} w=${RATE_GATE}`, out);
  check("observation_keys", OBSERVATION_KEYS.length === 10, `n=${OBSERVATION_KEYS.length}`, out);

  const obs = {
    timestamp: 0.2,
    quaternionMeasured: plant.qTarget,
    gyroMeasured: [0, 0, 0] as [number, number, number],
    sliderPosition: 0,
    sliderVelocity: 0,
    tankWallPressure1: 2500,
    tankWallPressure2: 2500,
    remainingFuelEstimate: 4.5,
    thrusterCurrentFeedback: [0, 0, 0, 0, 0, 0] as [number, number, number, number, number, number],
    actuatorResponseAbnormal: false,
  };
  for (const mode of ["baseline", "discrete-pulse-v2"] as const) {
    const c = createFlightController(plant, { mode });
    const cmd = c.step(obs);
    check(
      `arena_step_${mode}`,
      cmd.thrusters.length <= 2 && Number.isFinite(cmd.sliderForceN),
      `nOn=${cmd.thrusters.length}`,
      out,
    );
  }
  check("arena_score_fn", typeof scoreFromFiles === "function", "scoreFromFiles", out);

  const here = dirname(fileURLToPath(import.meta.url));
  const arenaSrc = readFileSync(join(here, "arena.ts"), "utf8");
  const coreSrc = readFileSync(join(here, "core.ts"), "utf8");
  const importsOf = (src: string) =>
    [...src.matchAll(/^import[\s\S]*?from\s+["']([^"']+)["']/gm)].map((m) => m[1]!);
  const bannedMod = (id: string) =>
    /simulator|robust-terminal|nullspace-search|action-macros|capture-value/.test(id);
  check("arena_no_research_hooks", !importsOf(arenaSrc).some(bannedMod), importsOf(arenaSrc).join(","), out);
  check("core_no_research_hooks", !importsOf(coreSrc).some(bannedMod), importsOf(coreSrc).join(","), out);
  check("arena_no_simulator_export", !importsOf(arenaSrc).some((id) => /simulator/.test(id)), "no Simulator", out);
  return out;
}
