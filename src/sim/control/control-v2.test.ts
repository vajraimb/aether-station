import { readFileSync, readdirSync, statSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { defaultPublicConfig } from "../constants";
import { createFlightController } from "./factory";
import { runBeamPlannerTests } from "./beam-planner.test";
import { runRolloutTests } from "./rollout-model.test";
import { runRolloutErrorTests } from "./rollout-error.test";
import { runTerminalReachableTests } from "./terminal-reachable.test";
import { runHierarchicalTests } from "./hierarchical.test";
import { runCaptureReachabilityTests } from "./capture-reachability.test";
import { runCaptureValueTests } from "./capture-value.test";
import { runBeliefMismatchTests } from "./belief-mismatch.test";
import { runActionMacroTests } from "./action-macros.test";
import { runWrenchGeometryTests } from "./wrench-geometry.test";
import { runNullspaceSearchTests } from "./nullspace-search.test";
import { runRobustTerminalTests } from "./robust-terminal.test";

interface T {
  name: string;
  pass: boolean;
  detail: string;
}

function check(name: string, pass: boolean, detail: string, out: T[]): void {
  out.push({ name, pass, detail });
}

function walk(dir: string, acc: string[] = []): string[] {
  for (const name of readdirSync(dir)) {
    const p = join(dir, name);
    const st = statSync(p);
    if (st.isDirectory()) walk(p, acc);
    else if (name.endsWith(".ts") && !name.endsWith(".test.ts")) acc.push(p);
  }
  return acc;
}

export function runControlV2Tests(): T[] {
  const out: T[] = [];
  out.push(...runRolloutTests());
  out.push(...runRolloutErrorTests());
  out.push(...runTerminalReachableTests());
  out.push(...runHierarchicalTests());
  out.push(...runBeamPlannerTests());
  out.push(...runCaptureReachabilityTests());
  out.push(...runCaptureValueTests());
  out.push(...runBeliefMismatchTests());
  out.push(...runActionMacroTests());
  out.push(...runWrenchGeometryTests());
  out.push(...runNullspaceSearchTests());
  out.push(...runRobustTerminalTests());

  const plant = defaultPublicConfig({ duration: 0.8 });
  const v2 = createFlightController(plant, { mode: "discrete-pulse-v2" });
  check("test_factory_v2_mode", v2.diagnostics().mode === "discrete-pulse-v2", v2.diagnostics().mode, out);
  const cmd = v2.step({
    timestamp: 0.1,
    quaternionMeasured: [1, 0, 0, 0],
    gyroMeasured: [0, 0, 0],
    sliderPosition: 0,
    sliderVelocity: 0,
    tankWallPressure1: 2500,
    tankWallPressure2: 2500,
    remainingFuelEstimate: 5,
    thrusterCurrentFeedback: [0, 0, 0, 0, 0, 0],
    actuatorResponseAbnormal: false,
  });
  const nOn = cmd.thrusters.length;
  check("test_controller_safe_fallback_step", nOn <= 2, `nOn=${nOn}`, out);

  const here = dirname(fileURLToPath(import.meta.url));
  const files = walk(here);
  const bannedHits: string[] = [];
  for (const f of files) {
    const src = readFileSync(f, "utf8");
    if (/\b73\.4\b/.test(src)) bannedHits.push(`${f}:73.4`);
    if (/\bFAULT_TIME\b/.test(src) || /\bFAULT_THRUSTER\b/.test(src)) bannedHits.push(`${f}:FAULT_*`);
    if (/from ["']\.\.\/simulator["']/.test(src)) bannedHits.push(`${f}:Simulator`);
    if (/PrivateScenario/.test(src) && !f.endsWith("control-v2.test.ts")) bannedHits.push(`${f}:PrivateScenario`);
    if (/800000|20260831/.test(src) && !f.includes(".test.")) bannedHits.push(`${f}:seed`);
  }
  check("test_no_truth_imports", bannedHits.length === 0, bannedHits.length ? bannedHits.join(",") : "clean", out);
  check("test_no_seed_branches", bannedHits.every((h) => !h.endsWith(":seed")), bannedHits.join(",") || "clean", out);

  return out;
}
