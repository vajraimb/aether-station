import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { AgentController } from "../controller";
import { defaultPublicConfig, Q0, W0 } from "../constants";
import { generateScenario } from "../scenario";
import { Simulator } from "../simulator";
import { OBSERVATION_KEYS, type Observation } from "../types";
import { BaselineController } from "./baseline";
import { createFlightController, createPlantController } from "./factory";
import { controlFromCommand, commandFromControl } from "./convert";

interface T {
  name: string;
  pass: boolean;
  detail: string;
}

const here = dirname(fileURLToPath(import.meta.url));

function check(name: string, pass: boolean, detail: string, out: T[]): void {
  out.push({ name, pass, detail });
}

function sampleObs(t = 0.4): Observation {
  return {
    timestamp: t,
    quaternionMeasured: Q0,
    gyroMeasured: W0,
    sliderPosition: 0.2,
    sliderVelocity: -0.05,
    tankWallPressure1: 2500,
    tankWallPressure2: 2500,
    remainingFuelEstimate: 5,
    thrusterCurrentFeedback: [0, 0, 0, 0, 0, 0],
    actuatorResponseAbnormal: false,
  };
}

export function runControllerProtocolTests(): T[] {
  const out: T[] = [];
  const plant = defaultPublicConfig({ duration: 2.5, seed: 800000 });

  {
    const flight = createFlightController(plant, { mode: "baseline" });
    check("test_controller_factory", flight.diagnostics().mode === "baseline", `mode=${flight.diagnostics().mode}`, out);
    flight.reset({
      mode: "baseline",
      fuelFloorKg: 2.8,
      planningHorizonS: 8,
      replanPeriodS: 0.5,
      beamWidth: 32,
    });
    const cmd = flight.step(sampleObs());
    check(
      "test_factory_mode_selection",
      Array.isArray(cmd.thrusters) && typeof cmd.sliderForceN === "number",
      `thrusters=${cmd.thrusters.length} slider=${cmd.sliderForceN.toFixed(2)}`,
      out,
    );
    try {
      createFlightController(plant, { mode: "discrete-pulse-v2" });
      check("test_factory_v2_unregistered_or_present", true, "v2 registered", out);
    } catch (e) {
      check(
        "test_factory_v2_unregistered_or_present",
        String(e).includes("not registered"),
        String(e),
        out,
      );
    }
  }

  {
    const cfg = defaultPublicConfig({ duration: 1.6, seed: 800000 });
    const sc = generateScenario(800000, false);
    const a = new Simulator(cfg, sc, new AgentController(cfg));
    const b = new Simulator(cfg, sc, new BaselineController(cfg).asPlant());
    a.runAll();
    b.runAll();
    const dq = Math.hypot(
      a.state.q[1] - b.state.q[1],
      a.state.q[2] - b.state.q[2],
      a.state.q[3] - b.state.q[3],
    );
    const dw = Math.hypot(
      a.state.w[0] - b.state.w[0],
      a.state.w[1] - b.state.w[1],
      a.state.w[2] - b.state.w[2],
    );
    const dfuel = Math.abs(a.state.fuel - b.state.fuel);
    const sameEvents = a.events.length === b.events.length && a.events.every((e, i) => e.type === b.events[i]?.type && Math.abs(e.t - (b.events[i]?.t ?? 0)) < 1e-12);
    check(
      "test_baseline_adapter_parity",
      dq < 1e-15 && dw < 1e-15 && dfuel < 1e-15 && sameEvents,
      `Δq=${dq.toExponential(2)} Δw=${dw.toExponential(2)} Δfuel=${dfuel.toExponential(2)} events=${a.events.length}/${b.events.length}`,
      out,
    );
  }

  {
    const flight = createFlightController(plant, { mode: "baseline" });
    check("test_observation_only_boundary", flight.step.length === 1, `step.length=${flight.step.length}`, out);
    check("test_controller_no_simulator_field", !("sim" in flight) && !("scenario" in flight), "no sim/scenario", out);
    const allowed = new Set<string>(OBSERVATION_KEYS);
    const accessed = new Set<string>();
    const raw = sampleObs(1.2);
    const proxy = new Proxy(raw, {
      get(t, prop) {
        const key = String(prop);
        accessed.add(key);
        if (!allowed.has(key) && key !== "then") {
          throw new Error(`controller read forbidden obs field ${key}`);
        }
        return t[prop as keyof Observation];
      },
    });
    try {
      flight.step(proxy);
      const extra = [...accessed].filter((k) => !allowed.has(k) && k !== "then");
      check("test_observation_proxy_no_truth", extra.length === 0, `accessed=${[...accessed].join(",")}`, out);
    } catch (e) {
      check("test_observation_proxy_no_truth", false, String(e), out);
    }
  }

  {
    const d = createFlightController(plant, { mode: "baseline" }).diagnostics();
    const blob = JSON.stringify(d);
    check(
      "test_diagnostics_truth_exclusion",
      !blob.includes("faultTime") && !blob.includes("c1") && !("truth" in d) && !("scenario" in d),
      blob.slice(0, 180),
      out,
    );
  }

  {
    const agent = new AgentController(plant);
    const cmd = agent.step(sampleObs(0.3));
    const round = commandFromControl(controlFromCommand(cmd));
    check(
      "test_command_roundtrip",
      cmd.sliderForce === round.sliderForce && cmd.pulseWidth.every((w: number, i: number) => w === round.pulseWidth[i]),
      `pulse=${cmd.pulseWidth.join(",")}`,
      out,
    );
  }

  {
    const src = readFileSync(join(here, "baseline.ts"), "utf8") + readFileSync(join(here, "factory.ts"), "utf8");
    const banned = [];
    if (/\b73\.4\b/.test(src)) banned.push("73.4");
    if (/\bFAULT_TIME\b/.test(src)) banned.push("FAULT_TIME");
    if (/\bFAULT_THRUSTER\b/.test(src)) banned.push("FAULT_THRUSTER");
    if (/\bPrivateScenario\b/.test(src)) banned.push("PrivateScenario");
    if (/\bSimulator\b/.test(src) && /from ["']\.\.\/simulator["']/.test(src)) banned.push("Simulator import");
    check("test_no_truth_imports", banned.length === 0, banned.length ? banned.join(",") : "clean", out);
    check("test_no_seed_branches", !/800000|20260831/.test(src), "no seed literals in factory/baseline", out);
  }

  {
    const plantCtrl = createPlantController(plant, { mode: "baseline" });
    check("test_plant_controller_name", typeof plantCtrl.step === "function", plantCtrl.name, out);
  }

  return out;
}
