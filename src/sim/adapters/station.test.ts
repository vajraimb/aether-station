import { mkdtempSync, readFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { runArenaLeakTests } from "../../../packages/agent-arena/src/leak.test.ts";
import { scoreEpisode } from "../../../packages/agent-arena/src/runner.ts";
import { defaultPublicConfig } from "../constants.ts";
import { createPlantController } from "../control/factory.ts";
import { generateScenario } from "../scenario.ts";
import { parseEventsJsonl, parseTrajectoryCsv } from "../scoring.ts";
import { Simulator } from "../simulator.ts";
import { SpaceStationAgent, SpaceStationDriver, SpaceStationScorer, stationScenario } from "./station.ts";

interface T {
  name: string;
  pass: boolean;
  detail: string;
}

function check(name: string, pass: boolean, detail: string, out: T[]): void {
  out.push({ name, pass, detail });
}

export function runStationAdapterTests(): T[] {
  const out: T[] = [];
  out.push(...runArenaLeakTests());

  const plant = defaultPublicConfig({ duration: 0.5, seed: 800_000 });
  const sc = generateScenario(800_000, false);
  const raw = new Simulator(plant, sc, createPlantController(plant, { mode: "baseline" }));
  raw.runAll();

  const dir = mkdtempSync(join(tmpdir(), "aether-arena-"));
  const agent = new SpaceStationAgent(plant, { mode: "baseline" });
  agent.reset({
    mode: "baseline",
    fuelFloorKg: 2.8,
    planningHorizonS: 8,
    replanPeriodS: 0.5,
    beamWidth: 28,
  });
  const driver = new SpaceStationDriver(plant, dir);
  const art = driver.run(agent, { mode: "baseline" }, stationScenario(800_000));
  const log = parseTrajectoryCsv(readFileSync(art.trajectoryPath, "utf8"));
  const events = parseEventsJsonl(readFileSync(art.eventsPath, "utf8"));

  check("adapter_log_len", log.length === raw.log.length, `csv=${log.length} raw=${raw.log.length}`, out);
  const n = Math.min(log.length, raw.log.length);
  let maxDq = 0;
  let maxDw = 0;
  let maxDf = 0;
  for (let i = 0; i < n; i++) {
    const a = log[i]!;
    const b = raw.log[i]!;
    maxDq = Math.max(maxDq, Math.abs(a.attitudeErrorDeg - b.attitudeErrorDeg));
    maxDw = Math.max(maxDw, Math.abs(a.w[0] - b.w[0]) + Math.abs(a.w[1] - b.w[1]) + Math.abs(a.w[2] - b.w[2]));
    maxDf = Math.max(maxDf, Math.abs(a.fuelTrue - b.fuelTrue));
  }
  check("adapter_att_fieldwise", maxDq < 1e-6, `maxΔatt=${maxDq}`, out);
  check("adapter_w_fieldwise", maxDw < 1e-6, `maxΔw=${maxDw}`, out);
  check("adapter_fuel_fieldwise", maxDf < 1e-6, `maxΔfuel=${maxDf}`, out);
  check("adapter_events_len", events.length === raw.events.length, `csv=${events.length} raw=${raw.events.length}`, out);

  const report = scoreEpisode(new SpaceStationScorer(), art);
  check("scorer_has_gates", Object.keys(report.gates).length >= 6, `n=${Object.keys(report.gates).length}`, out);
  check("scorer_protocol_pass_boolean", typeof report.pass === "boolean", String(report.pass), out);
  return out;
}
