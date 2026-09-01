import { mkdtempSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { ReorderPointAgent } from "./agent";
import { InventoryEnvironment } from "./environment";
import { writeInventoryArtifacts } from "./recorder";
import { defaultInventoryConfig, makeScenario } from "./scenario";
import { InventoryScorer } from "./scorer";

interface T {
  name: string;
  pass: boolean;
  detail: string;
}

function check(name: string, pass: boolean, detail: string, out: T[]): void {
  out.push({ name, pass, detail });
}

export function runInventoryScorerTests(): T[] {
  const out: T[] = [];
  const cfg = defaultInventoryConfig({ horizonDays: 12 });
  const env = new InventoryEnvironment(cfg);
  const agent = new ReorderPointAgent();
  const sc = makeScenario("steady", 2);
  agent.reset(cfg);
  let obs = env.reset(sc);
  for (;;) {
    const a = agent.step(obs);
    const r = env.step(a);
    obs = r.observation;
    if (r.terminated) break;
  }
  const dir = mkdtempSync(join(tmpdir(), "inv-score-"));
  const art = writeInventoryArtifacts(dir, env.log, env.events, sc);
  const report = new InventoryScorer().score(art.trajectoryPath, art.eventsPath);
  check("scorer_gates_present", "fill_rate" in report.gates && "ending_cash" in report.gates, Object.keys(report.gates).join(","), out);
  check("scorer_pass_boolean", typeof report.pass === "boolean", String(report.pass), out);
  check("scorer_no_env_arg", new InventoryScorer().score.length >= 2, "file paths only", out);
  check("log_nonempty", env.log.length === 12, `n=${env.log.length}`, out);
  return out;
}
