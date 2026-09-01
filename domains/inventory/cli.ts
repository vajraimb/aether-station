#!/usr/bin/env npx tsx
/**
 * Inventory domain smoke / eval. Platform check, not a solved policy.
 *   npm run inventory:smoke
 */
import { mkdirSync } from "node:fs";
import { join } from "node:path";
import { writeJson } from "../../src/sim/io.ts";
import { runInventoryEpisode } from "./driver.ts";
import { defaultInventoryConfig, makeScenario, SCENARIO_IDS } from "./scenario.ts";
import { InventoryScorer } from "./scorer.ts";

const smoke = process.argv.includes("--smoke") || process.argv.includes("--set") && process.argv.includes("smoke");
const cfg = defaultInventoryConfig({ horizonDays: smoke ? 30 : 60 });
const root = "outputs/inventory";
mkdirSync(root, { recursive: true });
const scorer = new InventoryScorer();
const rows = [];
for (const id of SCENARIO_IDS) {
  const sc = makeScenario(id, 1);
  const dir = join(root, id);
  mkdirSync(dir, { recursive: true });
  const art = runInventoryEpisode(cfg, sc, dir);
  const report = scorer.score(art.trajectoryPath, art.eventsPath);
  rows.push({
    id,
    pass: report.pass,
    fillRate: report.gates.fill_rate?.value ?? null,
    cash: report.gates.ending_cash?.value ?? null,
    stockoutDays: report.gates.stockout_days?.value ?? null,
    violations: report.gates.constraint_violations?.value ?? null,
  });
  console.log(`${report.pass ? "PASS" : "fail"}  ${id}  fill=${report.gates.fill_rate?.value?.toFixed(3)} cash=${report.gates.ending_cash?.value?.toFixed(1)}`);
}
const summary = {
  domain: "inventory",
  agent: "reorder-point",
  horizonDays: cfg.horizonDays,
  allGates: rows.filter((r) => r.pass).length / rows.length,
  rows,
  note: "Baseline is expected to FAIL all-gates. This smoke checks the platform, not policy quality.",
};
writeJson(join(root, "smoke.json"), summary);
console.log(`allGates=${summary.allGates} → ${root}/smoke.json`);
process.exit(0);
