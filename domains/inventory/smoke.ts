import { mkdirSync } from "node:fs";
import { join } from "node:path";
import { writeJson } from "../../src/sim/io.ts";
import { runInventoryEpisode } from "./driver.ts";
import { defaultInventoryConfig, makeScenario, SCENARIO_IDS } from "./scenario.ts";
import { InventoryScorer } from "./scorer.ts";

export const INVENTORY_SMOKE_PATH = "outputs/inventory/smoke.json";
export const INVENTORY_RUN_ID = "inventory-reorder-smoke";

export interface InventorySmokeRow {
  id: string;
  pass: boolean;
  fillRate: number | null;
  cash: number | null;
  stockoutDays: number | null;
  violations: number | null;
}

export function runInventorySmoke(horizonDays = 30): {
  allGates: number;
  rows: InventorySmokeRow[];
  path: string;
} {
  const cfg = defaultInventoryConfig({ horizonDays });
  const root = "outputs/inventory";
  mkdirSync(root, { recursive: true });
  const scorer = new InventoryScorer();
  const rows: InventorySmokeRow[] = [];
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
    console.log(
      `${report.pass ? "PASS" : "fail"}  ${id}  fill=${report.gates.fill_rate?.value?.toFixed(3)} cash=${report.gates.ending_cash?.value?.toFixed(1)}`,
    );
  }
  const summary = {
    domain: "inventory",
    agent: "reorder-point",
    horizonDays: cfg.horizonDays,
    allGates: rows.filter((r) => r.pass).length / rows.length,
    rows,
    run_id: INVENTORY_RUN_ID,
    note: "Baseline is expected to FAIL all-gates. This smoke checks the platform, not policy quality.",
  };
  writeJson(INVENTORY_SMOKE_PATH, summary);
  console.log(`allGates=${summary.allGates} → ${INVENTORY_SMOKE_PATH}`);
  return { allGates: summary.allGates, rows, path: INVENTORY_SMOKE_PATH };
}
