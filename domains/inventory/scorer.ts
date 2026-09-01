import { readFileSync } from "node:fs";
import type { ArtifactScorer, ScoreReport } from "../../packages/agent-arena/src/scorer.ts";
import type { InventorySample } from "./types";

function parseCsv(text: string): InventorySample[] {
  const lines = text.trim().split(/\n/);
  if (lines.length < 2) return [];
  const keys = lines[0]!.split(",");
  return lines.slice(1).map((line) => {
    const p = line.split(",");
    const n = (k: string) => Number(p[keys.indexOf(k)] ?? NaN);
    return {
      t: n("t"),
      onHand: n("onHand"),
      cash: n("cash"),
      demandToday: n("demandToday"),
      filledToday: n("filledToday"),
      backlog: n("backlog"),
      reportedOnHand: n("reportedOnHand"),
      demandForecast: n("demandForecast"),
      supplierAlert: n("supplierAlert") === 1,
      actionKind: p[keys.indexOf("actionKind")] ?? "coast",
      actionQty: n("actionQty"),
      demandAcc: n("demandAcc"),
      filledAcc: n("filledAcc"),
      holdingCostAcc: n("holdingCostAcc"),
      rushCostAcc: n("rushCostAcc"),
      stockoutDays: n("stockoutDays"),
      constraintViolations: n("constraintViolations"),
    };
  });
}

export class InventoryScorer implements ArtifactScorer {
  score(trajectoryPath: string, eventsPath: string): ScoreReport {
    const rows = parseCsv(readFileSync(trajectoryPath, "utf8"));
    const events = readFileSync(eventsPath, "utf8")
      .trim()
      .split(/\n/)
      .filter(Boolean)
      .map((l) => JSON.parse(l) as { type: string; t: number });
    const last = rows[rows.length - 1];
    const fillRate = last && last.demandAcc > 0 ? last.filledAcc / last.demandAcc : 0;
    const endingCash = last?.cash ?? 0;
    const stockout = last?.stockoutDays ?? 99;
    const viol = last?.constraintViolations ?? 99;
    const holding = last?.holdingCostAcc ?? 0;
    const rush = last?.rushCostAcc ?? 0;
    const down = events.find((e) => e.type === "supplier_down");
    const firstAlert = rows.find((r) => r.supplierAlert);
    const detectDelay = down && firstAlert ? firstAlert.t - down.t : 0;
    const detectOk = !down || detectDelay < 3;
    const gates = {
      fill_rate: { pass: fillRate >= 0.95, value: fillRate },
      ending_cash: { pass: endingCash > 0, value: endingCash },
      stockout_days: { pass: stockout < 5, value: stockout },
      constraint_violations: { pass: viol === 0, value: viol },
      supplier_detect_delay: { pass: detectOk, value: down ? detectDelay : 0 },
      holding_cost: { pass: true, value: holding },
      rush_cost: { pass: true, value: rush },
    };
    const required = ["fill_rate", "ending_cash", "stockout_days", "constraint_violations", "supplier_detect_delay"] as const;
    const pass = required.every((k) => gates[k].pass);
    return { pass, gates };
  }
}
