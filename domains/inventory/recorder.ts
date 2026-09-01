import { fileRecorder } from "../../packages/agent-arena/src/recorder.ts";
import type { InventoryEvent } from "./environment";
import type { InventoryPrivateScenario, InventorySample } from "./types";

export function inventoryCsv(rows: readonly InventorySample[]): string {
  const header = [
    "t","onHand","cash","demandToday","filledToday","backlog","reportedOnHand","demandForecast",
    "supplierAlert","actionKind","actionQty","demandAcc","filledAcc","holdingCostAcc","rushCostAcc",
    "stockoutDays","constraintViolations",
  ].join(",");
  const lines = rows.map((r) =>
    [
      r.t, r.onHand, r.cash, r.demandToday, r.filledToday, r.backlog, r.reportedOnHand, r.demandForecast,
      r.supplierAlert ? 1 : 0, r.actionKind, r.actionQty, r.demandAcc, r.filledAcc, r.holdingCostAcc,
      r.rushCostAcc, r.stockoutDays, r.constraintViolations,
    ].join(","),
  );
  return [header, ...lines].join("\n");
}

export function inventoryEventsJsonl(events: readonly InventoryEvent[], sc: InventoryPrivateScenario): string {
  const scenario = { t: 0, type: "scenario", data: { id: sc.id, seed: sc.seed } };
  return [scenario, ...events].map((e) => JSON.stringify(e)).join("\n");
}

export function writeInventoryArtifacts(
  dir: string,
  rows: readonly InventorySample[],
  events: readonly InventoryEvent[],
  sc: InventoryPrivateScenario,
): { trajectoryPath: string; eventsPath: string } {
  const trajectoryPath = `${dir}/trajectory.csv`;
  const eventsPath = `${dir}/events.jsonl`;
  fileRecorder.write(trajectoryPath, inventoryCsv(rows));
  fileRecorder.write(eventsPath, inventoryEventsJsonl(events, sc));
  return { trajectoryPath, eventsPath };
}
