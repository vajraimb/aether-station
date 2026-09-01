import { ReorderPointAgent } from "./agent";
import { OBSERVATION_KEYS } from "./types";

interface T {
  name: string;
  pass: boolean;
  detail: string;
}

function check(name: string, pass: boolean, detail: string, out: T[]): void {
  out.push({ name, pass, detail });
}

export function runInventoryAgentTests(): T[] {
  const out: T[] = [];
  const a = new ReorderPointAgent();
  a.reset({ minOrderQty: 10, normalLead: 4 });
  const high: import("./types").InventoryObservation = {
    timestamp: 3,
    reportedOnHand: 80,
    delayedSales: 8,
    demandForecast: 8,
    orderUpdates: [],
    supplierAlert: false,
  };
  const low: import("./types").InventoryObservation = {
    timestamp: 4,
    reportedOnHand: 5,
    delayedSales: 0,
    demandForecast: 8,
    orderUpdates: [],
    supplierAlert: false,
  };
  const coast = a.step(high);
  check("agent_coast_when_full", coast.kind === "coast", coast.kind, out);
  const b = new ReorderPointAgent();
  b.reset({ minOrderQty: 10, normalLead: 4 });
  const act = b.step(low);
  check("agent_orders_when_low", act.kind !== "coast", act.kind, out);
  if (act.kind !== "coast") check("agent_min_qty", act.quantity >= 10, String(act.quantity), out);
  const keys = Object.keys(high);
  check("agent_obs_shape", keys.every((k) => (OBSERVATION_KEYS as readonly string[]).includes(k)), keys.join(","), out);
  return out;
}
