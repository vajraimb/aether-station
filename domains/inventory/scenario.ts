import type { InventoryPrivateScenario, InventoryPublicConfig, InventoryTruth } from "./types";

export function defaultInventoryConfig(over: Partial<InventoryPublicConfig> = {}): InventoryPublicConfig {
  return {
    horizonDays: 60,
    capacity: 200,
    minOrderQty: 10,
    unitCost: 8,
    rushMultiplier: 2,
    salePrice: 16,
    holdingCostPerUnit: 0.15,
    startOnHand: 40,
    startCash: 2000,
    normalLead: 4,
    rushLead: 1,
    maxOrdersPerDay: 1,
    obsDelay: 1,
    salesDelay: 1,
    ...over,
  };
}

export const SCENARIO_IDS = [
  "steady",
  "demand-spike",
  "supplier-outage",
  "delay-anomaly",
  "forecast-bias",
  "combined-stress",
] as const;

export type ScenarioId = (typeof SCENARIO_IDS)[number];

export function makeScenario(id: ScenarioId, seed = 1): InventoryPrivateScenario {
  const base: InventoryPrivateScenario = {
    id,
    seed,
    demandMean: 8,
    demandSpikeDay: null,
    demandSpikeMult: 1,
    supplierFailDay: null,
    supplierFailDur: 0,
    obsDelayExtra: 0,
    forecastBias: 0,
    salesDelayExtra: 0,
  };
  if (id === "demand-spike") return { ...base, demandSpikeDay: 20, demandSpikeMult: 3 };
  if (id === "supplier-outage") return { ...base, supplierFailDay: 15, supplierFailDur: 8 };
  if (id === "delay-anomaly") return { ...base, obsDelayExtra: 2, salesDelayExtra: 2 };
  if (id === "forecast-bias") return { ...base, forecastBias: 0.4 };
  if (id === "combined-stress") {
    return {
      ...base,
      demandMean: 10,
      demandSpikeDay: 18,
      demandSpikeMult: 2.5,
      supplierFailDay: 22,
      supplierFailDur: 6,
      obsDelayExtra: 1,
      forecastBias: 0.25,
    };
  }
  return base;
}

export function initialTruth(cfg: InventoryPublicConfig, sc: InventoryPrivateScenario): InventoryTruth {
  return {
    time: 0,
    onHand: cfg.startOnHand,
    cash: cfg.startCash,
    demandRate: sc.demandMean,
    supplierHealthy: true,
    pendingOrders: [],
    backlog: 0,
    filledToday: 0,
    demandToday: 0,
    holdingCostAcc: 0,
    rushCostAcc: 0,
    stockoutDays: 0,
    demandAcc: 0,
    filledAcc: 0,
    constraintViolations: 0,
    ordersToday: 0,
  };
}
