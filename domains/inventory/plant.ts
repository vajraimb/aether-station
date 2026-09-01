import { poisson } from "./rng";
import type { InventoryPrivateScenario, InventoryPublicConfig, InventoryTruth } from "./types";

export function supplierHealthyAt(sc: InventoryPrivateScenario, day: number): boolean {
  if (sc.supplierFailDay === null) return true;
  return day < sc.supplierFailDay || day >= sc.supplierFailDay + sc.supplierFailDur;
}

export function demandMeanAt(sc: InventoryPrivateScenario, day: number): number {
  if (sc.demandSpikeDay !== null && day >= sc.demandSpikeDay && day < sc.demandSpikeDay + 5) {
    return sc.demandMean * sc.demandSpikeMult;
  }
  return sc.demandMean;
}

/** Advance one day: arrivals, demand, holding. No agent action. */
export function stepPlant(
  truth: InventoryTruth,
  cfg: InventoryPublicConfig,
  sc: InventoryPrivateScenario,
  rng: () => number,
): InventoryTruth {
  const day = truth.time;
  const healthy = supplierHealthyAt(sc, day);
  const arrived = healthy
    ? truth.pendingOrders.filter((o) => o.etaDay <= day)
    : [];
  const stillPending = healthy
    ? truth.pendingOrders.filter((o) => o.etaDay > day)
    : truth.pendingOrders.map((o) => (o.etaDay <= day ? { ...o, etaDay: day + 1 } : o));
  let onHand = truth.onHand + arrived.reduce((s, o) => s + o.quantity, 0);
  onHand = Math.min(cfg.capacity, onHand);

  const demand = poisson(rng, demandMeanAt(sc, day));
  const available = onHand;
  const fromStock = Math.min(available, demand + truth.backlog);
  const filled = Math.min(fromStock, demand + truth.backlog);
  onHand = available - filled;
  const leftoverDemand = demand + truth.backlog - filled;
  const backlog = leftoverDemand;

  const holding = cfg.holdingCostPerUnit * onHand;
  const cash = truth.cash - holding + filled * cfg.salePrice;
  return {
    ...truth,
    time: day + 1,
    onHand,
    cash,
    demandRate: demandMeanAt(sc, day),
    supplierHealthy: healthy,
    pendingOrders: stillPending,
    backlog,
    filledToday: filled,
    demandToday: demand,
    holdingCostAcc: truth.holdingCostAcc + holding,
    stockoutDays: truth.stockoutDays + (backlog > 0 || filled < demand ? 1 : 0),
    demandAcc: truth.demandAcc + demand,
    filledAcc: truth.filledAcc + Math.min(filled, demand),
    ordersToday: 0,
  };
}
