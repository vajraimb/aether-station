import type { Agent } from "../../packages/agent-arena/src/agent.ts";
import type { InventoryAction, InventoryObservation, InventoryPublicConfig } from "./types";
import { defaultInventoryConfig } from "./scenario";

/** Transparent reorder-point policy. Observation only. */
export class ReorderPointAgent implements Agent<InventoryObservation, InventoryAction> {
  cfg: InventoryPublicConfig = defaultInventoryConfig();
  private onOrderEst = 0;

  reset(publicConfig: unknown): void {
    this.cfg = { ...defaultInventoryConfig(), ...(publicConfig as Partial<InventoryPublicConfig>) };
    this.onOrderEst = 0;
  }

  step(observation: Readonly<InventoryObservation>): InventoryAction {
    for (const u of observation.orderUpdates) {
      if (u.status === "placed") this.onOrderEst += u.quantity;
      if (u.status === "arrived") this.onOrderEst = Math.max(0, this.onOrderEst - u.quantity);
    }
    const position = observation.reportedOnHand + this.onOrderEst;
    const cover = Math.max(this.cfg.minOrderQty, Math.round(observation.demandForecast * this.cfg.normalLead * 1.5));
    if (position >= cover) return { kind: "coast" };
    const qty = Math.max(this.cfg.minOrderQty, cover - position);
    const rush = observation.supplierAlert || observation.delayedSales === 0 && observation.reportedOnHand < this.cfg.minOrderQty;
    const action: InventoryAction = rush
      ? { kind: "rush-order", quantity: qty }
      : { kind: "normal-order", quantity: qty };
    return action;
  }
}
