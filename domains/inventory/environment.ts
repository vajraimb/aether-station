import type { Environment, StepResult } from "../../packages/agent-arena/src/environment.ts";
import { ObservationChannel } from "./observe";
import { applyAction, stepPlant } from "./plant";
import { mulberry32 } from "./rng";
import { initialTruth } from "./scenario";
import type {
  InventoryAction,
  InventoryObservation,
  InventoryPrivateScenario,
  InventoryPublicConfig,
  InventorySample,
  InventoryTruth,
  OrderUpdate,
} from "./types";

export interface InventoryEvent {
  t: number;
  type: "order_placed" | "order_rejected" | "stockout" | "supplier_down" | "supplier_up" | "bankrupt";
  data?: Record<string, number | string | boolean>;
}

export class InventoryEnvironment implements Environment<InventoryObservation, InventoryAction> {
  truth: InventoryTruth | null = null;
  events: InventoryEvent[] = [];
  log: InventorySample[] = [];
  sc: InventoryPrivateScenario | null = null;
  private ch = new ObservationChannel();
  private rng: () => number = mulberry32(1);
  private lastHealthy = true;

  constructor(readonly cfg: InventoryPublicConfig) {}

  reset(privateScenario: unknown): InventoryObservation {
    this.sc = privateScenario as InventoryPrivateScenario;
    this.rng = mulberry32(this.sc.seed);
    this.truth = initialTruth(this.cfg, this.sc);
    this.events = [];
    this.log = [];
    this.ch = new ObservationChannel();
    this.lastHealthy = true;
    this.ch.push(this.truth);
    return this.ch.sample(this.cfg, this.sc, this.rng);
  }

  step(action: InventoryAction): StepResult<InventoryObservation> {
    if (!this.truth || !this.sc) throw new Error("reset first");
    const applied = applyAction(this.truth, this.cfg, action);
    this.truth = applied.truth;
    if (applied.rejected) this.events.push({ t: this.truth.time, type: "order_rejected" });
    if (!applied.rejected && action.kind !== "coast") {
      this.events.push({ t: this.truth.time, type: "order_placed", data: { qty: action.quantity } });
    }
    const updates: OrderUpdate[] = [...applied.updates];
    const before = this.truth;
    this.truth = stepPlant(this.truth, this.cfg, this.sc, this.rng);
    if (this.truth.backlog > 0) this.events.push({ t: this.truth.time, type: "stockout", data: { backlog: this.truth.backlog } });
    if (this.lastHealthy && !this.truth.supplierHealthy) this.events.push({ t: this.truth.time, type: "supplier_down" });
    if (!this.lastHealthy && this.truth.supplierHealthy) this.events.push({ t: this.truth.time, type: "supplier_up" });
    this.lastHealthy = this.truth.supplierHealthy;
    if (this.truth.cash < 0) this.events.push({ t: this.truth.time, type: "bankrupt" });
    for (const o of before.pendingOrders) {
      if (!this.truth.pendingOrders.some((p) => p.id === o.id)) {
        updates.push({ id: o.id, status: "arrived", quantity: o.quantity });
      } else if (this.truth.pendingOrders.find((p) => p.id === o.id)!.etaDay > o.etaDay) {
        updates.push({ id: o.id, status: "delayed", quantity: o.quantity });
      }
    }
    this.ch.push(this.truth, updates);
    const observation = this.ch.sample(this.cfg, this.sc, this.rng);
    this.log.push({
      t: this.truth.time,
      onHand: this.truth.onHand,
      cash: this.truth.cash,
      demandToday: this.truth.demandToday,
      filledToday: this.truth.filledToday,
      backlog: this.truth.backlog,
      reportedOnHand: observation.reportedOnHand,
      demandForecast: observation.demandForecast,
      supplierAlert: observation.supplierAlert,
      actionKind: action.kind,
      actionQty: action.kind === "coast" ? 0 : action.quantity,
      demandAcc: this.truth.demandAcc,
      filledAcc: this.truth.filledAcc,
      holdingCostAcc: this.truth.holdingCostAcc,
      rushCostAcc: this.truth.rushCostAcc,
      stockoutDays: this.truth.stockoutDays,
      constraintViolations: this.truth.constraintViolations,
    });
    const terminated = this.truth.time >= this.cfg.horizonDays || this.truth.cash < 0;
    return { observation, terminated, truncated: false };
  }
}
