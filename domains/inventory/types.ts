export interface PendingOrder {
  readonly id: string;
  readonly quantity: number;
  readonly kind: "normal" | "rush";
  readonly placedDay: number;
  readonly etaDay: number;
  readonly unitCost: number;
}

export interface InventoryTruth {
  time: number;
  onHand: number;
  cash: number;
  demandRate: number;
  supplierHealthy: boolean;
  pendingOrders: PendingOrder[];
  backlog: number;
  filledToday: number;
  demandToday: number;
  holdingCostAcc: number;
  rushCostAcc: number;
  stockoutDays: number;
  demandAcc: number;
  filledAcc: number;
  constraintViolations: number;
  ordersToday: number;
}

export interface OrderUpdate {
  readonly id: string;
  readonly status: "placed" | "arrived" | "delayed";
  readonly quantity: number;
}

export interface InventoryObservation {
  readonly timestamp: number;
  readonly reportedOnHand: number;
  readonly delayedSales: number;
  readonly demandForecast: number;
  readonly orderUpdates: readonly OrderUpdate[];
  readonly supplierAlert: boolean;
}

export type InventoryAction =
  | { readonly kind: "coast" }
  | { readonly kind: "normal-order"; readonly quantity: number }
  | { readonly kind: "rush-order"; readonly quantity: number };

export interface InventoryPublicConfig {
  horizonDays: number;
  capacity: number;
  minOrderQty: number;
  unitCost: number;
  rushMultiplier: number;
  salePrice: number;
  holdingCostPerUnit: number;
  startOnHand: number;
  startCash: number;
  normalLead: number;
  rushLead: number;
  maxOrdersPerDay: number;
  obsDelay: number;
  salesDelay: number;
}

export interface InventoryPrivateScenario {
  id: string;
  seed: number;
  demandMean: number;
  demandSpikeDay: number | null;
  demandSpikeMult: number;
  supplierFailDay: number | null;
  supplierFailDur: number;
  obsDelayExtra: number;
  forecastBias: number;
  salesDelayExtra: number;
}

export const OBSERVATION_KEYS = [
  "timestamp",
  "reportedOnHand",
  "delayedSales",
  "demandForecast",
  "orderUpdates",
  "supplierAlert",
] as const;

export const HIDDEN_TRUTH_KEYS = ["demandRate", "supplierHealthy", "pendingOrders", "backlog"] as const;
