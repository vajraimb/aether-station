import type {
  InventoryObservation,
  InventoryPrivateScenario,
  InventoryPublicConfig,
  InventoryTruth,
  OrderUpdate,
} from "./types";
import { OBSERVATION_KEYS } from "./types";

interface Snap {
  day: number;
  onHand: number;
  filledToday: number;
  supplierHealthy: boolean;
  updates: OrderUpdate[];
}

/** Delayed, noisy observation. Never copies hidden truth fields. */
export class ObservationChannel {
  private snaps: Snap[] = [];
  private failSeenOn: number | null = null;

  push(truth: InventoryTruth, updates: readonly OrderUpdate[] = []): void {
    this.snaps.push({
      day: truth.time,
      onHand: truth.onHand,
      filledToday: truth.filledToday,
      supplierHealthy: truth.supplierHealthy,
      updates: [...updates],
    });
    if (!truth.supplierHealthy && this.failSeenOn === null) this.failSeenOn = truth.time;
  }

  sample(cfg: InventoryPublicConfig, sc: InventoryPrivateScenario, rng: () => number): InventoryObservation {
    const delay = cfg.obsDelay + sc.obsDelayExtra;
    const salesDelay = cfg.salesDelay + sc.salesDelayExtra;
    const now = this.snaps[this.snaps.length - 1];
    const t = now?.day ?? 0;
    const lagged = this.snaps.find((s) => s.day === t - delay);
    const salesSnap = this.snaps.find((s) => s.day === t - salesDelay);
    const noise = Math.round((rng() - 0.5) * 2);
    const reportedOnHand = Math.max(0, (lagged?.onHand ?? cfg.startOnHand) + noise);
    const forecast = Math.max(0, sc.demandMean * (1 + sc.forecastBias) + (rng() - 0.5) * 2);
    const alertDelay = 2;
    const supplierAlert =
      this.failSeenOn !== null && t >= this.failSeenOn + alertDelay && (now ? !now.supplierHealthy || t < this.failSeenOn + 20 : false);
    const orderUpdates = this.snaps
      .filter((s) => s.day >= t - delay && s.day <= t)
      .flatMap((s) => s.updates);
    return {
      timestamp: t,
      reportedOnHand,
      delayedSales: salesSnap?.filledToday ?? 0,
      demandForecast: forecast,
      orderUpdates,
      supplierAlert,
    };
  }
}

export function observationLeaksTruth(obs: InventoryObservation): string[] {
  const json = JSON.stringify(obs);
  const hits: string[] = [];
  for (const k of ["demandRate", "supplierHealthy", "pendingOrders", "backlog", "forecastBias"]) {
    if (json.includes(`"${k}"`)) hits.push(k);
  }
  const extra = Object.keys(obs).filter((k) => !(OBSERVATION_KEYS as readonly string[]).includes(k));
  return [...hits, ...extra.map((k) => `extra:${k}`)];
}
