import { defaultPublicConfig } from "../constants";
import { rolloutFromSimLike } from "./rollout-model";
import { captureFeatures, proveInfeasible, captureCostFromLabel } from "./reachability-label";
import { splitOf } from "./reachability-dataset";
import { fitKnn, queryTable, setCaptureValueTable, captureCost } from "./capture-value";
import { capturedGates } from "./capture-reachability";

interface T {
  name: string;
  pass: boolean;
  detail: string;
}

function check(name: string, pass: boolean, detail: string, out: T[]): void {
  out.push({ name, pass, detail });
}

export function runCaptureValueTests(): T[] {
  const out: T[] = [];
  const plant = defaultPublicConfig();

  {
    const rest = rolloutFromSimLike({
      time: 0,
      q: [0.9962, 0, 0.08716, 0],
      w: [0, 0, 0],
      s: 0,
      sd: 0,
      th1: 0,
      th1d: 0,
      th2: 0,
      th2d: 0,
      fuel: 2.8,
    });
    check("test_infeasible_fuel_floor_at_rest", proveInfeasible(rest, plant, []), `att=${capturedGates(rest, plant.qTarget).attDeg}`, out);
    const spinning = rolloutFromSimLike({
      time: 0,
      q: [0.9962, 0, 0.08716, 0],
      w: [0.05, 0, 0],
      s: 0,
      sd: 0,
      th1: 0,
      th1d: 0,
      th2: 0,
      th2d: 0,
      fuel: 2.8,
    });
    check(
      "test_fuel_floor_with_rate_is_not_a_proof",
      !proveInfeasible(spinning, plant, []),
      "coasting rate can still walk attitude",
      out,
    );
  }

  {
    const capturedCost = captureCostFromLabel("captured", 4, 0.1, 0.4, 0.004);
    const unreached = captureCostFromLabel("search_unreached", null, 0.2, 6, 0.02);
    const inf = captureCostFromLabel("proven_infeasible", null, 0, 12, 0);
    check("test_costs_ordered", capturedCost < unreached && unreached < inf, `${capturedCost} ${unreached} ${inf}`, out);
  }

  {
    check("test_split_is_deterministic", splitOf("ds-3") === splitOf("ds-3"), splitOf("ds-3"), out);
    const ids = Array.from({ length: 40 }, (_, i) => splitOf(`ds-${i}`));
    const nVal = ids.filter((s) => s === "val").length;
    check("test_split_has_both", nVal >= 4 && nVal <= 16, `nVal=${nVal}`, out);
  }

  {
    const plant2 = plant;
    const st = rolloutFromSimLike({
      time: 0,
      q: [1, 0, 0, 0],
      w: [0, 0, 0],
      s: 0.1,
      sd: 0,
      th1: 0,
      th1d: 0,
      th2: 0,
      th2d: 0,
      fuel: 3.4,
    });
    const x = captureFeatures(st, [], plant2);
    check("test_feature_len_8", x.length === 8, `${x.length}`, out);
    const table = fitKnn([
      {
        id: "ds-0",
        split: "train",
        isolated: [],
        features: x,
        label: "captured",
        cost: 2,
        firstPrimitiveId: "coast:0.040",
        captureTimeS: 1,
        minAttDeg: 0.2,
        finalOmega: 0.002,
        fuelUsedKg: 0.04,
        method: "eigen",
        horizonS: 8,
      },
      {
        id: "ds-1",
        split: "val",
        isolated: [],
        features: x.map((v, i) => v + (i === 0 ? 0.01 : 0)),
        label: "captured",
        cost: 2.2,
        firstPrimitiveId: "coast:0.040",
        captureTimeS: 1.1,
        minAttDeg: 0.3,
        finalOmega: 0.003,
        fuelUsedKg: 0.04,
        method: "eigen",
        horizonS: 8,
      },
    ]);
    setCaptureValueTable(table);
    const q = queryTable(table, x);
    check("test_knn_recovers_train_cost", Math.abs(q.cost - 2) < 0.05, `cost=${q.cost}`, out);
    const c = captureCost(st, [], plant2);
    check("test_capture_cost_wired", Number.isFinite(c) && c < 50, `c=${c}`, out);
  }

  return out;
}
