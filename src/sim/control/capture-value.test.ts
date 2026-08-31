import { defaultPublicConfig } from "../constants";
import { rolloutFromSimLike } from "./rollout-model";
import { captureFeatures, proveInfeasible, captureCostFromLabel } from "./reachability-label";
import { assertGroupedSplit, splitOfGroup, type DatasetSample } from "./reachability-dataset";
import { fitKnn, queryTable, setCaptureValueTable, captureCost, heuristicCost } from "./capture-value";
import { capturedGates } from "./capture-reachability";

interface T {
  name: string;
  pass: boolean;
  detail: string;
}

function check(name: string, pass: boolean, detail: string, out: T[]): void {
  out.push({ name, pass, detail });
}

function row(partial: Partial<DatasetSample> & Pick<DatasetSample, "id" | "groupId" | "split" | "features" | "label" | "cost">): DatasetSample {
  return {
    isolated: [],
    firstPrimitiveId: null,
    captureTimeS: null,
    minAttDeg: 1,
    finalOmega: 0.01,
    fuelUsedKg: 0,
    method: "eigen",
    horizonS: 8,
    ...partial,
  };
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
    check("test_fuel_floor_with_rate_is_not_a_proof", !proveInfeasible(spinning, plant, []), "coasting rate can still walk attitude", out);
  }

  {
    const capturedCost = captureCostFromLabel("captured", 4, 0.1, 0.4, 0.004);
    const unreached = captureCostFromLabel("search_unreached", null, 0.2, 6, 0.02);
    const inf = captureCostFromLabel("proven_infeasible", null, 0, 12, 0);
    check("test_costs_ordered", capturedCost < unreached && unreached < inf, `${capturedCost} ${unreached} ${inf}`, out);
  }

  {
    check("test_group_split_deterministic", splitOfGroup("fam-3") === splitOfGroup("fam-3"), splitOfGroup("fam-3"), out);
    const samples = [
      row({ id: "fam-1::0", groupId: "fam-1", split: splitOfGroup("fam-1"), features: [0.1, 0, 0, 0, 1, 0, 0, 0], label: "captured", cost: 1 }),
      row({ id: "fam-1::1", groupId: "fam-1", split: splitOfGroup("fam-1"), features: [0.2, 0, 0, 0, 1, 0, 0, 0], label: "search_unreached", cost: 80 }),
      row({ id: "fam-2::0", groupId: "fam-2", split: splitOfGroup("fam-2"), features: [0.3, 0, 0, 0, 1, 0, 0, 0], label: "captured", cost: 2 }),
    ];
    const leaks = assertGroupedSplit(samples);
    check("test_grouped_split_no_leak", leaks.length === 0 && samples[0]!.split === samples[1]!.split, `leaks=${leaks.join(",")} s=${samples[0]!.split}/${samples[1]!.split}`, out);
  }

  {
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
    const x = captureFeatures(st, [], plant);
    check("test_feature_len_8", x.length === 8, `${x.length}`, out);
    const table = fitKnn([
      row({ id: "fam-0::0", groupId: "fam-0", split: "train", features: x, label: "captured", cost: 2, firstPrimitiveId: "coast:0.040" }),
      row({
        id: "fam-9::0",
        groupId: "fam-9",
        split: "val",
        features: x.map((v, i) => v + (i === 0 ? 0.01 : 0)),
        label: "captured",
        cost: 2.2,
        firstPrimitiveId: "coast:0.040",
      }),
    ]);
    setCaptureValueTable(table);
    const q = queryTable(table, x);
    check("test_knn_recovers_train_cost", !q.ood && Math.abs(q.cost - 2) < 0.05, `cost=${q.cost} ood=${q.ood}`, out);
    const far = x.map((v, i) => (i === 0 ? 4 : v + 10));
    const o = queryTable(table, far);
    check("test_ood_falls_back_to_heuristic", o.ood && Math.abs(o.cost - heuristicCost(far)) < 1e-9, `ood=${o.ood} nn=${o.nnDist} thr=${table.oodThreshold}`, out);
    const c = captureCost(st, [], plant);
    check("test_capture_cost_wired", Number.isFinite(c) && c < 50, `c=${c}`, out);
  }

  return out;
}
