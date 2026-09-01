import { initialTruth, makeScenario, SCENARIO_IDS, defaultInventoryConfig } from "./scenario";
import { HIDDEN_TRUTH_KEYS } from "./types";

interface T {
  name: string;
  pass: boolean;
  detail: string;
}

function check(name: string, pass: boolean, detail: string, out: T[]): void {
  out.push({ name, pass, detail });
}

export function runInventoryScenarioTests(): T[] {
  const out: T[] = [];
  check("scenario_count", SCENARIO_IDS.length === 6, `n=${SCENARIO_IDS.length}`, out);
  const cfg = defaultInventoryConfig();
  for (const id of SCENARIO_IDS) {
    const sc = makeScenario(id, 7);
    const t = initialTruth(cfg, sc);
    check(`scenario_${id}_id`, sc.id === id, sc.id, out);
    check(`scenario_${id}_seed`, sc.seed === 7, String(sc.seed), out);
    check(`truth_${id}_no_neg`, t.onHand >= 0 && t.cash >= 0, `q=${t.onHand} cash=${t.cash}`, out);
    check(`truth_${id}_demandRate`, t.demandRate === sc.demandMean, String(t.demandRate), out);
  }
  const spike = makeScenario("demand-spike");
  check("spike_day", spike.demandSpikeDay === 20 && spike.demandSpikeMult === 3, String(spike.demandSpikeDay), out);
  const outage = makeScenario("supplier-outage");
  check("outage_window", outage.supplierFailDay === 15 && outage.supplierFailDur === 8, String(outage.supplierFailDur), out);
  check("hidden_keys", HIDDEN_TRUTH_KEYS.includes("demandRate") && HIDDEN_TRUTH_KEYS.includes("supplierHealthy"), "hidden", out);
  return out;
}
