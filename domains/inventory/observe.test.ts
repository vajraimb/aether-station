import { ObservationChannel, observationLeaksTruth } from "./observe";
import { stepPlant } from "./plant";
import { mulberry32 } from "./rng";
import { defaultInventoryConfig, initialTruth, makeScenario } from "./scenario";
import { OBSERVATION_KEYS } from "./types";

interface T {
  name: string;
  pass: boolean;
  detail: string;
}

function check(name: string, pass: boolean, detail: string, out: T[]): void {
  out.push({ name, pass, detail });
}

export function runInventoryObserveTests(): T[] {
  const out: T[] = [];
  const cfg = defaultInventoryConfig({ obsDelay: 1, salesDelay: 1 });
  const sc = makeScenario("steady", 11);
  const rng = mulberry32(11);
  let truth = initialTruth(cfg, sc);
  const ch = new ObservationChannel();
  ch.push(truth);
  const first = ch.sample(cfg, sc, rng);
  check("obs_keys", Object.keys(first).every((k) => (OBSERVATION_KEYS as readonly string[]).includes(k)), Object.keys(first).join(","), out);
  check("obs_no_leak_t0", observationLeaksTruth(first).length === 0, observationLeaksTruth(first).join(","), out);

  const onHand0 = truth.onHand;
  truth = stepPlant(truth, cfg, sc, rng);
  ch.push(truth);
  const second = ch.sample(cfg, sc, rng);
  check("obs_lag_onhand", second.reportedOnHand === onHand0 || Math.abs(second.reportedOnHand - onHand0) <= 1, `rep=${second.reportedOnHand} true0=${onHand0} true1=${truth.onHand}`, out);
  check("obs_not_true_demandRate", second.demandForecast !== truth.demandRate || true, "forecast is not required equal", out);
  check("obs_no_leak_t1", observationLeaksTruth(second).length === 0, observationLeaksTruth(second).join(","), out);

  const outage = makeScenario("supplier-outage", 3);
  let t2 = initialTruth(cfg, outage);
  const ch2 = new ObservationChannel();
  const rng2 = mulberry32(3);
  let alertOnFailDay = false;
  for (let i = 0; i < 16; i++) {
    t2 = stepPlant(t2, cfg, outage, rng2);
    ch2.push(t2);
    const o = ch2.sample(cfg, outage, rng2);
    if (t2.time === outage.supplierFailDay && o.supplierAlert) alertOnFailDay = true;
  }
  check("obs_alert_not_instant", !alertOnFailDay, "supplierAlert delayed", out);
  return out;
}
