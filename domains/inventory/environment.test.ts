import { InventoryEnvironment } from "./environment";
import { observationLeaksTruth } from "./observe";
import { defaultInventoryConfig, makeScenario } from "./scenario";

interface T {
  name: string;
  pass: boolean;
  detail: string;
}

function check(name: string, pass: boolean, detail: string, out: T[]): void {
  out.push({ name, pass, detail });
}

export function runInventoryEnvTests(): T[] {
  const out: T[] = [];
  const cfg = defaultInventoryConfig({ horizonDays: 8, minOrderQty: 10, capacity: 50, startCash: 200 });
  const env = new InventoryEnvironment(cfg);
  const obs0 = env.reset(makeScenario("steady", 4));
  check("env_reset_obs", Number.isFinite(obs0.reportedOnHand), `q=${obs0.reportedOnHand}`, out);
  check("env_reset_noleak", observationLeaksTruth(obs0).length === 0, observationLeaksTruth(obs0).join(","), out);

  const tiny = env.step({ kind: "normal-order", quantity: 1 });
  check("env_min_qty_reject", (env.truth?.constraintViolations ?? 0) >= 1, `viol=${env.truth?.constraintViolations}`, out);
  void tiny;

  const env2 = new InventoryEnvironment(cfg);
  env2.reset(makeScenario("steady", 4));
  env2.step({ kind: "normal-order", quantity: 1000 });
  check("env_capacity_or_cash_reject", (env2.truth?.constraintViolations ?? 0) >= 1, `viol=${env2.truth?.constraintViolations}`, out);

  const env3 = new InventoryEnvironment(defaultInventoryConfig({ horizonDays: 5 }));
  env3.reset(makeScenario("steady", 1));
  let steps = 0;
  let term = false;
  while (steps < 20) {
    const r = env3.step({ kind: "coast" });
    steps += 1;
    if (r.terminated) {
      term = true;
      break;
    }
  }
  check("env_horizon_terminates", term && steps === 5, `steps=${steps}`, out);
  return out;
}
