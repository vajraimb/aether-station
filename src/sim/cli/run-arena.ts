#!/usr/bin/env npx tsx
/** AgentArena leak + domain adapter tests. No 180s physics. */
import { runStationAdapterTests } from "../adapters/station.test.ts";
import { runInventoryEnvTests } from "../../../domains/inventory/environment.test.ts";
import { runInventoryObserveTests } from "../../../domains/inventory/observe.test.ts";
import { runInventoryScenarioTests } from "../../../domains/inventory/scenario.test.ts";

const rows = [
  ...runStationAdapterTests(),
  ...runInventoryScenarioTests(),
  ...runInventoryObserveTests(),
  ...runInventoryEnvTests(),
];
for (const r of rows) console.log(`${r.pass ? "PASS" : "FAIL"}  ${r.name}  ${r.detail}`);
const fail = rows.filter((r) => !r.pass).length;
console.log(`\n${rows.length - fail}/${rows.length} passed`);
process.exit(fail ? 1 : 0);
