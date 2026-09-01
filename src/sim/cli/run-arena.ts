#!/usr/bin/env npx tsx
/** AgentArena leak + domain adapter tests. No 180s physics. */
import { runCislunarTests } from "../../../domains/cislunar/trajectory.test.ts";
import { runInventoryAgentTests } from "../../../domains/inventory/agent.test.ts";
import { runInventoryEnvTests } from "../../../domains/inventory/environment.test.ts";
import { runInventoryObserveTests } from "../../../domains/inventory/observe.test.ts";
import { runInventoryScenarioTests } from "../../../domains/inventory/scenario.test.ts";
import { runInventoryScorerTests } from "../../../domains/inventory/scorer.test.ts";
import { runStationAdapterTests } from "../adapters/station.test.ts";
import { runBundleTests } from "./run-bundle.test.ts";

const rows = [
  ...runBundleTests(),
  ...runStationAdapterTests(),
  ...runInventoryScenarioTests(),
  ...runInventoryObserveTests(),
  ...runInventoryEnvTests(),
  ...runInventoryAgentTests(),
  ...runInventoryScorerTests(),
  ...runCislunarTests(),
];
for (const r of rows) console.log(`${r.pass ? "PASS" : "FAIL"}  ${r.name}  ${r.detail}`);
const fail = rows.filter((r) => !r.pass).length;
console.log(`\n${rows.length - fail}/${rows.length} passed`);
process.exit(fail ? 1 : 0);
