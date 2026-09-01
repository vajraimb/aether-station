#!/usr/bin/env npx tsx
import { join } from "node:path";
import { INVENTORY_LEDGER } from "../../../domains/inventory/catalog.ts";
import { LEDGER } from "../ledger/catalog.ts";
import { claimsHonest } from "../ledger/schema.ts";
import { writeJson } from "../io.ts";

const ROOT = join(import.meta.dirname, "../../..");
const ALL = [...LEDGER, ...INVENTORY_LEDGER];
const bad = ALL.filter((r) => !claimsHonest(r));
if (bad.length) {
  console.error("dishonest claims", bad.map((r) => r.run_id));
  process.exit(1);
}

writeJson(join(ROOT, "outputs/ledger/index.json"), {
  domains: ["aether", "inventory"],
  environment_version: "aether-station@0.1.0-benchmark",
  physics_sha: "bdfff5b4c62733d7156d2f9cdeeaa75661d6c9f4",
  generatedAt: new Date().toISOString(),
  count: ALL.length,
  runs: ALL,
});
for (const r of ALL) {
  writeJson(join(ROOT, `outputs/ledger/runs/${r.run_id}.json`), r);
}
console.log(`ledger stamped ${ALL.length} runs → outputs/ledger/`);
