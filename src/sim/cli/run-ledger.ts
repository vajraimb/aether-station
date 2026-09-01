#!/usr/bin/env npx tsx
import { join } from "node:path";
import { LEDGER } from "../ledger/catalog.ts";
import { claimsHonest } from "../ledger/schema.ts";
import { writeJson } from "../io.ts";

const ROOT = join(import.meta.dirname, "../../..");
const bad = LEDGER.filter((r) => !claimsHonest(r));
if (bad.length) {
  console.error("dishonest claims", bad.map((r) => r.run_id));
  process.exit(1);
}

writeJson(join(ROOT, "outputs/ledger/index.json"), {
  domain: "aether",
  environment_version: "aether-station@0.1.0-benchmark",
  physics_sha: "bdfff5b4c62733d7156d2f9cdeeaa75661d6c9f4",
  generatedAt: new Date().toISOString(),
  count: LEDGER.length,
  runs: LEDGER,
});
for (const r of LEDGER) {
  writeJson(join(ROOT, `outputs/ledger/runs/${r.run_id}.json`), r);
}
console.log(`ledger stamped ${LEDGER.length} runs → outputs/ledger/`);
