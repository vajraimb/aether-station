#!/usr/bin/env npx tsx
import { join } from "node:path";
import { LEDGER } from "../ledger/catalog.ts";
import { ExperimentStore } from "../ledger/store.ts";

const ROOT = join(import.meta.dirname, "../../..");
const path = process.argv.includes("--memory")
  ? ":memory:"
  : (process.argv.find((a, i, arr) => arr[i - 1] === "--out") ?? join(ROOT, "outputs/ledger/experiments.sqlite"));

const store = new ExperimentStore(path);
const n = store.ingestAll(LEDGER);
console.log(`ingested ${n} runs → ${path}`);
console.log(`search_unreached=${store.byClaimType("search_unreached").length} proven=${store.byClaimType("proven").length}`);
if (path === ":memory:") store.close();
else store.close();
