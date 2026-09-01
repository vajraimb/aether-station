import { INVENTORY_LEDGER } from "../../../domains/inventory/catalog";
import { LEDGER } from "./catalog";
import { ExperimentStore, SCHEMA_VERSION } from "./store";
import type { RunRecord } from "./schema";

interface T {
  name: string;
  pass: boolean;
  detail: string;
}

function check(name: string, pass: boolean, detail: string, out: T[]): void {
  out.push({ name, pass, detail });
}

export function runStoreTests(): T[] {
  const out: T[] = [];
  const store = new ExperimentStore(":memory:");
  store.ingestAll(LEDGER);
  check("store_count", store.count() === LEDGER.length, `n=${store.count()}`, out);
  check("store_schema_version", store.schemaVersion() === SCHEMA_VERSION, `v=${store.schemaVersion()}`, out);
  const base = store.get("aether-baseline-train10");
  check("store_get_baseline", base?.status === "failed" && base.domain === "aether", String(base?.status), out);
  const kids = store.children("aether-physics-conservation");
  check("store_children", kids.length >= 1, `n=${kids.length}`, out);
  const line = store.lineage("aether-robust-terminal");
  check(
    "store_lineage",
    line[0] === "aether-robust-terminal" && line.includes("aether-physics-conservation"),
    line.join(">"),
    out,
  );
  const search = store.byClaimType("search_unreached");
  check("store_search_unreached", search.length >= 2, `n=${search.length}`, out);
  const proven = store.byClaimType("proven");
  check("store_no_proven", proven.length === 0, `n=${proven.length}`, out);

  const n1 = store.count();
  const c1 = store.claimCount();
  store.ingestAll(LEDGER);
  check("store_idempotent_runs", store.count() === n1, `n=${store.count()}`, out);
  check("store_idempotent_claims", store.claimCount() === c1, `c=${store.claimCount()}`, out);

  const orphan: RunRecord = {
    ...LEDGER[0]!,
    run_id: "orphan-should-rollback",
    parent_run_id: "does-not-exist",
  };
  let threw = false;
  try {
    store.ingest(orphan);
  } catch {
    threw = true;
  }
  check("store_parent_required", threw, "missing parent throws", out);
  check("store_rollback_no_orphan", store.get("orphan-should-rollback") === undefined, "no orphan row", out);
  check("store_rollback_count", store.count() === n1, `n=${store.count()}`, out);
  const n2 = store.count();
  store.ingestAll(INVENTORY_LEDGER);
  check("store_inventory_domain", store.get("inventory-reorder-smoke")?.domain === "inventory", String(store.get("inventory-reorder-smoke")?.domain), out);
  check("store_inventory_parent", store.lineage("inventory-reorder-smoke").includes("aether-benchmark-v0.1.0"), "cross-domain parent", out);
  check("store_inventory_count", store.count() === n2 + 1, `n=${store.count()}`, out);
  store.ingestAll(INVENTORY_LEDGER);
  check("store_inventory_idempotent", store.count() === n2 + 1, `n=${store.count()}`, out);
  store.close();
  return out;
}
