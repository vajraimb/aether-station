import { LEDGER } from "./catalog";
import { ExperimentStore } from "./store";

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
  store.close();
  return out;
}
