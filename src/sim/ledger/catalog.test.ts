import { existsSync } from "node:fs";
import { LEDGER } from "./catalog";
import { CLAIM_TYPES, claimsHonest, PHYSICS_SHA } from "./schema";

interface T {
  name: string;
  pass: boolean;
  detail: string;
}

function check(name: string, pass: boolean, detail: string, out: T[]): void {
  out.push({ name, pass, detail });
}

export function runLedgerTests(): T[] {
  const out: T[] = [];
  check("ledger_nonempty", LEDGER.length >= 8, `n=${LEDGER.length}`, out);
  const ids = new Set(LEDGER.map((r) => r.run_id));
  check("ledger_unique_ids", ids.size === LEDGER.length, `unique=${ids.size}`, out);
  for (const r of LEDGER) {
    check(`physics_sha_${r.run_id}`, r.physics_sha === PHYSICS_SHA, r.physics_sha.slice(0, 7), out);
    check(`claims_honest_${r.run_id}`, claimsHonest(r), r.claims.map((c) => c.type).join(","), out);
    check(`has_claims_${r.run_id}`, r.claims.length >= 1, `n=${r.claims.length}`, out);
    for (const c of r.claims) {
      check(`claim_type_${r.run_id}_${c.type}`, (CLAIM_TYPES as readonly string[]).includes(c.type), c.type, out);
    }
    for (const a of r.artifacts) {
      if (a.startsWith("outputs/") || a.startsWith("docs/")) {
        check(`artifact_exists_${a}`, existsSync(a), a, out);
      }
    }
  }
  const proven = LEDGER.flatMap((r) => r.claims).filter((c) => c.type === "proven");
  check("no_proven_unreachability", proven.length === 0, `proven=${proven.length}`, out);
  const search = LEDGER.filter((r) => r.claims.some((c) => c.type === "search_unreached"));
  check("search_unreached_present", search.length >= 2, `n=${search.length}`, out);
  return out;
}
