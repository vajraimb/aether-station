/**
 * Unified experiment ledger. JSON first; SQLite ingest comes later.
 * claim_type stops "search did not find" being written as "physically impossible".
 */
export const PHYSICS_SHA = "bdfff5b4c62733d7156d2f9cdeeaa75661d6c9f4";

export const CLAIM_TYPES = ["measured", "search_unreached", "proven", "hypothesis", "deprecated"] as const;
export type ClaimType = (typeof CLAIM_TYPES)[number];

export const RUN_KINDS = ["benchmark_run", "physics_audit", "research_study", "benchmark_release"] as const;
export type RunKind = (typeof RUN_KINDS)[number];

export const RUN_STATUSES = ["passed", "failed", "stopped", "archived"] as const;
export type RunStatus = (typeof RUN_STATUSES)[number];

export interface Claim {
  readonly type: ClaimType;
  readonly statement: string;
  readonly evidence_artifact: string;
}

export interface RunRecord {
  readonly run_id: string;
  readonly kind: RunKind;
  readonly status: RunStatus;
  readonly domain: "aether";
  readonly environment_version: string;
  readonly agent_version: string;
  readonly code_sha: string;
  readonly physics_sha: string;
  readonly agent_sha: string | null;
  readonly scenario_hash: string | null;
  readonly config_hash: string | null;
  readonly seed_set: readonly number[] | null;
  readonly parent_run_id: string | null;
  readonly failure_class: string | null;
  readonly metrics: Readonly<Record<string, number | string | boolean | null>>;
  readonly artifacts: readonly string[];
  readonly reproduction_command: string;
  readonly claims: readonly Claim[];
}

export function isClaimType(x: string): x is ClaimType {
  return (CLAIM_TYPES as readonly string[]).includes(x);
}

/** Search misses must not be filed as proven unreachability. */
export function claimsHonest(run: RunRecord): boolean {
  return run.claims.every((c) => {
    if (c.type === "proven" && /unreach|impossible|cannot/i.test(c.statement)) return false;
    if (c.type === "search_unreached" && /impossible|unreachable in the plant/i.test(c.statement)) return false;
    return isClaimType(c.type);
  });
}
