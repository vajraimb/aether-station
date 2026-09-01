/**
 * ExperimentStore — SQLite ingest of the JSON ledger.
 * Shared columns for AETHER and later domains. Not a new database product.
 */
import { createHash } from "node:crypto";
import { DatabaseSync } from "node:sqlite";
import { mkdirSync } from "node:fs";
import { dirname } from "node:path";
import type { RunRecord } from "./schema";

export const SCHEMA_VERSION = 1;

export const STORE_SCHEMA = `
CREATE TABLE IF NOT EXISTS schema_meta (
  key TEXT PRIMARY KEY,
  value TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS runs (
  run_id TEXT PRIMARY KEY,
  kind TEXT NOT NULL,
  status TEXT NOT NULL,
  domain TEXT NOT NULL,
  environment_version TEXT NOT NULL,
  agent_version TEXT NOT NULL,
  code_sha TEXT NOT NULL,
  physics_sha TEXT NOT NULL,
  agent_sha TEXT,
  scenario_hash TEXT,
  config_hash TEXT,
  parent_run_id TEXT,
  failure_class TEXT,
  reproduction_command TEXT NOT NULL,
  seed_set TEXT,
  artifact_manifest TEXT NOT NULL,
  artifact_hash TEXT NOT NULL,
  created_at TEXT NOT NULL DEFAULT (datetime('now'))
);
CREATE TABLE IF NOT EXISTS metrics (
  run_id TEXT NOT NULL,
  key TEXT NOT NULL,
  value REAL,
  text_value TEXT,
  PRIMARY KEY (run_id, key)
);
CREATE TABLE IF NOT EXISTS claims (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  run_id TEXT NOT NULL,
  claim_type TEXT NOT NULL,
  statement TEXT NOT NULL,
  evidence_artifact TEXT NOT NULL,
  UNIQUE (run_id, claim_type, statement, evidence_artifact)
);
CREATE INDEX IF NOT EXISTS idx_runs_parent ON runs(parent_run_id);
CREATE INDEX IF NOT EXISTS idx_runs_domain ON runs(domain);
CREATE INDEX IF NOT EXISTS idx_claims_type ON claims(claim_type);
`;

export function artifactHash(run: RunRecord): string {
  return createHash("sha256")
    .update(JSON.stringify({ artifacts: run.artifacts, code_sha: run.code_sha, metrics: run.metrics }))
    .digest("hex");
}

function topo(runs: readonly RunRecord[]): RunRecord[] {
  const byId = new Map(runs.map((r) => [r.run_id, r]));
  const out: RunRecord[] = [];
  const seen = new Set<string>();
  const visit = (r: RunRecord) => {
    if (seen.has(r.run_id)) return;
    seen.add(r.run_id);
    if (r.parent_run_id && byId.has(r.parent_run_id)) visit(byId.get(r.parent_run_id)!);
    out.push(r);
  };
  for (const r of runs) visit(r);
  return out;
}

export class ExperimentStore {
  readonly db: DatabaseSync;

  constructor(path: string) {
    if (path !== ":memory:") mkdirSync(dirname(path), { recursive: true });
    this.db = new DatabaseSync(path);
    this.db.exec(STORE_SCHEMA);
    this.db.prepare("INSERT OR IGNORE INTO schema_meta(key, value) VALUES ('version', ?)").run(String(SCHEMA_VERSION));
  }

  schemaVersion(): number {
    const row = this.db.prepare("SELECT value FROM schema_meta WHERE key = 'version'").get() as { value: string };
    return Number(row.value);
  }

  close(): void {
    this.db.close();
  }

  ingest(run: RunRecord, opts: { requireParent?: boolean } = {}): "inserted" | "skipped" {
    if (!run.claims.every((c) => c.evidence_artifact.length > 0)) {
      throw new Error(`claim missing evidence_artifact: ${run.run_id}`);
    }
    const hash = artifactHash(run);
    const existing = this.get(run.run_id);
    if (existing && String(existing.artifact_hash) === hash) return "skipped";
    if (opts.requireParent !== false && run.parent_run_id && !this.get(run.parent_run_id)) {
      throw new Error(`missing parent_run_id ${run.parent_run_id} for ${run.run_id}`);
    }
    this.db.exec("BEGIN");
    try {
      this.db.prepare(`INSERT OR REPLACE INTO runs (
        run_id, kind, status, domain, environment_version, agent_version,
        code_sha, physics_sha, agent_sha, scenario_hash, config_hash,
        parent_run_id, failure_class, reproduction_command, seed_set, artifact_manifest, artifact_hash
      ) VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)`).run(
        run.run_id,
        run.kind,
        run.status,
        run.domain,
        run.environment_version,
        run.agent_version,
        run.code_sha,
        run.physics_sha,
        run.agent_sha,
        run.scenario_hash,
        run.config_hash,
        run.parent_run_id,
        run.failure_class,
        run.reproduction_command,
        run.seed_set ? JSON.stringify(run.seed_set) : null,
        JSON.stringify(run.artifacts),
        hash,
      );
      this.db.prepare("DELETE FROM metrics WHERE run_id = ?").run(run.run_id);
      const insM = this.db.prepare("INSERT INTO metrics (run_id, key, value, text_value) VALUES (?,?,?,?)");
      for (const [k, v] of Object.entries(run.metrics)) {
        if (typeof v === "number") insM.run(run.run_id, k, v, null);
        else insM.run(run.run_id, k, null, v === null ? null : String(v));
      }
      this.db.prepare("DELETE FROM claims WHERE run_id = ?").run(run.run_id);
      const insC = this.db.prepare(
        "INSERT INTO claims (run_id, claim_type, statement, evidence_artifact) VALUES (?,?,?,?)",
      );
      for (const c of run.claims) insC.run(run.run_id, c.type, c.statement, c.evidence_artifact);
      this.db.exec("COMMIT");
      return "inserted";
    } catch (e) {
      this.db.exec("ROLLBACK");
      throw e;
    }
  }

  ingestAll(runs: readonly RunRecord[]): number {
    let n = 0;
    for (const r of topo(runs)) {
      if (this.ingest(r) === "inserted") n += 1;
    }
    return n;
  }

  get(runId: string): Record<string, unknown> | undefined {
    const row = this.db.prepare("SELECT * FROM runs WHERE run_id = ?").get(runId) as
      | Record<string, unknown>
      | undefined;
    return row;
  }

  children(parentId: string): Record<string, unknown>[] {
    return this.db.prepare("SELECT * FROM runs WHERE parent_run_id = ?").all(parentId) as Record<string, unknown>[];
  }

  lineage(runId: string): string[] {
    const out: string[] = [];
    let id: string | null = runId;
    const seen = new Set<string>();
    while (id && !seen.has(id)) {
      seen.add(id);
      out.push(id);
      const row = this.get(id);
      id = (row?.parent_run_id as string | null) ?? null;
    }
    return out;
  }

  byClaimType(type: string): Record<string, unknown>[] {
    return this.db.prepare(
      "SELECT DISTINCT r.* FROM runs r JOIN claims c ON c.run_id = r.run_id WHERE c.claim_type = ?",
    ).all(type) as Record<string, unknown>[];
  }

  count(): number {
    const row = this.db.prepare("SELECT COUNT(*) AS n FROM runs").get() as { n: number };
    return Number(row.n);
  }

  claimCount(): number {
    const row = this.db.prepare("SELECT COUNT(*) AS n FROM claims").get() as { n: number };
    return Number(row.n);
  }
}
