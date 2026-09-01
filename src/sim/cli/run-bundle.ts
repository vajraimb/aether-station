import { existsSync } from "node:fs";
import { join } from "node:path";
import { writeJson } from "../io.ts";
import type { Claim } from "../ledger/schema.ts";

export interface RunBundle {
  run_id: string;
  domain: string;
  agent: string;
  scenario: string;
  status: "passed" | "failed";
  metrics: Record<string, number | string | boolean | null>;
  claims: readonly Claim[];
  reproduction_command: string;
  artifacts: readonly string[];
}

export function runDir(runId: string): string {
  return join("outputs/runs", runId);
}

export function writeRunBundle(bundle: RunBundle): string {
  const dir = runDir(bundle.run_id);
  const manifest = {
    run_id: bundle.run_id,
    domain: bundle.domain,
    agent: bundle.agent,
    scenario: bundle.scenario,
    status: bundle.status,
    reproduction_command: bundle.reproduction_command,
    artifacts: bundle.artifacts,
  };
  writeJson(join(dir, "manifest.json"), manifest);
  writeJson(join(dir, "metrics.json"), bundle.metrics);
  writeJson(join(dir, "claims.json"), bundle.claims);
  return dir;
}

export function committedRunFiles(runId: string): string[] {
  const dir = runDir(runId);
  return ["manifest.json", "metrics.json", "claims.json"].map((f) => join(dir, f)).filter(existsSync);
}
