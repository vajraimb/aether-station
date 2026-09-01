import { existsSync, readFileSync, rmSync } from "node:fs";
import { join } from "node:path";
import { writeRunBundle } from "./run-bundle";

interface T {
  name: string;
  pass: boolean;
  detail: string;
}

function check(name: string, pass: boolean, detail: string, out: T[]): void {
  out.push({ name, pass, detail });
}

export function runBundleTests(): T[] {
  const out: T[] = [];
  const runId = "test-bundle-tmp";
  const dir = writeRunBundle({
    run_id: runId,
    domain: "inventory",
    agent: "reorder-point",
    scenario: "smoke",
    status: "failed",
    metrics: { allGates: 0 },
    claims: [{ type: "measured", statement: "tmp", evidence_artifact: "outputs/inventory/smoke.json" }],
    reproduction_command: "npm run arena -- --domain inventory --agent reorder-point --scenario smoke",
    artifacts: ["outputs/inventory/smoke.json"],
  });
  check("bundle_dir", dir === join("outputs/runs", runId), dir, out);
  check("bundle_manifest", existsSync(join(dir, "manifest.json")), "manifest.json", out);
  check("bundle_metrics", existsSync(join(dir, "metrics.json")), "metrics.json", out);
  check("bundle_claims", existsSync(join(dir, "claims.json")), "claims.json", out);
  const man = JSON.parse(readFileSync(join(dir, "manifest.json"), "utf8")) as { run_id: string };
  check("bundle_run_id", man.run_id === runId, man.run_id, out);
  rmSync(dir, { recursive: true, force: true });
  return out;
}
