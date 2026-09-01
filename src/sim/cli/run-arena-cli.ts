#!/usr/bin/env npx tsx
/**
 * Unified domain CLI. Does not change AgentArena interfaces.
 *
 *   npm run arena -- --domain station --agent baseline --scenario smoke
 *   npm run arena -- --domain inventory --agent reorder-point --scenario smoke
 */
import { execSync } from "node:child_process";
import { readFileSync } from "node:fs";
import { INVENTORY_RUN_ID, INVENTORY_SMOKE_PATH, runInventorySmoke } from "../../../domains/inventory/smoke.ts";
import { writeRunBundle } from "./run-bundle.ts";

function arg(flag: string, fallback = ""): string {
  const i = process.argv.indexOf(flag);
  if (i >= 0 && i + 1 < process.argv.length) return process.argv[i + 1]!;
  return fallback;
}

const domain = (arg("--domain", "station") || "station").toLowerCase();
const agent = (arg("--agent", domain === "inventory" ? "reorder-point" : "baseline") || "baseline").toLowerCase();
const scenario = (arg("--scenario", "smoke") || "smoke").toLowerCase();

if (scenario !== "smoke") {
  console.error("only --scenario smoke is wired in v0.2.0-arena");
  process.exit(2);
}

if (domain === "inventory") {
  if (agent !== "reorder-point") {
    console.error("inventory agent must be reorder-point");
    process.exit(2);
  }
  const smoke = runInventorySmoke(30);
  const dir = writeRunBundle({
    run_id: INVENTORY_RUN_ID,
    domain: "inventory",
    agent: "reorder-point",
    scenario: "smoke",
    status: smoke.allGates >= 1 ? "passed" : "failed",
    metrics: {
      allGates: smoke.allGates,
      fillSteady: smoke.rows.find((r) => r.id === "steady")?.fillRate ?? null,
      fillSpike: smoke.rows.find((r) => r.id === "demand-spike")?.fillRate ?? null,
      fillOutage: smoke.rows.find((r) => r.id === "supplier-outage")?.fillRate ?? null,
      fillDelay: smoke.rows.find((r) => r.id === "delay-anomaly")?.fillRate ?? null,
      fillBias: smoke.rows.find((r) => r.id === "forecast-bias")?.fillRate ?? null,
      fillCombo: smoke.rows.find((r) => r.id === "combined-stress")?.fillRate ?? null,
    },
    claims: [
      {
        type: "measured",
        statement: "Reorder-point inventory smoke 0/6 all-gates. Platform ran; policy is not the test.",
        evidence_artifact: INVENTORY_SMOKE_PATH,
      },
    ],
    reproduction_command: "npm run arena -- --domain inventory --agent reorder-point --scenario smoke",
    artifacts: [INVENTORY_SMOKE_PATH, `outputs/runs/${INVENTORY_RUN_ID}/metrics.json`],
  });
  console.log(`wrote ${dir} (trajectory gitignored)`);
  process.exit(0);
}

if (domain === "station" || domain === "aether") {
  const runId = "station-baseline-smoke";
  const evalOut = `outputs/runs/${runId}/eval.json`;
  execSync(`npx tsx src/sim/cli/run-eval.ts --controller ${agent} --set smoke --out ${evalOut}`, {
    stdio: "inherit",
  });
  const evalJson = JSON.parse(readFileSync(evalOut, "utf8")) as {
    rates: { allGates: number; attitude: number; fuel: number };
    pass: { allGates: boolean };
  };
  const dir = writeRunBundle({
    run_id: runId,
    domain: "station",
    agent,
    scenario: "smoke",
    status: evalJson.pass.allGates ? "passed" : "failed",
    metrics: {
      allGates: evalJson.rates.allGates,
      attitude: evalJson.rates.attitude,
      fuel: evalJson.rates.fuel,
    },
    claims: [
      {
        type: "measured",
        statement: "Station baseline smoke all-gates FAIL is expected. Task solved: NO.",
        evidence_artifact: `outputs/runs/${runId}/metrics.json`,
      },
    ],
    reproduction_command: "npm run arena -- --domain station --agent baseline --scenario smoke",
    artifacts: [`outputs/runs/${runId}/metrics.json`],
  });
  console.log(`wrote ${dir}`);
  process.exit(0);
}

console.error(`unknown domain ${domain}`);
process.exit(2);
