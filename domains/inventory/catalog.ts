import { PHYSICS_SHA, type RunRecord } from "../../src/sim/ledger/schema";

export const INVENTORY_LEDGER: readonly RunRecord[] = [
  {
    run_id: "inventory-reorder-smoke",
    kind: "benchmark_run",
    status: "failed",
    domain: "inventory",
    environment_version: "aether-station@0.1.0-benchmark",
    agent_version: "reorder-point",
    code_sha: "domain/inventory-validation",
    physics_sha: PHYSICS_SHA,
    agent_sha: "reorder-point",
    scenario_hash: "steady+spike+outage+delay+bias+combo",
    config_hash: "horizon-30",
    seed_set: [1],
    parent_run_id: "aether-benchmark-v0.1.0",
    failure_class: "online_gate_fail",
    metrics: {
      allGates: 0,
      fillSteady: 0.555,
      fillSpike: 0.452,
      fillOutage: 0.543,
      fillDelay: 0.42,
      fillBias: 0.89,
      fillCombo: 0.457,
    },
    artifacts: ["outputs/inventory/smoke.json"],
    reproduction_command: "npm run inventory:smoke",
    claims: [
      {
        type: "measured",
        statement: "Reorder-point inventory smoke 0/6 all-gates. Platform ran; policy is not the test.",
        evidence_artifact: "outputs/inventory/smoke.json",
      },
    ],
  },
];
