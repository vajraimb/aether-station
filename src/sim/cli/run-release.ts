#!/usr/bin/env npx tsx
/**
 * Stamp release-manifest.json and checksums.sha256.
 * Does not run a controller. Does not touch the physics kernel.
 */
import { createHash } from "node:crypto";
import { execSync } from "node:child_process";
import { existsSync, readFileSync, readdirSync, statSync } from "node:fs";
import { join, relative } from "node:path";
import { writeJson, writeText } from "../io.ts";

export const PHYSICS_SHA = "bdfff5b4c62733d7156d2f9cdeeaa75661d6c9f4";
export const FREEZE_SHA = "4b5d7a6b2450f3e4613e50a9d77d9a5dfbaf15e4";
export const RELEASE_ID = "v0.2.0-arena";

const ROOT = join(import.meta.dirname, "../../..");

function git(cmd: string): string {
  try {
    return execSync(cmd, { encoding: "utf8", cwd: ROOT }).trim();
  } catch {
    return "unknown";
  }
}

function sha256File(path: string): string {
  const h = createHash("sha256");
  h.update(readFileSync(path));
  return h.digest("hex");
}

function walk(dir: string, acc: string[] = []): string[] {
  if (!existsSync(dir)) return acc;
  for (const name of readdirSync(dir)) {
    const p = join(dir, name);
    const st = statSync(p);
    if (st.isDirectory()) walk(p, acc);
    else acc.push(p);
  }
  return acc;
}

const ARTIFACT_DIRS = ["docs/release", "docs/research-phase.md", "docs/benchmark.md", "docs/cross-domain.md", "outputs/ARTIFACTS.md"];
const ARTIFACT_GLOBS = [
  "outputs/eval-baseline-train10.json",
  "outputs/eval-v2-train10.json",
  "outputs/eval-ablation-train10.json",
  "outputs/conservation.json",
  "outputs/reachability.json",
  "outputs/v2-failure-traces/summary.json",
  "outputs/inventory/smoke.json",
  "outputs/runs/station-baseline-smoke/manifest.json",
  "outputs/runs/station-baseline-smoke/metrics.json",
  "outputs/runs/station-baseline-smoke/claims.json",
  "outputs/runs/inventory-reorder-smoke/manifest.json",
  "outputs/runs/inventory-reorder-smoke/metrics.json",
  "outputs/runs/inventory-reorder-smoke/claims.json",
  "src/sim/core.ts",
  "src/sim/arena.ts",
  "src/sim/math3d.ts",
  "src/sim/dynamics.ts",
  "src/sim/audit.ts",
  "packages/agent-arena/src/index.ts",
];

function collectFiles(): string[] {
  const out: string[] = [];
  for (const rel of ARTIFACT_DIRS) {
    const p = join(ROOT, rel);
    if (!existsSync(p)) continue;
    if (statSync(p).isDirectory()) out.push(...walk(p));
    else out.push(p);
  }
  for (const rel of ARTIFACT_GLOBS) {
    const p = join(ROOT, rel);
    if (existsSync(p)) out.push(p);
  }
  return [...new Set(out)].sort();
}

const files = collectFiles();
const checksums = files.map((abs) => {
  const rel = relative(ROOT, abs).replaceAll("\\", "/");
  return { path: rel, sha256: sha256File(abs), bytes: statSync(abs).size };
});

const kernelDiff = git(`git diff --name-only ${PHYSICS_SHA} -- src/sim/math3d.ts src/sim/dynamics.ts src/sim/audit.ts`);

const manifest = {
  release: RELEASE_ID,
  kind: "benchmark_release",
  generatedAt: new Date().toISOString(),
  codeSha: git("git rev-parse HEAD"),
  branch: git("git rev-parse --abbrev-ref HEAD"),
  freezeSha: FREEZE_SHA,
  physicsSha: PHYSICS_SHA,
  physicsKernelUnchanged: kernelDiff === "",
  tags: {
    "v0.2.0-arena": "this release",
    "v0.1.0-benchmark": "486099c2f7378bd3bea3fca9e7e9296b3bcb4e4e",
    "benchmark/research-phase-complete-v1": FREEZE_SHA,
    "archive/control-v2-final": "0d871b4e45dd918edbce08b02d234b981aadf3a1",
    "archive/action-macro-belief-audit": "7c3eca07c6f3f91386d302d806c3e39d6348109e",
    "archive/wrench-nullspace-study": "38d2c741ebad6cb3ae6443337fd0678609b2c6bc",
    "archive/robust-terminal-study": "3b6b5daa9a0de9719a7eaf191c894fbc2be6d5d3",
  },
  card: {
    physicsValidity: "PASS",
    benchmarkInfrastructure: "PASS",
    agentArena: "PASS",
    crossDomainValidation: "PASS",
    baselineControl: "FAIL",
    inventoryPolicy: "FAIL",
    taskSolved: "NO",
    simCoreExtraction: "DEFERRED",
  },
  claims: [
    {
      type: "measured",
      statement: "Physics kernel matches bdfff5b (math3d/dynamics/audit).",
      evidence_artifact: "outputs/conservation.json",
    },
    {
      type: "measured",
      statement: "Baseline train-10 all-gates 0/10.",
      evidence_artifact: "outputs/eval-baseline-train10.json",
    },
    {
      type: "measured",
      statement: "discrete-pulse-v2 / kNN-value train-10 all-gates 0/10. kNN STOPPED.",
      evidence_artifact: "outputs/eval-v2-train10.json",
    },
    {
      type: "search_unreached",
      statement: "Robust terminal cancellation 6/42 nominal capture. Not a proof of impossibility.",
      evidence_artifact: "outputs/robust-terminal-study.json",
    },
    {
      type: "measured",
      statement: "Inventory reorder-point smoke 0/6 all-gates. Cross-domain infrastructure PASS.",
      evidence_artifact: "outputs/inventory/smoke.json",
    },
    {
      type: "measured",
      statement: "AgentArena core files unchanged vs acb6d8f.",
      evidence_artifact: "docs/cross-domain.md",
    },
  ],
  artifacts: checksums,
};

const sumLines = checksums.map((c) => `${c.sha256}  ${c.path}`).join("\n");
writeJson(join(ROOT, "outputs/release/release-manifest.json"), manifest);
writeText(join(ROOT, "outputs/release/checksums.sha256"), sumLines);
console.log(`release ${RELEASE_ID} files=${checksums.length} kernel=${manifest.physicsKernelUnchanged ? "PASS" : "FAIL"}`);
console.log("wrote outputs/release/release-manifest.json");
console.log("wrote outputs/release/checksums.sha256");
if (!manifest.physicsKernelUnchanged) process.exit(1);
