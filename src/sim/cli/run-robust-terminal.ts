#!/usr/bin/env npx tsx
/**
 * Low-rate η_T-robust terminal cancellation study.
 *
 *   npx tsx src/sim/cli/run-robust-terminal.ts
 *   npx tsx src/sim/cli/run-robust-terminal.ts --quick
 *
 * Does not retune or wire a controller. Does not run train-50 / hidden.
 */
import { execSync } from "node:child_process";
import { defaultPublicConfig } from "../constants.ts";
import { runRobustTerminalStudy } from "../control/robust-terminal.ts";
import { writeJson } from "../io.ts";

function hasFlag(flag: string): boolean {
  return process.argv.includes(flag);
}

function arg(flag: string, fallback?: string): string {
  const i = process.argv.indexOf(flag);
  if (i >= 0 && i + 1 < process.argv.length) return process.argv[i + 1]!;
  return fallback ?? "";
}

function gitSha(): string {
  try {
    return execSync("git rev-parse HEAD", { encoding: "utf8" }).trim();
  } catch {
    return "unknown";
  }
}

const quick = hasFlag("--quick");
const outPath = arg("--out", quick ? "outputs/robust-terminal-study.quick.json" : "outputs/robust-terminal-study.json");
const maxSegRaw = arg("--max-seg");
const beamRaw = arg("--beam");
const plant = defaultPublicConfig();
const t0 = Date.now();
const study = runRobustTerminalStudy(plant, {
  quick,
  maxSeg: maxSegRaw ? Number(maxSegRaw) : undefined,
  beamWidth: beamRaw ? Number(beamRaw) : undefined,
});

const payload = {
  commitSha: gitSha(),
  elapsed_ms: Date.now() - t0,
  timestamp: new Date().toISOString(),
  wiredToController: false,
  knnValueLine: "STOPPED",
  physicsBaselineSha: "bdfff5b4c62733d7156d2f9cdeeaa75661d6c9f4",
  spec: {
    rateCap: 0.015,
    attCapDeg: 15,
    maxSeg: study.maxSeg,
    beamWidth: study.beamWidth,
    etaInterval: study.interval,
    etaGrid: study.grid,
    weights: study.weights,
    peakWPerpLimit: 0.03,
    fuelFloor: 2.8,
    attGateDeg: 1,
    rateGate: 0.008,
    nStates: study.nStates,
  },
  byMethod: study.byMethod,
  gates: study.gates,
  results: study.results.map((r) => ({
    stateId: r.stateId,
    family: r.family,
    isolated: r.isolated,
    att0: r.att0,
    omega0: r.omega0,
    methods: Object.fromEntries(
      Object.entries(r.methods).map(([k, v]) => [
        k,
        {
          nSeg: v.nSeg,
          ids: v.ids,
          nominal: v.nominal,
          p10: v.p10,
          worst: v.worst,
        },
      ]),
    ),
  })),
  notes:
    "Low-rate (‖ω‖≤0.015, e_q≤15°) post-failure terminal cancellation. Worst-case over estimator 2σ η_T interval clipped to etaRange. Not wired to DiscretePulseV2Controller.",
  status: {
    physics: "PASS",
    onlineControl: "FAIL",
    overall: study.gates.overall80 && study.gates.perMask70 && study.gates.worstCaseFuel && study.gates.beatsTerminal ? "UNPROVEN" : "FAIL",
    readyToWire: false,
  },
};

writeJson(outPath, payload);
console.log(`wrote ${outPath} in ${payload.elapsed_ms} ms  n=${study.nStates} maxSeg=${study.maxSeg} beam=${study.beamWidth}`);
for (const [k, v] of Object.entries(study.byMethod)) {
  console.log(
    `method ${k} nomCap=${v.nominalCaptureRate.toFixed(2)} worstCap=${v.worstCaptureRate.toFixed(2)} fuelHold=${v.fuelHoldWorstRate.toFixed(2)} medJnom=${v.medianJnom.toFixed(2)} medJworst=${v.medianJworst.toFixed(2)}`,
  );
}
console.log(
  `gates overall80=${study.gates.overall80} perMask70=${study.gates.perMask70} worstFuel=${study.gates.worstCaseFuel} beatsTerminal=${study.gates.beatsTerminal} readyToWire=${study.gates.readyToWire}`,
);
