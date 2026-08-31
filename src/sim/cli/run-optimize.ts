#!/usr/bin/env npx tsx
import { runOfflineOptimizer } from "../optimizer.ts";

const n = Math.max(1, Number(process.argv.includes("--n") ? process.argv[process.argv.indexOf("--n") + 1] : 6));
const t0 = Date.now();
const r = runOfflineOptimizer(n);
console.log(JSON.stringify(r, null, 2));
console.log("elapsed_ms", Date.now() - t0);
