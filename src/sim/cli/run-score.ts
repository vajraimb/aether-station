#!/usr/bin/env npx tsx
/**
 * File-only scorer. Does not import Simulator or Controller.
 *   npm run score -- outputs/trajectory.csv outputs/events.jsonl
 */
import { readFileSync } from "node:fs";
import { parseEventsJsonl, parseTrajectoryCsv, scoreFromLog } from "../scoring.ts";
import { writeMetrics } from "../io.ts";

const trajPath = process.argv[2] ?? "outputs/trajectory.csv";
const evPath = process.argv[3] ?? "outputs/events.jsonl";
const outPath = process.argv[4] ?? "outputs/recomputed-metrics.json";

const traj = readFileSync(trajPath, "utf8");
const ev = readFileSync(evPath, "utf8");
const log = parseTrajectoryCsv(traj);
const events = parseEventsJsonl(ev);
const m = scoreFromLog(log, events);
writeMetrics(outPath, m);
console.log(JSON.stringify({
  source: { trajectory: trajPath, events: evPath },
  samples: log.length,
  events: events.length,
  scorecard: m.scorecard,
  fdir: {
    faultInjectionTime: m.faultInjectionTime,
    abnormalFlagTime: m.abnormalFlagTime,
    detectionTime: m.detectionTime,
    isolationTime: m.isolationTime,
    detectionDelay: m.detectionDelay,
    isolationDelay: m.isolationDelay,
    isolatedThrusterId: m.isolatedThrusterId,
    confidence: m.confidence,
  },
}, null, 2));
