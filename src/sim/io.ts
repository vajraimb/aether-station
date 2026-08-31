import { mkdirSync, writeFileSync } from "node:fs";
import { dirname } from "node:path";
import type { Metrics, Sample, SimEvent } from "./types";
import { eventsJsonl, trajectoryCsv } from "./scoring";

export function writeText(path: string, text: string) {
  mkdirSync(dirname(path), { recursive: true });
  writeFileSync(path, text.endsWith("\n") ? text : text + "\n", "utf8");
}

export function writeJson(path: string, value: unknown) {
  writeText(path, JSON.stringify(value, null, 2));
}

export function writeTrajectory(path: string, log: Sample[]) {
  writeText(path, trajectoryCsv(log));
}

export function writeEvents(path: string, events: SimEvent[]) {
  writeText(path, eventsJsonl(events));
}

export function writeMetrics(path: string, m: Metrics) {
  writeJson(path, m);
}
