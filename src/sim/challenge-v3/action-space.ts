/**
 * Challenge V3 discrete action space.
 *
 * An action sequence is pi = [(action_0, duration_0), ..., (action_N, duration_N)]
 * with
 *   action   in {coast} U {single legal thruster} U {legal thruster pair}
 *   duration in {40, 80, 120, 160, 240, 320} ms
 *
 * The action space is a property of the plant (six nozzles, at most two
 * concurrent, minimum pulse 40 ms). It is not tuned per scenario and it never
 * keys off a seed, a wall-clock fault time or a specific thruster identity.
 */
import { MAX_ACTIVE, MIN_PULSE, THRUSTERS } from "../constants";

export const DURATION_GRID_S = [0.04, 0.08, 0.12, 0.16, 0.24, 0.32] as const;
export type DurationS = (typeof DURATION_GRID_S)[number];

/** Thruster ids on for this action. Empty = coast. */
export type Action = readonly number[];

export interface Segment {
  readonly action: Action;
  readonly durationS: number;
}

export const COAST: Action = [];

function pairKey(a: number, b: number): string {
  return `${a}|${b}`;
}

/** All actions: coast, 6 singles, 15 pairs = 22. */
export const ALL_ACTIONS: Action[] = (() => {
  const out: Action[] = [COAST];
  for (let i = 0; i < THRUSTERS.length; i++) out.push([i]);
  for (let i = 0; i < THRUSTERS.length; i++) {
    for (let j = i + 1; j < THRUSTERS.length; j++) out.push([i, j]);
  }
  return out;
})();

export function actionId(a: Action): string {
  return a.length === 0 ? "coast" : a.join("+");
}

/** A failure mask is the set of thruster ids believed / known to be dead. */
export function legalActions(failed: ReadonlySet<number>): Action[] {
  return ALL_ACTIONS.filter((a) => a.every((i) => !failed.has(i)) && a.length <= MAX_ACTIVE);
}

/** Cache legal-action lists per mask so search loops do not re-allocate. */
const legalCache = new Map<string, Action[]>();
export function legalActionsCached(failed: ReadonlySet<number>): Action[] {
  const key = [...failed].sort((x, y) => x - y).join(",");
  let v = legalCache.get(key);
  if (!v) {
    v = legalActions(failed);
    legalCache.set(key, v);
  }
  return v;
}

export function isLegalSegment(seg: Segment, failed: ReadonlySet<number>): boolean {
  if (seg.action.length > MAX_ACTIVE) return false;
  if (seg.action.some((i) => failed.has(i))) return false;
  if (seg.action.length > 0 && seg.durationS < MIN_PULSE - 1e-12) return false;
  return DURATION_GRID_S.some((d) => Math.abs(d - seg.durationS) < 1e-12);
}

/** Number of 0.1 s controller ticks a segment occupies. */
export function segmentTicks(durationS: number, ctrlDt: number): number {
  return Math.max(1, Math.round(Math.ceil(durationS / ctrlDt - 1e-9)));
}

export function sequenceDurationS(seq: readonly Segment[], ctrlDt: number): number {
  let t = 0;
  for (const s of seq) t += segmentTicks(s.durationS, ctrlDt) * ctrlDt;
  return t;
}

/** Thruster-on time of a sequence (fuel proxy, before eta / mass-flow). */
export function sequenceOnTimeS(seq: readonly Segment[]): number {
  let t = 0;
  for (const s of seq) t += s.action.length * s.durationS;
  return t;
}

export function sequencePulseCount(seq: readonly Segment[]): number {
  let n = 0;
  for (const s of seq) n += s.action.length;
  return n;
}

void pairKey;
