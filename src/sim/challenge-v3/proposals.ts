/**
 * Action-sequence proposal generator (spec section 11).
 *
 * A proposal is a *candidate* action sequence, never the answer: every
 * proposal is scored by rolling it out on the plant (or on a parameter
 * ensemble) against the lexicographic objective, and the optimizers in
 * `optimizers.ts` are free to reject or rewrite all of them.
 *
 * Mechanisms used here, all explicitly allowed by the spec:
 *   - signed angular-impulse target: pick a desired delta-omega, then solve for
 *     the nozzle on-times that deliver it;
 *   - exact minimum-fuel allocation: min sum(x) s.t. A x = delta-omega, x >= 0
 *     is a linear program whose optimum sits at a basic feasible solution with
 *     at most three non-zeros, so enumerating the <= C(6,3) bases solves it
 *     exactly instead of heuristically;
 *   - mixed-integer pulse schedule: on-times are then snapped onto the 40 ms
 *     quantum and decomposed into legal duration-grid segments.
 *
 * Nothing here contains a gain that was tuned by hand against a scenario, and
 * nothing branches on seed, mission time, axis or thruster id.
 */
import {
  attitudeErrorVector,
  mv,
  solve3,
  vadd,
  vnorm,
  vscale,
  vsub,
  type Mat3,
  type Quat,
  type Vec3,
} from "../math3d";
import { DURATION_GRID_S, segmentTicks, type Action, type Segment } from "./action-space";
import type { SurrogateModel } from "./surrogate";

/** 40 ms quantum expressed in duration-grid units. */
const Q = 0.04;
/** Duration grid in quanta, descending, used for the segment decomposition. */
const GRID_Q = [8, 6, 4, 3, 2, 1] as const;

export interface Allocation {
  /** On-time in seconds per nozzle id (0 = unused). */
  onTime: number[];
  /** Total on-time, i.e. the fuel-proportional cost. */
  total: number;
  /** Residual delta-omega not delivered by this allocation. */
  residual: Vec3;
}

/**
 * Exact minimum-total-on-time allocation of `dw` onto the live nozzles.
 * Enumerates every basis of size 1..3 over the live columns of
 * `B = Iinv * tau` and keeps the cheapest non-negative exact solution; falls
 * back to the least-squares basis with the smallest residual when `dw` is not
 * reachable (e.g. after a failure the sign structure can block a direction).
 */
/** The 40 ms actuator quantum, seconds. */
const MIN_PULSE_S = 0.04;

export function allocateDeltaOmega(
  model: SurrogateModel,
  dw: Vec3,
  live: readonly number[],
): Allocation {
  const B: Vec3[] = model.cols.map((c) => mv(model.Iinv, c));
  const zero = (): number[] => [0, 0, 0, 0, 0, 0];
  if (vnorm(dw) < 1e-12) return { onTime: zero(), total: 0, residual: [0, 0, 0] };

  let best: Allocation | null = null;
  let bestInexact: Allocation | null = null;
  const consider = (ids: number[], xs: number[]) => {
    const onTime = zero();
    let total = 0;
    for (let k = 0; k < ids.length; k++) {
      const x = xs[k]!;
      if (x < -1e-12) return;
      onTime[ids[k]!] = Math.max(0, x);
      total += Math.max(0, x);
    }
    let got: Vec3 = [0, 0, 0];
    for (let i = 0; i < 6; i++) if (onTime[i]! > 0) got = vadd(got, vscale(B[i]!, onTime[i]!));
    const residual = vsub(dw, got);
    const cand: Allocation = { onTime, total, residual };
    if (vnorm(residual) < 1e-10 * Math.max(1, vnorm(dw))) {
      if (!best || total < best.total - 1e-12) best = cand;
    } else if (
      !bestInexact ||
      vnorm(residual) < vnorm(bestInexact.residual) - 1e-15 ||
      (Math.abs(vnorm(residual) - vnorm(bestInexact.residual)) <= 1e-15 && total < bestInexact.total)
    ) {
      bestInexact = cand;
    }
  };

  const n = live.length;
  // Bases of size 3 give exact solutions when the columns are independent.
  for (let a = 0; a < n; a++) {
    for (let b = a + 1; b < n; b++) {
      for (let c = b + 1; c < n; c++) {
        const ids = [live[a]!, live[b]!, live[c]!];
        const A: Mat3 = [
          [B[ids[0]!]![0], B[ids[1]!]![0], B[ids[2]!]![0]],
          [B[ids[0]!]![1], B[ids[1]!]![1], B[ids[2]!]![1]],
          [B[ids[0]!]![2], B[ids[1]!]![2], B[ids[2]!]![2]],
        ];
        const det =
          A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
          A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
          A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
        if (Math.abs(det) < 1e-14) continue;
        consider(ids, solve3(A, dw));
      }
    }
  }
  // Sizes 1 and 2: least-squares, used only as inexact fallbacks.
  for (let a = 0; a < n; a++) {
    const i = live[a]!;
    const d = B[i]![0] * B[i]![0] + B[i]![1] * B[i]![1] + B[i]![2] * B[i]![2];
    if (d > 1e-18) consider([i], [(B[i]![0] * dw[0] + B[i]![1] * dw[1] + B[i]![2] * dw[2]) / d]);
    for (let b = a + 1; b < n; b++) {
      const j = live[b]!;
      const g11 = d;
      const g22 = B[j]![0] ** 2 + B[j]![1] ** 2 + B[j]![2] ** 2;
      const g12 = B[i]![0] * B[j]![0] + B[i]![1] * B[j]![1] + B[i]![2] * B[j]![2];
      const r1 = B[i]![0] * dw[0] + B[i]![1] * dw[1] + B[i]![2] * dw[2];
      const r2 = B[j]![0] * dw[0] + B[j]![1] * dw[1] + B[j]![2] * dw[2];
      const dd = g11 * g22 - g12 * g12;
      if (Math.abs(dd) < 1e-18) continue;
      consider([i, j], [(r1 * g22 - r2 * g12) / dd, (r2 * g11 - r1 * g12) / dd]);
    }
  }
  return best ?? bestInexact ?? { onTime: zero(), total: 0, residual: dw };
}

/**
 * Snap an allocation onto the 40 ms quantum and decompose it into legal
 * duration-grid segments. `rounding` selects the integer relaxation: nearest,
 * floor (never over-shoot) or ceil (never under-shoot).
 */
export function scheduleAllocation(
  alloc: Allocation,
  rounding: "nearest" | "floor" | "ceil",
  pairUp: boolean,
  maxSegments: number,
): Segment[] {
  const quanta: Array<{ id: number; n: number }> = [];
  for (let i = 0; i < 6; i++) {
    const x = alloc.onTime[i] ?? 0;
    if (x <= 0) continue;
    const raw = x / Q;
    let n: number;
    if (rounding === "floor") n = Math.floor(raw + 1e-9);
    else if (rounding === "ceil") n = Math.ceil(raw - 1e-9);
    else n = Math.round(raw);
    if (n >= 1) quanta.push({ id: i, n });
  }
  // Stable, id-independent ordering: longest burn first, id as tie-break.
  quanta.sort((a, b) => b.n - a.n || a.id - b.id);

  const chunks: Array<{ id: number; d: number }> = [];
  for (const q of quanta) {
    let left = q.n;
    while (left > 0) {
      let take = 1;
      for (const g of GRID_Q) {
        if (g <= left) {
          take = g;
          break;
        }
      }
      chunks.push({ id: q.id, d: take * Q });
      left -= take;
    }
  }

  const segs: Segment[] = [];
  if (pairUp) {
    // Fire two nozzles concurrently when their chunk durations match, which
    // halves the mission time spent on the same angular impulse.
    const used = new Array(chunks.length).fill(false);
    for (let i = 0; i < chunks.length; i++) {
      if (used[i]) continue;
      let partner = -1;
      for (let j = i + 1; j < chunks.length; j++) {
        if (used[j]) continue;
        if (chunks[j]!.id !== chunks[i]!.id && Math.abs(chunks[j]!.d - chunks[i]!.d) < 1e-12) {
          partner = j;
          break;
        }
      }
      used[i] = true;
      if (partner >= 0) {
        used[partner] = true;
        const action: Action = [chunks[i]!.id, chunks[partner]!.id].sort((a, b) => a - b);
        segs.push({ action, durationS: chunks[i]!.d });
      } else {
        segs.push({ action: [chunks[i]!.id], durationS: chunks[i]!.d });
      }
      if (segs.length >= maxSegments) break;
    }
  } else {
    for (const c of chunks) {
      segs.push({ action: [c.id], durationS: c.d });
      if (segs.length >= maxSegments) break;
    }
  }
  return segs;
}

export interface ProposalTarget {
  label: string;
  dw: Vec3;
}

/**
 * Deterministic grid of signed angular-impulse targets: partial and full
 * detumble, plus eigen-axis slew rates sized by a grid of arrival times, plus
 * the terminal-targeting rate that makes the free drift arrive at the target
 * attitude exactly at `tFinal`.
 */
export function impulseTargets(
  q: Quat,
  w: Vec3,
  qTarget: Quat,
  tNow: number,
  tFinal: number,
): ProposalTarget[] {
  const out: ProposalTarget[] = [];
  for (const a of [0.35, 0.7, 1.0]) {
    out.push({ label: `detumble:${a}`, dw: vscale(w, -a) });
  }
  const e = attitudeErrorVector(q, qTarget);
  const eAng = vscale(e, 2); // body-frame error-angle vector, small-angle exact
  const remaining = Math.max(1e-3, tFinal - tNow);
  for (const frac of [0.25, 0.4, 0.6, 0.85, 1.0]) {
    const T = Math.max(2, remaining * frac);
    const wRef = vscale(eAng, -1 / T);
    out.push({ label: `slew:T=${T.toFixed(1)}`, dw: vsub(wRef, w) });
  }
  out.push({ label: "coast", dw: [0, 0, 0] });
  return out;
}

/** Emit a pure-coast stretch on the legal duration grid. */
export function coastSegments(durationS: number, ctrlDt: number): Segment[] {
  const out: Segment[] = [];
  let left = Math.round(durationS / ctrlDt) * ctrlDt;
  const big = DURATION_GRID_S[DURATION_GRID_S.length - 1]!;
  while (left > 1e-9) {
    let d = big;
    if (left < big) {
      d = DURATION_GRID_S[0]!;
      for (const g of DURATION_GRID_S) if (g <= left + 1e-9) d = g;
    }
    out.push({ action: [], durationS: d });
    left -= Math.ceil(d / ctrlDt - 1e-9) * ctrlDt;
  }
  return out;
}

/**
 * Rest-to-rest capture manoeuvres: accelerate onto an eigen-axis slew rate,
 * coast, then null the rate so that the station *arrives* at the target
 * attitude with zero residual rate at a chosen epoch.
 *
 * This is the structure the single-burn impulse targets cannot express, and it
 * is what turns a "timed fly-by" of the target attitude into an actual capture.
 * The manoeuvre time is solved by two fixed-point passes over the burn
 * durations; the exact outcome is always decided by the full roll-out.
 */
export function restToRestProposals(
  model: SurrogateModel,
  q: Quat,
  w: Vec3,
  qTarget: Quat,
  tNow: number,
  tFinal: number,
  live: readonly number[],
  ctrlDt: number,
  maxSegments: number,
): Array<{ label: string; seq: Segment[] }> {
  const out: Array<{ label: string; seq: Segment[] }> = [];
  const eAng = vscale(attitudeErrorVector(q, qTarget), 2);
  const remaining = tFinal - tNow;
  if (remaining <= 1) return out;
  const arrivals = new Set<number>();
  for (const back of [0.5, 2, 5, 12, 25, 40]) {
    const T = remaining - back;
    if (T > 1) arrivals.add(Math.round(T * 10) / 10);
  }
  for (const frac of [0.2, 0.35, 0.5, 0.7]) {
    const T = remaining * frac;
    if (T > 1) arrivals.add(Math.round(T * 10) / 10);
  }
  for (const T of [...arrivals].sort((a, b) => a - b)) {
    let t1 = 0;
    let t2 = 0;
    let wRef: Vec3 = [0, 0, 0];
    let a1 = allocateDeltaOmega(model, [0, 0, 0], live);
    let a2 = a1;
    let ok = false;
    for (let iter = 0; iter < 3; iter++) {
      // Rotation accumulated over the manoeuvre, counting half of each burn.
      const eff = Math.max(0.2, T - t1 / 2 - t2 / 2);
      wRef = vscale(eAng, -1 / eff);
      a1 = allocateDeltaOmega(model, vsub(wRef, w), live);
      a2 = allocateDeltaOmega(model, vscale(wRef, -1), live);
      if (!Number.isFinite(a1.total) || !Number.isFinite(a2.total)) break;
      t1 = a1.total / 0.8;
      t2 = a2.total / 0.8;
      if (t1 + t2 > T) break;
      ok = true;
    }
    if (!ok || t1 + t2 > T) continue;
    for (const pairUp of [false, true]) {
      const s1 = scheduleAllocation(a1, "nearest", pairUp, maxSegments);
      const s2 = scheduleAllocation(a2, "nearest", pairUp, maxSegments);
      const used1 = s1.reduce((acc, x) => acc + Math.ceil(x.durationS / ctrlDt - 1e-9) * ctrlDt, 0);
      const used2 = s2.reduce((acc, x) => acc + Math.ceil(x.durationS / ctrlDt - 1e-9) * ctrlDt, 0);
      const coastT = T - used1 - used2;
      if (coastT < 0) continue;
      const seq = [...s1, ...coastSegments(coastT, ctrlDt), ...s2];
      if (seq.length > 0) out.push({ label: `r2r:T=${T.toFixed(1)}${pairUp ? "/pair" : ""}`, seq });
    }
  }
  return out;
}

/** Full deterministic proposal set for one replanning epoch. */
export function generateProposals(
  model: SurrogateModel,
  q: Quat,
  w: Vec3,
  qTarget: Quat,
  tNow: number,
  tFinal: number,
  live: readonly number[],
  maxSegments: number,
  ctrlDt = 0.1,
): Array<{ label: string; seq: Segment[] }> {
  const out: Array<{ label: string; seq: Segment[] }> = [];
  const seen = new Set<string>();
  const push = (label: string, seq: Segment[]) => {
    const key = seq.map((s) => `${s.action.join("+")}@${s.durationS}`).join(",");
    if (seen.has(key)) return;
    seen.add(key);
    out.push({ label, seq });
  };
  push("coast", []);
  for (const tgt of impulseTargets(q, w, qTarget, tNow, tFinal)) {
    if (vnorm(tgt.dw) < 1e-9) continue;
    const alloc = allocateDeltaOmega(model, tgt.dw, live);
    if (alloc.total <= 0) continue;
    for (const rounding of ["nearest", "floor", "ceil"] as const) {
      for (const pairUp of [false, true]) {
        const seq = scheduleAllocation(alloc, rounding, pairUp, maxSegments);
        if (seq.length > 0) push(`${tgt.label}/${rounding}${pairUp ? "/pair" : ""}`, seq);
      }
    }
  }
  for (const p of restToRestProposals(
    model,
    q,
    w,
    qTarget,
    tNow,
    tFinal,
    live,
    ctrlDt,
    maxSegments,
  )) {
    push(p.label, p.seq);
  }
  // Single-primitive probes keep the proposal set spanning even when the
  // impulse solver is blocked by the failure mask.
  for (const id of live) {
    for (const d of DURATION_GRID_S) {
      push(`primitive:${id}@${d}`, [{ action: [id], durationS: d }]);
    }
  }
  return out;
}

/**
 * Explicit search over the commandable time quantisation (follow-up item C).
 *
 * The commandable grid is not a single number: pulse *start* times live on the
 * 100 ms controller tick, pulse *widths* live on the 40 ms quantum, and the
 * 120 ms command delay shifts every burn by a fixed amount. Arrival time,
 * braking start, dwell start and coast duration are therefore all controlled
 * by two integers - how many idle ticks precede the terminal burn group, and
 * how many 40 ms quanta wide each of its pulses is - and this function
 * enumerates both explicitly instead of accepting whichever arrival a
 * generator happened to propose.
 *
 * The enumeration is fixed and ordered, so it is deterministic.
 */
export function slotVariants(
  seq: readonly Segment[],
  ctrlDt: number,
  tickShifts: readonly number[] = [-8, -6, -4, -3, -2, -1, 1, 2, 3, 4, 6, 8],
  quantaShifts: readonly number[] = [-2, -1, 1, 2],
): Array<{ label: string; seq: Segment[] }> {
  const out: Array<{ label: string; seq: Segment[] }> = [];
  if (seq.length === 0) return out;
  // Locate the terminal burn group: the last maximal run of firing segments.
  let end = seq.length - 1;
  while (end >= 0 && seq[end]!.action.length === 0) end -= 1;
  if (end < 0) return out;
  let begin = end;
  while (begin > 0 && seq[begin - 1]!.action.length > 0) begin -= 1;

  const head = seq.slice(0, begin);
  const group = seq.slice(begin, end + 1);
  const tail = seq.slice(end + 1);
  // Idle ticks immediately before the terminal group, which is what sets the
  // arrival slot and, with it, the dwell start.
  let leadIdle = 0;
  for (let i = head.length - 1; i >= 0 && head[i]!.action.length === 0; i--) {
    leadIdle += segmentTicks(head[i]!.durationS, ctrlDt);
  }

  // Arrival / braking-start / coast-duration slots: move the terminal group by
  // whole controller ticks by re-timing the idle stretch in front of it.
  for (const k of tickShifts) {
    const ticks = leadIdle + k;
    if (ticks < 0) continue;
    let cut = head.length;
    let removed = 0;
    while (cut > 0 && head[cut - 1]!.action.length === 0) {
      removed += segmentTicks(head[cut - 1]!.durationS, ctrlDt);
      cut -= 1;
    }
    void removed;
    const rebuilt: Segment[] = [...head.slice(0, cut)];
    if (ticks > 0) rebuilt.push(...coastSegments(ticks * ctrlDt, ctrlDt));
    out.push({
      label: `slot:tick${k >= 0 ? "+" : ""}${k}`,
      seq: [...rebuilt, ...group, ...tail],
    });
  }

  // Pulse phase / width: widen or narrow every pulse of the terminal group by
  // whole 40 ms quanta. This is the only sub-tick authority the actuator has.
  for (const q of quantaShifts) {
    const scaled: Segment[] = group.map((s) => {
      if (s.action.length === 0) return s;
      const quanta = Math.round(s.durationS / MIN_PULSE_S) + q;
      if (quanta < 1) return { action: s.action, durationS: MIN_PULSE_S };
      return { action: s.action, durationS: quanta * MIN_PULSE_S };
    });
    out.push({
      label: `slot:quanta${q >= 0 ? "+" : ""}${q}`,
      seq: [...head, ...scaled, ...tail],
    });
  }
  return out;
}
