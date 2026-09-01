/**
 * Two action-sequence optimizers over the discrete decision variable
 *   pi = [(action_0, duration_0), ..., (action_{L-1}, duration_{L-1})],
 * action in {coast} U {legal single nozzles} U {legal nozzle pairs},
 * duration in {40, 80, 120, 160, 240, 320} ms  (spec section 6).
 *
 *   - `branchAndBound`: deterministic best-first branch-and-bound. Fixed node
 *     expansion budget, stable tie-breaks, and a sound prune on the fuel floor
 *     using the braking on-time lower bound. No wall-clock deadline anywhere.
 *   - `cem`: stochastic population-based cross-entropy method over the
 *     per-slot categorical distribution. Fixed population, generations and
 *     elite count, seeded RNG stream.
 *
 * Both optimize the *same* objective, evaluated by rolling the candidate out
 * on the plant (or on a parameter ensemble) - not a surrogate scalar - so the
 * comparison in `truth-optimizer-train10.json` is apples to apples.
 */
import { makeRng } from "../math3d";
import { lexCompare, type TerminalEval } from "./objective";
import type { Action, Segment } from "./action-space";

export interface SeqScore {
  cost: number;
  key: number[];
  term: TerminalEval;
  /** The scored sequence after auto-completion, if the evaluator appends one. */
  full?: Segment[];
}

export type SeqEvaluator = (seq: readonly Segment[]) => SeqScore;

export interface OptimizerResult {
  seq: Segment[];
  score: SeqScore;
  /** Number of plant roll-outs consumed. */
  rollouts: number;
  /** Number of nodes expanded / candidates sampled. */
  iterations: number;
  optimizer: string;
}

export interface ExpansionSet {
  actions: Action[];
  durations: number[];
}

function seqKey(seq: readonly Segment[]): string {
  return seq.map((s) => `${s.action.join("+")}@${s.durationS.toFixed(3)}`).join(",");
}

/**
 * Deterministic best-first branch-and-bound.
 *
 * Nodes are partial sequences; the score of a node is the objective of the
 * sequence completed by coasting to the end of the mission, which is a real
 * feasible completion, so every node score is an achievable value and the
 * incumbent is monotone. `pruneFuel` rejects nodes that cannot respect the
 * fuel floor even under the optimistic braking bound.
 */
export function branchAndBound(
  evaluate: SeqEvaluator,
  seeds: ReadonlyArray<readonly Segment[]>,
  expansion: ExpansionSet,
  opts: {
    maxDepth: number;
    nodeExpansions: number;
    beamWidth: number;
    /** Optional sound prune: node -> true when it cannot be completed legally. */
    prune?: (seq: readonly Segment[], score: SeqScore) => boolean;
  },
): OptimizerResult {
  interface Node {
    seq: Segment[];
    score: SeqScore;
    order: number;
  }
  let rollouts = 0;
  let order = 0;
  const seen = new Set<string>();
  const evalNode = (seq: Segment[]): Node => {
    rollouts += 1;
    return { seq, score: evaluate(seq), order: order++ };
  };

  const cmp = (a: Node, b: Node) => {
    const c = lexCompare(a.score.key, b.score.key);
    if (c !== 0) return c;
    if (a.seq.length !== b.seq.length) return a.seq.length - b.seq.length;
    return a.order - b.order;
  };

  let frontier: Node[] = [];
  const root = evalNode([]);
  seen.add(seqKey([]));
  frontier.push(root);
  let best = root;

  for (const s of seeds) {
    const k = seqKey(s);
    if (seen.has(k)) continue;
    seen.add(k);
    const n = evalNode([...s]);
    frontier.push(n);
    if (cmp(n, best) < 0) best = n;
  }

  let expansions = 0;
  while (expansions < opts.nodeExpansions && frontier.length > 0) {
    frontier.sort(cmp);
    if (frontier.length > opts.beamWidth) frontier = frontier.slice(0, opts.beamWidth);
    const node = frontier.shift()!;
    if (node.seq.length >= opts.maxDepth) continue;
    expansions += 1;
    for (const action of expansion.actions) {
      const durations = action.length === 0 ? [expansion.durations[0]!] : expansion.durations;
      for (const d of durations) {
        const seq = [...node.seq, { action, durationS: d }];
        const k = seqKey(seq);
        if (seen.has(k)) continue;
        seen.add(k);
        const child = evalNode(seq);
        if (opts.prune?.(seq, child.score)) continue;
        frontier.push(child);
        if (cmp(child, best) < 0) best = child;
      }
    }
  }

  return {
    seq: best.seq,
    score: best.score,
    rollouts,
    iterations: expansions,
    optimizer: "branch-and-bound",
  };
}

/**
 * Cross-entropy method over per-slot categorical distributions.
 * Fixed population, generations, elite count and RNG seed: the result is a
 * deterministic function of its inputs.
 */
export function cem(
  evaluate: SeqEvaluator,
  expansion: ExpansionSet,
  opts: {
    slots: number;
    population: number;
    generations: number;
    elite: number;
    seed: number;
    smoothing?: number;
    /** Sequences used to initialise the distribution. */
    warmStart?: ReadonlyArray<readonly Segment[]>;
  },
): OptimizerResult {
  const nA = expansion.actions.length;
  const nD = expansion.durations.length;
  const smooth = opts.smoothing ?? 0.7;
  const rng = makeRng(opts.seed);
  let rollouts = 0;

  const pA: number[][] = Array.from({ length: opts.slots }, () => new Array(nA).fill(1 / nA));
  const pD: number[][] = Array.from({ length: opts.slots }, () => new Array(nD).fill(1 / nD));

  const actIndex = new Map<string, number>();
  expansion.actions.forEach((a, i) => actIndex.set(a.join("+"), i));
  const durIndex = new Map<number, number>();
  expansion.durations.forEach((d, i) => durIndex.set(Number(d.toFixed(3)), i));

  // Bias the initial distribution toward the warm-start sequences.
  if (opts.warmStart) {
    for (const ws of opts.warmStart) {
      for (let k = 0; k < Math.min(opts.slots, ws.length); k++) {
        const ai = actIndex.get(ws[k]!.action.join("+"));
        const di = durIndex.get(Number(ws[k]!.durationS.toFixed(3)));
        if (ai !== undefined) pA[k]![ai] = pA[k]![ai]! + 1 / Math.max(1, opts.warmStart.length);
        if (di !== undefined) pD[k]![di] = pD[k]![di]! + 1 / Math.max(1, opts.warmStart.length);
      }
    }
    for (let k = 0; k < opts.slots; k++) {
      normalize(pA[k]!);
      normalize(pD[k]!);
    }
  }

  const draw = (p: number[]): number => {
    const u = rng.u01();
    let acc = 0;
    for (let i = 0; i < p.length; i++) {
      acc += p[i]!;
      if (u <= acc) return i;
    }
    return p.length - 1;
  };

  let best: { seq: Segment[]; score: SeqScore } | null = null;
  let sampled = 0;

  for (let g = 0; g < opts.generations; g++) {
    const pop: Array<{ seq: Segment[]; ai: number[]; di: number[]; score: SeqScore }> = [];
    for (let n = 0; n < opts.population; n++) {
      const ai: number[] = [];
      const di: number[] = [];
      const seq: Segment[] = [];
      for (let k = 0; k < opts.slots; k++) {
        const a = draw(pA[k]!);
        const d = draw(pD[k]!);
        ai.push(a);
        di.push(d);
        const action = expansion.actions[a]!;
        if (action.length === 0) continue; // coast slot: contributes no segment
        seq.push({ action, durationS: expansion.durations[d]! });
      }
      rollouts += 1;
      sampled += 1;
      const score = evaluate(seq);
      pop.push({ seq, ai, di, score });
      if (!best || lexCompare(score.key, best.score.key) < 0) best = { seq, score };
    }
    pop.sort((x, y) => lexCompare(x.score.key, y.score.key));
    const elite = pop.slice(0, Math.max(1, opts.elite));
    for (let k = 0; k < opts.slots; k++) {
      const na = new Array(nA).fill(0);
      const nd = new Array(nD).fill(0);
      for (const e of elite) {
        na[e.ai[k]!] += 1;
        nd[e.di[k]!] += 1;
      }
      for (let i = 0; i < nA; i++) pA[k]![i] = smooth * (na[i] / elite.length) + (1 - smooth) * pA[k]![i]!;
      for (let i = 0; i < nD; i++) pD[k]![i] = smooth * (nd[i] / elite.length) + (1 - smooth) * pD[k]![i]!;
      normalize(pA[k]!);
      normalize(pD[k]!);
    }
  }

  const fallback = { seq: [] as Segment[], score: evaluate([]) };
  const chosen = best ?? fallback;
  return {
    seq: chosen.seq,
    score: chosen.score,
    rollouts,
    iterations: sampled,
    optimizer: "cem",
  };
}

function normalize(p: number[]): void {
  let s = 0;
  for (const x of p) s += x;
  if (s <= 0) {
    p.fill(1 / p.length);
    return;
  }
  for (let i = 0; i < p.length; i++) p[i] = p[i]! / s;
}
