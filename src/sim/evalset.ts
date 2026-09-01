/**
 * Public training seeds and held-out validation seeds.
 * Gains / planner weights may be inspected against TRAIN only.
 * HIDDEN is the acceptance set and is not a tuning target.
 */
export const TRAIN_SEEDS: number[] = Array.from({ length: 50 }, (_, i) => 800_000 + i * 17);
export const HIDDEN_SEEDS: number[] = Array.from({ length: 50 }, (_, i) => 910_000 + i * 97);
/** Publication smoke: first two public train seeds. Not a capture gate. */
export const SMOKE_SEEDS: number[] = TRAIN_SEEDS.slice(0, 2);

export const FUEL_FLOOR = 2.82;
export const FUEL_HARD = 2.8;
/** Estimate-side RCS cutoff. Fuel sensor is ±4 %, so stop above the hard gate. */
export const FUEL_STOP = 2.94;
export const ATT_GATE_DEG = 1;
export const RATE_GATE = 0.008;
export const PARAM_GATE = 0.15;
export const HORIZON_S = 8;
