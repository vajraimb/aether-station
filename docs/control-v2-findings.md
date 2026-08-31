# Control V2 findings

Physics baseline: `bdfff5b4c62733d7156d2f9cdeeaa75661d6c9f4`

Status: **PHYSICS PASS / CONTROL FAIL / OVERALL FAIL**

The previous findings snapshot named `f37f8b45`. That commit is **not** HEAD.
`80e2be5d` only added `outputs/eval-v2-smoke.json` and did not change control
logic. This round starts from `80e2be5d` and adds envelope, reachable set,
hierarchical capture, traces, and a new train-10.

Stage-one train-10 is still not met. Train-50 and hidden were not run.

## Stage-one gates (train-10, discrete-pulse-v2, this round)

| Gate | Target | Result |
|---|---|---|
| Full physics | 155/155 | **pass** (order 3.87, 180 s drift < 1e-4) |
| Fuel | 100% | **90%** (800051 remaining 2.788 kg) |
| FDIR | 100% | **100%** |
| Attitude < 1° | ≥ 60% | **0%** (best 10.25°) |
| All-gates | > 0% | **0%** |

Eval artifact: `outputs/eval-v2-train10.json`. Comparison: `outputs/eval-comparison-train10.json`.

## Rollout error envelope

Public synthetic grid (3 regimes × 2 fault states × 2 pending queues × 4 action classes × 7 horizons). Reduced `DEFAULT_ROLLOUT_CONFIG` vs frozen `PARITY_ROLLOUT_CONFIG`, same belief parameters. See `outputs/rollout-error-envelope.json`.

| Horizon | att p50 | att p90 | vs 1° gate |
|---|---|---|---|
| 0.5 s | 0.0041 rad (0.23°) | 0.0042 rad | below gate, near terminal tol 0.005 |
| 2 s | 0.026 rad (1.49°) | 0.026 rad | **above** 1° |
| 5 s | 0.026 rad (1.48°) | 0.027 rad | **above** 1° |
| 8–10 s | 0.033–0.035 rad (~2°) | ~0.035 rad | not usable for capture |

Conclusion: 0.5 s high-fidelity can own terminal capture. 3–5 s reduced is acceptable for guidance into a ~8–15° basin. 8–15 s reduced rollout cannot be responsible for the 1° gate.

## Terminal reachable set

`canCaptureWithinHorizon` uses the frozen rk4 kernel, 40–160 ms pulses, and delay-complete queues. Entry candidates 8/10/12/15° were measured on a public synthetic basin set (`outputs/terminal-entry-selection.json`). Closing capture rate inside one 1.6 s horizon is **0** at every candidate: a 2 s slew from rest is only ~3–4°.

Chosen public constant: **`TERMINAL_ENTRY_DEG = 12`**. Guidance hands off into a receding-horizon walk-in, not a one-shot 1° claim. No per-seed branch.

## Hierarchical planner

- **Guidance:** existing reduced beam, horizon 5 s, original expansion budget. Objective is the 12° basin (H_perp, wrong-way rate, fuel margin). It does not claim predicted final < 1°.
- **Terminal:** frozen-kernel rk4 (dt 10 ms, no event locator in search), horizon ~1 s, pulses 40/80/120/160 ms, 0.24 s replan, every healthy single plus one pair. Hard gates att < 1°, |ω| < 0.008, fuel > 2.8 kg at captured states. Rate gate is applied only near 2°.
- Existing `planBeam` is kept as the guidance backend and as a fallback. It is not overwritten.

## Failure traces

`outputs/v2-failure-traces/` for public train seeds 800017, 800068, 800119, 800102.

| Seed | Prior V2 att | This round | Notes |
|---|---|---|---|
| 800017 | 99.6° | 99.2° | Still wrong-way. FDIR 3/3. Terminal reachable rate 0. |
| 800068 | 79.5° | 79.9° | Still wrong-way. FDIR 5/5. |
| 800119 | 2.48° (best / closest to 1°) | 23.8° | Guidance reached the basin; terminal walk-in did not finish. |ω| 0.0066. |
| 800102 | 5.86° (next closest) | 28.1° | Same pattern: basin entry without 1° capture. |

Each file includes attitude, w_parallel, |w_perp|, selected primitive, predicted vs actual, fuel margin, planner phase, reachable flag, FDIR mask.

## Why stage-one still fails

- Reduced 2–5 s attitude error (~1.5°) is larger than the 1° gate, so guidance cannot own capture.
- High-fid 1 s terminal authority of a 160 ms pulse is ~0.004 rad/s. From 8–12° with |ω| already small, receding search often ranks coast first or applies a weakly aligned single that does not close geodesic error.
- 800017 / 800068 never enter the basin; pair-induced ω_perp from the reduced guidance beam still dominates.
- 800051 spent to 2.788 kg (below 2.8). Fuel-floor pruning is active in search but the plant delay queue can finish a committed pulse after the estimate crosses the floor.
- Parameter relative error still sits on RLS bounds for c1/c2/k12 (20% pass). No seed-specific patches.

## Frozen physics

No edit to `math3d.ts`, `dynamics.ts`, or `audit.ts`. Actuator geometry, 18 N, 40 ms, 120 ms delay, two-jet limit, and Isp are unchanged. Full physics 155/155.

## Not run

- No train-50.
- No hidden evaluation.
- No demo-seed special case.
- No beam-width increase as the main lever.
- 1° gate was not relaxed.

Draft PR #1 remains open and unmerged.
