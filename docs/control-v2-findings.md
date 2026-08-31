# Control V2 findings

Physics baseline: `bdfff5b4c62733d7156d2f9cdeeaa75661d6c9f4`

Status: **PHYSICS PASS / CONTROL FAIL / OVERALL FAIL**

PR: **DRAFT / DO NOT MERGE**

This round stops handwritten planner patches. It adds a labeled
offline dataset and an auditable k-NN capture-cost V used as the
guidance / beam terminal heuristic. Beam width, physics, and the demo
seed are unchanged. Train-10, train-50, and hidden were not run.

## Offline dataset + capture-value (this round)

Artifacts: `outputs/reachability-dataset.json`, `src/sim/control/data/capture-value-knn.json`.

400 public states from a seed-free sampler (log-att 1.2–35°, closing /
rest / opening, optional one-jet isolation, optional pending pulse).
Each state is labeled by a budgeted high-fid search (8 s, eigen+beam):

| Label | Meaning | Count |
|---|---|---|
| `captured` | committed trajectory met att<1° AND \|ω\|<0.008 AND fuel>2.8 | **78** |
| `search_unreached` | no sequence inside the budget — **not a proof** | **322** |
| `proven_infeasible` | no torque and near-zero rate with att≥1° | **0** |

Train/val split by id hash: 323 / 77. k-NN (k=7) val MAE ≈ 18.6 cost
units — usable as a ranking heuristic, not a calibrated time predictor.

Online: lexicographic beam and guidance rank by V after fuel/hard
gates. Terminal phase still the short high-fid search. Handoff is
`V < 6` or att≤8, not “inside the 12° ball”. `TERMINAL_ENTRY_DEG`
stays 12 as a published constant.

## Previous offline 20-state study

Artifact: `outputs/capture-reachability-study.json`.

Public harvest: 20 states, 4 per bucket at 15 / 10 / 5 / 2.5 / 1.5°,
closing vs rest rate, healthy vs +Y isolated. Frozen-kernel rk4
(dt 10 ms), coast + every legal single and unordered pair, pulses
40–320 ms, 120 ms delay, max two jets, fuel floor 2.8 kg. Methods:
eigen, one-step beam lookahead (offline width 8 — `DEFAULT_BEAM_CONFIG`
untouched), CEM-style shortlist. Capture is the conjunction
att < 1° AND |ω| < 0.008 AND fuel > 2.8 kg. Attitude-ball-only is
logged and is not capture. Peek candidates were not counted.

| Gate | Result |
|---|---|
| Conjunctive capture (best method/horizon per state) | **15/20** |
| Attitude ball < 1° | **15/20** |
| 15° bucket | 3/4 |
| 10° bucket | 3/4 |
| 5° bucket | 3/4 |
| 2.5° bucket | 3/4 |
| 1.5° bucket | 3/4 |
| Fastest att-ball | 0.68 s (0.025 kg) |
| Fastest 5° capture | ~8.2 s |
| Fastest 15° capture | ~15.1 s |

The four misses are exactly `rest` + `plusY-isolated` at every bucket,
including 1.5°. Closing-rate and healthy-at-rest states captured, often
only with a pair, and usually only after > 1.6 s.

### Answers

- **Can these states enter the 1° set inside the fuel budget?** Partial. 15/20 yes. Resting states with +Y isolated did not in 30 s.
- **Minimum time?** Att-ball from 0.68 s; conjunctive capture from ~1.8 s (1.5° closing) up to ~22 s (15° with +Y failed, closing).
- **Minimum fuel?** 0.025 kg to att-ball; captures used on the order of 0.03–0.3 kg, well above the 2.8 kg floor from a 3.4 kg start.
- **Must attitude increase first?** Not observed on the recorded bests.
- **Must use dual thrusters?** Yes on every recorded conjunctive capture.
- **Need longer than 1.6 s non-monotonic trajectories?** Yes — att-ball time exceeded 1.6 s on successful captures outside the smallest closing cases. The previous 1.6 s terminal horizon cannot see these sequences.
- **Geometry review?** Not the first action for ≤5° closing/healthy states: those captured offline. Rest + +Y isolated failed even at 1.5°, so authority with that fault at rest is a real constraint. The remaining online gap is planner horizon / value, not “5° is unreachable”.

## Previous hierarchical round (still FAIL)

The previous findings snapshot named `f37f8b45`. That commit is **not** HEAD.
`80e2be5d` only added `outputs/eval-v2-smoke.json` and did not change control
logic. Envelope, reachable set, hierarchical capture, traces, and train-10
remain on this branch from `b96eb07`–`1db5ca6`.

Stage-one train-10 is still not met. Train-50 and hidden were not run.

## Stage-one gates (train-10, discrete-pulse-v2, previous round)

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

Conclusion: 0.5 s high-fid can own terminal capture. 3–5 s reduced is acceptable for guidance into a ~8–15° basin. 8–15 s reduced rollout cannot be responsible for the 1° gate.

## Terminal reachable set

`canCaptureWithinHorizon` uses the frozen rk4 kernel, 40–160 ms pulses, and delay-complete queues. Entry candidates 8/10/12/15° were measured on a public synthetic basin set (`outputs/terminal-entry-selection.json`). Closing capture rate inside one 1.6 s horizon is **0** at every candidate: a 2 s slew from rest is only ~3–4°.

Chosen public constant: **`TERMINAL_ENTRY_DEG = 12`**. Guidance hands off into a receding-horizon walk-in, not a one-shot 1° claim. No per-seed branch.

## Hierarchical planner

- **Guidance:** existing reduced beam, horizon 5 s, original expansion budget. Objective is the 12° basin (H_perp, wrong-way rate, fuel margin). It does not claim predicted final < 1°.
- **Terminal:** frozen-kernel rk4 (dt 10 ms, no event locator in search), horizon ~1 s, pulses 40/80/120/160 ms, 0.24 s replan, every healthy single plus one pair. Hard gates att < 1°, |ω| < 0.008, fuel > 2.8 kg at captured states. Rate gate is applied only near 2°.
- Existing `planBeam` is kept as the guidance backend and as a fallback. It is not overwritten.

## Failure traces

`outputs/v2-failure-traces/` for public train seeds 800017, 800068, 800119, 800102. Full CSV/JSON traces are generated by `npm run eval:traces` and are not stored in git (see `outputs/ARTIFACTS.md`). Summary remains in `outputs/v2-failure-traces/summary.json`.

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
- Offline study now shows 15/20 bucket states *are* conjunctively reachable in 5–30 s with pairs, so the online 1.6 s terminal horizon is too short rather than “1° is impossible”. Rest + +Y isolated remains unsolved even offline.

## Frozen physics

No edit to `math3d.ts`, `dynamics.ts`, or `audit.ts`. Actuator geometry, 18 N, 40 ms, 120 ms delay, two-jet limit, and Isp are unchanged. Full physics 155/155 from the previous round; this round did not re-run train-10.

## Not run

- No train-50.
- No hidden evaluation.
- No demo-seed special case.
- No beam-width increase as the main lever.
- 1° gate was not relaxed.
- Online hierarchical planner was not patched.

Draft PR #1 remains open and unmerged.
