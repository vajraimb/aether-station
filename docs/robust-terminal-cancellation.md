# Low-rate η_T-robust terminal cancellation

Branch: `research/robust-terminal-cancellation`

Not a controller. kNN-value remains stopped. Short macros, wrench library, and null-space search remain unwired. Old V2 Draft PR is untouched. Physics kernel frozen at `bdfff5b`. High-rate null-space expansion stays paused.

Status: **PHYSICS PASS (261/261) / NOMINAL CAPTURE 14% (6/42) / FUEL HOLD 100% / GATES FAIL / NOT WIRED / OVERALL FAIL**

## Commands

```
npm run check
npm run test:physics -- --full
npm run eval:robust-terminal
```

Artifact: `outputs/robust-terminal-study.json`

## Spec

- States: ‖ω‖ ≤ 0.015 rad/s, e_q ≤ 15°. Six families × seven masks (healthy + isolated 0..5) = 42 public states. Empty pending queue. Fuel 3.2 kg.
- Actions: singles 40/80/160 ms, pairs 40 ms, coast 80 ms. Delay 120 ms, max two jets, fuel floor 2.8 kg.
- Horizon: 24 segments (in the requested 8–32), beam 10.
- Intermediate ω_⊥ may grow; peak ω_⊥ > 0.03 rad/s is illegal.
- η_T interval is the estimator 2σ band around η̂ = 0.873 with P = 0.01, clipped to published `etaRange` [0.82, 0.96]. That interval is the whole published range — not a hand-set ±20% on (η, c1, c2, k12). Grid: 5 points including η̂. Search scores the worst of {η_min, η̂, η_max}.
- Reported cost J = |H_∥| + 2‖H_⊥‖ + 50 e_q + 10 Δm. Search ranks lex on (illegal, −nCaptured, att residual above 1°, rate residual above 0.008, J_worst, fuel).
- Baselines: max-projection single pulse, previous greedy-then-cancel, current `searchTerminal`.

## Offline gates

| Gate | Result |
|---|---|
| ≥ 80% att∧rate capture on nominal η | **14% (6/42)** FAIL |
| each failure mask ≥ 70% | 0–17% FAIL |
| worst-case η still fuel > 2.8 | **100%** PASS |
| beats `searchTerminal` by ≥ 15 pp and lower median J_worst | +14 pp, J_worst **higher** FAIL |
| READY TO WIRE | **NO** |

## Results

14 s for 42 states. Capture = att < 1° AND |ω| < 0.008 AND fuel > 2.8.

| Method | nominal capture | worst-case capture | fuel hold | median J_nom / J_worst |
|---|---:|---:|---:|---|
| single-pulse | 0/42 | 0/42 | 1.00 | 15.2 / 15.3 |
| greedy-then-cancel | 0/42 | 0/42 | 1.00 | 18.4 / 18.5 |
| terminal-search | 0/42 | 0/42 | 1.00 | 20.5 / 20.6 |
| robust-cancel | **6/42 (14%)** | 6/42 | **1.00** | 44.7 / 47.4 |

Per family, robust-cancel only:

| Family | ‖ω‖, e_q | capture | median att / ‖ω‖ after |
|---|---|---:|---|
| near-close | 0.006, 2.5°, closing | **6/7** | 0.46°, 0.005 |
| near-rest | 0.004, 3°, rest | 0/7 | 1.11°, 0.010 |
| mid-close | 0.010, 8°, closing | 0/7 | 1.65°, 0.028 |
| mid-rest | 0.008, 8°, rest | 0/7 | 4.32°, 0.024 |
| mid-open | 0.006, 8°, opening | 0/7 | 8.03°, 0.006 (coast) |
| entry-close | 0.012, 14°, closing | 0/7 | 6.95°, 0.031 |

near-close / isolated-1 is the only miss in that family: att 0.77° but ‖ω‖ = 0.012.

η_min vs η_hat vs η_max did not flip capture on the 6 successes (worst-case capture = nominal capture). Fuel never approached 2.8 (used ~0.07 kg). Peak ω_⊥ stayed under 0.03 except as a search cutoff.

## What this means

Cancel **is** a terminal-phase behaviour, and only if the state is already closing and inside a few degrees. From 2.5° closing, 18 segments can hit both gates on 6/7 masks. From 8° the beam can drive att to ~1.6° but dumps ‖ω‖ to 0.028 — the leaky projector again, now in time. From 14°, 24 segments take off about half the angle and still spin up. Opening-rate states never take the first step: att-first lex refuses a pulse that grows e_q while reversing ω.

Label: **search_unreached_under_budget** for 8–15° and for rest/opening, not physically impossible. 32 segments, a different first-step ranking, or a two-stage “reverse then cancel” planner might move that boundary. None of that is a reason to wire this search.

```
PHYSICS: PASS
REPRODUCIBILITY: PASS
LOW-RATE SET: 42 STATES, 7 MASKS, ‖ω‖≤0.015, e_q≤15°
η_T INTERVAL: ESTIMATOR 2σ CLIPPED TO [0.82, 0.96]
NOMINAL CAPTURE: 14% (near-close only)
PER-MASK: 0–17%
WORST-CASE FUEL: PASS
VS TERMINAL-SEARCH: MORE CAPTURE, WORSE J
READY TO WIRE: NO
ONLINE CONTROL: FAIL
OVERALL: FAIL
```
