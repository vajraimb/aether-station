# Wrench geometry and long-sequence null-space

Branch: `research/wrench-nullspace-and-long-sequence`

Not a controller. kNN-value remains stopped. Short 2–4 macros remain unwired. Old V2 Draft PR is untouched. Physics kernel frozen at `bdfff5b`.

Status: **PHYSICS PASS (236/236) / LINEAR SPAN RANK-3 / SHORT-PULSE ρ = O(1) / CANCEL HELPS NET ⊥ ON SOME STATES / NOT WIRED / OVERALL FAIL**

## Commands

```
npm run check
npm run test:physics -- --full
npm run eval:wrench-nullspace
```

Artifact: `outputs/wrench-nullspace-study.json`

## 1. Wrench set (40 ms primitives, η = 0.873)

200 Fibonacci-sphere target axes. Body-frame τ = (r − r_cm) × F. SVD of the 3 × n torque matrix.

| Mask | rank (singles) | σ | cond | best-primitive ρ median / p90 | ρ>10 |
|---|---:|---|---:|---|---:|
| healthy | 3 | 10.10, 10.01, 5.24 | 1.93 | 2.83 / 6.83 | 4% |
| isolated 0 or 1 | 3 | 10.07, 8.71, 4.27 | 2.36 | 2.83 / 6.83 | 4% |
| isolated 2 or 3 | 3 | 10.09, 8.42, 4.41 | 2.29 | 2.83 / 6.83 | 4% |
| isolated 4 or 5 | 3 | 10.01, 8.02, 4.67 | 2.14 | 2.83 / 6.83 | 4% |

Rank stays 3 after any one-jet fail. Condition is mild. That is **linear controllability**, not eigenaxis alignment: for a random e, the best legal 40 ms primitive still has ‖ΔH_⊥‖ ~ ⅓ of |ΔH_∥| (median ρ 2.8). Only 4% of axes admit ρ>10.

This is why 2–4 segment macros never reduced ω_∥ without growing ω_⊥: each primitive is a leaky projector.

## 2. 8–16 segment null-space search

Actions: singles 40/80 ms, pairs 40 ms, coast 80 ms. Delay 120 ms, max two jets, fuel floor 2.8 kg. Score uses **inertial angular impulse accumulated at fire time**, so intermediate ⊥ is allowed and only the net ∑ΔH is costed. Target = min(35% of |H_∥(0)|, 12 N·m·s).

Three methods on 8 public states:

| Method | mean ρ | mean |H_∥| fraction | target met |
|---|---:|---:|---:|
| greedy-par | 1.34 | 0.14 | 1/8 (terminal only) |
| greedy-then-cancel | **25.0** | 0.10 | 1/8 |
| beam-nullspace (w=18, 16 seg) | 4.90 | 0.14 | 1/8 |

Cancel is real: on medium-rate / terminal / closing / detumble, greedy-then-cancel drives net ‖ΔH_⊥‖ to 0.06–0.11 N·m·s while keeping most of the parallel impulse (ρ 31–77). Peak ⊥ during the sequence is still 1–3 N·m·s.

It does **not** solve high-rate: 16 pulses of 80 ms move only ~5% of |H_∥|. Fuel used ~0.033 kg. Beam, ranked lex on remaining parallel first, never switches into cancel until the parallel target is met — so it collapses to greedy-par on the 7/8 states that miss the target.

## 3. Parameter robustness (±20% on η, (c1,c2), k12; 8 corners)

Replayed beam sequences:

| State | target-met rate | median ρ | min ∥ fraction |
|---|---:|---:|---:|
| terminal healthy | 0.50 | 30.5 | 0.30 |
| all other 7 | 0.00 | 0.74–1.77 | 0.04–0.19 |

Impulse scales with η. A sequence sized on the nominal η misses the parallel target when η is 20% low. Not robust enough to wire.

## Answers

> In a failed mask, is there a legal long sequence that moves ΔH_∥ a lot while net ΔH_⊥ ≈ 0?

**Partly, and only after the rate is already small.** Terminal ( |ω|~0.01 ) : yes, cancel brings ρ from 1.8 to 30. High-rate tumble: no, 16 segments are impulse-poor and still leak. The geometry spans R³ but every short pulse is a leaky projector; cancel needs a second phase and a budget the high-rate case does not have in 16 steps.

## Do not wire

- Cancel is a **phase**, not a replacement for the planner.
- Beam-as-lex-on-parallel never uses that phase unless the parallel target is already cheap.
- Robustness to 15–24% parameter error is not established outside terminal.
- Online control remains FAIL. Train-50 / hidden not run.

```
PHYSICS: PASS
WRENCH RANK AFTER ONE FAIL: 3
RANDOM-AXIS SHORT-PULSE ρ: O(1)  (median 2.8, 4% > 10)
NULLSPACE CANCEL: HELPS NET ⊥ WHEN |ω| IS ALREADY SMALL
PARAMETER ROBUSTNESS: FAIL except terminal
READY TO WIRE: NO
ONLINE CONTROL: FAIL
OVERALL: FAIL
```
