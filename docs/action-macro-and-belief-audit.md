# Action macros and belief–truth audit

Branch: `control/action-macro-and-belief-audit` @ `99f0901` (parent `0d871b4`)

kNN-value line remains **STOPPED**. This branch does not retune k, table size, or beam width, and does not wire a new planner. Draft PR on `control/discrete-pulse-planner-v2` stays research record.

Status: **PHYSICS PASS (205/205) / KNN-VALUE ONLINE FAIL / MACROS NOT WIRED / CONTROL UNPROVEN / OVERALL FAIL**

Train-50 and hidden were not run. Physics kernel, beam width 28, and the demo seed are unchanged.

## Commands

```
npm run check
npm run test:physics -- --full
npm run eval:action-macros
npm run eval:belief-audit
```

## A. Belief–truth mismatch (train-10, original-v2)

Artifact: `outputs/belief-truth-audit-train10.json` (18 000 control cycles, ~217 s).

Closed loop is `discrete-pulse-v2` / `original-v2`. Estimator+FDIR never see truth. Phases: terminal if truth att ≤ 12°, else post-fault / pre-fault (10 s window) / nominal. `byClock` splits only on fault time.

| Phase | n | att geodesic ° mean / p90 / worst | \|Δω\| mean / p90 | fuel err p90 | slosh ∠ mean | param rel | FDIR mismatch | pending mismatch |
|---|---:|---|---|---|---|---|---|---|
| nominal | 6309 | 0.192 / 0.476 / 13.2 | 0.0023 / 0.0074 | 0.108 kg | 0.019 | 0.235 | 0.0% | 0.3% |
| pre_fault | 700 | 0.057 / 0.088 / 0.35 | 0.0005 / 0.0008 | 0.097 kg | 0.004 | 0.190 | 0.0% | 0.1% |
| post_fault | 3579 | 0.050 / 0.079 / 0.32 | 0.0005 / 0.0008 | 0.085 kg | 0.004 | 0.166 | 1.6% | 1.0% |
| terminal | 7412 | 0.052 / 0.079 / 0.15 | 0.0005 / 0.0008 | 0.083 kg | 0.004 | 0.149 | 0.4% | 4.5% |
| before fault | 7734 | 0.167 / 0.424 | 0.0020 | — | — | 0.220 | 0.0% | 0.4% |
| after fault | 10266 | 0.051 / 0.079 | 0.0005 | — | — | 0.157 | 0.9% | 3.4% |

Attitude geodesic and rate errors after startup are **small**: ~0.05° / 5e-4 rad/s. That cannot explain 95% kNN OOD, nor 0/10 capture.

What *is* large:

- Parameter relative error 0.15–0.24 (the 0.15 gate). Rollout and wrench both use (c1, c2, k12, η).
- FDIR mask mismatch is isolation delay, 1.6% of post-fault cycles, not a persistent wrong jet.
- Pending occupancy vs plant actual is 0.3–4.5% (delay + 50 ms log alignment), not a systematic queue bug.
- Terminal pending rate is higher because pulses are short and the 120 ms delay straddles the sample.

Closed-loop original-v2 on the same 10 seeds still **misses 1°**:

| seed | att ° | \|ω\| | fuel kg |
|---|---:|---:|---:|
| 800000 | 5.99 | 0.0019 | 3.427 |
| 800017 | 99.58 | 0.0418 | 2.837 |
| 800034 | 9.31 | 0.0044 | 2.868 |
| 800051 | 11.92 | 0.0043 | 2.831 |
| 800068 | 79.46 | 0.0155 | 2.995 |
| 800085 | 13.61 | 0.0026 | 3.383 |
| 800102 | 5.86 | 0.0022 | 3.118 |
| 800119 | 2.48 | 0.0018 | 3.420 |
| 800136 | 9.91 | 0.0046 | 2.845 |
| 800153 | 10.84 | 0.0022 | 2.835 |

Best att 2.48°. Fuel always > 2.8. Isolation matches the injected jet on these 10. All-gates 0/10. Same FAIL as the ablation, now with the estimator ruled out as the primary att failure.

## B. Action macro library (offline, not wired)

Artifact: `outputs/action-macro-library.json`.

222 legal 2–4 segment macros: pulse-coast 54, pulse-pulse 108, pulse-coast-pulse 48, pulse-coast-pulse-coast 12. Constraints: 40 ms min pulse, 120 ms delay on every pulse, max two jets, fuel floor 2.8 kg. Isolated jets are dropped. `controller-v2.ts`, `beam-planner.ts`, and `guidance-planner.ts` do not import this module.

10 public representative states (envelope × closing/opening/detumble). 1960 legal evaluations, 260 illegal (mostly isolated-jet).

| Metric | Value |
|---|---|
| Pareto size (hParReduction, ‖ΔH⊥‖, fuel, attDrift) | 102 |
| States where some macro dominates max-projection single | 8/10 (80%) |
| Evals with Δω_∥ < 0 | 820 / 1960 |
| Δω_∥ < 0 and Δω_⊥ ≤ 0 | **0 / 1960** |
| Δω_∥ < 0 and Δω_⊥ < 0.001 | 642 / 1960 |
| Robust hParReduction > 0 (signAgree ≥ 0.75) | 601 |

Max-projection single (the current eigen heuristic) itself leaks ⊥ (~6–8×10⁻⁴ rad/s per ~0.36 s) and on closing/detumble states has the **wrong sign** on ω_∥. Opposite-jet `pulse-pulse` and `pulse-coast-pulse` can flip that sign, but every reducing macro still grows ω_⊥. Best extra ∥ reduction vs the single-pulse baseline is ~0.0003 rad/s per macro — not a capture-scale effect.

**Do not wire.** The recommendation was to connect macros only if they dominate on target-axis momentum without inflating the perpendicular component. They do not.

## C. What this rules out

1. kNN-value as an online cost — already stopped; 95% OOD.
2. “The estimator’s q/ω is too wrong for any planner” — geodesic 0.05° after startup, ω error 5e-4. The planner sees essentially the right rigid-body attitude.
3. FDIR pointing at the wrong jet as the att failure mode — 0% mismatch before fault, 1.6% after (delay), isolation correct on these 10.
4. A short 2–4 pulse/coast macro that cancels ∥ without ⊥ — **not in this library**, on these states, with this thruster geometry.

## D. What remains

1. **Wrench geometry.** Offset 6-jet layout is rank-3 after one fail, but a single 40–160 ms pulse is not aligned with a random eigenaxis. Perp leakage per pulse is comparable to the useful ∥ component. A longer sequenced *null-space* trim (many shorts, not 2–4) or a slider-assisted torque may be required — that is a new study, not a controller.
2. **Parameter error in the rollout model.** 15–24% on (c, k12, η) is enough to rank the wrong pulse even when q/ω are right. Recalibrating the reduced model / using η-robust wrench ranking is in scope for a later planner, not this audit.
3. **Stage A (rate kill) vs Stage C (slew).** Several train-10 seeds sit at 5–14° with |ω| already < 0.005 and fuel > 2.8 — they are past detumble and still not in the 1° ball. That is a pointing/slew gap, not a rate gap. Two seeds (99°, 79°) never left the tumble.

## Status

```
PHYSICS: PASS (205/205, kernel frozen at bdfff5b)
TEST/REPRODUCIBILITY: PASS
FDIR: PASS on this train-10
FUEL CONSTRAINT HANDLING: PASS
BELIEF q/ω: ADEQUATE (~0.05° / 5e-4 rad/s after startup)
BELIEF parameters: WEAK (rel err ~0.15–0.24)
ACTION MACROS: NO CLEAN ∥/⊥ DOMINANCE — NOT WIRED
KNN-VALUE ONLINE CONTROL: FAIL (stopped)
OVERALL: FAIL
```
