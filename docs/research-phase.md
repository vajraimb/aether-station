# AETHER Research Benchmark v1

Physics baseline: `bdfff5b4c62733d7156d2f9cdeeaa75661d6c9f4`

Aether Station is a **post-fault, partially observed, discrete-pulse, sloshing attitude-recovery benchmark**. Online control did not pass the published gates. This document freezes that result. It is not a controller.

```text
AETHER Research Benchmark v1

PHYSICS: PASS
BENCHMARK INFRASTRUCTURE: PASS
TRUTH/OBSERVATION ISOLATION: PASS
REPRODUCIBILITY: PASS
CONTROL BASELINES: FAIL
ROBUST TERMINAL CANCELLATION: FAIL
READY TO WIRE: NO
OVERALL CONTROL TASK: FAIL
RESEARCH PHASE: COMPLETE
```

Immutable tags (annotated) — prefer these over live branch tips:

| Tag | SHA | Branch archived |
|---|---|---|
| `archive/control-v2-final` | `0d871b4` | `control/discrete-pulse-planner-v2` |
| `archive/action-macro-belief-audit` | `7c3eca0` | `control/action-macro-and-belief-audit` |
| `archive/wrench-nullspace-study` | `38d2c74` | `research/wrench-nullspace-and-long-sequence` |
| `archive/robust-terminal-study` | `3b6b5da` | `research/robust-terminal-cancellation` |
| `benchmark/research-phase-complete-v1` | `4b5d7a6` | `benchmark/research-phase-complete` |

## Frozen algorithm branches

Do not accumulate new planners on these. They are the research archive.

| Branch | Tip | Record |
|---|---|---|
| `control/discrete-pulse-planner-v2` | `0d871b4` | kNN-value / hierarchical / original-v2. Train-10 all-gates **0/10**. kNN-value **STOPPED**. Draft PR, do not merge. |
| `control/action-macro-and-belief-audit` | `7c3eca0` | Estimator q/ω is not the att failure. 2–4 macros: **0/1960** reduce ω_∥ without growing ω_⊥. Not wired. |
| `research/wrench-nullspace-and-long-sequence` | `38d2c74` | Rank-3 after one fail; short-pulse ρ median 2.83; 16-seg cancel is terminal-only. Not wired. |
| `research/robust-terminal-cancellation` | `3b6b5da` | 42 low-rate states, estimator 2σ η_T. Nominal capture **6/42 (14%)**, near-close only. Not wired. |

This branch (`benchmark/research-phase-complete`) only names the layers and the freeze. Physics tests **270/270**. It does not retune, wire, or run train-50 / hidden. v1 is tag `benchmark/research-phase-complete-v1` @ `4b5d7a6`; later commits on this branch are ledger only.

## Lines that are closed

1. Continuous torque map + PD (demo-seed gain hunt)
2. Hand-tuned BCB
3. Long-horizon reduced-model beam as a capture planner
4. High-fidelity short-horizon terminal search as a capture planner
5. kNN capture-value
6. 2–4 segment action macros
7. 16-segment null-space cancellation
8. Parameter-robust terminal cancellation

Failures are `search_unreached_under_budget` or `online_gate_fail`, not proofs of physical impossibility. They are still not a reason to wire.

## What stays

- Frozen Newton–Euler kernel (`math3d` / `dynamics` / `audit` @ `bdfff5b`)
- Observation-only `FlightController` contract
- Independent file scorer
- Public train / hidden seed lists (hidden unused)
- Visualization
- Baseline controller
- Reachability, belief–truth, wrench, and robust-terminal artifacts as ledger entries
- Failed V2 families as named baselines (`baseline`, `discrete-pulse-v2`)

## What does not happen next on this freeze

- No new online controller
- No wiring of macros / null-space / robust-terminal
- No train-50 / hidden campaign
- No Draft PR merge
- No physics-kernel edit
- No UI retune

Hardware co-design (`research/actuator-codesign`) is a **new** question: minimal plant change that makes train-10 reachable. It is not opened here.
