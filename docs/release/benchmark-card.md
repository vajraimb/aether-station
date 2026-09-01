# AETHER Benchmark Card v0.1.0

**Name:** AETHER Research Benchmark v0.1.0  
**Tag:** `v0.1.0-benchmark`  
**Freeze:** `benchmark/research-phase-complete-v1` @ `4b5d7a6`  
**Physics kernel:** `bdfff5b4c62733d7156d2f9cdeeaa75661d6c9f4` (`math3d.ts` / `dynamics.ts` / `audit.ts`)

```text
Physics validity: PASS
Benchmark infrastructure: PASS
Baseline control: FAIL
Task solved: NO
Research benchmark ready: YES
```

This is **not** a successful attitude-recovery controller. It is a post-fault, partially observed, discrete-pulse, sloshing 6-DOF lab with honest FAIL labels.

## Task

Recover a tumbling cylindrical station after one RCS jet fails in flight. The agent sees only a noisy, delayed `Observation`. It never sees truth, hidden slosh coefficients `c1 / c2 / k12 / ηT`, or the wall-clock fault time.

## Gates (all required)

| Gate | Threshold |
|---|---|
| Attitude | geodesic error < 1° |
| Rate | ‖ω‖ < 0.008 rad/s |
| Fuel | remaining > 2.8 kg |
| Parameters | relative error < 0.15 |
| FDIR | isolated thruster matches plant, isolation delay < 3 s |
| Slosh | energy ratio < 0.08 |
| Impact | slider impact < 0.25 m/s |

Isolation delay is `isolationTime − faultInjectionTime`.

## What ships

- Frozen Newton–Euler plant
- Observation-only `FlightController` (`src/sim/arena.ts`)
- Baseline agent (fails the gates)
- File scorer (CSV + JSONL, no live Simulator)
- Public train-50 / hidden-50 seed lists (hidden blocked)
- Research archive tags, not live planners

## What does not ship as an agent

- kNN capture-value (`archive/control-v2-final`)
- 2–4 pulse macros (`archive/action-macro-belief-audit`)
- Null-space sequences (`archive/wrench-nullspace-study`)
- Robust terminal cancellation (`archive/robust-terminal-study`)

Those are research records. Draft PR #1 must not merge.

## Reproduce

See [`reproduction.md`](reproduction.md). Smoke:

```bash
npm ci
npm run check
npm run test:physics -- --full
npm run eval -- --controller baseline --set smoke
```
