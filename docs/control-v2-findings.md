# Control V2 findings

Physics baseline: `bdfff5b4c62733d7156d2f9cdeeaa75661d6c9f4`

Current branch HEAD at the start of the hierarchical-capture round:
`80e2be5d17b8c4745780ca568b1b9636426be5c7`

The previous findings snapshot named `f37f8b45b6bf41c71c5782cdcb47dd7beeeaaae5`.
That commit is **not** the current HEAD. `80e2be5d` only added
`outputs/eval-v2-smoke.json` and does **not** change control logic.

Status: **PHYSICS PASS / CONTROL FAIL / OVERALL FAIL**

Stage-one train-10 was not met, so train-50 and hidden were not run.

## Stage-one gates (train-10, discrete-pulse-v2)

| Gate | Target | Result |
|---|---|---|
| Full physics | all pass | pass |
| Fuel | 100% | 100% |
| FDIR | 100% | 100% |
| Attitude < 1° | ≥ 60% | 0% |
| All-gates | > 0% | 0% |

## Failure clustering (V2 train-10)

- **late / missing terminal capture** (800000, 800102, 800119, 800034, 800051, 800085, 800136, 800153): attitude 2.5–14° with rate often already under 0.008. Beam coasts after the predicted geodesic looks “good enough” over an 8 s horizon that cannot finish a 1° hold.
- **perpendicular momentum / wrong-way slew** (800017, 800068): terminal attitude 80–100°. Pair pulses that score well on the reduced model inject ω_perp in the plant. Isolation is still correct.
- **planner–plant mismatch**: parity is tight for 0.5–2 s open-loop (see `outputs/rollout-parity.json`); 8–15 s closed-loop search still uses the fast rigid step, so delay + CM motion + slosh accumulate. A 5 s reduced-rollout attitude error of ~0.028 rad (~1.6°) already exceeds the 1° terminal gate, so the reduced model cannot own sub-degree capture.
- **quantization / expansion budget**: 2800 deterministic expansions, 0.32–0.4 s primitives. Near 1° the 40 ms grid is fine; the miss is earlier, during eigenaxis tracking.
- **parameter estimation**: 50% of seeds meet the 0.15 relative-error gate (baseline 30%). Failures still sit on RLS bounds for c1/c2/k12; excitation is FDIR-driven, not information-optimal. No seed-specific patches.

## Rollout error envelope

Public synthetic grid (3 regimes × 2 fault states × 2 pending queues × 4 action classes × 7 horizons). Reduced `DEFAULT_ROLLOUT_CONFIG` vs frozen `PARITY_ROLLOUT_CONFIG`, same belief parameters. See `outputs/rollout-error-envelope.json`.

| Horizon | att p50 | att p90 | vs 1° gate |
|---|---|---|---|
| 0.5 s | 0.0041 rad (0.23°) | 0.0042 rad | below gate, near terminal tol 0.005 |
| 2 s | 0.026 rad (1.49°) | 0.026 rad | **above** 1° |
| 5 s | 0.026 rad (1.48°) | 0.027 rad | **above** 1° |
| 8–10 s | 0.033–0.035 rad (~2°) | ~0.035 rad | not usable for capture |

Conclusion: 0.5 s high-fidelity (or reduced, barely) can own terminal capture. 3–5 s reduced is acceptable for guidance into a ~8–15° basin. 8–15 s reduced rollout cannot be responsible for the 1° gate.

## Frozen physics

No bug was found that required editing `math3d.ts`, `dynamics.ts`, or `audit.ts`. Actuator geometry, 18 N, 40 ms, 120 ms delay, two-jet limit, and Isp are unchanged.

## CI

The intended GitHub Actions workflow is checked in as `docs/control-v2-ci.yml`.
Creating `.github/workflows/*.yml` requires the `workflow` OAuth scope, which
the device-auth token used to push this branch does not have. Copy the file
into `.github/workflows/control-v2.yml` from an account with that scope.

- No train-50.
- No hidden evaluation.
- No demo-seed special case.
- The offline pulse optimizer from main remains a truth-fed restart search, not a mixed-integer global optimum.
