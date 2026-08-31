# Control V2 findings

Physics baseline: `bdfff5b4c62733d7156d2f9cdeeaa75661d6c9f4`

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
- **planner–plant mismatch**: parity is tight for 0.5–2 s open-loop (see `outputs/rollout-parity.json`); 8–15 s closed-loop search still uses the fast rigid step, so delay + CM motion + slosh accumulate.
- **quantization / expansion budget**: 2800 deterministic expansions, 0.32–0.4 s primitives. Near 1° the 40 ms grid is fine; the miss is earlier, during eigenaxis tracking.
- **parameter estimation**: 50% of seeds meet the 0.15 relative-error gate (baseline 30%). Failures still sit on RLS bounds for c1/c2/k12; excitation is FDIR-driven, not information-optimal. No seed-specific patches.

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
