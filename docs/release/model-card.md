# Model card — there is no successful control model

AETHER v0.1.0 ships a **benchmark**, not a solved policy.

| Slot | What it is | Gate result |
|---|---|---|
| `baseline` | Observation-only PD / pulse adapter (`AgentController`) | FAIL (train-10 all-gates 0/10, att 1/10) |
| `discrete-pulse-v2` | Archived failed planner (kNN-value **STOPPED**) | FAIL (train-10 all-gates 0/10) |
| Oracle (`--oracle`) | Truth-state reference, same actuator limits | Diagnostic only, not an agent |

## Intended use

1. Score new agents against the published gates without touching the physics kernel.
2. Compare against the frozen baseline.
3. Cite archive tags when discussing closed lines.

## Out of scope

- Demo-seed PD gain hunting
- Wiring archived planners
- Train-50 / hidden until a train-10 all-gates pass exists
- Treating `search_unreached` as physical impossibility

## Training data

None for a successful policy. Public `TRAIN_SEEDS` may be inspected. `HIDDEN_SEEDS` are blocked (`--force-hidden` required).

## Ethics / safety

Desktop research simulation. No flight hardware. Fuel floor 2.8 kg is a scoring constraint, not a safety-certified limit.
