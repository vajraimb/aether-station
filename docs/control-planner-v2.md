# Discrete Pulse Planner V2

## Status

- Physics baseline: `bdfff5b4c62733d7156d2f9cdeeaa75661d6c9f4`
- Physics kernel, actuator constraints, scoring gates, and UI are frozen.
- Existing observation-only controller remains the baseline (`mode=baseline`).
- V2 discrete pulse planner is selected with `mode=discrete-pulse-v2`.
- Overall benchmark status remains **PHYSICS PASS / CONTROL FAIL / OVERALL FAIL**.
- Train-10 stage-one attitude gate (60%) was not met. Train-50 and hidden were not run.

See `docs/control-v2-findings.md` and `outputs/eval-comparison-train10.json`.

## Objective

Replace continuous desired-torque projection and hand-patched phase logic with direct planning over legal thruster pulse primitives. The planner must improve cross-scenario success without tuning against the demo seed.

## Trust boundary

The flight controller receives only `Observation` values and its private belief state. It must not receive a simulator, truth state, private scenario, true parameters, fault injection time, or failed-thruster identity.

The rollout model is a controller-side approximate model. It uses estimated attitude, angular velocity, slosh state, fuel, thruster effectiveness, and FDIR beliefs only.

## Controller interface

```ts
export interface FlightController {
  reset(config: PublicControllerConfig): void
  step(observation: Readonly<Observation>): ControlCommand
  diagnostics(): Readonly<ControllerDiagnostics>
}
```

The baseline and V2 controller implement the same interface. Selection is configuration-driven; the simulator does not branch on controller implementation.

## Discrete action space

A primitive contains zero, one, or two healthy thrusters and a legal pulse duration.

```ts
type PulseDurationMs = 40 | 80 | 120 | 160 | 240 | 320

interface PulsePrimitive {
  thrusterIds: readonly number[]
  durationMs: PulseDurationMs
}
```

Rules:

- Include an explicit coast action.
- Generate all legal single-thruster actions.
- Generate all legal unordered two-thruster actions.
- Exclude isolated thrusters.
- Respect the two-active-thruster limit.
- Preserve the existing 18 N limit, 40 ms minimum pulse, 120 ms delay, fuel model, and command queue semantics.
- Deduplicate primitives with equivalent commanded wrench when useful, but retain alternatives when their force or fuel consequences differ.

## Prediction state

The reduced rollout state contains:

```ts
interface RolloutState {
  qBI: Quat
  omegaB: Vec3
  sliderS: number
  sliderV: number
  theta1: number
  theta1Dot: number
  theta2: number
  theta2Dot: number
  fuelMass: number
  pendingPulses: readonly PendingPulse[]
}
```

The first implementation may freeze slow parameter estimates over one horizon. It must retain actuator delay and pending pulse effects. Every approximation must be documented and covered by a rollout-versus-plant error test.

## Receding-horizon search

Initial operating range:

- Replan period: 0.5–1.0 s.
- Prediction horizon: 8–15 s.
- Decision interval: compatible with legal pulse durations and command delay.
- Beam width: 32–128.
- Execute only the first primitive, then replan from the next observation.

Each node stores state, command history, resource use, hard-gate status, and deterministic tie-break metadata. Search ordering must be deterministic for a fixed observation sequence and seed.

## Lexicographic objective

Do not collapse all requirements into an unconstrained weighted sum. Candidate plans are ordered by:

1. No actuator, slider, numerical, or fuel constraint violations.
2. Predicted terminal fuel at or above 2.8 kg.
3. Predicted terminal angular speed below 0.008 rad/s.
4. Predicted terminal attitude error below 1 degree.
5. Reduced perpendicular angular momentum and slosh energy.
6. Higher remaining fuel.
7. Lower terminal attitude and angular-speed error.
8. Fewer pulse transitions.

A smooth heuristic cost may rank candidates within the same lexicographic class, but cannot make an infeasible candidate outrank a feasible one.

## Terminal heuristic

Use quaternion shortest-arc attitude error and reject unwinding. The heuristic should include:

- geodesic attitude error;
- angular-speed norm;
- angular momentum perpendicular to the target eigenaxis;
- predicted slosh energy;
- fuel consumed;
- pulse switching count;
- margin to hard constraints.

The terminal heuristic is versioned and its constants are selected on a public training set only.

## Evaluation protocol

Keep three disjoint sets:

- Smoke: cheap deterministic development cases.
- Train: public fixed scenarios for controller selection.
- Hidden: private scenarios used only for final evaluation.

Ranking order:

1. Constraint violations.
2. All-gates success rate.
3. Attitude pass rate.
4. Fuel pass rate.
5. Parameter-estimation pass rate.
6. Median fuel remaining.
7. Median terminal attitude error.

Report median, p90, worst case, and per-seed outcomes. A demo-seed pass never substitutes for aggregate success.

## Required tests

- Action generator emits only legal, unique primitives.
- Failed thrusters disappear from the action set after isolation.
- Minimum pulse width and two-thruster concurrency are preserved.
- Fuel-floor violations are pruned, not merely penalized.
- Delay and pending pulse queues affect rollouts.
- Quaternion shortest-arc error is sign invariant.
- Planner output is deterministic.
- Controller cannot access truth or private scenario types.
- Reduced rollout agrees with the frozen plant over short open-loop pulse sequences within documented tolerances.
- Existing full physics audit remains unchanged and passing.

## Delivery stages

1. Introduce the controller interface and preserve baseline behavior.
2. Add discrete action generation and legality tests.
3. Add reduced rollout model and plant-parity tests.
4. Add deterministic beam search with hard fuel pruning.
5. Integrate V2 behind configuration and run baseline/V2 A/B evaluation.
6. Tune only on the public train set.
7. Run hidden evaluation once the public acceptance threshold is met.

## Acceptance

Stage-one target:

- Existing full physics suite remains passing.
- Fuel pass rate stays at 100% on train-10.
- Attitude pass rate reaches at least 60% on train-10.
- All-gates rate improves from 0%.

Final target:

- Hidden all-gates success rate at least 80%.
- Hidden attitude pass rate at least 90%.
- Hidden fuel pass rate at least 90%.
- FDIR isolation accuracy at least 95%.
- Parameter-estimation pass rate at least 80%.

Until those gates are met, the project status remains `PHYSICS PASS / CONTROL FAIL / OVERALL FAIL`.
