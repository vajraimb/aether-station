# Cross-domain abstraction findings

Branch: `domain/inventory-validation`  
Base: `release/v0.1.0-benchmark@acb6d8f`  
AgentArena core files changed: **0**

```text
packages/agent-arena/src/agent.ts
packages/agent-arena/src/environment.ts
packages/agent-arena/src/runner.ts
packages/agent-arena/src/scorer.ts
```

Inventory used those interfaces as-is. AETHER stays on `src/sim/adapters/station.ts`.

## What generalized

| Piece | Used by AETHER | Used by inventory |
|---|---|---|
| `Agent<O,A>` | `SpaceStationAgent` | `ReorderPointAgent` |
| `Environment<O,A>` | (driver still uses Simulator.runAll) | `InventoryEnvironment` |
| `ArtifactScorer` | `SpaceStationScorer` | `InventoryScorer` |
| `EpisodeDriver` | `SpaceStationDriver` | `InventoryDriver` |
| `fileRecorder` | station CSV | inventory CSV |
| ExperimentStore | aether runs | `inventory-reorder-smoke` |

## What did not need to move into SimCore

Each domain owns its clock, delay, RNG, and event list:

- AETHER: 5 ms RK4, 120 ms command delay, quaternion plant
- Inventory: 1 day steps, observation lag, mulberry32

Those are not the same implementation. Do **not** extract `packages/sim-core` yet.

## Gaps (do not patch in this branch)

1. No single CLI flag `--domain inventory`. Two entry points (`npm run eval`, `npm run inventory:smoke`).
2. AETHER driver still wraps `Simulator.runAll` instead of gym `env.step`.
3. Arena has no built-in delayed channel; inventory implemented its own.
4. `scoreEpisode` is a one-liner. Both domains call `scorer.score(paths)` equivalently.
5. Observation leak checks are per-domain (`OBSERVATION_KEYS`), not an arena helper.

## Inventory result (not a policy claim)

Reorder-point smoke **0/6 all-gates**. Fill rates 0.42–0.89. Cash stayed positive. This validates the platform, not the stocking rule.

## Reuse matrix (SimCore still deferred)

| Capability | AETHER | Inventory | Extract? |
|---|---|---|---|
| Seeded RNG | yes | yes | candidate later |
| Delay buffer | yes | yes | candidate later |
| Event queue | yes | yes | candidate later |
| Run manifest | yes | yes | already in Arena / ledger |
| Recorder | yes | yes | already in Arena |
| RK4 | yes | no | no |
| Collision | yes | no | no |
| Quaternion | yes | no | no |
| Inventory lead time | no | yes | no |

Do not extract `packages/sim-core` or `packages/sim-runtime` until a dual-domain trajectory parity test exists. Inventory policy remains FAIL on purpose.
