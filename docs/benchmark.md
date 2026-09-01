# Benchmark contract

Three reusable surfaces. Physics kernel stays where it is; these files only re-export.

| Layer | Module | Contains | Must not contain |
|---|---|---|---|
| **SimCore** | [`src/sim/core.ts`](../src/sim/core.ts) | Frozen plant: public config, observation keys, 3-D math, mass/H/slosh queries, actuator limits | Controllers, Simulator, hidden scenario |
| **AgentArena** | [`packages/agent-arena`](../packages/agent-arena/src/index.ts) | Generic `Agent` / `Environment` / file `ArtifactScorer` / episode runner | Domain physics, Simulator, quaternions, slosh |
| **AETHER adapter** | [`src/sim/adapters/station.ts`](../src/sim/adapters/station.ts) | `SpaceStationAgent`, `SpaceStationDriver`, `SpaceStationScorer` | Must not live inside `packages/agent-arena` |
| **Ledger** | [`outputs/ARTIFACTS.md`](../outputs/ARTIFACTS.md) | Commands that regenerate aggregate JSON; physics SHA | Per-seed CSV, `outputs/runs/` |

## Agent protocol

```ts
import { createFlightController } from "./arena";

const agent = createFlightController(plant, { mode: "baseline" });
const cmd = agent.step(observation); // Observation only
```

Gates (unchanged): att < 1°, |ω| < 0.008, fuel > 2.8 kg. Isolation delay is `isolationTime − faultInjectionTime`.

## Physics freeze

```
git diff bdfff5b4c62733d7156d2f9cdeeaa75661d6c9f4 -- src/sim/math3d.ts src/sim/dynamics.ts src/sim/audit.ts
```

must be empty. `npm run test:physics -- --full` is the kernel + contract suite.

## Ledger

See `outputs/ARTIFACTS.md`. Research studies (belief audit, macros, wrench, robust-terminal) are records, not flight code. Quick JSON is gitignored.
