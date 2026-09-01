# AgentArena card v0.2.0

**Tag:** `v0.2.0-arena`  
**Parent tag:** `v0.1.0-benchmark` @ `486099c` (immutable)  
**Physics kernel:** `bdfff5b` (unchanged)

```text
Physics validity: PASS
Benchmark infrastructure: PASS
AgentArena: PASS
AETHER adapter: PASS
Inventory adapter: PASS
Cross-domain validation: PASS
Baseline control: FAIL
Inventory policy: FAIL
Task solved: NO
SimCore extraction: DEFERRED
```

This release adds a second domain (inventory) that uses AgentArena **without** changing:

```text
packages/agent-arena/src/agent.ts
packages/agent-arena/src/environment.ts
packages/agent-arena/src/runner.ts
packages/agent-arena/src/scorer.ts
```

## Smoke

```bash
npm run arena -- --domain station --agent baseline --scenario smoke
npm run arena -- --domain inventory --agent reorder-point --scenario smoke
```

| Domain | run_id | Tracked evidence |
|---|---|---|
| station | `station-baseline-smoke` | `outputs/runs/station-baseline-smoke/{manifest,metrics,claims}.json` |
| inventory | `inventory-reorder-smoke` | `outputs/inventory/smoke.json` and `outputs/runs/inventory-reorder-smoke/{manifest,metrics,claims}.json` |

Per-scenario CSV/JSONL traces are gitignored. Re-score from those files after a local smoke.

There is **no** `outputs/inventory-smoke.json`. The aggregate is `outputs/inventory/smoke.json`.
