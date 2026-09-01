# Reproduction — AgentArena v0.2.0

Pinned references (prefer tags and hashes over branch names):

| Ref | Value |
|---|---|
| Release tag | `v0.2.0-arena` |
| Parent tag | `v0.1.0-benchmark` @ `486099c` (do not move) |
| Freeze tag | `benchmark/research-phase-complete-v1` @ `4b5d7a6` |
| Physics kernel | `bdfff5b4c62733d7156d2f9cdeeaa75661d6c9f4` |
| Branch (moving) | `release/v0.2.0-arena` |

## Commands

```bash
git checkout v0.2.0-arena
npm ci
npm run check
npm run test:physics -- --full
npm run test:arena
npm run arena -- --domain station --agent baseline --scenario smoke
npm run arena -- --domain inventory --agent reorder-point --scenario smoke
npm run ledger:ingest -- --memory
npm run release:manifest
```

## Expected

| Check | Result |
|---|---|
| physics kernel diff vs `bdfff5b` | empty |
| `test:physics --full` | PASS |
| station smoke | runner completes; **control FAIL** |
| inventory smoke | runner completes; **policy FAIL** (0/6) |
| AgentArena core vs `acb6d8f` | empty diff on agent/environment/runner/scorer |

## Inventory evidence

| Item | Value |
|---|---|
| Command | `npm run inventory:smoke` or `npm run arena -- --domain inventory --agent reorder-point --scenario smoke` |
| Tracked aggregate | `outputs/inventory/smoke.json` |
| Unified run | `outputs/runs/inventory-reorder-smoke/{manifest,metrics,claims}.json` |
| Version controlled | yes (JSON above). Per-scenario `outputs/inventory/<id>/*.csv` is gitignored |
| Re-score | `InventoryScorer.score(trajectory.csv, events.jsonl)` after a local smoke |
| Ledger `run_id` | `inventory-reorder-smoke` |

Not a path: `outputs/inventory-smoke.json`.
